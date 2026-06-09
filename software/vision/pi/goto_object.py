#!/usr/bin/env python3
"""
goto_object.py — click-to-go with built-in color detection (no depth node).

Flow:
  1. CENTER  : HSV blob detection on raw left image -> pixel IBVS on j1/j5.
               No SGBM running -> fast frame rate (Pi not overloaded).
  2. DEPTH   : one-shot stereo disparity on a single captured L/R pair after
               centering. Depth node is NOT needed at all.
  3. COMPUTE : FK(joints) + camera mount -> object's base-frame Cartesian position.
  4. MOVE    : IK for standoff, j5 corrected to aim camera, one joint command.
  5. VERIFY  : re-detect at the new pose, report residual.

HSV params come from the GUI click (run_approach_object.sh passes them as ROS params).
"""
import json, os, socket, sys, threading, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, JointState

sys.path.insert(0, "/home/armpi/ros_nodes")
import moveo_publisher as mp

CAM_T = np.array([-0.05, 0.03, 0.0])  # left camera in EE (model frame). Final reported positions (obj_user etc.) are rotated 180° Z into the public user frame via chain_to_user.
JOINT_LIM = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.75, 1.75)]
MIN_BLOB_AREA = 300  # px²


def fk_base_ee(q5):
    # Returns the 4x4 in the *internal model* frame.
    # CAM_T and depth rays are expressed in EE model; obj_base is therefore in base_model.
    # Callers then use to_user / mp.chain_to_user (the 180° Z) to get final public user coords.
    if hasattr(mp, 'forward_kinematics_matrix'):
        return mp.forward_kinematics_matrix(q5)
    return mp.MOVEO_CHAIN.forward_kinematics([0.0] + list(q5) + [0.0])


def to_user(p):
    # Apply 180° Z (model → final public user frame).
    if hasattr(mp, "chain_to_user"):
        return mp.chain_to_user(p)
    # fallback 180 Z
    return [-float(p[0]), -float(p[1]), float(p[2])]


def img_to_np(msg):
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class GotoObject(Node):
    def __init__(self):
        super().__init__("goto_object")
        p = self.declare_parameter
        self.standoff    = p("standoff_m",    0.22).value
        self.center_tol  = p("center_tol_px", 15.0).value  # pixels from image centre
        self.jog         = p("jog_rad",       0.05).value
        self.gain        = p("center_gain",   0.4).value
        self.settle      = p("settle_s",      1.2).value
        self.n_samples   = p("measure_samples", 5).value
        self.max_center  = p("max_center_iters", 12).value
        self.max_reach   = p("max_reach_m",   0.42).value
        self.calib_path  = p("calib", os.path.expanduser("~/vision/stereo_calib.yaml")).value
        self.host        = p("host", "127.0.0.1").value
        self.port        = p("port", 9000).value
        # HSV params passed from run_approach_object.sh
        self.hsv_lo = np.array([p("h_lo", 0).value,   p("s_lo", 50).value,  p("v_lo", 50).value])
        self.hsv_hi = np.array([p("h_hi", 179).value, p("s_hi", 255).value, p("v_hi", 255).value])

        # Load stereo calibration for one-shot depth
        self._cal = self._load_cal(self.calib_path)

        # State
        self._left  = None   # (stamp_ns, bgr_ndarray)
        self._right = None   # (stamp_ns, bgr_ndarray)
        self._centroid_px = None   # (cx, cy) in left image
        self._centroid_stamp = None  # Time
        self._cmd_time = None
        self._q = None
        self._abort = False
        self._lock = threading.Lock()

        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, "/stereo/left/image_raw",  self._lcb, q)
        self.create_subscription(Image, "/stereo/right/image_raw", self._rcb, q)
        self.create_subscription(Empty, "/approach/stop",
                                 lambda m: setattr(self, "_abort", True), 10)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = None
        for _ in range(15):
            try:
                self._sock = socket.create_connection((self.host, self.port), timeout=5); break
            except OSError:
                time.sleep(1.0)
        if self._sock is None:
            raise RuntimeError(f"could not reach moveo_publisher {self.host}:{self.port}")
        self._sf = self._sock.makefile("r")
        self.get_logger().info(
            f"goto_object: standoff {self.standoff*100:.0f}cm, "
            f"HSV lo={list(self.hsv_lo)} hi={list(self.hsv_hi)}. "
            f"Object in view + Send All Joints. j4=0.  !! E-STOP ready.")
        threading.Thread(target=self._run, daemon=True).start()

    # ── calibration ──
    def _load_cal(self, path):
        """Load stereo calibration and compute rectification maps at half resolution.
        Uses the same approach as stereo_depth_node: scale K, then call stereoRectify
        at the scaled size so OpenCV produces a correctly-scaled Q matrix."""
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f"cannot open calibration: {path}")
        g = lambda n: fs.getNode(n).mat()
        K1, D1, K2, D2 = g("K1"), g("D1"), g("K2"), g("D2")
        R, T = g("R"), g("T")
        iw = int(fs.getNode("image_width").real())
        ih = int(fs.getNode("image_height").real())
        fs.release()
        scale = 0.5   # half-res for SGBM (same as depth node default)
        sw, sh = int(round(iw*scale)), int(round(ih*scale))
        S = np.array([[scale,0,0],[0,scale,0],[0,0,1]], np.float64)
        K1s, K2s = S @ K1, S @ K2
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1s, D1, K2s, D2, (sw,sh), R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        mapL = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, (sw,sh), cv2.CV_16SC2)
        mapR = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, (sw,sh), cv2.CV_16SC2)
        sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=64, blockSize=7,
            P1=8*3*49, P2=32*3*49, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        # Keep full-res K1 for raw-pixel -> normalised-ray conversion
        return {"mapL": mapL, "mapR": mapR, "Q": Q,
                "sgbm": sgbm, "sw": sw, "sh": sh,
                "fx": float(K1[0,0]), "fy": float(K1[1,1]),
                "cx": float(K1[0,2]), "cy": float(K1[1,2])}

    # ── callbacks ──
    def _lcb(self, msg):
        bgr = img_to_np(msg)
        cx, cy = self._detect_color(bgr)
        with self._lock:
            self._left = (stamp_ns(msg), bgr)
            if cx is not None:
                self._centroid_px = (cx, cy)
                self._centroid_stamp = Time.from_msg(msg.header.stamp)
            else:
                self._centroid_px = None

    def _rcb(self, msg):
        with self._lock:
            self._right = (stamp_ns(msg), img_to_np(msg))

    def _jc(self, msg):
        if len(msg.position) >= 5:
            with self._lock:
                self._q = np.array(msg.position[:5], float)

    # ── color detection (fast, no SGBM) ──
    def _detect_color(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if self.hsv_lo[0] > self.hsv_hi[0]:  # hue wrap (red/magenta ~0/179)
            m1 = cv2.inRange(hsv, np.array([self.hsv_lo[0], self.hsv_lo[1], self.hsv_lo[2]]),
                             np.array([179, self.hsv_hi[1], self.hsv_hi[2]]))
            m2 = cv2.inRange(hsv, np.array([0, self.hsv_lo[1], self.hsv_lo[2]]),
                             np.array([self.hsv_hi[0], self.hsv_hi[1], self.hsv_hi[2]]))
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(hsv, self.hsv_lo, self.hsv_hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_BLOB_AREA:
            return None, None
        M = cv2.moments(c)
        if M['m00'] == 0:
            return None, None
        return int(M['m10']/M['m00']), int(M['m01']/M['m00'])

    # ── one-shot depth after centering ──
    def _measure_depth(self):
        """Capture one synced L/R pair and return metric depth (m) at the
        colour blob centroid, plus the normalised camera-plane (x,y) offset."""
        t0 = time.time()
        while time.time() - t0 < 8.0:
            with self._lock:
                l = self._left; r = self._right; cxy = self._centroid_px
            if l and r and cxy and abs(l[0]-r[0]) < 5_000_000:
                break
            time.sleep(0.05)
        else:
            return None, None, None
        sw, sh = self._cal["sw"], self._cal["sh"]
        li = cv2.resize(l[1], (sw,sh)); ri = cv2.resize(r[1], (sw,sh))
        lr = cv2.remap(li, *self._cal["mapL"], cv2.INTER_LINEAR)
        rr = cv2.remap(ri, *self._cal["mapR"], cv2.INTER_LINEAR)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        disp = self._cal["sgbm"].compute(gl, gr).astype(np.float32) / 16.0
        pts3d = cv2.reprojectImageTo3D(disp, self._cal["Q"])
        # Scale centroid coords to the proc resolution
        scale_x = sw / l[1].shape[1]; scale_y = sh / l[1].shape[0]
        u = int(np.clip(cxy[0]*scale_x, 0, sw-1))
        v = int(np.clip(cxy[1]*scale_y, 0, sh-1))
        # Sample depth in a window around the centroid
        win = 8
        y0,y1 = max(0,v-win), min(sh,v+win+1)
        x0,x1 = max(0,u-win), min(sw,u+win+1)
        patch = pts3d[y0:y1, x0:x1, 2]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if len(valid) < 5:
            # Fall back to blob-wide median
            all_valid = pts3d[:,:,2]
            all_valid = all_valid[(all_valid>0) & np.isfinite(all_valid)]
            if len(all_valid) < 10:
                return None, None, None
            z_mm = float(np.median(all_valid))
        else:
            z_mm = float(np.median(valid))
        z_m = z_mm / 1000.0
        # Camera-plane normalised offset at the centroid (for object position calc)
        x_n = (cxy[0] - self._cal["cx"]) / self._cal["fx"]  # metres/metre (dimensionless)
        y_n = (cxy[1] - self._cal["cy"]) / self._cal["fy"]
        return z_m, x_n, y_n

    # ── pixel-only measurement for centering ──
    def _measure_px(self):
        """Return median pixel offset (dx, dy) from image centre over N fresh frames."""
        with self._lock:
            cmd_t = self._cmd_time
        deadline = (cmd_t + Duration(seconds=float(self.settle))) if cmd_t else None
        samples = []; last_stamp = None
        t0 = time.time()
        while time.time() - t0 < self.settle + 6.0:
            with self._lock:
                cxy   = self._centroid_px
                stamp = self._centroid_stamp
                left  = self._left
            if cxy is not None and stamp is not None and left is not None:
                h, w = left[1].shape[:2]
                fresh  = (deadline is None) or (stamp > deadline)
                newer  = (last_stamp is None) or (stamp > last_stamp)
                if fresh and newer:
                    dx = cxy[0] - w/2.0
                    dy = cxy[1] - h/2.0
                    samples.append(np.array([dx, dy]))
                    last_stamp = stamp
                    if len(samples) >= self.n_samples:
                        break
            time.sleep(0.03)
        if len(samples) < max(1, self.n_samples // 2):
            return None
        return np.median(np.array(samples), axis=0)  # (dx_px, dy_px)

    # ── joint send ──
    def _send_joints(self, q):
        q = q.copy()
        for i,(lo,hi) in enumerate(JOINT_LIM):
            q[i] = float(np.clip(q[i], lo, hi))
        self._sock.sendall((json.dumps({"position": [round(float(a),5) for a in q]})+"\n").encode())
        try: self._sf.readline()
        except Exception: pass
        with self._lock:
            self._cmd_time = self.get_clock().now()
        return q

    # ── j5 aim correction ──
    def _aim_j5(self, q_ik, obj_base):
        T = fk_base_ee(q_ik); R = T[:3,:3]
        cam_pos = (T @ np.append(CAM_T, 1.0))[:3]
        aim = obj_base - cam_pos
        if float(np.linalg.norm(aim)) < 0.01: return q_ik
        aim_dir = aim / np.linalg.norm(aim)
        j5_axis = R[:,0]; cur_z = R[:,2]
        def proj(v):
            p = v - float(np.dot(v, j5_axis))*j5_axis
            n = float(np.linalg.norm(p)); return p/n if n > 1e-6 else None
        cp = proj(cur_z); ap = proj(aim_dir)
        if cp is None or ap is None: return q_ik
        cos_a = float(np.clip(np.dot(cp, ap), -1.0, 1.0))
        sin_a = float(np.dot(np.cross(cp, ap), j5_axis))
        delta = float(np.arctan2(sin_a, cos_a))
        q_new = list(q_ik); q_new[4] = float(np.clip(q_ik[4]+delta, -1.75, 1.75))
        return q_new

    def _fin(self):
        self.get_logger().info("=== goto_object finished ===")

    # ── main sequence ──
    def _run(self):
        self.get_logger().info("waiting for /joint_commands — click 'Send All Joints'...")
        q = None
        for i in range(3000):
            with self._lock:
                if self._q is not None: q = self._q.copy(); break
            if i and i % 100 == 0:
                self.get_logger().info("still waiting for /joint_commands...")
            time.sleep(0.1)
        if q is None:
            self.get_logger().error("ERROR: no /joint_commands after 300s."); self._fin(); return
        with self._lock:
            self._cmd_time = self.get_clock().now()

        # ── Phase 1: pixel-only centering (fast — no SGBM) ──
        f0 = self._measure_px()
        if f0 is None:
            self.get_logger().error(
                "ERROR: no target detected — check HSV and that object is in view."); self._fin(); return
        h = w = None
        with self._lock:
            if self._left: h, w = self._left[1].shape[:2]
        self.get_logger().info(
            f"centering (start dx={f0[0]:+.0f} dy={f0[1]:+.0f} px)...")

        # 2×2 Jacobian d(dx,dy)/d(j1,j5) in PIXELS
        ctrl = [0, 4]
        J = np.zeros((2, 2))
        for k, ji in enumerate(ctrl):
            qp = q.copy(); qp[ji] += self.jog
            self._send_joints(qp); fp = self._measure_px()
            self._send_joints(q);  self._measure_px()
            if fp is None:
                self.get_logger().error(
                    f"ERROR: lost target during j{ji+1} probe."); self._fin(); return
            J[:,k] = (fp - f0) / self.jog

        f_prev = dq_prev = None; centered = False
        for it in range(self.max_center):
            if self._abort: self.get_logger().warn("ABORT."); self._fin(); return
            f = self._measure_px()
            if f is None:
                self.get_logger().warn(f"centering [{it}]: target lost."); continue
            if f_prev is not None and dq_prev is not None:
                den = float(dq_prev @ dq_prev)
                if den > 1e-6:
                    J = J + np.outer((f - f_prev) - J @ dq_prev, dq_prev) / den
            e = f
            self.get_logger().info(
                f"centering [{it}]: offset dx={f[0]:+.0f} dy={f[1]:+.0f} px")
            if np.linalg.norm(e) < self.center_tol:
                centered = True; break
            Jpinv = J.T @ np.linalg.inv(J @ J.T + 0.02*np.eye(2))
            dq = -self.gain * (Jpinv @ e)
            n = np.linalg.norm(dq)
            if n > self.jog*2: dq *= self.jog*2 / n
            qn = q.copy()
            for k, ji in enumerate(ctrl): qn[ji] += dq[k]
            q = self._send_joints(qn)
            f_prev, dq_prev = f.copy(), dq.copy()
        if not centered:
            self.get_logger().error(
                f"ERROR: could not center in {self.max_center} steps — proceeding.")
        with self._lock: qc = self._q.copy()
        self.get_logger().info(f"centered: j = {[round(float(a),3) for a in qc]}")

        # ── Phase 2: one-shot depth ──
        self.get_logger().info("computing depth (single disparity frame)...")
        z_m, x_n, y_n = self._measure_depth()
        if z_m is None:
            self.get_logger().error("ERROR: could not compute depth."); self._fin(); return
        self.get_logger().info(f"depth from camera: {z_m*100:.1f} cm")

        # ── Phase 3: compute object position, IK + j5 aim, one move ──
        with self._lock: qcur = self._q.copy()
        T = fk_base_ee(qcur)
        obj_ee   = CAM_T + np.array([x_n*z_m, y_n*z_m, z_m])
        obj_base = (T @ np.append(obj_ee, 1.0))[:3]
        obj_user = to_user(obj_base)
        self.get_logger().info(
            f"estimated location of target (base): "
            f"[{obj_user[0]:.3f}, {obj_user[1]:.3f}, {obj_user[2]:.3f}] m  "
            f"(tan(j1)={np.tan(qcur[0]):.3f}  y/x={obj_user[1]/obj_user[0]:.3f})"
            if abs(obj_user[0]) > 0.01 else
            f"estimated location of target (base): "
            f"[{obj_user[0]:.3f}, {obj_user[1]:.3f}, {obj_user[2]:.3f}] m"
        )

        tgt_ee   = CAM_T + np.array([x_n*z_m, y_n*z_m, max(0.05, z_m - self.standoff)])
        tgt_base = (T @ np.append(tgt_ee, 1.0))[:3]
        ref = np.array([0.0, 0.0, 0.17])
        vec = tgt_base - ref; dist_reach = float(np.linalg.norm(vec))
        if dist_reach > self.max_reach:
            tgt_base = ref + vec * (self.max_reach / dist_reach)
            self.get_logger().warn(
                f"object beyond reach ({dist_reach*100:.0f}cm) — clamping.")
        tgt_user = to_user(tgt_base)
        self.get_logger().info(
            f"approaching to standoff {self.standoff*100:.0f}cm -> "
            f"[{tgt_user[0]:.3f}, {tgt_user[1]:.3f}, {tgt_user[2]:.3f}] m")

        try:
            ik_angles, fk_err = mp.solve_ik(tgt_user, qcur.tolist())
        except Exception as e:
            self.get_logger().error(f"ERROR: IK failed: {e}"); self._fin(); return
        self.get_logger().info(
            f"[IK] j = {[round(float(a),3) for a in ik_angles]}  err {fk_err:.1f}mm")

        # Constrain IK to stay in the same arm configuration as the viewing pose:
        # j1: centering already put it at the right horizontal aim — keep it.
        # j2: shoulder was at 0 during viewing; IK can jump it wildly (e.g. 0->1.676),
        #     causing massive j5 corrections and unpredictable motion — pin it.
        # j4: roll joint with uncalibrated scale — always 0.
        # Only j3 (depth / reach) changes, which is what we actually want.
        ik_angles = list(ik_angles)
        ik_angles[0] = float(qcur[0])   # j1 from centering
        ik_angles[1] = float(qcur[1])   # j2 from viewing pose (usually 0)
        ik_angles[3] = 0.0              # j4 forced 0 (uncalibrated)
        self.get_logger().info(
            f"constrained: j1={ik_angles[0]:.3f} j2={ik_angles[1]:.3f} "
            f"j3={ik_angles[2]:.3f} j4=0 (IK free j3 used for depth)")

        # Compute j5 so the camera aims at the object at this constrained pose.
        ik_aimed = self._aim_j5(ik_angles, obj_base)
        delta_j5 = float(ik_aimed[4]) - float(ik_angles[4])
        self.get_logger().info(
            f"j5 aim: {delta_j5:+.3f} rad ({np.degrees(delta_j5):+.1f}°)")
        self.get_logger().info(f"[tx] {[round(float(a),3) for a in ik_aimed]}")

        if self._abort: self.get_logger().warn("ABORT."); self._fin(); return
        self._send_joints(ik_aimed)
        time.sleep(3.0)

        # ── Phase 4: verify ──
        if self._abort: self._fin(); return
        f = self._measure_px()
        if f is None:
            self.get_logger().warn("verify: object not visible after move.")
        else:
            self.get_logger().info(
                f"verify: offset dx={f[0]:+.0f} dy={f[1]:+.0f} px. Re-click to refine.")
        self._fin()

    def destroy_node(self):
        try: self._sock.close()
        except Exception: pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GotoObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
