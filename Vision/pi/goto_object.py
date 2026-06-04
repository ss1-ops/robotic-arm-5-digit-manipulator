#!/usr/bin/env python3
"""
goto_object.py — go to a clicked object the sensible way:

  1. CENTER  : small IBVS on j1 (yaw) + j5 (pitch) to put the object on the
               optical axis (closed-loop -> immune to FK/joint-scale error).
  2. COMPUTE : read stereo depth d; with the object centered it sits at EE-frame
               (CAM_X, CAM_Y, d). FK(current joints) -> base pose; transform to
               get the object's Cartesian position in the base frame.
  3. MOVE    : Cartesian-approach to a standoff short of the object (one IK move).
  4. VERIFY  : re-acquire the object and report the residual (and could refine).

Only j1/j5 move during centering, so the camera barely swings and the target
stays in view -- unlike a full multi-joint Jacobian probe.

Frames: /stereo/target/point is the object in the LEFT camera optical frame
(x right, y down, z = depth along the optical axis, metres). The camera mounts at
EE-frame offset CAM_T with optical axis = EE +Z (URDF mount). The publisher's
cartesian command is in the USER frame (+X forward, +Y left); the ikpy chain is
the INTERNAL frame (rotated +90 deg about Z), so we convert before sending.

Prereqs: camera + depth(color) + moveo_publisher running, ESP up, object in view,
'Send All Joints' to seed /joint_commands.  Keep j4 = 0.  !! E-STOP ready.

Run:  python3 goto_object.py --ros-args -p standoff_m:=0.15
"""
import json, socket, sys, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty
from sensor_msgs.msg import JointState

sys.path.insert(0, "/home/armpi/ros_nodes")
import moveo_publisher as mp          # MOVEO_CHAIN for FK (internal frame)

CAM_T = np.array([-0.05, -0.03, 0.0])  # left camera position in the EE frame (URDF mount)
JOINT_LIM = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.75, 1.75)]


def fk_base_ee(q5):
    """4x4 base<-EE transform (internal chain frame) for joint vector q5."""
    return mp.MOVEO_CHAIN.forward_kinematics([0.0] + list(q5) + [0.0])


def to_user(p):
    """internal base frame -> user frame (+X forward, +Y left): x_u=-y_i, y_u=x_i."""
    return [-float(p[1]), float(p[0]), float(p[2])]


class GotoObject(Node):
    def __init__(self):
        super().__init__("goto_object")
        p = self.declare_parameter
        self.standoff = p("standoff_m", 0.15).value      # stop this far short of the object
        self.center_tol = p("center_tol_m", 0.02).value  # |x|,|y| in camera frame (m)
        self.jog = p("jog_rad", 0.05).value              # probe + max step for centering
        self.gain = p("center_gain", 0.5).value
        self.settle = p("settle_s", 1.2).value
        self.max_center = p("max_center_iters", 12).value
        self.n_samples = p("measure_samples", 5).value   # median over N frames (noise reject)
        self.max_reach = p("max_reach_m", 0.42).value    # clamp far targets to this (from J1 axis)
        self.approach_step = p("approach_step_m", 0.06).value   # forward step per approach iteration
        self.max_approach = p("max_approach_steps", 12).value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value

        self._pt = None; self._state = "NONE"; self._q = None; self._abort = False
        self._pt_stamp = None       # capture time of the latest /stereo/target/point
        self._cmd_time = None       # ROS time the last joint command was sent
        self._lock = threading.Lock()
        self.create_subscription(PointStamped, "/stereo/target/point", self._ptcb, 10)
        self.create_subscription(String, "/stereo/target/state",
                                 lambda m: setattr(self, "_state", m.data), 10)
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
            f"goto_object: standoff {self.standoff*100:.0f}cm. Object in view + Send All Joints. "
            f"j4=0. E-STOP / /approach/stop ready.")
        threading.Thread(target=self._run, daemon=True).start()

    # ── callbacks ──
    def _ptcb(self, m):
        x, y, z = m.point.x, m.point.y, m.point.z
        # Reject implausible points (stereo/detection glitches, e.g. x=-4m) so a
        # single bad frame can't throw the centering servo off the rails.
        if abs(x) > 0.6 or abs(y) > 0.6 or not (0.05 < z < 1.5):
            return
        with self._lock:
            self._pt = np.array([x, y, z])
            self._pt_stamp = Time.from_msg(m.header.stamp)

    def _jc(self, m):
        if len(m.position) >= 5:
            with self._lock:
                self._q = np.array(m.position[:5], float)

    # ── io ──
    def _send_joints(self, q):
        q = q.copy()
        for i, (lo, hi) in enumerate(JOINT_LIM):
            q[i] = float(np.clip(q[i], lo, hi))
        self._sock.sendall((json.dumps({"position": [round(float(a), 5) for a in q]}) + "\n").encode())
        try: self._sf.readline()
        except Exception: pass
        with self._lock:
            self._cmd_time = self.get_clock().now()   # mark when this move was issued
        return q

    def _send_cartesian(self, xyz_user):
        self._sock.sendall((json.dumps({"cartesian": [round(float(a), 5) for a in xyz_user]}) + "\n").encode())
        try:
            return json.loads(self._sf.readline().strip())
        except Exception:
            return {}

    def _measure(self):
        """Return the MEDIAN of N TRACK frames captured AFTER the last commanded
        move has had time to execute. Frames are gated by capture timestamp
        (stamp > cmd_time + settle), so the servo never acts on a stale frame
        taken before/while the arm was still moving. Collects N DISTINCT frames."""
        with self._lock:
            cmd_t = self._cmd_time
        deadline = (cmd_t + Duration(seconds=float(self.settle))) if cmd_t is not None else None
        samples = []
        last_stamp = None
        t0 = time.time()
        while time.time() - t0 < float(self.settle) + 8.0:
            with self._lock:
                pt = None if self._pt is None else self._pt.copy()
                stamp = self._pt_stamp
                s = self._state
            if pt is not None and s == "TRACK" and stamp is not None:
                fresh = (deadline is None) or (stamp > deadline)        # after the move settled
                newer = (last_stamp is None) or (stamp > last_stamp)    # a distinct frame
                if fresh and newer:
                    samples.append(pt); last_stamp = stamp
                    if len(samples) >= self.n_samples:
                        break
            time.sleep(0.03)
        if len(samples) < max(1, self.n_samples // 2):
            return None
        return np.median(np.array(samples), axis=0)

    # ── centering helper (reused at each approach step) ──
    def _center(self, q, label="center"):
        """Run j1/j5 IBVS to put the object on the optical axis.
        Returns (q, f) — updated joint vector and the final measured point.
        Returns (q, None) if the target is lost."""
        ctrl = [0, 4]
        f0 = self._measure()
        if f0 is None:
            return q, None
        self.get_logger().info(
            f"[{label}] start x={f0[0]*100:+.1f} y={f0[1]*100:+.1f} cm  depth={f0[2]*100:.0f}cm")

        # 2×2 Jacobian probe (j1, j5)
        J = np.zeros((2, 2))
        for k, ji in enumerate(ctrl):
            qp = q.copy(); qp[ji] += self.jog
            self._send_joints(qp); fp = self._measure()
            self._send_joints(q); self._measure()
            if fp is None:
                self.get_logger().error(
                    f"[{label}] ERROR: lost target during j{ji+1} probe — "
                    "use a bigger/brighter object, more centered.")
                return q, None
            J[:, k] = (fp[:2] - f0[:2]) / self.jog

        f_prev = dq_prev = None
        for it in range(self.max_center):
            if self._abort:
                return q, None
            f = self._measure()
            if f is None:
                self.get_logger().warn(f"[{label}:{it}] target lost — holding."); continue
            if f_prev is not None and dq_prev is not None:
                den = float(dq_prev @ dq_prev)
                if den > 1e-6:
                    J = J + np.outer((f[:2] - f_prev[:2]) - J @ dq_prev, dq_prev) / den
            e = f[:2]
            self.get_logger().info(
                f"[{label}:{it}] offset x={f[0]*100:+.1f} y={f[1]*100:+.1f} cm")
            if np.linalg.norm(e) < self.center_tol:
                break
            Jpinv = J.T @ np.linalg.inv(J @ J.T + 0.02 * np.eye(2))
            dq = -self.gain * (Jpinv @ e)
            n = np.linalg.norm(dq)
            if n > self.jog * 2:
                dq *= self.jog * 2 / n
            qn = q.copy()
            for k, ji in enumerate(ctrl):
                qn[ji] += dq[k]
            q = self._send_joints(qn)
            f_prev, dq_prev = f.copy(), dq.copy()
        else:
            self.get_logger().warn(
                f"[{label}] could not fully center in {self.max_center} steps — proceeding.")

        f = self._measure()
        return q, f

    # ── main sequence ──
    def _run(self):
        self.get_logger().info("waiting for /joint_commands — click 'Send All Joints'...")
        q = None
        for i in range(3000):
            with self._lock:
                if self._q is not None:
                    q = self._q.copy(); break
            if i and i % 100 == 0:
                self.get_logger().info("still waiting for /joint_commands...")
            time.sleep(0.1)
        if q is None:
            self.get_logger().error("ERROR: no /joint_commands after 300s."); self._fin(); return
        with self._lock:
            self._cmd_time = self.get_clock().now()

        # ── Phase 1: initial centering ──
        q, f = self._center(q, label="center")
        if f is None:
            self.get_logger().error("ERROR: cannot see target."); self._fin(); return
        with self._lock:
            qc = self._q.copy()
        self.get_logger().info(f"centered: j = {[round(float(a), 3) for a in qc]}")

        # ── Phase 2: compute object's Cartesian position ──
        d = float(f[2])
        self.get_logger().info(f"depth from camera: {d*100:.1f} cm")
        with self._lock:
            qcur = self._q.copy()
        T = fk_base_ee(qcur)
        obj_ee = CAM_T + np.array([f[0], f[1], d])
        obj_base = (T @ np.append(obj_ee, 1.0))[:3]
        obj_user = to_user(obj_base)
        self.get_logger().info(
            f"estimated location of target (base): "
            f"[{obj_user[0]:.3f}, {obj_user[1]:.3f}, {obj_user[2]:.3f}] m")

        # ── Phase 3: incremental approach — step forward, re-center, repeat ──
        # Each step moves a fraction of the remaining distance toward the object,
        # then re-centers j1/j5 so the camera stays aimed at the object throughout.
        for step in range(self.max_approach):
            if self._abort:
                self.get_logger().warn("ABORT."); self._fin(); return
            with self._lock:
                qcur = self._q.copy()
            T = fk_base_ee(qcur)
            f_now = self._measure()
            if f_now is None:
                self.get_logger().error(
                    f"ERROR: lost target during approach step {step}."); self._fin(); return
            d_now = float(f_now[2])
            remaining = d_now - self.standoff
            self.get_logger().info(
                f"[approach:{step}] depth={d_now*100:.0f}cm  remaining={remaining*100:.0f}cm")
            if remaining <= 0.01:
                self.get_logger().info("standoff reached."); break

            # Step forward by approach_step (or the remaining distance if smaller)
            step_d = min(self.approach_step, remaining)
            tgt_ee = CAM_T + np.array([f_now[0], f_now[1], max(0.05, d_now - step_d)])
            tgt_base = (T @ np.append(tgt_ee, 1.0))[:3]
            ref = np.array([0.0, 0.0, 0.17])
            vec = tgt_base - ref; dist_reach = float(np.linalg.norm(vec))
            if dist_reach > self.max_reach:
                tgt_base = ref + vec * (self.max_reach / dist_reach)
                self.get_logger().warn(
                    f"object beyond reach ({dist_reach*100:.0f}cm) — clamping to reach limit.")
            tgt_user = to_user(tgt_base)
            ack = self._send_cartesian(tgt_user)
            if not ack.get("ok"):
                self.get_logger().error(
                    f"ERROR: IK rejected at step {step}: {ack.get('error', ack)}"); self._fin(); return
            ang = ack.get("angles", [])
            self.get_logger().info(
                f"[IK] {[f'{a:.3f}' for a in tgt_user]}  err {ack.get('fk_err_mm','?')}mm")
            self.get_logger().info(f"[tx] {[round(float(a), 3) for a in ang]}")
            with self._lock:
                q = self._q.copy() if self._q is not None else q
            time.sleep(2.5)

            # Re-center: adjust j1/j5 to keep the camera on the object
            q, f_after = self._center(q, label=f"re-aim:{step}")
            if f_after is None:
                self.get_logger().error(
                    f"ERROR: lost target after step {step} — stopping."); self._fin(); return
        else:
            self.get_logger().warn("max approach steps reached.")

        # ── Phase 4: final verify ──
        f = self._measure()
        if f is None:
            self.get_logger().warn("verify: object not visible at standoff.")
            self._fin(); return
        self.get_logger().info(
            f"verify: offset x={f[0]*100:+.1f} y={f[1]*100:+.1f} cm, "
            f"depth {f[2]*100:.1f}cm (target {self.standoff*100:.0f}cm). Re-click to refine.")
        self._fin()

    def _fin(self):
        self.get_logger().info("=== goto_object finished ===")

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
