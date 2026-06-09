#!/usr/bin/env python3
"""
mac_goto_object.py — FULL port of the Pi goto_object logic to run on your Mac/compute.

This completely offloads the vision workload (blob detection, SGBM depth, object pose,
IK target computation, j5 aiming) to your stronger machine.

Pi only needs to:
  - Run stereo_camera_node + the enhanced mjpeg_stream (for /left /right JPEGs)
  - Run moveo_publisher (for joint commands via TCP port 9000)

Mac does the rest at full speed, sending only the final joint commands (or cartesian) to Pi.

Usage example (after jogging the arm to a starting pose with the GUI and clicking "Send All Joints"):
    python3 mac_goto_object.py \
        --pi-host 192.168.1.142 \
        --stream-port 8080 \
        --cmd-port 9000 \
        --calib ../calibration/stereo_calib.yaml \
        --standoff 0.22 \
        --h-lo 0 --h-hi 179 --s-lo 50 --s-hi 255 --v-lo 50 --v-hi 255 \
        --initial-joints "0,0,0,0,0"

The script will do the full sequence:
  1. Pixel-only centering (j1 + j5) using fast blob detection on the stream.
  2. One-shot depth at the centered blob.
  3. Compute object in base frame (using local kinematics for FK).
  4. IK for standoff pose (sends cartesian to Pi's moveo_publisher so Pi does the IK with its warm-start).
  5. j5 aim correction to keep camera on target.
  6. Send the move.
  7. Verify residual pixel offset.

HSV values usually come from a click in the GUI (you can copy them from depth_test_gui or previous runs).

This should give much higher effective vision rates because SGBM and detection run on your compute,
not the Pi.
"""

import argparse
import json
import os
import socket
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np

# Make kinematics available (single source of truth, pure Python)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'mac-gui'))
import kinematics as kin

from mac_stereo_grabber import StereoGrabber

# Same limits as Pi side
JOINT_LIM = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.75, 1.75)]
MIN_BLOB_AREA = 300
CAM_T = np.array([-0.05, 0.03, 0.0])  # left camera in EE (model frame)

class MacGotoObject:
    def __init__(self, pi_host: str, stream_port: int, cmd_port: int,
                 calib_path: str, standoff: float = 0.22,
                 h_lo: int = 0, h_hi: int = 179, s_lo: int = 50, s_hi: int = 255,
                 v_lo: int = 50, v_hi: int = 255,
                 initial_joints: Optional[list] = None,
                 center_tol_px: float = 15.0, jog_rad: float = 0.05,
                 center_gain: float = 0.4, settle_s: float = 1.2,
                 measure_samples: int = 5, max_center: int = 12,
                 max_reach_m: float = 0.42,
                 log_func=print,
                 send_cartesian_func=None,
                 send_joints_func=None):
        self.pi_host = pi_host
        self.stream_port = stream_port
        self.cmd_port = cmd_port
        self.standoff = standoff
        self.hsv_lo = np.array([h_lo, s_lo, v_lo])
        self.hsv_hi = np.array([h_hi, s_hi, v_hi])
        self.center_tol = center_tol_px
        self.jog = jog_rad
        self.gain = center_gain
        self.settle = settle_s
        self.n_samples = measure_samples
        self.max_center = max_center
        self.max_reach = max_reach_m
        self.log = log_func
        self._external_send_cart = send_cartesian_func
        self._external_send_j = send_joints_func

        self.log("Loading calib for Mac processing...")
        self._cal = self._load_cal(calib_path)
        self.log(f"  Processing resolution: {self._cal['sw']}x{self._cal['sh']}")

        # Grabber for Pi stream
        self.grabber = StereoGrabber(pi_host, stream_port)

        # Command socket (only if no external send funcs provided)
        self._sock = None
        self._sf = None
        if not (self._external_send_cart or self._external_send_j):
            self._connect_command()

        # Local joint state estimate
        self._q = np.array(initial_joints if initial_joints is not None else [0.0]*5, dtype=float)
        self._cmd_time = time.time()

        self.log(f"MacGotoObject ready. Standoff={standoff*100:.0f}cm  HSV={list(self.hsv_lo)}-{list(self.hsv_hi)}")

    def _load_cal(self, path: str):
        """Exact same loading as the Pi goto_object / stereo_depth_node."""
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f"cannot open calibration: {path}")
        g = lambda n: fs.getNode(n).mat()
        K1, D1, K2, D2 = g("K1"), g("D1"), g("K2"), g("D2")
        R, T = g("R"), g("T")
        iw = int(fs.getNode("image_width").real())
        ih = int(fs.getNode("image_height").real())
        fs.release()
        scale = 0.5
        sw, sh = int(round(iw*scale)), int(round(ih*scale))
        S = np.array([[scale,0,0],[0,scale,0],[0,0,1]], np.float64)
        K1s, K2s = S @ K1, S @ K2
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1s, D1, K2s, D2, (sw,sh), R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        mapL = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, (sw,sh), cv2.CV_16SC2)
        mapR = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, (sw,sh), cv2.CV_16SC2)
        sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=160, blockSize=7,
            P1=8*3*49, P2=32*3*49, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        return {
            "mapL": mapL, "mapR": mapR, "Q": Q,
            "sgbm": sgbm, "sw": sw, "sh": sh,
            "fx": float(K1[0,0]), "fy": float(K1[1,1]),
            "cx": float(K1[0,2]), "cy": float(K1[1,2])
        }

    def _connect_command(self):
        for _ in range(15):
            try:
                self._sock = socket.create_connection((self.pi_host, self.cmd_port), timeout=5)
                break
            except OSError:
                time.sleep(1.0)
        if self._sock is None:
            raise RuntimeError(f"could not reach moveo_publisher at {self.pi_host}:{self.cmd_port}")
        self._sf = self._sock.makefile("r")
        self.log(f"Connected to command server on Pi")

    def _send_joints(self, q):
        if self._external_send_j:
            self._q = self._external_send_j(q)
            self._cmd_time = time.time()
            return self._q.copy()
        q = np.clip(np.array(q, dtype=float), [lo for lo,hi in JOINT_LIM], [hi for lo,hi in JOINT_LIM])
        payload = json.dumps({"position": [round(float(a), 5) for a in q]}) + "\n"
        self._sock.sendall(payload.encode())
        try:
            resp = json.loads(self._sf.readline().strip())
            if "angles" in resp:
                self._q = np.array(resp["angles"], dtype=float)
            self._cmd_time = time.time()
        except Exception:
            self._q = q.copy()
        return self._q.copy()

    def _send_cartesian(self, xyz):
        if self._external_send_cart:
            self._q = self._external_send_cart(xyz)
            self._cmd_time = time.time()
            return self._q.copy()
        payload = json.dumps({"cartesian": [round(float(v), 4) for v in xyz]}) + "\n"
        self._sock.sendall(payload.encode())
        try:
            resp = json.loads(self._sf.readline().strip())
            if resp.get("ok") and "angles" in resp:
                self._q = np.array(resp["angles"], dtype=float)
                if "achieved" in resp:
                    self.log(f"  IK achieved (user frame): {resp['achieved']}")
            self._cmd_time = time.time()
            return np.array(resp.get("angles", self._q), dtype=float)
        except Exception as e:
            self.log(f"  Cartesian send error: {e}")
            return self._q.copy()

    # --- Vision (ported from Pi side, using grabber) ---
    def _grab_pair(self):
        return self.grabber.get_pair()

    def _detect_color(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if self.hsv_lo[0] > self.hsv_hi[0]:  # hue wrap
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
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_BLOB_AREA:
            return None
        M = cv2.moments(c)
        if M['m00'] == 0:
            return None
        return int(M['m10']/M['m00']), int(M['m01']/M['m00'])

    def _measure_depth(self):
        """One-shot depth at current blob centroid (adapted for non-ROS grabber)."""
        t0 = time.time()
        pair = None
        cxy = None
        while time.time() - t0 < 8.0:
            pair = self._grab_pair()
            if pair is None:
                time.sleep(0.05)
                continue
            left, _ = pair
            cxy = self._detect_color(left)
            if cxy is not None:
                break
            time.sleep(0.05)
        if pair is None or cxy is None:
            return None, None, None

        left, right = pair
        sw, sh = self._cal["sw"], self._cal["sh"]
        li = cv2.resize(left, (sw, sh))
        ri = cv2.resize(right, (sw, sh))
        lr = cv2.remap(li, *self._cal["mapL"], cv2.INTER_LINEAR)
        rr = cv2.remap(ri, *self._cal["mapR"], cv2.INTER_LINEAR)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        disp = self._cal["sgbm"].compute(gl, gr).astype(np.float32) / 16.0
        pts3d = cv2.reprojectImageTo3D(disp, self._cal["Q"])

        scale_x = sw / left.shape[1]
        scale_y = sh / left.shape[0]
        u = int(np.clip(cxy[0] * scale_x, 0, sw-1))
        v = int(np.clip(cxy[1] * scale_y, 0, sh-1))

        win = 8
        y0, y1 = max(0, v-win), min(sh, v+win+1)
        x0, x1 = max(0, u-win), min(sw, u+win+1)
        patch = pts3d[y0:y1, x0:x1, 2]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if len(valid) < 5:
            all_valid = pts3d[:, :, 2]
            all_valid = all_valid[(all_valid > 0) & np.isfinite(all_valid)]
            if len(all_valid) < 10:
                return None, None, None
            z_mm = float(np.median(all_valid))
        else:
            z_mm = float(np.median(valid))
        z_m = z_mm / 1000.0

        x_n = (cxy[0] - self._cal["cx"]) / self._cal["fx"]
        y_n = (cxy[1] - self._cal["cy"]) / self._cal["fy"]
        return z_m, x_n, y_n

    def _measure_px(self):
        """Median pixel offset from center (fast, no depth)."""
        samples = []
        t0 = time.time()
        while time.time() - t0 < self.settle + 6.0:
            pair = self._grab_pair()
            if pair is None:
                time.sleep(0.03)
                continue
            left, _ = pair
            cxy = self._detect_color(left)
            if cxy is not None:
                h, w = left.shape[:2]
                dx = cxy[0] - w / 2.0
                dy = cxy[1] - h / 2.0
                samples.append(np.array([dx, dy]))
                if len(samples) >= self.n_samples:
                    break
            time.sleep(0.03)
        if len(samples) < max(1, self.n_samples // 2):
            return None
        return np.median(np.array(samples), axis=0)

    # --- FK / transforms (using local kinematics) ---
    def _fk_base_ee(self, q):
        return kin.forward_kinematics_matrix(q)

    def _to_user(self, p):
        # Use the same transform as the Pi (chain_to_user)
        return kin.chain_to_user(p) if hasattr(kin, 'chain_to_user') else list(p)

    def _aim_j5(self, q_ik, obj_base):
        T = self._fk_base_ee(q_ik)
        R = T[:3, :3]
        cam_pos = (T @ np.append(CAM_T, 1.0))[:3]
        aim = obj_base - cam_pos
        if float(np.linalg.norm(aim)) < 0.01:
            return q_ik
        aim_dir = aim / np.linalg.norm(aim)
        j5_axis = R[:, 0]
        cur_z = R[:, 2]
        def proj(v):
            p = v - float(np.dot(v, j5_axis)) * j5_axis
            n = float(np.linalg.norm(p))
            return p / n if n > 1e-6 else None
        cp = proj(cur_z)
        ap = proj(aim_dir)
        if cp is None or ap is None:
            return q_ik
        cos_a = float(np.clip(np.dot(cp, ap), -1.0, 1.0))
        sin_a = float(np.dot(np.cross(cp, ap), j5_axis))
        delta = float(np.arctan2(sin_a, cos_a))
        q_new = list(q_ik)
        q_new[4] = float(np.clip(q_ik[4] + delta, -1.75, 1.75))
        return q_new

    # --- Main sequence (ported) ---
    def run(self):
        self.log("Starting full goto sequence on Mac (offloaded)...")

        # Use initial q (user should have sent joints recently via GUI)
        q = self._q.copy()
        self.log(f"Starting with joints: {np.round(q, 3)}")

        # Phase 1: pixel centering (fast blob only)
        f0 = self._measure_px()
        if f0 is None:
            self.log("ERROR: no target visible at start. Check HSV / object in view.")
            return
        self.log(f"Initial pixel offset: dx={f0[0]:+.0f} dy={f0[1]:+.0f}")

        ctrl = [0, 4]  # j1, j5
        J = np.zeros((2, 2))
        for k, ji in enumerate(ctrl):
            qp = q.copy()
            qp[ji] += self.jog
            self._send_joints(qp)
            fp = self._measure_px()
            self._send_joints(q)
            self._measure_px()
            if fp is None:
                print(f"ERROR: lost target during probe j{ji+1}")
                return
            J[:, k] = (fp - f0) / self.jog

        f_prev = dq_prev = None
        centered = False
        for it in range(self.max_center):
            f = self._measure_px()
            if f is None:
                print(f"centering [{it}]: target lost")
                continue
            if f_prev is not None and dq_prev is not None:
                den = float(dq_prev @ dq_prev)
                if den > 1e-6:
                    J = J + np.outer((f - f_prev) - (J @ dq_prev), dq_prev) / den
            e = f
            print(f"centering [{it}]: dx={f[0]:+.0f} dy={f[1]:+.0f} px")
            if np.linalg.norm(e) < self.center_tol:
                centered = True
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

        if not centered:
            print(f"WARNING: could not fully center in {self.max_center} steps")
        print(f"Centered at joints: {np.round(q, 3)}")

        # Phase 2: depth
        print("Measuring depth...")
        z_m, x_n, y_n = self._measure_depth()
        if z_m is None:
            print("ERROR: depth measurement failed")
            return
        print(f"Depth: {z_m*100:.1f} cm")

        # Phase 3: object pose + approach target
        T = self._fk_base_ee(q)
        obj_ee = CAM_T + np.array([x_n * z_m, y_n * z_m, z_m])
        obj_base = (T @ np.append(obj_ee, 1.0))[:3]
        obj_user = self._to_user(obj_base)
        print(f"Target (user frame): {np.round(obj_user, 3)} m")

        tgt_ee = CAM_T + np.array([x_n * z_m, y_n * z_m, max(0.05, z_m - self.standoff)])
        tgt_base = (T @ np.append(tgt_ee, 1.0))[:3]
        ref = np.array([0.0, 0.0, 0.17])
        vec = tgt_base - ref
        dist_reach = float(np.linalg.norm(vec))
        if dist_reach > self.max_reach:
            tgt_base = ref + vec * (self.max_reach / dist_reach)
            print("WARNING: clamped to max reach")
        tgt_user = self._to_user(tgt_base)
        print(f"Approach target (user): {np.round(tgt_user, 3)} m  standoff={self.standoff*100:.0f}cm")

        # Send cartesian — Pi does IK (warm starts from its current joints)
        print("Sending cartesian target to Pi for IK...")
        ik_angles = self._send_cartesian(tgt_user)

        # Apply the same constraints the Pi version used (j1/j2 from viewing pose, j4=0)
        ik_angles = list(ik_angles)
        ik_angles[0] = float(q[0])
        ik_angles[1] = float(q[1])
        ik_angles[3] = 0.0
        print(f"Constrained IK angles: {np.round(ik_angles, 3)}")

        # j5 aim
        ik_aimed = self._aim_j5(ik_angles, obj_base)
        print(f"Final aimed joints: {np.round(ik_aimed, 3)}")

        # Send the move
        self._send_joints(ik_aimed)
        time.sleep(3.0)

        # Verify
        f = self._measure_px()
        if f is None:
            print("Verify: target not visible after move")
        else:
            print(f"Verify residual: dx={f[0]:+.0f} dy={f[1]:+.0f} px")

        print("=== Mac goto_object finished ===")
        try:
            self._sock.close()
        except:
            pass


def main():
    ap = argparse.ArgumentParser(description="Full vision offload goto on Mac")
    ap.add_argument("--pi-host", default="192.168.1.142")
    ap.add_argument("--stream-port", type=int, default=8080)
    ap.add_argument("--cmd-port", type=int, default=9000)
    ap.add_argument("--calib", default="software/vision/calibration/stereo_calib.yaml")
    ap.add_argument("--standoff", type=float, default=0.22)
    ap.add_argument("--h-lo", type=int, default=0)
    ap.add_argument("--h-hi", type=int, default=179)
    ap.add_argument("--s-lo", type=int, default=50)
    ap.add_argument("--s-hi", type=int, default=255)
    ap.add_argument("--v-lo", type=int, default=50)
    ap.add_argument("--v-hi", type=int, default=255)
    ap.add_argument("--initial-joints", default="0,0,0,0,0",
                    help="Comma separated 5 joint values in radians from a recent 'Send All Joints'")
    args = ap.parse_args()

    initial = [float(x) for x in args.initial_joints.split(",")]
    if len(initial) != 5:
        initial = [0.0]*5

    servo = MacGotoObject(
        pi_host=args.pi_host,
        stream_port=args.stream_port,
        cmd_port=args.cmd_port,
        calib_path=args.calib,
        standoff=args.standoff,
        h_lo=args.h_lo, h_hi=args.h_hi,
        s_lo=args.s_lo, s_hi=args.s_hi,
        v_lo=args.v_lo, v_hi=args.v_hi,
        initial_joints=initial
    )
    servo.run()


if __name__ == "__main__":
    main()
