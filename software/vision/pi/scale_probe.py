#!/usr/bin/env python3
"""
scale_probe.py — measure each joint's true steps/rad scale via the camera.

For each joint it commands a known angle +/-delta and measures the camera's ACTUAL
rotation from the fixed board's pose (solvePnP rotation magnitude = camera rotation
= the joint's real physical rotation). The ratio commanded/actual is the correction
for CALIB_FACTOR:  new_CALIB = old_CALIB * (commanded/actual).
  ratio > 1  -> arm under-rotates (steps/rad too low) -> raise CALIB
  ratio < 1  -> arm over-rotates                       -> lower CALIB

Rotation magnitude is unaffected by translation, so this works even for joints
that also move the camera (j1/j2/j3), not just the wrist joints.

Needs: camera node + moveo_publisher, board in view, Send All Joints to seed
/joint_commands, and stereo_calib.yaml (for the raw left intrinsics K1/D1).

Run:
    python3 scale_probe.py --ros-args -p delta_rad:=0.30
"""

import json
import socket
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from sensor_msgs.msg import Image, JointState

JOINT_LIM = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.75, 1.75)]
NAMES = ["j1(waist)", "j2(shoulder)", "j3(elbow)", "j4(wristroll)", "j5(wristpitch)"]
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def rot_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class ScaleProbe(Node):
    def __init__(self):
        super().__init__("scale_probe")
        p = self.declare_parameter
        self.joints = list(p("joints", [0, 1, 2, 3, 4]).value)
        self.delta = p("delta_rad", 0.30).value
        self.settle = p("settle_s", 1.6).value
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.square = p("square_mm", 23.25).value / 1000.0
        self.calib = p("calib", "/home/armpi/vision/stereo_calib.yaml").value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value

        fs = cv2.FileStorage(self.calib, cv2.FILE_STORAGE_READ)
        self.K1 = fs.getNode("K1").mat(); self.D1 = fs.getNode("D1").mat(); fs.release()
        self.objp = np.zeros((self.cols * self.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square

        self._R = None
        self._q0 = None
        self._lock = threading.Lock()

        self.create_subscription(Image, "/stereo/left/image_raw", self._img, qos_profile_sensor_data)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = socket.create_connection((self.host, self.port), timeout=5)
        self._sf = self._sock.makefile("r")
        self.get_logger().info(f"scale_probe: joints {self.joints}, delta {self.delta} rad. "
                               f"Board in view + Send All Joints. E-STOP ready.")
        threading.Thread(target=self._run, daemon=True).start()

    def _img(self, m):
        gray = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3),
                            cv2.COLOR_BGR2GRAY)
        ok, c = cv2.findChessboardCorners(gray, (self.cols, self.rows),
                                          cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        R = None
        if ok:
            c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), SUBPIX)
            ok2, rvec, _ = cv2.solvePnP(self.objp, c, self.K1, self.D1)
            if ok2:
                R, _ = cv2.Rodrigues(rvec)
        with self._lock:
            self._R = R

    def _jc(self, m):
        if len(m.position) >= 5:
            with self._lock:
                self._q0 = np.array(m.position[:5], float)

    def _send(self, q):
        q = q.copy()
        for i, (lo, hi) in enumerate(JOINT_LIM):
            q[i] = float(np.clip(q[i], lo, hi))
        self._sock.sendall((json.dumps({"position": [round(float(a), 5) for a in q]}) + "\n").encode())
        try:
            self._sf.readline()
        except Exception:
            pass

    def _measureR(self):
        time.sleep(self.settle)
        for _ in range(25):
            with self._lock:
                R = None if self._R is None else self._R.copy()
            if R is not None:
                return R
            time.sleep(0.1)
        return None

    def _run(self):
        self.get_logger().info("waiting for /joint_commands — click 'Send All Joints'...")
        q0 = None
        for i in range(3000):
            with self._lock:
                if self._q0 is not None:
                    q0 = self._q0.copy(); break
            if i and i % 100 == 0:
                self.get_logger().info("still waiting — click 'Send All Joints'...")
            time.sleep(0.1)
        if q0 is None:
            self.get_logger().error("no /joint_commands after 300s."); return
        if self._measureR() is None:
            self.get_logger().error("board pose not detected — center the board in view."); return

        self.get_logger().info(f"base pose {q0.round(3)} — measuring scale at +/-{self.delta} rad\n")
        for ji in self.joints:
            Rb = self._measureR()
            qp = q0.copy(); qp[ji] += self.delta; self._send(qp); Rp = self._measureR()
            qm = q0.copy(); qm[ji] -= self.delta; self._send(qm); Rm = self._measureR()
            self._send(q0); self._measureR()
            if None in (Rb, Rp, Rm):
                self.get_logger().warn(f"{NAMES[ji]:14s}: board lost — skip"); continue
            a_p = rot_angle_deg(Rb.T @ Rp)
            a_m = rot_angle_deg(Rb.T @ Rm)
            actual = np.radians((a_p + a_m) / 2.0)
            ratio = self.delta / actual if actual > 1e-4 else float("inf")
            self.get_logger().info(
                f"{NAMES[ji]:14s}: commanded {self.delta:.3f} rad -> actual {actual:.3f} rad "
                f"(+{np.radians(a_p):.3f}/-{np.radians(a_m):.3f})  ratio cmd/act = {ratio:.2f}  "
                f"=> multiply CALIB_FACTOR[{ji}] by {ratio:.2f}")
        self.get_logger().info("\nscale probe done. new_CALIB = old_CALIB * ratio (ratio>1 => raise).")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ScaleProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
