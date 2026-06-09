#!/usr/bin/env python3
"""
joint_probe.py — measure each joint's true image response (direction + scale).

Detects the checkerboard centroid directly in the raw left image (robust — no
rectification/depth needed) and, for each joint, commands +delta and -delta from
the current pose (central difference) to measure how the board centroid moves in
the image: d(u,v)/dq in pixels/rad. Use this to find:
  * REVERSED joints  — dominant-axis sign opposite to what you expect
  * MIS-SCALED joints — small |move| (the arm barely moved => steps/rad too low)
so you can fix DIR pins / CALIB_FACTOR / the URDF, which may also make FK accurate
enough for hand-eye.

NOTE: the camera is mounted upside down, so image +u is world-left and image +v is
world-up (a consistent global flip across all joints — fine for comparing them).

Needs the camera node running (/stereo/left/image_raw) + moveo_publisher, board
in view, and a jog (Send All Joints) to seed /joint_commands.

Run:
    python3 joint_probe.py --ros-args -p delta_rad:=0.12 -p joints:="[0,1,2,3,4]"
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
NAMES = ["j1(waist/yaw)", "j2(shoulder)", "j3(elbow)", "j4(wristroll)", "j5(wristpitch)"]


class ProbeNode(Node):
    def __init__(self):
        super().__init__("joint_probe")
        p = self.declare_parameter
        self.joints = list(p("joints", [0, 1, 2, 3, 4]).value)
        self.delta = p("delta_rad", 0.12).value
        self.settle = p("settle_s", 1.6).value
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value

        self._centroid = None   # latest board centroid (u,v) px, or None
        self._q0 = None
        self._lock = threading.Lock()

        self.create_subscription(Image, "/stereo/left/image_raw", self._img, qos_profile_sensor_data)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = socket.create_connection((self.host, self.port), timeout=5)
        self._sf = self._sock.makefile("r")
        self.get_logger().info(f"joint_probe: joints {self.joints}, delta {self.delta} rad. "
                               f"Board in view + Send All Joints to start.")
        threading.Thread(target=self._run, daemon=True).start()

    def _img(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ds = 640.0 / gray.shape[1]
        small = cv2.resize(gray, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA)
        ok, c = cv2.findChessboardCorners(
            small, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        with self._lock:
            self._centroid = (c.reshape(-1, 2).mean(axis=0) / ds) if ok else None

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

    def _measure(self):
        time.sleep(self.settle)
        for _ in range(25):
            with self._lock:
                c = None if self._centroid is None else self._centroid.copy()
            if c is not None:
                return c
            time.sleep(0.1)
        return None

    def _run(self):
        self.get_logger().info("waiting for /joint_commands (Send All Joints) + board in view...")
        for _ in range(600):
            with self._lock:
                q0 = None if self._q0 is None else self._q0.copy()
            if q0 is not None:
                break
            time.sleep(0.1)
        if q0 is None:
            self.get_logger().error("no /joint_commands after 60s."); return
        if self._measure() is None:
            self.get_logger().error("board not detected — center it in view (check the stream)."); return

        self.get_logger().info(f"base pose {q0.round(3)} — probing each joint +/-{self.delta} rad\n")
        for ji in self.joints:
            qp = q0.copy(); qp[ji] += self.delta
            self._send(qp); cp = self._measure()
            qm = q0.copy(); qm[ji] -= self.delta
            self._send(qm); cm = self._measure()
            self._send(q0); self._measure()  # return to base
            if cp is None or cm is None:
                self.get_logger().warn(f"{NAMES[ji]:16s}: board left view during probe (smaller delta?)"); continue
            d = (cp - cm) / (2 * self.delta)   # d(u,v)/dq, px/rad
            ax = "u(horiz)" if abs(d[0]) >= abs(d[1]) else "v(vert)"
            val = d[0] if ax.startswith("u") else d[1]
            self.get_logger().info(
                f"{NAMES[ji]:16s}: d(u,v)/dq = [{d[0]:+7.0f} {d[1]:+7.0f}] px/rad   "
                f"dominant {'+' if val >= 0 else '-'}{ax}  |move|={np.linalg.norm(d):.0f} px/rad")
        self.get_logger().info("\nprobe done — returned to base. Reversed = sign opposite expected; "
                               "tiny |move| = under-scaled (steps/rad too low).")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
