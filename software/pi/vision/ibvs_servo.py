#!/usr/bin/env python3
"""
ibvs_servo.py — image-based visual servoing ("point at object") for the Moveo.

Closes the loop on the CAMERA, not on forward kinematics — immune to the open-loop
arm's commanded-vs-actual pose error (and to reversed/mis-scaled joints) because it
MEASURES the joint->image relationship online and corrects from camera feedback.

Pixel-based + robust: detects the checkerboard centroid directly in the raw left
image (no depth node / rectification) and drives it to the image centre using a
pan/tilt pair:
    j1 (waist/yaw)   -> horizontal (u)
    j5 (wrist pitch) -> vertical   (v)

  1. estimate J = d(u,v)/dq by jogging each control joint (central difference)
  2. servo: dq = -gain * J^+ * (centroid - image_centre)   until centred

Prereqs: stereo_camera_node + moveo_publisher running, ESP connected (arm moves),
board in view, and a jog (Send All Joints) to seed /joint_commands.

Run:
    python3 ibvs_servo.py --ros-args -p control_joints:="[0,4]"
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


class IBVSNode(Node):
    def __init__(self):
        super().__init__("ibvs_servo")
        p = self.declare_parameter
        self.ctrl = list(p("control_joints", [0, 4]).value)   # j1 (yaw), j5 (pitch)
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value
        self.jog = p("jog_rad", 0.08).value
        self.gain = p("gain", 0.4).value
        self.tol_px = p("tol_px", 25.0).value
        self.max_step = p("max_step_rad", 0.10).value
        self.settle_s = p("settle_s", 1.4).value
        self.max_iters = p("max_iters", 40).value

        self._c = None           # latest board centroid (u,v) at detect scale
        self._center = None      # image centre at detect scale
        self._q0 = None
        self._lock = threading.Lock()

        self.create_subscription(Image, "/stereo/left/image_raw", self._img, qos_profile_sensor_data)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = socket.create_connection((self.host, self.port), timeout=5)
        self._sf = self._sock.makefile("r")
        self.get_logger().info(f"IBVS pointing (pixel/board): control joints {self.ctrl} "
                               f"(j1=yaw, j5=pitch). Board in view + Send All Joints to start.")
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
            self._center = np.array([small.shape[1] / 2.0, small.shape[0] / 2.0])
            self._c = c.reshape(-1, 2).mean(axis=0) if ok else None

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
        return q

    def _measure(self):
        time.sleep(self.settle_s)
        for _ in range(25):
            with self._lock:
                c = None if self._c is None else self._c.copy()
            if c is not None:
                return c
            time.sleep(0.1)
        return None

    def _run(self):
        self.get_logger().info("waiting for /joint_commands — click 'Send All Joints' in the GUI...")
        q = None
        for i in range(3000):  # ~300s, with periodic reminders
            with self._lock:
                if self._q0 is not None:
                    q = self._q0.copy()
                    break
            if i and i % 100 == 0:
                self.get_logger().info("still waiting — click 'Send All Joints' in the GUI...")
            time.sleep(0.1)
        if q is None:
            self.get_logger().error("no /joint_commands after 300s."); return
        self.get_logger().info(f"got joints {q.round(3)} — starting.")
        base = self._measure()
        if base is None:
            self.get_logger().error("board not detected — center it in view."); return

        # 1) Jacobian J = d(u,v)/dq (central difference) for the control joints.
        self.get_logger().info("estimating joint->image Jacobian...")
        J = np.zeros((2, len(self.ctrl)))
        for k, ji in enumerate(self.ctrl):
            qp = q.copy(); qp[ji] += self.jog; self._send(qp); cp = self._measure()
            qm = q.copy(); qm[ji] -= self.jog; self._send(qm); cm = self._measure()
            self._send(q); self._measure()
            if cp is None or cm is None:
                self.get_logger().error("lost board during Jacobian probe."); return
            J[:, k] = (cp - cm) / (2 * self.jog)
            self.get_logger().info(f"  j{ji+1}: d(u,v)/dq = {J[:,k].round(0)}")
        Jpinv = J.T @ np.linalg.inv(J @ J.T + 1e1 * np.eye(2))  # damped (px units)

        # 2) Servo the centroid to the image centre.
        self.get_logger().info("servoing to centre the board...")
        for it in range(self.max_iters):
            c = self._measure()
            if c is None:
                self.get_logger().warn(f"[{it}] board lost — holding."); continue
            with self._lock:
                ctr = self._center.copy()
            e = c - ctr
            err = float(np.linalg.norm(e))
            if err < self.tol_px:
                self.get_logger().info(f"CENTERED: |err|={err:.0f}px in {it} steps. Pointing done.")
                return
            dq = -self.gain * (Jpinv @ e)
            n = np.linalg.norm(dq)
            if n > self.max_step:
                dq *= self.max_step / n
            for k, ji in enumerate(self.ctrl):
                q[ji] += dq[k]
            q = self._send(q)
            self.get_logger().info(f"[{it}] err={err:5.0f}px  centroid={c.round(0)}  dq={dq.round(3)}")
        self.get_logger().warn("max iterations — lower gain / raise settle, or recheck the board.")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = IBVSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
