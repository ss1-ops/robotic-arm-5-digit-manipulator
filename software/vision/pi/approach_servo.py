#!/usr/bin/env python3
"""
approach_servo.py — distance-gated visual approach ("move to the object").

Extends the pointing IBVS with a third image feature: the board's apparent SIZE
(mean corner spread in px), which grows as the eye-in-hand camera nears a fixed
board — a monocular distance proxy (no stereo/FK needed, robust like pointing).

Servo drives the 3-vector error to zero:
    e = (u - cx, v - cy, spread - spread_target)
using control joints (default j1,j2,j3,j5) via an online 3xN Jacobian + damped
least squares. Centring (u,v) and approach (spread) happen together; the size
error shrinks as it nears the target, so the arm naturally SLOWS into range
(distance-gated). Converges when centred AND at the target size.

spread_target: set explicitly, or leave 0 to auto-target approach_factor x the
initial spread (move closer by that factor).

Prereqs: stereo_camera_node + moveo_publisher running, ESP connected, board in
view, jog (Send All Joints) to seed /joint_commands. Keep j4 = 0.

  !! Approaches a FIXED board — keep clearance and the E-STOP ready; start gentle.

Run:
    python3 approach_servo.py --ros-args -p approach_factor:=1.3
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


class ApproachNode(Node):
    def __init__(self):
        super().__init__("approach_servo")
        p = self.declare_parameter
        self.ctrl = list(p("control_joints", [0, 1, 2, 4]).value)  # j1,j2,j3,j5 (j4=0)
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value
        self.jog = p("jog_rad", 0.08).value
        self.gain = p("gain", 0.25).value          # lower = smoother (less overshoot)
        self.damp = p("damp", 20.0).value          # DLS damping (higher = smoother/slower)
        self.tol_px = p("tol_px", 25.0).value          # centring tolerance
        self.tol_spread = p("tol_spread_px", 8.0).value # size tolerance
        self.target_spread = p("spread_target_px", 0.0).value  # 0 => auto
        self.approach_factor = p("approach_factor", 1.3).value  # grow board by this (closer)
        self.max_step = p("max_step_rad", 0.08).value
        self.settle_s = p("settle_s", 1.4).value
        self.max_iters = p("max_iters", 60).value

        self._feat = None       # (u, v, spread) at detect scale
        self._center = None
        self._q0 = None
        self._lock = threading.Lock()

        self.create_subscription(Image, "/stereo/left/image_raw", self._img, qos_profile_sensor_data)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = socket.create_connection((self.host, self.port), timeout=5)
        self._sf = self._sock.makefile("r")
        self.get_logger().info(f"approach servo: control joints {self.ctrl}, gain {self.gain}. "
                               f"Board in view + Send All Joints. Keep j4=0. E-STOP ready.")
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
            if ok:
                pts = c.reshape(-1, 2)
                ctr = pts.mean(axis=0)
                spread = float(np.linalg.norm(pts - ctr, axis=1).mean())
                self._feat = np.array([ctr[0], ctr[1], spread])
            else:
                self._feat = None

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
                f = None if self._feat is None else self._feat.copy()
            if f is not None:
                return f
            time.sleep(0.1)
        return None

    def _run(self):
        self.get_logger().info("waiting for /joint_commands — click 'Send All Joints'...")
        q = None
        for i in range(3000):
            with self._lock:
                if self._q0 is not None:
                    q = self._q0.copy(); break
            if i and i % 100 == 0:
                self.get_logger().info("still waiting — click 'Send All Joints'...")
            time.sleep(0.1)
        if q is None:
            self.get_logger().error("no /joint_commands after 300s."); return
        f0 = self._measure()
        if f0 is None:
            self.get_logger().error("board not detected — center it in view."); return

        with self._lock:
            ctr = self._center.copy()
        s_tgt = self.target_spread if self.target_spread > 0 else f0[2] * self.approach_factor
        goal = np.array([ctr[0], ctr[1], s_tgt])
        self.get_logger().info(f"initial spread {f0[2]:.0f}px -> target {s_tgt:.0f}px "
                               f"(factor {s_tgt/f0[2]:.2f}); centring + approaching together.")

        # Jacobian J = d(u,v,spread)/dq for each control joint (central difference).
        self.get_logger().info("estimating 3xN Jacobian...")
        J = np.zeros((3, len(self.ctrl)))
        for k, ji in enumerate(self.ctrl):
            qp = q.copy(); qp[ji] += self.jog; self._send(qp); fp = self._measure()
            qm = q.copy(); qm[ji] -= self.jog; self._send(qm); fm = self._measure()
            self._send(q); self._measure()
            if fp is None or fm is None:
                self.get_logger().error("lost board during Jacobian probe."); return
            J[:, k] = (fp - fm) / (2 * self.jog)
            self.get_logger().info(f"  j{ji+1}: d(u,v,s)/dq = {J[:,k].round(0)}")
        Jpinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(3))

        self.get_logger().info("servoing (centre + approach)...")
        f_prev = dq_prev = None
        for it in range(self.max_iters):
            f = self._measure()
            if f is None:
                self.get_logger().warn(f"[{it}] board lost — holding."); continue
            # Broyden rank-1 update: keep J current as the arm moves (the centring
            # response changes as the arm extends), which removes servo oscillation.
            if f_prev is not None and dq_prev is not None:
                denom = float(dq_prev @ dq_prev)
                if denom > 1e-6:
                    J = J + np.outer((f - f_prev) - J @ dq_prev, dq_prev) / denom
                    Jpinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(3))
            with self._lock:
                ctr = self._center.copy()
            goal[:2] = ctr
            e = f - goal
            centre_err = float(np.linalg.norm(e[:2]))
            size_err = float(e[2])
            if centre_err < self.tol_px and abs(size_err) < self.tol_spread:
                self.get_logger().info(f"REACHED: centre {centre_err:.0f}px, size err {size_err:+.0f}px "
                                       f"in {it} steps. Approach done.")
                return
            dq = -self.gain * (Jpinv @ e)
            n = np.linalg.norm(dq)
            if n > self.max_step:
                dq *= self.max_step / n
            for k, ji in enumerate(self.ctrl):
                q[ji] += dq[k]
            q = self._send(q)
            f_prev, dq_prev = f.copy(), dq.copy()
            self.get_logger().info(f"[{it}] centre={centre_err:4.0f}px size_err={size_err:+5.0f}px "
                                   f"spread={f[2]:4.0f}  dq={dq.round(3)}")
        self.get_logger().warn("max iterations — lower gain / raise settle, or recheck the board.")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
