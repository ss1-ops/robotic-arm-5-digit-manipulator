#!/usr/bin/env python3
"""
approach_object.py — visual-servo to an object and stop at a target distance.

Closed-loop on the camera (immune to FK/joint errors). Uses the depth node's
metric target point (/stereo/target/point, camera-frame XYZ in metres) so it can
stop at a real distance:
    centre it:  drive (x, y) -> 0   (j1 yaw, j5 pitch)
    approach:   drive  z -> target_dist_m   (j2/j3 extend the arm)
Online 3xN Jacobian d(x,y,z)/dq + Broyden update; the z-error shrinks as it nears
target, so it slows into range (distance-gated). Stops when centred AND at range.

Pair with the depth node running a stable object detector (color from the GUI
click, or nearest):
    stereo_depth_node ... -p detector:=color -p h_lo:=.. (see run_approach_object.sh)

Prereqs: camera + depth + moveo_publisher running, ESP connected, target in view,
jog (Send All Joints) to seed /joint_commands. Keep j4 = 0.  !! E-STOP ready.

Run:
    python3 approach_object.py --ros-args -p target_dist_m:=0.22
"""

import json
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty

JOINT_LIM = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.75, 1.75)]


class ApproachObject(Node):
    def __init__(self):
        super().__init__("approach_object")
        p = self.declare_parameter
        self.ctrl = list(p("control_joints", [0, 1, 2, 4]).value)  # j1,j2,j3,j5
        self.target = p("target_dist_m", 0.22).value
        self.host = p("host", "127.0.0.1").value
        self.port = p("port", 9000).value
        self.jog = p("jog_rad", 0.08).value
        self.gain = p("gain", 0.3).value
        self.tol_xy = p("tol_xy_m", 0.015).value
        self.tol_z = p("tol_z_m", 0.02).value
        self.max_step = p("max_step_rad", 0.08).value
        self.settle_s = p("settle_s", 1.4).value
        self.damp = p("damp", 0.02).value
        self.max_iters = p("max_iters", 60).value

        self._pt = None
        self._state = "NONE"
        self._q0 = None
        self._lock = threading.Lock()
        self._abort = False

        self.create_subscription(PointStamped, "/stereo/target/point", self._ptcb, 10)
        self.create_subscription(String, "/stereo/target/state", self._stcb, 10)
        self.create_subscription(Empty, "/approach/stop", lambda m: setattr(self, "_abort", True), 10)
        jq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, "/joint_commands", self._jc, jq)

        self._sock = None
        for _ in range(15):   # moveo_publisher may not be listening yet — retry
            try:
                self._sock = socket.create_connection((self.host, self.port), timeout=5)
                break
            except OSError:
                time.sleep(1.0)
        if self._sock is None:
            raise RuntimeError(f"could not connect to moveo_publisher {self.host}:{self.port} (running?)")
        self._sf = self._sock.makefile("r")
        self.get_logger().info(f"approach_object: stop at {self.target*100:.0f}cm, joints {self.ctrl}. "
                               f"Object in view + Send All Joints. j4=0. E-STOP / /approach/stop ready.")
        threading.Thread(target=self._run, daemon=True).start()

    def _ptcb(self, m):
        with self._lock:
            self._pt = np.array([m.point.x, m.point.y, m.point.z])

    def _stcb(self, m):
        self._state = m.data

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
                t, s = (None if self._pt is None else self._pt.copy()), self._state
            if t is not None and s == "TRACK":
                return t
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
        if self._measure() is None:
            self.get_logger().error("no TRACK target — is the object detected? (depth node detector/HSV)"); return

        self.get_logger().info("estimating 3xN Jacobian...")
        J = np.zeros((3, len(self.ctrl)))
        for k, ji in enumerate(self.ctrl):
            qp = q.copy(); qp[ji] += self.jog; self._send(qp); fp = self._measure()
            qm = q.copy(); qm[ji] -= self.jog; self._send(qm); fm = self._measure()
            self._send(q); self._measure()
            if fp is None or fm is None:
                self.get_logger().error("lost target during Jacobian probe."); return
            J[:, k] = (fp - fm) / (2 * self.jog)
            self.get_logger().info(f"  j{ji+1}: d(x,y,z)/dq = {J[:,k].round(3)}")
        Jpinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(3))

        goal = np.array([0.0, 0.0, self.target])
        self.get_logger().info("servoing (centre + approach)...")
        f_prev = dq_prev = None
        for it in range(self.max_iters):
            if self._abort:
                self.get_logger().warn("ABORT received — stopping."); return
            f = self._measure()
            if f is None:
                self.get_logger().warn(f"[{it}] target lost — holding."); continue
            if f_prev is not None and dq_prev is not None:
                den = float(dq_prev @ dq_prev)
                if den > 1e-6:
                    J = J + np.outer((f - f_prev) - J @ dq_prev, dq_prev) / den
                    Jpinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(3))
            e = f - goal
            cxy = float(np.linalg.norm(e[:2]))
            cz = float(e[2])
            if cxy < self.tol_xy and abs(cz) < self.tol_z:
                self.get_logger().info(f"REACHED object: centre {cxy*100:.1f}cm, dist {f[2]*100:.0f}cm "
                                       f"(target {self.target*100:.0f}) in {it} steps.")
                return
            dq = -self.gain * (Jpinv @ e)
            n = np.linalg.norm(dq)
            if n > self.max_step:
                dq *= self.max_step / n
            for k, ji in enumerate(self.ctrl):
                q[ji] += dq[k]
            q = self._send(q)
            f_prev, dq_prev = f.copy(), dq.copy()
            self.get_logger().info(f"[{it}] centre={cxy*100:4.1f}cm dist={f[2]*100:4.0f}cm "
                                   f"(->{self.target*100:.0f})  dq={dq.round(3)}")
        self.get_logger().warn("max iterations — lower gain / raise settle, or recheck the target.")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ApproachObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
