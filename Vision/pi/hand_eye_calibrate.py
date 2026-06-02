#!/usr/bin/env python3
"""
hand_eye_calibrate.py — eye-in-hand calibration for the wrist-mounted stereo cam.

Solves T_ee_cam (camera pose in the end-effector frame) so camera-frame targets
from stereo_depth_node can be mapped into the arm base frame:

    p_base = T_base_ee @ T_ee_cam @ p_cam

Pose source: this arm runs moveo_publisher only (no robot_state_publisher / TF),
so T_base_ee is computed by FORWARD KINEMATICS on the commanded joint angles,
reusing moveo_publisher's own ikpy MOVEO_CHAIN (single source of truth, and
consistent with how cartesian commands are interpreted). Joint angles come from
/joint_commands (the arm is open-loop, so commanded == best estimate of actual).

  T_cam_target : solvePnP of the board in the RECTIFIED left image (the same frame
                 stereo_depth_node reports targets in).
  stationary   : detected from the FIXED board's image stillness (the board only
                 looks still when the camera/arm is not moving) — no encoders needed.

Procedure: clamp the checkerboard somewhere FIXED, jog the arm to ~18 varied poses
that keep the board in the left lens, pausing at each. It auto-captures when the
camera is still and the pose is new; after --count poses it runs
cv2.calibrateHandEye, writes hand_eye.yaml, and prints a consistency residual.

  !! CAVEAT: wrist roll (j4) scale is currently mis-calibrated, so FK is wrong for
     poses where j4 != 0. Until j4 scale is fixed, keep j4 FIXED (ideally 0) across
     all capture poses and vary j1/j2/j3/j5.

Prereqs running: stereo_camera_node (for /stereo/left/image_raw) and moveo_publisher.

Run:
    python3 hand_eye_calibrate.py --ros-args \
        -p calib:=/home/armpi/vision/stereo_calib.yaml \
        -p inner_cols:=8 -p inner_rows:=6 -p square_mm:=23.25
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty


def Rt_to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).flatten()
    return T


def img_to_np(msg):
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def rot_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class HandEyeNode(Node):
    def __init__(self):
        super().__init__("hand_eye_calibrate")
        p = self.declare_parameter
        self.calib_path = p("calib", "").value
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.square_m = p("square_mm", 23.25).value / 1000.0
        self.count = p("count", 18).value
        self.min_move_m = p("min_move_m", 0.04).value     # new-pose ee-translation threshold
        self.still_px = p("still_px", 3.0).value          # board-centroid stillness threshold
        self.reproj_max = p("reproj_max_px", 2.0).value   # reject scrambled detections; oblique views run ~1-1.5px
        self.j4_max = p("j4_max_rad", 0.1).value           # reject poses with wrist roll off zero (bad scale -> bad FK)
        self.joint_topic = p("joint_topic", "/joint_commands").value
        self.moveo_dir = os.path.expanduser(p("moveo_pub_dir", "~/ros_nodes").value)
        self.out = p("out", "hand_eye.yaml").value

        if not self.calib_path:
            raise RuntimeError("set -p calib:=/path/to/stereo_calib.yaml")
        self._load_rectify(self.calib_path)

        # Reuse moveo_publisher's exact kinematic chain for FK.
        if self.moveo_dir not in sys.path:
            sys.path.insert(0, self.moveo_dir)
        try:
            from moveo_publisher import MOVEO_CHAIN
        except Exception as e:
            raise RuntimeError(f"could not import MOVEO_CHAIN from {self.moveo_dir}: {e}")
        if MOVEO_CHAIN is None:
            raise RuntimeError("MOVEO_CHAIN is None (ikpy missing in moveo_publisher)")
        self.chain = MOVEO_CHAIN  # links: [Origin, j1..j5, ee]; FK returns ee in base

        # Board points in the BOARD frame, metres (so PnP t matches FK metres).
        self.objp = np.zeros((self.cols * self.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square_m

        self._joints = None          # latest [j1..j5]
        self.R_g2b, self.t_g2b, self.R_t2c, self.t_t2c = [], [], [], []
        self._prev_centroid = None
        self._last_cap_t = None      # ee translation of last captured pose
        self._done = False

        # /joint_commands is published BEST_EFFORT/VOLATILE by moveo_publisher (to
        # match the ESP micro-ROS subscriber) — match it or we receive nothing.
        joints_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Image, "/stereo/left/image_raw", self.image_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joint_topic, self.joints_cb, joints_qos)
        self.create_subscription(Empty, "/hand_eye/finish", lambda m: self._finish(), 10)
        self.get_logger().info(
            f"hand-eye (FK from {self.joint_topic}): board {self.cols}x{self.rows} @ "
            f"{self.square_m*1000:.2f}mm, target {self.count} poses. Clamp board FIXED, jog arm, "
            f"pause at each pose. CAVEAT: keep wrist roll (j4) fixed until its scale is calibrated. "
            f"Publish std_msgs/Empty to /hand_eye/finish to stop early.")

    def _load_rectify(self, path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(path)
        g = lambda n: fs.getNode(n).mat()
        K1, D1, R1, P1 = g("K1"), g("D1"), g("R1"), g("P1")
        w = int(fs.getNode("image_width").real()); h = int(fs.getNode("image_height").real())
        fs.release()
        self.mapL = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_16SC2)
        self.Kr = P1[:3, :3].copy()        # rectified-left intrinsics
        self.dist0 = np.zeros(5)           # rectified image has no distortion

    def joints_cb(self, msg):
        if len(msg.position) >= 5:
            self._joints = list(msg.position[:5])

    def _T_base_ee(self):
        full = np.array([0.0] + list(self._joints) + [0.0])  # [Origin, j1..j5, ee]
        return self.chain.forward_kinematics(full)

    def image_cb(self, msg):
        if self._done:
            return
        rect = cv2.remap(img_to_np(msg), *self.mapL, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        n = len(self.R_g2b)
        if not ok:
            self.get_logger().info(f"[{n}/{self.count}] board not visible", throttle_duration_sec=2.0)
            self._prev_centroid = None
            return
        if self._joints is None:
            self.get_logger().warn(f"[{n}/{self.count}] no {self.joint_topic} yet — move the arm once",
                                   throttle_duration_sec=2.0)
            return

        centroid = corners.reshape(-1, 2).mean(axis=0)
        moving = self._prev_centroid is not None and np.linalg.norm(centroid - self._prev_centroid) > self.still_px
        self._prev_centroid = centroid
        if moving:
            self.get_logger().info(f"[{n}/{self.count}] hold still...", throttle_duration_sec=1.5)
            return
        if abs(self._joints[3]) > self.j4_max:
            self.get_logger().warn(
                f"[{n}/{self.count}] wrist roll j4={self._joints[3]:+.2f} != 0 — keep j4 fixed (its "
                f"scale is uncalibrated -> FK wrong). Get angle variety from j5 pitch / j1 yaw.",
                throttle_duration_sec=1.5)
            return

        T_be = self._T_base_ee()
        t_ee = T_be[:3, 3]
        if self._last_cap_t is not None and np.linalg.norm(t_ee - self._last_cap_t) <= self.min_move_m:
            self.get_logger().info(f"[{n}/{self.count}] move to a NEW pose", throttle_duration_sec=1.5)
            return

        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        ok2, rvec, tvec = cv2.solvePnP(self.objp, corners, self.Kr, self.dist0)
        if not ok2:
            return
        # Quality gate: reject scrambled/mislocalized detections (would corrupt the calib).
        proj, _ = cv2.projectPoints(self.objp, rvec, tvec, self.Kr, self.dist0)
        reproj = float(np.linalg.norm(corners.reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean())
        if reproj > self.reproj_max:
            self.get_logger().warn(f"[{n}/{self.count}] rejected: reproj {reproj:.2f}px > "
                                   f"{self.reproj_max:.1f}px (bad detection)", throttle_duration_sec=1.0)
            return
        R_tc, _ = cv2.Rodrigues(rvec)
        self.R_g2b.append(T_be[:3, :3].copy()); self.t_g2b.append(t_ee.reshape(3, 1))
        self.R_t2c.append(R_tc); self.t_t2c.append(tvec.reshape(3, 1))
        self._last_cap_t = t_ee
        self.get_logger().info(f"captured pose {len(self.R_g2b)}/{self.count}  "
                               f"j4(roll)={self._joints[3]:+.3f}  target {np.linalg.norm(tvec)*100:.1f}cm  "
                               f"reproj {reproj:.2f}px")
        if len(self.R_g2b) >= self.count:
            self._finish()

    def _finish(self):
        if self._done:
            return
        n = len(self.R_g2b)
        if n < 5:
            self.get_logger().warn(f"only {n} poses; need >=5 (12-20 recommended). Keep going.")
            return
        self._done = True

        # Diagnose pose diversity: hand-eye needs varied ROTATIONS, not just
        # translations. Mean angular spread of the camera->target rotations.
        def ang_spread(Rs):
            return float(np.mean([rot_angle_deg(R @ Rs[0].T) for R in Rs]))
        tgt_spread = ang_spread(self.R_t2c)
        self.get_logger().info(
            f"pose diversity: camera-orientation spread {tgt_spread:.1f}deg "
            f"(need >~20-30deg of varied tilt/yaw for a good solve)")
        if tgt_spread < 15.0:
            self.get_logger().warn("LOW rotation diversity — recapture viewing the board "
                                   "from genuinely different angles (tilt j5, yaw j1), not just distances.")

        # FK-vs-camera consistency: the camera is rigid to the gripper, so the
        # rotation ANGLE between any two poses must match on the robot side (FK) and
        # the camera side, independent of T_ee_cam. Large mismatch => inaccurate robot
        # poses (open-loop joint-scale / link-length / homing errors), which no
        # T_ee_cam can fix.
        mism = []
        for i in range(len(self.R_g2b)):
            for j in range(i + 1, len(self.R_g2b)):
                a = rot_angle_deg(self.R_g2b[i].T @ self.R_g2b[j])      # robot relative rot (FK)
                b = rot_angle_deg(self.R_t2c[i] @ self.R_t2c[j].T)      # camera relative rot (observed)
                mism.append(abs(a - b))
        self.get_logger().info(
            f"FK-vs-camera rotation mismatch: mean {np.mean(mism):.1f}deg max {np.max(mism):.1f}deg "
            f"(<~2deg = robot FK accurate; large = open-loop joint-scale/kinematic error)")

        R_ce, t_ce = cv2.calibrateHandEye(
            self.R_g2b, self.t_g2b, self.R_t2c, self.t_t2c, method=cv2.CALIB_HAND_EYE_PARK)
        T_ee_cam = Rt_to_T(R_ce, t_ce)
        self.get_logger().info(f"\nT_ee_cam (camera in end-effector frame):\n{np.array2string(T_ee_cam, precision=4)}")

        # Consistency: T_base_target should be constant across poses.
        tbt = [Rt_to_T(Rg, tg) @ T_ee_cam @ Rt_to_T(Rt, tt)
               for Rg, tg, Rt, tt in zip(self.R_g2b, self.t_g2b, self.R_t2c, self.t_t2c)]
        trans = np.array([T[:3, 3] for T in tbt])
        spread_mm = np.linalg.norm(trans - trans.mean(0), axis=1).mean() * 1000
        ang = float(np.mean([rot_angle_deg(T[:3, :3] @ tbt[0][:3, :3].T) for T in tbt]))
        self.get_logger().info(
            f"consistency residual: board position spread {spread_mm:.1f}mm, "
            f"orientation spread {ang:.2f}deg  (lower is better; <~5mm / <~1deg is good)")

        fs = cv2.FileStorage(self.out, cv2.FILE_STORAGE_WRITE)
        fs.write("T_ee_cam", T_ee_cam)
        fs.write("camera_frame", "rectified_left_optical")
        fs.write("ee_frame", "moveo_chain_ee")
        fs.write("num_poses", n)
        fs.write("residual_mm", float(spread_mm))
        fs.write("residual_deg", ang)
        fs.release()
        self.get_logger().info(f"wrote {self.out}. Ctrl-C to exit.")


def main():
    rclpy.init()
    node = HandEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
