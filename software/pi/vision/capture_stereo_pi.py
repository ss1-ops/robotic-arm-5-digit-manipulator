#!/usr/bin/env python3
"""
capture_stereo_pi.py — headless stereo-pair capture for recalibration on the Pi.

Subscribes to the running stereo_camera_node topics (/stereo/{left,right}/image_raw)
and auto-saves a left/right PNG pair whenever the checkerboard is detected in BOTH
lenses, held still, and in a pose meaningfully different from the last save. Frame
the board with the browser stream (http://armpi.local:8080/); aim for ~25 pairs
covering varied positions, distances, and tilts (including the frame corners).

Captures the RAW as-mounted (upside-down) frames, so the resulting calibration is
valid for this mount — no rotation hacks. Then run, on the Pi:

    python3 ~/vision/stereo_calibrate.py --captures ~/vision/recal/captures \
        --inner-cols 8 --inner-rows 6 --square 23.25 --show

(stereo_calibrate.py lives in Vision/calibration/ — copy it to ~/vision/ if needed.)

Run:
    python3 capture_stereo_pi.py --ros-args -p count:=25
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DETECT_WIDTH = 640
STABLE_PX = 6.0
COOLDOWN_S = 0.5


def img_to_np(msg):
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def stamp_key(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def find_board(bgr, pattern):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    w = gray.shape[1]
    ds = DETECT_WIDTH / w if w > DETECT_WIDTH else 1.0
    small = cv2.resize(gray, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA) if ds != 1.0 else gray
    ok, c = cv2.findChessboardCorners(
        small, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if ok and ds != 1.0:
        c = c / ds
    return ok, c


class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture_stereo_pi")
        p = self.declare_parameter
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.count = p("count", 25).value
        self.out = os.path.expanduser(p("out", "~/vision/recal/captures").value)
        self.min_move = p("min_move_px", 0.0).value  # default 5% of half-width below

        os.makedirs(os.path.join(self.out, "left"), exist_ok=True)
        os.makedirs(os.path.join(self.out, "right"), exist_ok=True)

        self.pattern = (self.cols, self.rows)
        self._left = None
        self._right = None
        self._prev_centroid = None
        self._last_saved_centroid = None
        self._last_save_t = 0.0
        self.saved = 0

        self.create_subscription(Image, "/stereo/left/image_raw", self.left_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/stereo/right/image_raw", self.right_cb, qos_profile_sensor_data)
        self.create_timer(2.0, self._tick)
        self.get_logger().info(
            f"recal capture: board {self.cols}x{self.rows}, target {self.count} pairs -> {self.out}. "
            f"Frame the board (both lenses) via http://armpi.local:8080/ ; vary pos/dist/tilt.")

    def left_cb(self, msg):
        self._left = (stamp_key(msg), img_to_np(msg))
        self._try()

    def right_cb(self, msg):
        self._right = (stamp_key(msg), img_to_np(msg))
        self._try()

    def _tick(self):
        self.get_logger().info(f"[{self.saved}/{self.count}] capturing... (move the board to vary the view)")

    def _try(self):
        if self._left is None or self._right is None or self.saved >= self.count:
            return
        if abs(self._left[0] - self._right[0]) > 5_000_000:  # pair same-frame L/R
            return
        left, right = self._left[1], self._right[1]
        self._left = self._right = None

        okl, cl = find_board(left, self.pattern)
        if not okl:
            self._prev_centroid = None
            return
        okr, _ = find_board(right, self.pattern)
        if not okr:
            return  # need the board in BOTH lenses

        if self.min_move <= 0:
            self.min_move = 0.05 * left.shape[1]
        centroid = cl.reshape(-1, 2).mean(axis=0)
        moving = self._prev_centroid is not None and np.linalg.norm(centroid - self._prev_centroid) > STABLE_PX
        self._prev_centroid = centroid
        if moving:
            return
        novel = (self._last_saved_centroid is None
                 or np.linalg.norm(centroid - self._last_saved_centroid) > self.min_move)
        now = time.time()
        if not novel or (now - self._last_save_t) < COOLDOWN_S:
            return

        n = self.saved
        cv2.imwrite(os.path.join(self.out, "left", f"pair_{n:03d}.png"), left)
        cv2.imwrite(os.path.join(self.out, "right", f"pair_{n:03d}.png"), right)
        self.saved += 1
        self._last_saved_centroid = centroid
        self._last_save_t = now
        self.get_logger().info(f"saved pair {n:03d}  ({self.saved}/{self.count})")
        if self.saved >= self.count:
            self.get_logger().info(
                f"DONE — {self.saved} pairs in {self.out}. Now run:\n"
                f"  python3 ~/vision/stereo_calibrate.py --captures {self.out} "
                f"--inner-cols {self.cols} --inner-rows {self.rows} --square 23.25")


def main():
    rclpy.init()
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
