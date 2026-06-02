#!/usr/bin/env python3
"""
stereo_camera_node.py — ROS 2 driver for the ELP dual-lens USB stereo camera.

Runs on the Raspberry Pi (ROS 2 Jazzy, Linux/V4L2). The ELP enumerates as ONE
UVC /dev/video* device producing a side-by-side frame (left | right). This node
opens it with the V4L2 backend, forces MJPEG (required for 2560x960 over USB2),
splits each frame, and publishes:

    /stereo/left/image_raw     sensor_msgs/Image   (bgr8)
    /stereo/right/image_raw    sensor_msgs/Image   (bgr8)
    /stereo/left/camera_info   sensor_msgs/CameraInfo
    /stereo/right/camera_info  sensor_msgs/CameraInfo

CameraInfo is filled from the Mac-side calibration (stereo_calib.yaml) so that
downstream rectification (our depth node, or stereo_image_proc) has K/D/R/P.

This is a loose script (run with `python3`, like moveo_publisher.py) — no colcon
build needed. Image messages are built by hand to avoid a cv_bridge dependency.

Run:
    python3 stereo_camera_node.py --ros-args \
        -p device:=0 -p calib:=/home/armpi/vision/stereo_calib.yaml

Verify on the Pi (or Mac via ROS):
    ros2 topic hz /stereo/left/image_raw
    ros2 topic echo /stereo/left/camera_info --once
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


def load_calib(path):
    """Read the Mac-side stereo calibration; return per-camera K/D/R/P + size."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None
    g = lambda n: fs.getNode(n).mat()
    cal = dict(
        K1=g("K1"), D1=g("D1"), R1=g("R1"), P1=g("P1"),
        K2=g("K2"), D2=g("D2"), R2=g("R2"), P2=g("P2"),
        width=int(fs.getNode("image_width").real()),
        height=int(fs.getNode("image_height").real()),
    )
    fs.release()
    return cal


def make_camera_info(stamp, frame_id, w, h, K, D, R, P):
    ci = CameraInfo()
    ci.header.stamp = stamp
    ci.header.frame_id = frame_id
    ci.width = w
    ci.height = h
    d = np.asarray(D).flatten()
    ci.distortion_model = "rational_polynomial" if d.size >= 8 else "plumb_bob"
    ci.d = d.tolist()
    ci.k = np.asarray(K).flatten().tolist()
    ci.r = np.asarray(R).flatten().tolist()
    ci.p = np.asarray(P).flatten().tolist()
    return ci


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_camera")
        # Parameters
        self.device = self.declare_parameter("device", 0).value            # /dev/video index
        self.device_path = self.declare_parameter("device_path", "").value  # e.g. /dev/video0; overrides index if set
        self.width = self.declare_parameter("width", 2560).value      # full side-by-side width
        self.height = self.declare_parameter("height", 960).value
        self.fps = self.declare_parameter("fps", 30).value
        # The camera is mounted upside down, but flipping a side-by-side stereo frame
        # in software can't recover the (upright) calibration's rectification (it swaps
        # the physical L/R cameras; rectification is role-specific). So we stream RAW
        # as-mounted frames and recalibrate in this orientation; keep this False.
        # (Per-half rotation kept only as an option; it does NOT fix stereo geometry.)
        self.rotate_180 = self.declare_parameter("rotate_180", False).value
        self.calib_path = self.declare_parameter("calib", "").value
        self.left_frame = self.declare_parameter("left_frame", "stereo_left_optical").value
        self.right_frame = self.declare_parameter("right_frame", "stereo_right_optical").value

        # Calibration (optional — without it we still stream, but no CameraInfo K/D/R/P)
        self.calib = load_calib(self.calib_path) if self.calib_path else None
        if self.calib_path and self.calib is None:
            self.get_logger().warn(f"could not load calib '{self.calib_path}'; CameraInfo will be empty")
        elif self.calib:
            self.get_logger().info(f"loaded calibration from {self.calib_path}")

        # Publishers (best-effort sensor QoS, standard for camera streams)
        q = qos_profile_sensor_data
        self.pub_l = self.create_publisher(Image, "/stereo/left/image_raw", q)
        self.pub_r = self.create_publisher(Image, "/stereo/right/image_raw", q)
        self.pub_li = self.create_publisher(CameraInfo, "/stereo/left/camera_info", q)
        self.pub_ri = self.create_publisher(CameraInfo, "/stereo/right/camera_info", q)

        self.cap = self._open_camera()
        self._stop = False
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _open_camera(self):
        dev = self.device_path if self.device_path else int(self.device)
        # Retry: on restart the previous node's USB handle can take a moment to release.
        cap = None
        for attempt in range(6):
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if cap.isOpened():
                break
            cap.release()
            self.get_logger().warn(f"camera '{dev}' open attempt {attempt + 1}/6 failed; retrying...")
            time.sleep(1.0)
        if cap is None or not cap.isOpened():
            raise RuntimeError(
                f"cannot open camera '{dev}' after retries. Check `lsusb | grep 32e4`, "
                "that /dev/video0 exists, and that the user is in the 'video' group.")
        # Order matters: MJPG first, then resolution (MJPG needed for 2560x960 / USB2).
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # low latency for visual servoing
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"camera open: requested {self.width}x{self.height}, got {aw}x{ah}")
        if aw < self.width:
            self.get_logger().warn(
                f"got width {aw} (< {self.width}); device may be in a single-lens fallback mode.")
        return cap

    def _to_image_msg(self, stamp, frame_id, img):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg

    def _capture_loop(self):
        frames, t0 = 0, time.time()
        while not self._stop and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.get_logger().warn("frame grab failed", throttle_duration_sec=2.0)
                time.sleep(0.02)
                continue
            half = frame.shape[1] // 2
            left = frame[:, :half]
            right = frame[:, half:2 * half]
            if self.rotate_180:
                # Upside-down mount rotates each lens image 180 on its sensor, but the
                # hardware still reads sensor0->left half / sensor1->right half. So
                # rotate each half INDEPENDENTLY (a full-frame rotation would wrongly
                # swap the lenses, feeding sensor1 into the left topic with K1's maps).
                left = cv2.rotate(left, cv2.ROTATE_180)
                right = cv2.rotate(right, cv2.ROTATE_180)

            stamp = self.get_clock().now().to_msg()
            self.pub_l.publish(self._to_image_msg(stamp, self.left_frame, np.ascontiguousarray(left)))
            self.pub_r.publish(self._to_image_msg(stamp, self.right_frame, np.ascontiguousarray(right)))
            if self.calib:
                c, h, w = self.calib, left.shape[0], left.shape[1]
                self.pub_li.publish(make_camera_info(stamp, self.left_frame, w, h,
                                                     c["K1"], c["D1"], c["R1"], c["P1"]))
                self.pub_ri.publish(make_camera_info(stamp, self.right_frame, w, h,
                                                     c["K2"], c["D2"], c["R2"], c["P2"]))

            frames += 1
            if time.time() - t0 >= 5.0:
                self.get_logger().info(f"streaming {frames / (time.time() - t0):.1f} fps")
                frames, t0 = 0, time.time()

    def destroy_node(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
