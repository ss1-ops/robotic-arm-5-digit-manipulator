#!/usr/bin/env python3
"""
stereo_depth_node.py — detect a target and estimate its distance from the stereo
stream (ROS 2, runs on the Pi). This is the perception half of "point at object".

Pipeline (per synced left/right pair):
  1. rectify both images (maps + Q recomputed from stereo_calib.yaml at --scale)
  2. StereoSGBM disparity -> 3D (camera frame, metres)
  3. detect target pixel (u,v):
       --detector nearest : closest valid-depth blob (object-agnostic; good for
                            bring-up and for monitoring distance during approach)
       --detector color   : largest blob within an HSV range (a specific object)
  4. sample robust median depth in a window around (u,v) and apply distance gating

Publishes (camera/left optical frame):
  /stereo/target/point     geometry_msgs/PointStamped   (only when depth valid)
  /stereo/target/distance  std_msgs/Float32             (metres; -1 if unknown)
  /stereo/target/state     std_msgs/String              TRACK | FAR | BEARING_ONLY | NONE
  /stereo/target/debug     sensor_msgs/Image            overlay (view in rqt_image_view)

The camera->base transform (hand-eye) is a separate step; once available, an
approach controller maps /stereo/target/point into the moveo_publisher cartesian
interface and paces speed off the gate state.

Run:
    python3 stereo_depth_node.py --ros-args \
        -p calib:=/home/armpi/vision/stereo_calib.yaml -p detector:=nearest
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String


def load_calib_scaled(path, scale):
    """Load K/D/R/T and recompute rectification maps + Q at the processing scale."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    g = lambda n: fs.getNode(n).mat()
    K1, D1, K2, D2 = g("K1"), g("D1"), g("K2"), g("D2")
    R, T = g("R"), g("T")
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()
    sw, sh = int(round(w * scale)), int(round(h * scale))
    S = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]], np.float64)
    K1s, K2s = S @ K1, S @ K2
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1s, D1, K2s, D2, (sw, sh), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, (sw, sh), cv2.CV_16SC2)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, (sw, sh), cv2.CV_16SC2)
    return dict(Q=Q, mapL=(mapLx, mapLy), mapR=(mapRx, mapRy), size=(sw, sh))


def img_to_np(msg):
    """sensor_msgs/Image (bgr8) -> HxWx3 ndarray (no cv_bridge)."""
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def stamp_key(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__("stereo_depth")
        p = self.declare_parameter
        self.calib_path = p("calib", "").value
        self.scale = p("scale", 0.5).value
        self.detector = p("detector", "nearest").value          # nearest | color | board
        self.bcols = p("inner_cols", 8).value                    # for detector=board
        self.brows = p("inner_rows", 6).value
        self.near_cm = p("near_cm", 8.0).value
        self.far_cm = p("far_cm", 120.0).value                   # the distance gate
        self.min_area = p("min_area_px", 400).value
        self.num_disp = p("num_disp", 128).value
        self.block = p("block", 5).value
        self.frame_id = p("frame_id", "stereo_left_optical").value
        # HSV bounds for --detector color
        self.hsv_lo = np.array([p("h_lo", 35).value, p("s_lo", 80).value, p("v_lo", 60).value])
        self.hsv_hi = np.array([p("h_hi", 85).value, p("s_hi", 255).value, p("v_hi", 255).value])

        if not self.calib_path:
            raise RuntimeError("set -p calib:=/path/to/stereo_calib.yaml")
        self.cal = load_calib_scaled(self.calib_path, self.scale)
        self.get_logger().info(
            f"loaded {self.calib_path}, detector={self.detector}, "
            f"gate {self.near_cm:.0f}-{self.far_cm:.0f}cm, proc size {self.cal['size']}")

        nd = max(16, (int(self.num_disp) // 16) * 16)
        bs = int(self.block) | 1
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=nd, blockSize=bs,
            P1=8 * 3 * bs ** 2, P2=32 * 3 * bs ** 2, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        q = qos_profile_sensor_data
        self.pub_pt = self.create_publisher(PointStamped, "/stereo/target/point", 10)
        self.pub_d = self.create_publisher(Float32, "/stereo/target/distance", 10)
        self.pub_s = self.create_publisher(String, "/stereo/target/state", 10)
        self.pub_dbg = self.create_publisher(Image, "/stereo/target/debug", q)
        self.create_subscription(Image, "/stereo/left/image_raw", self.left_cb, q)
        self.create_subscription(Image, "/stereo/right/image_raw", self.right_cb, q)

        self._left = None   # (stamp_key, ndarray, header)
        self._right = None  # (stamp_key, ndarray)
        self._fps = 0.0
        self._t = time.time()
        self._nl = self._nr = self._nmatch = 0
        self._lastdiff = 0
        self.create_timer(2.0, self._diag)

    def _diag(self):
        self.get_logger().info(
            f"rx L={self._nl} R={self._nr} matched={self._nmatch} "
            f"lastdiff_ms={self._lastdiff/1e6:.0f}")

    def right_cb(self, msg):
        self._nr += 1
        self._right = (stamp_key(msg), img_to_np(msg))
        self._try_process()

    def _detect(self, bgr, disp, valid, dist_cm):
        """Return ((u,v), mask_or_None) for the target, or (None, mask)."""
        if self.detector == "board":
            # Rock-stable target for servoing/diagnostics: the checkerboard centroid.
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            okb, corners = cv2.findChessboardCorners(
                gray, (self.bcols, self.brows),
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
            if not okb:
                return None, None
            c = corners.reshape(-1, 2).mean(axis=0)
            return (int(c[0]), int(c[1])), None
        if self.detector == "color":
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_lo, self.hsv_hi)
        else:  # nearest: closest 25% of valid-depth pixels within gate
            in_gate = valid & (dist_cm >= self.near_cm) & (dist_cm <= self.far_cm)
            mask = np.zeros(disp.shape, np.uint8)
            if in_gate.any():
                thr = np.percentile(dist_cm[in_gate], 25)
                mask[in_gate & (dist_cm <= thr)] = 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, mask
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area:
            return None, mask
        M = cv2.moments(c)
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])), mask

    def _sample_xyz_mask(self, pts3d, valid, mask):
        """Median 3D point (mm) over valid-depth pixels inside the colour blob.
        Robust for solid objects whose interior has no stereo disparity."""
        m = valid & (mask > 0)
        if int(m.sum()) < 10:
            return None
        pts = pts3d[m]                                  # N x 3 (mm)
        pts = pts[(pts[:, 2] > 0) & np.isfinite(pts[:, 2])]
        if len(pts) < 10:
            return None
        return np.median(pts, axis=0)                   # (x, y, z) mm

    def _sample_depth(self, pts3d, valid, u, v, win=4):
        h, w = valid.shape
        y0, y1 = max(0, v - win), min(h, v + win + 1)
        x0, x1 = max(0, u - win), min(w, u + win + 1)
        m = valid[y0:y1, x0:x1]
        if m.sum() < 5:
            return None
        z = pts3d[y0:y1, x0:x1, 2][m]
        return float(np.median(z))  # mm

    def left_cb(self, msg):
        self._nl += 1
        self._left = (stamp_key(msg), img_to_np(msg), msg.header)
        self._try_process()

    def _try_process(self):
        # Pair same-frame L/R. The driver stamps both halves identically, but R(N)
        # arrives just after L(N), so we buffer both and process on a stamp match.
        if self._left is None or self._right is None:
            return
        self._lastdiff = abs(self._left[0] - self._right[0])
        if self._lastdiff > 5_000_000:  # 5ms
            return
        self._nmatch += 1
        left, header = self._left[1], self._left[2]
        right = self._right[1]
        self._left = self._right = None
        self._process(left, right, header)

    def _process(self, left, right, header):
        sw, sh = self.cal["size"]
        l = cv2.remap(cv2.resize(left, (sw, sh)), *self.cal["mapL"], cv2.INTER_LINEAR)
        r = cv2.remap(cv2.resize(right, (sw, sh)), *self.cal["mapR"], cv2.INTER_LINEAR)
        gl = cv2.cvtColor(l, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
        disp = self.matcher.compute(gl, gr).astype(np.float32) / 16.0
        valid = disp > 0.5
        pts3d = cv2.reprojectImageTo3D(disp, self.cal["Q"])  # mm
        dist_cm = np.where(valid, pts3d[:, :, 2] / 10.0, 0.0)
        valid &= np.isfinite(dist_cm) & (dist_cm > 0)

        target, mask = self._detect(l, disp, valid, dist_cm)
        state, dist_m = "NONE", -1.0
        if target is not None:
            u, v = target
            z_mm = self._sample_depth(pts3d, valid, u, v)
            xyz_mm = None
            if z_mm is None and mask is not None:
                # Uniform/glossy blob -> no disparity at its centre. Fall back to
                # the median 3D point over valid pixels in the WHOLE colour blob
                # (its textured edges carry depth). Fixes BEARING_ONLY on solid
                # objects.
                xyz_mm = self._sample_xyz_mask(pts3d, valid, mask)
                if xyz_mm is not None:
                    z_mm = float(xyz_mm[2])
            if z_mm is None:
                state = "BEARING_ONLY"
            else:
                d_cm = z_mm / 10.0
                dist_m = z_mm / 1000.0
                if d_cm > self.far_cm:
                    state = "FAR"            # detected but beyond the gate
                else:
                    state = "TRACK"
                    ps = PointStamped()
                    ps.header = header
                    ps.header.frame_id = self.frame_id
                    if xyz_mm is not None:
                        ps.point.x = float(xyz_mm[0] / 1000.0)
                        ps.point.y = float(xyz_mm[1] / 1000.0)
                    else:
                        ps.point.x = float(pts3d[v, u, 0] / 1000.0)
                        ps.point.y = float(pts3d[v, u, 1] / 1000.0)
                    ps.point.z = dist_m
                    self.pub_pt.publish(ps)

        self.pub_d.publish(Float32(data=float(dist_m)))
        self.pub_s.publish(String(data=state))
        valid_pct = 100.0 * np.count_nonzero(valid) / valid.size
        self.get_logger().info(
            f"target: {state:12s} dist={dist_m*100:6.1f}cm  valid_depth={valid_pct:4.0f}%",
            throttle_duration_sec=1.0)
        self._publish_debug(header, l, target, state, dist_m)

    def _publish_debug(self, header, l, target, state, dist_m):
        now = time.time()
        dt = now - self._t
        self._t = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 / dt if self._fps else 1.0 / dt
        dbg = l.copy()
        color = {"TRACK": (0, 220, 0), "FAR": (0, 165, 255),
                 "BEARING_ONLY": (0, 165, 255), "NONE": (0, 0, 255)}[state]
        if target is not None:
            cv2.drawMarker(dbg, target, color, cv2.MARKER_CROSS, 28, 2)
            cv2.circle(dbg, target, 8, color, 2)
        txt = f"{state}  {dist_m*100:5.1f}cm" if dist_m > 0 else f"{state}  --"
        cv2.putText(dbg, f"{txt}  {self._fps:4.1f}fps", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        out = Image()
        out.header = header
        out.header.frame_id = self.frame_id
        out.height, out.width = dbg.shape[0], dbg.shape[1]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = dbg.shape[1] * 3
        out.data = np.ascontiguousarray(dbg).tobytes()
        self.pub_dbg.publish(out)


def main():
    rclpy.init()
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
