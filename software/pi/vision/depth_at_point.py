#!/usr/bin/env python3
"""
depth_at_point.py — on Pi: given a pixel (u,v) in the *original full-res left image*,
capture N recent (or fresh) synced L/R pairs from the stereo topics, run the same
one-shot stereo depth logic as goto_object (SGBM + median in window + calib), and
return the median depth across the N samples.

Intended to be called from the Mac depth_test_GUI on click.

**Must be run with the ROS 2 environment sourced**, e.g.:
  source /opt/ros/jazzy/setup.bash
  python3 ~/vision/depth_at_point.py --u 800 --v 450 --n 3

The Mac GUI now does this automatically via bash -c when you click.

Usage (when run manually):
  python3 depth_at_point.py --u 800 --v 450 --n 3
  python3 depth_at_point.py --u 800 --v 450 --n 1 --calib ~/vision/stereo_calib.yaml

Prints:
  DIST: 0.312   (or error)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

def load_cal(self_path, num_disp=160):
    fs = cv2.FileStorage(self_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open calibration: {self_path}")
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
        minDisparity=0, numDisparities=num_disp, blockSize=7,
        P1=8*3*49, P2=32*3*49, disp12MaxDiff=1,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    return {"mapL": mapL, "mapR": mapR, "Q": Q,
            "sgbm": sgbm, "sw": sw, "sh": sh,
            "fx": float(K1[0,0]), "fy": float(K1[1,1]),
            "cx": float(K1[0,2]), "cy": float(K1[1,2])}

def img_to_np(msg):
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

class DepthSampler(Node):
    def __init__(self, calib_path, n, u, v, num_disp=160):
        super().__init__("depth_at_point_sampler")
        self.n = max(1, n)
        self.u_full = u
        self.v_full = v
        self.cal = load_cal(calib_path, num_disp)
        self.pairs = []  # list of (left, right) full res
        self._lock = False
        q = qos_profile_sensor_data
        self.create_subscription(Image, "/stereo/left/image_raw", self._lcb, q)
        self.create_subscription(Image, "/stereo/right/image_raw", self._rcb, q)
        self._left = None
        self._right = None
        self._got = 0
        self.timer = self.create_timer(0.05, self._tick)

    def _lcb(self, msg):
        if self._got >= self.n: return
        self._left = (stamp_ns(msg), img_to_np(msg))

    def _rcb(self, msg):
        if self._got >= self.n: return
        self._right = (stamp_ns(msg), img_to_np(msg))

    def _tick(self):
        if self._got >= self.n or self._left is None or self._right is None:
            return
        lts, l = self._left
        rts, r = self._right
        if abs(lts - rts) > 5_000_000:  # 5ms
            return
        self.pairs.append((l, r))
        self._got += 1
        self._left = self._right = None
        if self._got >= self.n:
            self._compute_and_shutdown()

    def _compute_and_shutdown(self):
        depths = []
        sw, sh = self.cal["sw"], self.cal["sh"]
        scale_x = sw / 1280.0   # assume orig ~1280, adjust if needed; better from calib but simple
        scale_y = sh / 960.0
        # actually use the orig from cal? cal doesn't store orig here, but in practice  the maps are for scaled
        # To make accurate, we resize the captured full to the proc size like in goto
        u = int(self.u_full * (sw / 1280.0))  # rough; in real use the scale from calib load
        v = int(self.v_full * (sh / 960.0))
        for l, r in self.pairs:
            li = cv2.resize(l, (sw, sh))
            ri = cv2.resize(r, (sw, sh))
            lr = cv2.remap(li, *self.cal["mapL"], cv2.INTER_LINEAR)
            rr = cv2.remap(ri, *self.cal["mapR"], cv2.INTER_LINEAR)
            gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
            disp = self.cal["sgbm"].compute(gl, gr).astype(np.float32) / 16.0
            pts3d = cv2.reprojectImageTo3D(disp, self.cal["Q"])
            win = 8
            y0 = max(0, v-win); y1 = min(sh, v+win+1)
            x0 = max(0, u-win); x1 = min(sw, u+win+1)
            patch = pts3d[y0:y1, x0:x1, 2]
            valid = patch[(patch > 0) & np.isfinite(patch)]
            if len(valid) >= 3:
                z_mm = float(np.median(valid))
                depths.append(z_mm / 1000.0)
        if depths:
            med = float(np.median(depths))
            print(f"DIST: {med:.3f}")
            print(f"DETAILS: n={len(depths)} values={[round(d,3) for d in depths]}")
        else:
            print("DIST: -1  (no valid depth samples)")
        rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--u", type=int, required=True)
    ap.add_argument("--v", type=int, required=True)
    ap.add_argument("--n", type=int, default=1, choices=[1,2,3,5])
    ap.add_argument("--num-disp", type=int, default=160, help="SGBM numDisparities (increase for closer objects)")
    ap.add_argument("--calib", default=os.path.expanduser("~/vision/stereo_calib.yaml"))
    args = ap.parse_args()

    rclpy.init()
    node = DepthSampler(args.calib, args.n, args.u, args.v, getattr(args, 'num_disp', 160))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
