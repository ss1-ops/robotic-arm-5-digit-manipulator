#!/usr/bin/env python3
"""Check whether the existing stereo calibration still rectifies correctly after
the camera remount. Grabs one synced L/R pair from the running camera node,
rectifies with stereo_calib.yaml, ORB-matches features, and reports the median
vertical (epipolar) disparity of matches. ~<2 px = calibration still valid;
large = recalibration needed. Board-free (uses scene features).

Run:  python3 verify_rectify.py
"""
import os, time
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def img_to_np(m):
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)


class V(Node):
    def __init__(self):
        super().__init__("verify_rectify")
        # stereo_calib.yaml is OpenCV FileStorage format (has %YAML directive +
        # !!opencv-matrix tags) -> read with cv2.FileStorage, not PyYAML.
        fs = cv2.FileStorage(os.path.expanduser("~/vision/stereo_calib.yaml"),
                             cv2.FILE_STORAGE_READ)
        def m(k):
            nd = fs.getNode(k)
            return None if nd.empty() else nd.mat()
        self.K1 = m("K1"); self.D1 = m("D1").ravel()
        self.K2 = m("K2"); self.D2 = m("D2").ravel()
        self.R1 = m("R1"); self.P1 = m("P1")
        self.R2 = m("R2"); self.P2 = m("P2")
        fs.release()
        self.L = self.Rr = None
        self.maps = None
        self.done = False
        self.create_subscription(Image, "/stereo/left/image_raw", self.lcb, qos_profile_sensor_data)
        self.create_subscription(Image, "/stereo/right/image_raw", self.rcb, qos_profile_sensor_data)

    def _mk(self, w, h):
        m1 = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R1, self.P1, (w, h), cv2.CV_16SC2)
        m2 = cv2.initUndistortRectifyMap(self.K2, self.D2, self.R2, self.P2, (w, h), cv2.CV_16SC2)
        return m1, m2

    def lcb(self, m): self.L = img_to_np(m); self.go()
    def rcb(self, m): self.Rr = img_to_np(m); self.go()

    def go(self):
        if self.done or self.L is None or self.Rr is None:
            return
        self.done = True
        h, w = self.L.shape[:2]
        if self.maps is None:
            self.maps = self._mk(w, h)
        lr = cv2.remap(self.L, *self.maps[0], cv2.INTER_LINEAR)
        rr = cv2.remap(self.Rr, *self.maps[1], cv2.INTER_LINEAR)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY); gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(1500)
        k1, d1 = orb.detectAndCompute(gl, None); k2, d2 = orb.detectAndCompute(gr, None)
        if d1 is None or d2 is None:
            print("VERIFY: no features"); rclpy.shutdown(); return
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(d1, d2)
        dys = []
        for mm in matches:
            (x1, y1) = k1[mm.queryIdx].pt; (x2, y2) = k2[mm.trainIdx].pt
            if abs(x1 - x2) < 200:   # plausible disparity range, reject outliers
                dys.append(abs(y1 - y2))
        dys = np.array(dys)
        if len(dys) < 20:
            print(f"VERIFY: too few matches ({len(dys)})"); rclpy.shutdown(); return
        print(f"VERIFY: matches={len(dys)}  median|dy|={np.median(dys):.2f}px  "
              f"mean|dy|={dys.mean():.2f}px  p90={np.percentile(dys,90):.2f}px")
        print("  -> <~2px median = OLD CALIBRATION STILL VALID (no recal needed)")
        print("  -> >~5px       = recalibration needed")
        rclpy.shutdown()


def main():
    rclpy.init(); n = V()
    t = time.time()
    while rclpy.ok() and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.5)
    n.destroy_node()


if __name__ == "__main__":
    main()
