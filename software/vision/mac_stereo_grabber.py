#!/usr/bin/env python3
"""
mac_stereo_grabber.py — fetch latest stereo pairs from the Pi over HTTP
(for offloading all heavy vision processing to your compute / Mac).

Pi side (lightweight):
  python3 mjpeg_stream.py --ros-args -p port:=8080 -p proc_quality:=85

Usage from Mac:
  from vision.mac_stereo_grabber import StereoGrabber
  grabber = StereoGrabber("192.168.1.142", 8080)
  left, right = grabber.get_pair()   # BGR numpy arrays (rectified if Pi is doing it)
  # then do your SGBM, detection, etc. locally at full speed

This keeps the Pi doing almost nothing but camera capture + JPEG encoding.
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import requests


class StereoGrabber:
    def __init__(self, host: str, port: int = 8080, timeout: float = 1.5):
        self.host = host
        self.port = port
        self.base = f"http://{host}:{port}"
        self.timeout = timeout
        self.session = requests.Session()

    def get_pair(self, timeout: Optional[float] = None) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Return (left_bgr, right_bgr) or None on failure."""
        t = timeout or self.timeout
        try:
            l_resp = self.session.get(f"{self.base}/left", timeout=t)
            r_resp = self.session.get(f"{self.base}/right", timeout=t)
            if l_resp.status_code != 200 or r_resp.status_code != 200:
                return None
            l_buf = np.frombuffer(l_resp.content, np.uint8)
            r_buf = np.frombuffer(r_resp.content, np.uint8)
            left = cv2.imdecode(l_buf, cv2.IMREAD_COLOR)
            right = cv2.imdecode(r_buf, cv2.IMREAD_COLOR)
            if left is None or right is None:
                return None
            return left, right
        except Exception:
            return None

    def get_latest_jpeg_bytes(self, which: str = "left") -> Optional[bytes]:
        """Raw JPEG bytes for /left or /right (useful for minimal latency)."""
        try:
            resp = self.session.get(f"{self.base}/{which}", timeout=self.timeout)
            if resp.status_code == 200:
                return resp.content
        except Exception:
            pass
        return None


if __name__ == "__main__":
    # Quick test
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.142"
    g = StereoGrabber(host)
    print(f"Grabbing from {host}...")
    pair = g.get_pair()
    if pair:
        l, r = pair
        print(f"Got pair: left {l.shape} right {r.shape}")
        cv2.imwrite("/tmp/test_left.jpg", l)
        cv2.imwrite("/tmp/test_right.jpg", r)
        print("Wrote /tmp/test_left.jpg /tmp/test_right.jpg")
    else:
        print("Failed to grab pair (is the stream running on Pi?)")
