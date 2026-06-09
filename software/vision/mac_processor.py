#!/usr/bin/env python3
"""
mac_processor.py — example of running heavy stereo vision (SGBM + detection)
completely on your compute (Mac) while the Pi only streams frames.

This offloads the biggest CPU hogs (rectify + SGBM + multi-frame depth) from the Pi,
letting it run the camera at higher effective rates and have headroom for motor control.

Dependencies (on Mac):
    pip install opencv-python numpy requests

Usage:
    python3 mac_processor.py --host 192.168.1.142 --calib software/vision/calibration/stereo_calib.yaml

It will grab pairs, run the same SGBM pipeline the Pi used to run, print depths, etc.
You can then wire the results into IK (using the local kinematics.py) and send
commands over the existing TCP port 9000 to the Pi's moveo_publisher.

For full closed-loop goto/approach you would port the state machine from
goto_object.py here and use a TCP client (see moveo_simple_controller.py for the
socket protocol).

The Pi side only needs:
    python3 mjpeg_stream.py --ros-args -p calib:=... -p proc_quality:=85
(plus the stereo_camera_node and moveo_publisher as usual)
"""

import argparse
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from mac_stereo_grabber import StereoGrabber

# --- Calib loading (same logic as the Pi nodes) ---
def load_calib(path: str):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calib: {path}")
    g = lambda n: fs.getNode(n).mat()
    K1, D1, K2, D2 = g("K1"), g("D1"), g("K2"), g("D2")
    R, T = g("R"), g("T")
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()

    # Match the scale used on Pi for SGBM (0.5 in goto_object / stereo_depth)
    scale = 0.5
    sw, sh = int(round(w * scale)), int(round(h * scale))
    S = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]], np.float64)
    K1s, K2s = S @ K1, S @ K2

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1s, D1, K2s, D2, (sw, sh), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    mapL = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, (sw, sh), cv2.CV_16SC2)
    mapR = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, (sw, sh), cv2.CV_16SC2)

    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=160,   # same as current Pi setting for close range
        blockSize=7,
        P1=8*3*49, P2=32*3*49,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    # Full-res K for normalized rays
    fx, fy = float(K1[0, 0]), float(K1[1, 1])
    cx, cy = float(K1[0, 2]), float(K1[1, 2])

    return {
        "mapL": mapL, "mapR": mapR, "Q": Q,
        "sgbm": sgbm, "sw": sw, "sh": sh,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "orig_w": w, "orig_h": h,
    }


def measure_depth_at_centroid(cal: dict, left_bgr: np.ndarray, right_bgr: np.ndarray,
                              cxy: Tuple[int, int], n_samples: int = 1) -> Optional[float]:
    """Replicates the core of _measure_depth from the Pi goto_object.
    Returns depth in metres or None.
    """
    sw, sh = cal["sw"], cal["sh"]
    li = cv2.resize(left_bgr, (sw, sh))
    ri = cv2.resize(right_bgr, (sw, sh))

    lr = cv2.remap(li, *cal["mapL"], cv2.INTER_LINEAR)
    rr = cv2.remap(ri, *cal["mapR"], cv2.INTER_LINEAR)

    gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)

    disp = cal["sgbm"].compute(gl, gr).astype(np.float32) / 16.0
    pts3d = cv2.reprojectImageTo3D(disp, cal["Q"])

    # Scale click coords
    scale_x = sw / left_bgr.shape[1]
    scale_y = sh / left_bgr.shape[0]
    u = int(np.clip(cxy[0] * scale_x, 0, sw - 1))
    v = int(np.clip(cxy[1] * scale_y, 0, sh - 1))

    win = 8
    y0, y1 = max(0, v - win), min(sh, v + win + 1)
    x0, x1 = max(0, u - win), min(sw, u + win + 1)
    patch = pts3d[y0:y1, x0:x1, 2]
    valid = patch[(patch > 0) & np.isfinite(patch)]

    if len(valid) < 3:
        # fallback to broader median
        all_valid = pts3d[:, :, 2]
        all_valid = all_valid[(all_valid > 0) & np.isfinite(all_valid)]
        if len(all_valid) < 5:
            return None
        z_mm = float(np.median(all_valid))
    else:
        z_mm = float(np.median(valid))

    return z_mm / 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.142")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--calib", default="software/vision/calibration/stereo_calib.yaml")
    ap.add_argument("--n", type=int, default=5, help="median samples per measurement")
    args = ap.parse_args()

    print("Loading calib...")
    cal = load_calib(args.calib)
    print(f"Calib loaded. Processing at {cal['sw']}x{cal['sh']}")

    grabber = StereoGrabber(args.host, args.port)
    print(f"Grabbing from {args.host}:{args.port} ... (Ctrl-C to stop)")

    try:
        while True:
            pair = grabber.get_pair()
            if pair is None:
                time.sleep(0.1)
                continue
            left, right = pair

            # Simple center depth as demo (you would run blob detection here too)
            h, w = left.shape[:2]
            cxy = (w // 2, h // 2)
            z = measure_depth_at_centroid(cal, left, right, cxy, n_samples=args.n)
            if z is not None:
                print(f"Center depth ≈ {z*100:.1f} cm   (press Ctrl-C to quit)")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
