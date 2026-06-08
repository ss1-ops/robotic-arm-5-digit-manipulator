#!/usr/bin/env python3
"""
stereo_calibrate.py — compute stereo calibration from captured checkerboard pairs.

Pipeline:
  1. Detect + sub-pixel refine checkerboard corners in every left/right pair.
  2. Calibrate each lens individually (intrinsics K, distortion D).
  3. cv2.stereoCalibrate (FIX_INTRINSIC) -> relative rotation R, translation T.
  4. cv2.stereoRectify -> rectification R1/R2, projection P1/P2, reprojection Q.
  5. Save everything to an OpenCV FileStorage YAML that the Pi runtime loads
     directly with cv2.FileStorage (portable across machines; the calibration is
     a property of the camera, not the host).

Usage:
    python3 stereo_calibrate.py --captures captures \
        --inner-cols 9 --inner-rows 6 --square 25.0 [--show]
"""

import argparse
import glob
import os

import cv2
import numpy as np

SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
DETECT_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE


def detect(gray, pattern):
    ok, corners = cv2.findChessboardCorners(gray, pattern, DETECT_FLAGS)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRIT)
    return ok, corners


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", default="captures", help="Dir with left/ and right/ pairs")
    ap.add_argument("--inner-cols", type=int, default=8)
    ap.add_argument("--inner-rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=23.25, help="Square size in mm")
    ap.add_argument("--out", default="stereo_calib.yaml")
    ap.add_argument("--rational", action="store_true",
                    help="Use the 8-term rational distortion model (better for wide/fisheye lenses)")
    ap.add_argument("--show", action="store_true", help="Preview rectified pair with epipolar lines")
    args = ap.parse_args()

    intr_flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0

    pattern = (args.inner_cols, args.inner_rows)

    # Object points: a planar grid in board coordinates, scaled to mm.
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= args.square

    left_files = sorted(glob.glob(os.path.join(args.captures, "left", "*.png")))
    if not left_files:
        raise SystemExit(f"No images in {args.captures}/left/. Run capture_stereo.py first.")

    objpoints, imgpL, imgpR = [], [], []
    image_size = None
    used, skipped = 0, 0
    for lf in left_files:
        rf = lf.replace(os.sep + "left" + os.sep, os.sep + "right" + os.sep)
        if not os.path.exists(rf):
            skipped += 1
            continue
        il, ir = cv2.imread(lf), cv2.imread(rf)
        gl = cv2.cvtColor(il, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(ir, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gl.shape[1], gl.shape[0])  # (w, h)
        okl, cl = detect(gl, pattern)
        okr, cr = detect(gr, pattern)
        if okl and okr:
            objpoints.append(objp)
            imgpL.append(cl)
            imgpR.append(cr)
            used += 1
        else:
            skipped += 1
            print(f"  skip {os.path.basename(lf)} (L={'ok' if okl else 'no'} R={'ok' if okr else 'no'})")

    print(f"\n[calib] usable pairs: {used}  (skipped {skipped})")
    if used < 8:
        raise SystemExit("Need at least ~8 good pairs (20-30 recommended). Capture more.")

    # 1) Per-lens intrinsics
    model = "rational (8-term)" if args.rational else "standard (5-term)"
    print(f"[calib] distortion model: {model}")
    rmsL, K1, D1, _, _ = cv2.calibrateCamera(objpoints, imgpL, image_size, None, None, flags=intr_flags)
    rmsR, K2, D2, _, _ = cv2.calibrateCamera(objpoints, imgpR, image_size, None, None, flags=intr_flags)
    print(f"[calib] per-lens reprojection RMS  left={rmsL:.3f}px  right={rmsR:.3f}px")

    # 2) Stereo extrinsics with intrinsics fixed
    stereo_rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpL, imgpR, K1, D1, K2, D2, image_size,
        flags=cv2.CALIB_FIX_INTRINSIC | intr_flags, criteria=SUBPIX_CRIT)
    baseline_mm = float(np.linalg.norm(T))
    print(f"[calib] stereo reprojection RMS = {stereo_rms:.3f}px   baseline = {baseline_mm:.2f} mm")

    # 3) Rectification
    R1, R2, P1, P2, Q, roiL, roiR = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    # Rectification quality: after rectification, corresponding points must share
    # a row, so the vertical (epipolar) error of the corners is the metric that
    # actually governs disparity/depth accuracy. Lower is better; <0.5px is good.
    ep = []
    for cl, cr in zip(imgpL, imgpR):
        ul = cv2.undistortPoints(cl, K1, D1, R=R1, P=P1).reshape(-1, 2)
        ur = cv2.undistortPoints(cr, K2, D2, R=R2, P=P2).reshape(-1, 2)
        ep.append(np.abs(ul[:, 1] - ur[:, 1]))
    ep = np.concatenate(ep)
    epipolar_mean = float(ep.mean())
    print(f"[calib] rectified epipolar y-error: mean={epipolar_mean:.3f}px  "
          f"median={np.median(ep):.3f}px  95th={np.percentile(ep, 95):.3f}px  (<0.5 good)")

    # 4) Save (OpenCV FileStorage YAML — loadable on the Pi)
    fs = cv2.FileStorage(args.out, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("square_size_mm", args.square)
    fs.write("board_inner_cols", pattern[0])
    fs.write("board_inner_rows", pattern[1])
    fs.write("baseline_mm", baseline_mm)
    fs.write("rms_left", rmsL)
    fs.write("rms_right", rmsR)
    fs.write("rms_stereo", stereo_rms)
    fs.write("epipolar_mean_px", epipolar_mean)
    fs.write("K1", K1); fs.write("D1", D1)
    fs.write("K2", K2); fs.write("D2", D2)
    fs.write("R", R); fs.write("T", T)
    fs.write("E", E); fs.write("F", F)
    fs.write("R1", R1); fs.write("R2", R2)
    fs.write("P1", P1); fs.write("P2", P2)
    fs.write("Q", Q)
    fs.release()
    print(f"[calib] wrote {args.out}")
    if stereo_rms > 0.6:
        print("[WARN] stereo RMS > 0.6px — consider recapturing with more varied, "
              "sharper, well-lit views covering the frame corners.")

    if args.show:
        _preview_rectified(left_files, K1, D1, K2, D2, R1, R2, P1, P2, image_size)


def _preview_rectified(left_files, K1, D1, K2, D2, R1, R2, P1, P2, image_size):
    """Remap one pair and draw horizontal lines; rows should align across L/R."""
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)
    lf = left_files[len(left_files) // 2]
    rf = lf.replace(os.sep + "left" + os.sep, os.sep + "right" + os.sep)
    il, ir = cv2.imread(lf), cv2.imread(rf)
    rl = cv2.remap(il, mapLx, mapLy, cv2.INTER_LINEAR)
    rr = cv2.remap(ir, mapRx, mapRy, cv2.INTER_LINEAR)
    combined = cv2.hconcat([rl, rr])
    for y in range(0, combined.shape[0], 40):
        cv2.line(combined, (0, y), (combined.shape[1], y), (0, 255, 0), 1)
    if combined.shape[1] > 1600:
        s = 1600 / combined.shape[1]
        combined = cv2.resize(combined, None, fx=s, fy=s)
    cv2.imshow("rectified (rows should align) - any key to close", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
