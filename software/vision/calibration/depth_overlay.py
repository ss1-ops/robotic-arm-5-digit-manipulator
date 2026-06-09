#!/usr/bin/env python3
"""
depth_overlay.py — live translucent distance heatmap over the stereo stream.

Computes a disparity map (StereoSGBM) from the two lenses and overlays a
colour-coded distance heatmap on the left image: warm = near, cool = far.
Pixels with no valid match, or beyond the "far" cutoff, are left un-tinted —
which is exactly the distance-gating idea (anything past the gate is ignored).

Two modes:
  * METRIC  — if stereo_calib.yaml is found, depth is real (centimetres). The
    rectification + Q are recomputed at the processing scale so it stays correct.
  * RELATIVE — no calibration yet: shows normalised disparity (near/far is
    correct, absolute distance is NOT). Clearly labelled on-screen.

Live trackbars: blend alpha, near/far range, and the two key SGBM knobs.
Center crosshair prints the measured distance there.

Keys:  q/ESC quit   s save snapshot

Usage:
    python3 depth_overlay.py                       # auto-load stereo_calib.yaml
    python3 depth_overlay.py --calib stereo_calib.yaml --scale 0.5
    python3 depth_overlay.py --no-calib            # force relative mode
"""

import argparse
import os
import time

import cv2
import numpy as np

from stereo_io import open_stereo_camera, split_lr, describe_capture, fit_to_screen

WIN = "depth overlay (q=quit  s=snap)"


def load_calib(path, scale):
    """Load intrinsics/extrinsics and recompute rectification + Q at `scale`."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    K1 = fs.getNode("K1").mat(); D1 = fs.getNode("D1").mat()
    K2 = fs.getNode("K2").mat(); D2 = fs.getNode("D2").mat()
    R = fs.getNode("R").mat();   T = fs.getNode("T").mat()
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()

    sw, sh = int(round(w * scale)), int(round(h * scale))
    S = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]], dtype=np.float64)
    K1s, K2s = S @ K1, S @ K2  # scale fx,fy,cx,cy to the processing resolution
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1s, D1, K2s, D2, (sw, sh), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, (sw, sh), cv2.CV_16SC2)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, (sw, sh), cv2.CV_16SC2)
    return {"Q": Q, "mapL": (mapLx, mapLy), "mapR": (mapRx, mapRy), "size": (sw, sh)}


def make_matcher(num_disp, block):
    num_disp = max(16, (num_disp // 16) * 16)
    block = block if block % 2 == 1 else block + 1
    block = max(3, block)
    return cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=num_disp, blockSize=block,
        P1=8 * 3 * block ** 2, P2=32 * 3 * block ** 2,
        disp12MaxDiff=1, uniquenessRatio=10,
        speckleWindowSize=100, speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--width", type=int, default=2560)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--calib", default="stereo_calib.yaml", help="Stereo calibration YAML")
    ap.add_argument("--no-calib", action="store_true", help="Force relative (non-metric) mode")
    ap.add_argument("--scale", type=float, default=0.5, help="Processing downscale for speed (0.25-1.0)")
    ap.add_argument("--num-disp", type=int, default=128, help="numDisparities (multiple of 16)")
    ap.add_argument("--block", type=int, default=5, help="SGBM block size (odd)")
    args = ap.parse_args()

    calib = None
    if not args.no_calib and os.path.exists(args.calib):
        try:
            calib = load_calib(args.calib, args.scale)
            print(f"[depth] METRIC mode — loaded {args.calib} (scale {args.scale})")
        except Exception as e:
            print(f"[depth] could not load {args.calib} ({e}); falling back to RELATIVE mode")
    if calib is None:
        print("[depth] RELATIVE mode — no calibration; distances are NOT metric.")

    cap = open_stereo_camera(args.index, args.width, args.height)
    cw, ch, fps, fourcc = describe_capture(cap)
    print(f"[depth] camera {args.index}: {cw}x{ch} @ {fps:.0f}fps {fourcc}")

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("alpha %", WIN, 55, 100, lambda v: None)
    cv2.createTrackbar("near cm", WIN, 15, 300, lambda v: None)
    cv2.createTrackbar("far cm", WIN, 120, 400, lambda v: None)
    cv2.createTrackbar("numDisp/16", WIN, max(1, args.num_disp // 16), 16, lambda v: None)
    cv2.createTrackbar("block", WIN, args.block, 21, lambda v: None)

    matcher = make_matcher(args.num_disp, args.block)
    cur_nd, cur_bs = args.num_disp, args.block
    t_prev, fps_disp = time.time(), 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        left, right = split_lr(frame)
        if args.scale != 1.0:
            left = cv2.resize(left, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)

        if calib is not None:
            left = cv2.remap(left, *calib["mapL"], cv2.INTER_LINEAR)
            right = cv2.remap(right, *calib["mapR"], cv2.INTER_LINEAR)

        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        # Rebuild matcher only when the trackbars change.
        nd = max(1, cv2.getTrackbarPos("numDisp/16", WIN)) * 16
        bs = max(3, cv2.getTrackbarPos("block", WIN))
        if nd != cur_nd or bs != cur_bs:
            matcher = make_matcher(nd, bs)
            cur_nd, cur_bs = nd, bs

        disp = matcher.compute(gl, gr).astype(np.float32) / 16.0
        valid = disp > (matcher.getMinDisparity() + 0.5)

        near_cm = max(1, cv2.getTrackbarPos("near cm", WIN))
        far_cm = max(near_cm + 1, cv2.getTrackbarPos("far cm", WIN))
        alpha = cv2.getTrackbarPos("alpha %", WIN) / 100.0

        if calib is not None:
            pts = cv2.reprojectImageTo3D(disp, calib["Q"])
            dist_cm = pts[:, :, 2] / 10.0  # Q is in mm (square size was mm) -> cm
            valid &= np.isfinite(dist_cm) & (dist_cm > 0)
            metric = True
        else:
            # Relative: map disparity to a pseudo-distance (more disp = nearer).
            dmax = disp[valid].max() if valid.any() else 1.0
            dist_cm = np.where(valid, near_cm + (far_cm - near_cm) * (1.0 - disp / max(dmax, 1e-6)), 0)
            metric = False

        # Normalise to the [near, far] window; outside the window = ignored (gated).
        norm = np.clip((dist_cm - near_cm) / (far_cm - near_cm), 0, 1)
        gated = valid & (dist_cm >= near_cm) & (dist_cm <= far_cm)
        heat = cv2.applyColorMap(((1.0 - norm) * 255).astype(np.uint8), cv2.COLORMAP_JET)

        out = left.copy()
        m = gated[..., None]
        out = np.where(m, (out * (1 - alpha) + heat * alpha).astype(np.uint8), out)

        # Center crosshair + distance readout.
        h, w = out.shape[:2]
        cy, cx = h // 2, w // 2
        cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cd = dist_cm[cy, cx]
        center_txt = (f"center: {cd:5.1f} cm" if (metric and valid[cy, cx])
                      else ("center: --" if metric else "center: (relative)"))

        now = time.time(); dt = now - t_prev; t_prev = now
        if dt > 0:
            fps_disp = 0.9 * fps_disp + 0.1 / dt if fps_disp else 1.0 / dt
        mode = "METRIC cm" if metric else "RELATIVE (not metric)"
        cv2.putText(out, f"{mode}   {center_txt}   {fps_disp:4.1f} fps",
                    (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(out, f"range {near_cm}-{far_cm} cm   warm=near  cool=far",
                    (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.imshow(WIN, fit_to_screen(out))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            os.makedirs("snapshots", exist_ok=True)
            fn = os.path.join("snapshots", f"depth_{int(now)}.png")
            cv2.imwrite(fn, out)
            print(f"[depth] saved {fn}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
