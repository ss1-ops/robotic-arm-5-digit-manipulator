#!/usr/bin/env python3
"""
capture_stereo.py — capture checkerboard stereo pairs (macOS).

Opens the ELP side-by-side stream, splits it L/R, and live-detects the
checkerboard in BOTH halves. Aim for 20-30 pairs covering a wide range of
angles, distances, and positions across the frame (including the corners).

AUTO mode (default): the keyboard often doesn't reach OpenCV windows on macOS,
so capture is automatic — when the board is visible in both lenses, held STILL,
and in a pose meaningfully DIFFERENT from the last saved one, a pair is saved.
Just sweep the board slowly around the workspace. Stop by closing the window or
pressing Ctrl-C in the terminal.

MANUAL mode (--manual): press SPACE to save, q to quit (only if your build's
keyboard handling works).

The board geometry is INNER corners (squares - 1) and square size in mm:
a standard 10x7 squares board has 9x6 inner corners.

Usage:
    python3 capture_stereo.py --inner-cols 8 --inner-rows 6 --square 23.25
    python3 capture_stereo.py --manual           # keypress mode
"""

import argparse
import os
import time

import cv2
import numpy as np

from stereo_io import open_stereo_camera, split_lr, describe_capture, fit_to_screen

LIVE_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH
              | cv2.CALIB_CB_NORMALIZE_IMAGE
              | cv2.CALIB_CB_FAST_CHECK)

# Live detection runs on a downscaled image so the UI stays responsive (full-res
# findChessboardCorners is slow exactly when a board is present). Full-res
# sub-pixel detection happens later in stereo_calibrate.py on the saved PNGs.
DETECT_WIDTH = 640

STABLE_PX = 6.0        # max board-centroid motion (px) between frames to count as "still"
COOLDOWN_S = 0.5       # min time between auto-saves
SPREAD_NOVELTY = 0.12  # fractional change in board scale that also counts as a new pose
WIN = "stereo capture  (auto; close window or Ctrl-C to stop)"


def find_board(gray, pattern):
    w = gray.shape[1]
    ds = DETECT_WIDTH / w if w > DETECT_WIDTH else 1.0
    small = cv2.resize(gray, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA) if ds != 1.0 else gray
    ok, corners = cv2.findChessboardCorners(small, pattern, LIVE_FLAGS)
    if ok and ds != 1.0:
        corners = corners / ds  # back to full-res coords for drawing
    return ok, corners


def board_feat(corners):
    """Order-invariant board features: (centroid xy, mean radial spread).

    Uses centroid + scale instead of per-corner correspondence, so the 180-degree
    ordering flips findChessboardCorners can produce between frames (common when
    both pattern dimensions are even) don't register as huge phantom motion.
    """
    pts = corners.reshape(-1, 2)
    centroid = pts.mean(axis=0)
    spread = float(np.linalg.norm(pts - centroid, axis=1).mean())
    return centroid, spread


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0, help="Camera index (ELP=0)")
    ap.add_argument("--width", type=int, default=2560, help="Full combined frame width")
    ap.add_argument("--height", type=int, default=960, help="Frame height")
    ap.add_argument("--inner-cols", type=int, default=8, help="Inner corners per row (squares-1)")
    ap.add_argument("--inner-rows", type=int, default=6, help="Inner corners per column (squares-1)")
    ap.add_argument("--square", type=float, default=23.25, help="Square size in mm (recorded for reference)")
    ap.add_argument("--out", default="captures", help="Output directory for pairs")
    ap.add_argument("--count", type=int, default=30, help="Auto-mode target pair count (then stops)")
    ap.add_argument("--min-move", type=float, default=0.0,
                    help="Min board-centroid shift (px) vs last save to count as a new pose "
                         "(default: 5%% of half-frame width)")
    ap.add_argument("--manual", action="store_true", help="Use SPACE/q keypresses instead of auto-capture")
    args = ap.parse_args()

    pattern = (args.inner_cols, args.inner_rows)
    os.makedirs(os.path.join(args.out, "left"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "right"), exist_ok=True)

    cap = open_stereo_camera(args.index, args.width, args.height)
    w, h, fps, fourcc = describe_capture(cap)
    print(f"[capture] camera index {args.index}: {w}x{h} @ {fps:.0f}fps {fourcc}")
    if w < args.width:
        print(f"[WARN] got width {w}, requested {args.width}. The device may have")
        print("       fallen back to a single-lens mode. Check find_camera.py output.")
    min_move = args.min_move if args.min_move > 0 else 0.05 * (w / 2)
    print(f"[capture] board: {pattern[0]}x{pattern[1]} inner corners, {args.square}mm squares")
    print("[capture] (if no green grid appears on the board, the inner-corner count is wrong)")
    if args.manual:
        print("[capture] MANUAL: SPACE = save (both must show board) | q = quit\n")
    else:
        print(f"[capture] AUTO: sweep the board slowly; saves when still + a new pose "
              f"(>{min_move:.0f}px shift or scale change). Target {args.count} pairs.")
        print("[capture] stop by closing the window or Ctrl-C.\n")

    saved = 0
    last_save_t = 0.0
    prev_centroid = None      # board centroid from previous frame (stability)
    last_saved_feat = None    # (centroid, spread) of last saved pair (novelty)
    flash_until = 0.0
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    def save_pair(left, right, feat):
        nonlocal saved, last_save_t, last_saved_feat, flash_until
        n = saved
        cv2.imwrite(os.path.join(args.out, "left", f"pair_{n:03d}.png"), left)
        cv2.imwrite(os.path.join(args.out, "right", f"pair_{n:03d}.png"), right)
        saved += 1
        last_save_t = time.time()
        last_saved_feat = feat
        flash_until = last_save_t + 0.25
        print(f"[capture] saved pair {n:03d}  (total {saved})")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            left, right = split_lr(frame)
            gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            okl, cl = find_board(gl, pattern)
            okr, cr = find_board(gr, pattern)
            both = okl and okr

            disp_l, disp_r = left.copy(), right.copy()
            if okl:
                cv2.drawChessboardCorners(disp_l, pattern, cl, okl)
            if okr:
                cv2.drawChessboardCorners(disp_r, pattern, cr, okr)
            combined = cv2.hconcat([disp_l, disp_r])

            now = time.time()
            feat = board_feat(cl) if okl else None
            status, color = "", (0, 0, 255)
            if not args.manual and both:
                motion = (np.linalg.norm(feat[0] - prev_centroid)
                          if prev_centroid is not None else None)
                stable = motion is not None and motion < STABLE_PX
                if last_saved_feat is None:
                    distinct = True
                else:
                    c0, s0 = last_saved_feat
                    distinct = (np.linalg.norm(feat[0] - c0) > min_move
                                or abs(feat[1] - s0) > SPREAD_NOVELTY * s0)
                if stable and distinct and (now - last_save_t) > COOLDOWN_S:
                    save_pair(left, right, feat)
                    status, color = "saved!", (0, 220, 0)
                elif not stable:
                    status, color = "hold still...", (0, 165, 255)
                elif not distinct:
                    status, color = "move to a NEW pose", (0, 165, 255)
                else:
                    status, color = "ready", (0, 220, 0)
            elif both:
                status, color = "BOTH OK - SPACE to save", (0, 220, 0)
            else:
                status = f"L:{'ok' if okl else '--'}  R:{'ok' if okr else '--'}"
            prev_centroid = feat[0] if feat is not None else None

            cv2.putText(combined, f"pairs saved: {saved}/{args.count}   {status}",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            if now < flash_until:  # brief white border on save
                cv2.rectangle(combined, (0, 0), (combined.shape[1] - 1, combined.shape[0] - 1),
                              (255, 255, 255), 25)

            cv2.imshow(WIN, fit_to_screen(combined))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if args.manual and key == ord(" ") and both and (now - last_save_t) > 0.4:
                save_pair(left, right, feat)

            # Stop conditions that don't need the keyboard:
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
            if not args.manual and saved >= args.count:
                print(f"[capture] reached target of {args.count} pairs.")
                break
    except KeyboardInterrupt:
        print("\n[capture] stopped (Ctrl-C).")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[capture] done. {saved} pairs in '{args.out}/'.")
    if saved >= 12:
        print("Next: python3 stereo_calibrate.py "
              f"--captures {args.out} --inner-cols {args.inner_cols} "
              f"--inner-rows {args.inner_rows} --square {args.square}")
    else:
        print("Capture more pairs (aim for 20-30) before calibrating.")


if __name__ == "__main__":
    main()
