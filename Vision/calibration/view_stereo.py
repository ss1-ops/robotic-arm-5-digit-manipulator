#!/usr/bin/env python3
"""
view_stereo.py — live preview of the ELP dual-lens stream.

Shows the side-by-side frame (left | right) with a divider line and a live FPS
readout. Handy for checking focus, exposure, framing, and that both lenses are
working before calibrating.

Keys:
    v   toggle view: side-by-side  /  left only  /  right only
    s   save a snapshot (full side-by-side frame) to snapshots/
    q / ESC  quit

Usage:
    python3 view_stereo.py                 # index 0, 2560x960
    python3 view_stereo.py --index 1 --width 1280 --height 720
"""

import argparse
import os
import time

import cv2

from stereo_io import open_stereo_camera, split_lr, describe_capture, fit_to_screen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0, help="Camera index (ELP=0)")
    ap.add_argument("--width", type=int, default=2560, help="Full combined frame width")
    ap.add_argument("--height", type=int, default=960, help="Frame height")
    args = ap.parse_args()

    cap = open_stereo_camera(args.index, args.width, args.height)
    w, h, fps, fourcc = describe_capture(cap)
    print(f"[view] camera {args.index}: {w}x{h} @ {fps:.0f}fps {fourcc}")
    if w < args.width:
        print(f"[WARN] got width {w}, requested {args.width} — may be a single-lens fallback.")
    print("[view] keys: v=toggle view  s=snapshot  q/ESC=quit")

    modes = ("both", "left", "right")
    mode_i = 0
    os.makedirs("snapshots", exist_ok=True)
    win = "ELP stereo (v=view  s=snap  q=quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    t_prev, fps_disp = time.time(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[view] frame grab failed; retrying...")
            time.sleep(0.05)
            continue

        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt > 0:
            fps_disp = 0.9 * fps_disp + 0.1 * (1.0 / dt) if fps_disp else 1.0 / dt

        left, right = split_lr(frame)
        mode = modes[mode_i]
        if mode == "left":
            view = left.copy()
        elif mode == "right":
            view = right.copy()
        else:
            view = frame.copy()
            mid = view.shape[1] // 2
            cv2.line(view, (mid, 0), (mid, view.shape[0]), (0, 255, 255), 2)

        cv2.putText(view, f"{mode}  {view.shape[1]}x{view.shape[0]}  {fps_disp:4.1f} fps",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3)

        cv2.imshow(win, fit_to_screen(view))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # q or ESC
            break
        if key == ord("v"):
            mode_i = (mode_i + 1) % len(modes)
        if key == ord("s"):
            fn = os.path.join("snapshots", f"snap_{int(now)}.png")
            cv2.imwrite(fn, frame)
            print(f"[view] saved {fn}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
