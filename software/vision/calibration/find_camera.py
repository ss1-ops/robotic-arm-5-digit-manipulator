#!/usr/bin/env python3
"""
find_camera.py — probe macOS video indices and identify the ELP stereo device.

The built-in FaceTime camera is usually index 0. The ELP dual-lens module is the
one whose frame is roughly twice as wide as it is tall (side-by-side stereo).

Usage:
    python3 find_camera.py [--max-index 6]
"""

import argparse

import cv2

from stereo_io import open_stereo_camera, describe_capture


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-index", type=int, default=6,
                    help="Highest camera index to probe (default 6)")
    args = ap.parse_args()

    print("Probing camera indices (AVFoundation)...\n")
    stereo_candidates = []
    for idx in range(args.max_index + 1):
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"  index {idx}: opened but no frame (permission? in use?)")
            cap.release()
            continue
        h, w = frame.shape[:2]
        _, _, fps, fourcc = describe_capture(cap)
        aspect = w / h if h else 0
        stereo = aspect > 1.8  # side-by-side pair ~= 2.0 (or wider)
        tag = "  <-- likely STEREO (side-by-side)" if stereo else ""
        print(f"  index {idx}: {w}x{h} @ {fps:.0f}fps {fourcc} aspect={aspect:.2f}{tag}")
        if stereo:
            stereo_candidates.append(idx)
        cap.release()

    print()
    if stereo_candidates:
        print(f"Stereo camera likely at index: {stereo_candidates[0]}")
        print("Confirm the high-res side-by-side mode (e.g. 2560x960) with:")
        print(f"    python3 capture_stereo.py --index {stereo_candidates[0]}")
    else:
        print("No side-by-side device found. If the ELP is plugged in, it may be")
        print("defaulting to a single-lens/low-res mode — try forcing the mode:")
        print("    python3 capture_stereo.py --index <n> --width 2560 --height 960")
        print("Also check terminal Camera permission in System Settings > Privacy.")


if __name__ == "__main__":
    main()
