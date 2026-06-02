#!/usr/bin/env python3
"""
make_charuco.py — generate a printable ChArUco board for hand-eye calibration.

ChArUco (chessboard + embedded ArUco markers) gives an UNAMBIGUOUS pose — unlike a
plain checkerboard, whose planar pose rotation flips between two solutions and
corrupts hand-eye. The marker IDs fix the orientation.

Board params here MUST match the detector (hand_eye_calibrate.py charuco mode):
  dict = DICT_4X4_50, 7x5 squares, square 28 mm, marker 21 mm.

Print it (fit to page is fine), then MEASURE one black square edge with calipers
and pass that to hand-eye as -p square_mm:=<measured> (printers rescale).

Usage:  python3 make_charuco.py          # writes charuco_7x5.png
"""

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50
SQX, SQY = 7, 5            # squares
SQUARE_M = 0.028          # 28 mm (nominal — measure after printing)
MARKER_M = 0.021          # 21 mm
OUT = "charuco_7x5.png"
PX_W = 1680               # render resolution (aspect 7:5); ~240 px per square


def main():
    a = cv2.aruco
    d = (a.getPredefinedDictionary(DICT) if hasattr(a, "getPredefinedDictionary")
         else a.Dictionary_get(DICT))
    px = (PX_W, int(PX_W * SQY / SQX))
    # OpenCV 4.7+ (new API) vs 4.6 (legacy)
    if hasattr(a, "CharucoBoard") and not hasattr(a, "CharucoBoard_create"):
        board = a.CharucoBoard((SQX, SQY), SQUARE_M, MARKER_M, d)
        img = board.generateImage(px, marginSize=40)
    else:
        board = a.CharucoBoard_create(SQX, SQY, SQUARE_M, MARKER_M, d)
        img = board.draw(px, marginSize=40)
    cv2.imwrite(OUT, img)
    print(f"wrote {OUT}  ({SQX}x{SQY} squares, nominal {SQUARE_M*1000:.0f}mm sq / "
          f"{MARKER_M*1000:.0f}mm marker, DICT_4X4_50)")
    print("PRINT it, then measure a black square edge and use that mm value for hand-eye.")


if __name__ == "__main__":
    main()
