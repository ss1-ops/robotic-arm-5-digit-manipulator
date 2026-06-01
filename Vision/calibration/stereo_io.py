#!/usr/bin/env python3
"""
stereo_io.py — shared helpers for the ELP dual-lens stereo camera on macOS.

The ELP module enumerates as ONE UVC device that outputs a single side-by-side
synchronized frame (left half | right half). We open it with the AVFoundation
backend, force MJPEG (required for the high-res modes over USB2), and split each
frame down the middle.
"""

import cv2
import numpy as np


def open_stereo_camera(index, width=2560, height=960, fps=60, fourcc="MJPG"):
    """Open the ELP stereo device and request the side-by-side mode.

    `width` is the FULL combined width (both lenses). For the 960p mode that is
    2560x960 (two 1280x960 frames). AVFoundation may silently ignore some of
    these hints, so callers should always check the actual frame size returned
    by read() rather than trusting the request.
    """
    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open camera index {index}. Run find_camera.py to list "
            "indices, and make sure your terminal app has Camera permission "
            "(System Settings > Privacy & Security > Camera)."
        )
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def split_lr(frame):
    """Split a side-by-side frame into (left, right) halves."""
    w = frame.shape[1]
    half = w // 2
    return frame[:, :half].copy(), frame[:, half:2 * half].copy()


def describe_capture(cap):
    """Return (width, height, fps, fourcc_str) the device actually settled on."""
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    raw = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((raw >> 8 * i) & 0xFF) for i in range(4)) if raw else "?"
    return w, h, fps, fourcc


def fit_to_screen(img, max_w=1600):
    """Downscale an image for on-screen preview only (keeps aspect ratio)."""
    if img.shape[1] <= max_w:
        return img
    scale = max_w / img.shape[1]
    return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
