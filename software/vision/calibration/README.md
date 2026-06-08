# Stereo Calibration (Mac-side)

Interactive calibration tooling for the **ELP dual-lens** stereo camera. Runs on
the MacBook (needs a display); the resulting `stereo_calib.yaml` is copied to the
headless Pi where the runtime vision pipeline loads it. Stereo calibration is a
property of the camera, not the host, so calibrating on the Mac is correct.

> Mount context: the camera is **eye-in-hand** (just above the Aero Hand). This
> step produces the intrinsics + stereo extrinsics. The camera→end-effector
> transform (hand-eye) is a separate later step that needs the arm live.

## Setup

```bash
cd "Vision/calibration"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**macOS camera permission:** the first time OpenCV opens the camera, your
terminal app (Terminal / iTerm / VS Code) must be granted Camera access under
**System Settings > Privacy & Security > Camera**. If frames come back black or
`read()` fails, this is almost always the cause — grant access and relaunch the
terminal app.

## 1. Find the camera index

```bash
python3 find_camera.py
```

On this Mac the **ELP is index 0** (native max **2560×960** = two 1280×960) and
the built-in FaceTime camera is index 1. `capture_stereo.py` defaults to index 0
and requests 2560×960 (the highest mode; higher requests clamp to it).

## 2. Print a checkerboard

Use a flat, rigid checkerboard (mount on cardboard/clipboard — any warp ruins the
calibration). Note the **inner corners** = squares − 1 per side, and the real
**square size in mm** (measure the print; printers rescale). A 10×7-square board
= `9×6` inner corners.

## 3. Capture pairs

```bash
python3 capture_stereo.py --inner-cols 9 --inner-rows 6 --square 25.0
```

- **Auto-capture (default):** macOS often won't deliver keystrokes to OpenCV
  windows, so capture is automatic. Sweep the board slowly around the workspace;
  a pair saves whenever the board is in **both** lenses, held **still**, and in a
  pose meaningfully **different** from the last save (a white border flashes on
  save). Stop by **closing the window** or **Ctrl-C**. Stops itself at `--count`
  (default 30).
- `--manual` falls back to SPACE-to-save / `q`-to-quit if your build's keyboard
  handling works.
- Either way: capture **20–30** pairs covering varied distance and tilt, and make
  sure the board visits all regions of the frame including the **corners** (that's
  where distortion lives). Keep it sharp and well-lit.
- Saved to `captures/left/` and `captures/right/`.

## 4. Calibrate

```bash
python3 stereo_calibrate.py --captures captures \
    --inner-cols 9 --inner-rows 6 --square 25.0 --show
```

- Prints per-lens and stereo reprojection RMS (aim for **< 0.5 px**) and the
  measured **baseline** (sanity-check against the physical lens spacing, ~60 mm).
- `--show` remaps a sample pair and draws horizontal lines — corresponding points
  should land on the **same row** across left/right if rectification is good.
- Writes `stereo_calib.yaml` (OpenCV FileStorage: K1/D1/K2/D2/R/T/R1/R2/P1/P2/Q).

## 5. Ship to the Pi

```bash
scp stereo_calib.yaml armpi@armpi.local:~/vision/
```

The Pi runtime loads it with `cv2.FileStorage(path, cv2.FILE_STORAGE_READ)` to
build the rectification maps and reproject disparity to 3D via `Q`.

## Files

- `stereo_io.py` — camera open (AVFoundation + MJPEG) and L/R split helpers
- `find_camera.py` — list indices, flag the side-by-side stereo device
- `capture_stereo.py` — interactive checkerboard pair capture
- `stereo_calibrate.py` — calibrate, rectify, save YAML
- `.venv/`, `captures/` — git-ignored
