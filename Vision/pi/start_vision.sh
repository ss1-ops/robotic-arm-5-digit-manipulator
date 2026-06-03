#!/usr/bin/env bash
# Start the vision stack (stereo camera node + browser MJPEG stream), detached.
# Invoked by the Mac GUI on "Connect SSH". View at http://armpi.local:8080/
#
# Camera is mounted RIGHT-WAY-UP (remounted 2026-06-02): raw frames are already
# upright, so neither the node (rotate_180=false) nor the stream (display_rotate_180
# =false) rotates. Re-run safely: it clears old vision nodes first.
# NOTE: stereo_calib.yaml was captured with the OLD upside-down mount — K1/D1
# intrinsics still hold (sensor-physical, used by hand-eye), but the rectification
# maps are wrong for this orientation, so stereo DEPTH must be recalibrated before use.
source /opt/ros/jazzy/setup.bash 2>/dev/null
CALIB="$HOME/vision/stereo_calib.yaml"
cd "$HOME/vision" || exit 1

pkill -9 -f "stereo_camera_node|stereo_depth_node|hand_eye_calibrate|mjpeg_stream" 2>/dev/null
sleep 1

setsid python3 stereo_camera_node.py --ros-args \
    -p device:=0 -p calib:="$CALIB" -p rotate_180:=false \
    >/tmp/cam.log 2>&1 </dev/null &
sleep 4
# Stream shows the raw left view; no display rotation (camera is upright).
setsid python3 mjpeg_stream.py --ros-args \
    -p display_rotate_180:=false \
    >/tmp/mjpeg.log 2>&1 </dev/null &
sleep 1
echo "vision started: camera + stream (http://armpi.local:8080/)"
