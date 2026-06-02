#!/usr/bin/env bash
# Start the vision stack (stereo camera node + browser MJPEG stream), detached.
# Invoked by the Mac GUI on "Connect SSH". View at http://armpi.local:8080/
#
# Camera streams RAW as-mounted frames (rotate_180=false); the stream display-rotates
# 180 for human viewing only (cosmetic). Re-run safely: it clears old vision nodes first.
source /opt/ros/jazzy/setup.bash 2>/dev/null
CALIB="$HOME/vision/stereo_calib.yaml"
cd "$HOME/vision" || exit 1

pkill -9 -f "stereo_camera_node|stereo_depth_node|hand_eye_calibrate|mjpeg_stream" 2>/dev/null
sleep 1

setsid python3 stereo_camera_node.py --ros-args \
    -p device:=0 -p calib:="$CALIB" -p rotate_180:=false \
    >/tmp/cam.log 2>&1 </dev/null &
sleep 4
# No calib -> stream shows the raw left view (rectification is invalid until recalibrated);
# display_rotate makes the upside-down mount look upright in the browser.
setsid python3 mjpeg_stream.py --ros-args \
    -p display_rotate_180:=true \
    >/tmp/mjpeg.log 2>&1 </dev/null &
sleep 1
echo "vision started: camera + stream (http://armpi.local:8080/)"
