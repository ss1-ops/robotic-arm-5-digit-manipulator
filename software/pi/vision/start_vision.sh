#!/usr/bin/env bash
# Start the direct camera MJPEG stream (no ROS), detached.
# Invoked by the Mac GUI on "Connect SSH". View at http://armpi.local:8080/
# Re-run safely: kills any old vision processes first.
cd "$HOME/vision" || exit 1

pkill -9 -f "direct_stream|stereo_camera_node|stereo_depth_node|hand_eye_calibrate|mjpeg_stream" 2>/dev/null
sleep 1

setsid python3 direct_stream.py >/tmp/cam.log 2>&1 </dev/null &
sleep 3

pgrep -f direct_stream > /dev/null \
    && echo "vision started: http://armpi.local:8080/" \
    || { echo "WARNING: direct_stream not running"; cat /tmp/cam.log; }
