#!/usr/bin/env bash
# Start the MJPEG browser stream (rectified left + board overlay), detached.
# View at http://armpi.local:8080/   Stop: pkill -f mjpeg_stream
source /opt/ros/jazzy/setup.bash 2>/dev/null
CALIB="$HOME/vision/stereo_calib.yaml"
cd "$HOME/vision" || exit 1
pkill -9 -f mjpeg_stream 2>/dev/null
sleep 1
setsid python3 mjpeg_stream.py --ros-args -p calib:="$CALIB" >/tmp/mjpeg.log 2>&1 </dev/null &
sleep 2
echo "stream launched -> http://armpi.local:8080/  (log: /tmp/mjpeg.log)"
