#!/usr/bin/env bash
# Start the stereo camera + depth nodes on the Pi, fully detached (setsid), so it
# survives the SSH session. Logs to /tmp/cam.log and /tmp/depth.log.
# Usage (on the Pi):  bash run_vision.sh [detector]   # detector: nearest | color
source /opt/ros/jazzy/setup.bash 2>/dev/null
DIR="$HOME/vision"
CALIB="$DIR/stereo_calib.yaml"
DETECTOR="${1:-nearest}"
cd "$DIR" || exit 1

pkill -9 -f stereo_camera_node 2>/dev/null
pkill -9 -f stereo_depth_node 2>/dev/null
sleep 1

setsid python3 stereo_camera_node.py --ros-args \
    -p device:=0 -p calib:="$CALIB" >/tmp/cam.log 2>&1 </dev/null &
sleep 4
setsid python3 stereo_depth_node.py --ros-args \
    -p calib:="$CALIB" -p detector:="$DETECTOR" >/tmp/depth.log 2>&1 </dev/null &
sleep 1
echo "started: camera + depth ($DETECTOR)"
echo "logs: /tmp/cam.log  /tmp/depth.log"
