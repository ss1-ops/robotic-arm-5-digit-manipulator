#!/usr/bin/env bash
# Start the stereo camera + hand-eye calibration nodes on the Pi, detached.
# (No depth node — saves CPU so detection stays snappy during capture.)
# Usage (on the Pi):  bash run_handeye.sh [extra hand_eye --ros-args params]
# Watch progress:     tail -f /tmp/handeye.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
DIR="$HOME/vision"
CALIB="$DIR/stereo_calib.yaml"
cd "$DIR" || exit 1

pkill -9 -f stereo_depth_node 2>/dev/null
pkill -9 -f hand_eye_calibrate 2>/dev/null
pkill -9 -f stereo_camera_node 2>/dev/null
sleep 1

setsid python3 stereo_camera_node.py --ros-args \
    -p device:=0 -p calib:="$CALIB" >/tmp/cam.log 2>&1 </dev/null &
sleep 5
setsid python3 hand_eye_calibrate.py --ros-args \
    -p calib:="$CALIB" "$@" >/tmp/handeye.log 2>&1 </dev/null &
sleep 1
echo "started: camera + hand-eye"
echo "watch:  tail -f /tmp/handeye.log    (stop: pkill -f 'stereo_camera_node|hand_eye_calibrate')"
