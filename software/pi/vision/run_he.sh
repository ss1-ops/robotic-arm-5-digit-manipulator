#!/usr/bin/env bash
# Start ONLY the hand_eye_calibrate node (assumes camera + stream already running,
# e.g. started by the GUI on Connect SSH). Detached. Log: /tmp/handeye.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
pkill -9 -f hand_eye_calibrate 2>/dev/null
sleep 1
setsid python3 hand_eye_calibrate.py --ros-args \
    -p calib:="$HOME/vision/stereo_calib.yaml" "$@" >/tmp/handeye.log 2>&1 </dev/null &
sleep 1
echo "hand-eye started (camera+stream left running)"
