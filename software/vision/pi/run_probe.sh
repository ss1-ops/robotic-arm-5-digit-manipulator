#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
CALIB="$HOME/vision/stereo_calib.yaml"
pkill -9 -f "stereo_depth_node|ibvs_servo|joint_probe" 2>/dev/null
sleep 1
setsid python3 stereo_depth_node.py --ros-args -p calib:="$CALIB" -p detector:=board >/tmp/depth.log 2>&1 </dev/null &
sleep 3
setsid python3 joint_probe.py --ros-args "$@" >/tmp/probe.log 2>&1 </dev/null &
sleep 1
echo "started: depth(board) + joint_probe (watch /tmp/probe.log)"
