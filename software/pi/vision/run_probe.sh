#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
pkill -9 -f "stereo_depth_node|ibvs_servo|joint_probe" 2>/dev/null
sleep 1
setsid python3 joint_probe.py --ros-args "$@" >/tmp/probe.log 2>&1 </dev/null &
sleep 1
echo "joint_probe started (camera node assumed running). watch /tmp/probe.log"
