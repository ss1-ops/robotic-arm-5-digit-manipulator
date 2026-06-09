#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
pkill -9 -f "ibvs_servo|approach_servo|joint_probe|scale_probe|stereo_depth_node" 2>/dev/null
sleep 1
setsid python3 scale_probe.py --ros-args "$@" >/tmp/scale.log 2>&1 </dev/null &
sleep 1
echo "scale_probe started (watch /tmp/scale.log)"
