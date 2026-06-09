#!/usr/bin/env bash
# Start the distance-gated approach servo (board target on raw image; no depth node).
# Assumes camera node + moveo_publisher running, ESP connected, board in view,
# Send All Joints to seed /joint_commands. Watch: tail -f /tmp/approach.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
pkill -9 -f "ibvs_servo|approach_servo|joint_probe|stereo_depth_node" 2>/dev/null
sleep 1
setsid python3 approach_servo.py --ros-args "$@" >/tmp/approach.log 2>&1 </dev/null &
sleep 1
echo "approach servo started (watch /tmp/approach.log)"
