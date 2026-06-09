#!/usr/bin/env bash
# Start the IBVS pointing controller (detects board on raw image; no depth node).
# Assumes camera node + moveo_publisher running, ESP connected, board in view,
# and a jog (Send All Joints) to seed /joint_commands. Watch: tail -f /tmp/ibvs.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
pkill -9 -f "stereo_depth_node|ibvs_servo|joint_probe" 2>/dev/null
sleep 1
setsid python3 ibvs_servo.py --ros-args "$@" >/tmp/ibvs.log 2>&1 </dev/null &
sleep 1
echo "ibvs started (watch /tmp/ibvs.log)"
