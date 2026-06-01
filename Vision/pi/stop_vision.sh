#!/usr/bin/env bash
# Stop the stereo vision nodes on the Pi.
pkill -9 -f stereo_depth_node 2>/dev/null
pkill -9 -f stereo_camera_node 2>/dev/null
echo "stopped vision nodes"
