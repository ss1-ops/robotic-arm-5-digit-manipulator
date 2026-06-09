#!/usr/bin/env bash
# Stop the vision stream on the Pi.
pkill -9 -f direct_stream 2>/dev/null
pkill -9 -f stereo_camera_node 2>/dev/null
pkill -9 -f stereo_depth_node 2>/dev/null
pkill -9 -f mjpeg_stream 2>/dev/null
echo "stopped vision"
