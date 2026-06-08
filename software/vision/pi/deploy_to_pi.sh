#!/usr/bin/env bash
# Copy the vision nodes + scripts + calibration to the Pi.
# Usage: bash deploy_to_pi.sh   (override host with: PI=armpi@192.168.1.142 bash deploy_to_pi.sh)
set -e

PI="${PI:-armpi@armpi.local}"
DEST="${DEST:-~/vision}"
CALIB="../calibration/stereo_calib.yaml"

echo "[deploy] target: $PI:$DEST"
ssh "$PI" "mkdir -p $DEST"
scp stereo_camera_node.py stereo_depth_node.py hand_eye_calibrate.py \
    run_vision.sh stop_vision.sh README.md "$PI:$DEST/"
if [ -f "$CALIB" ]; then
    scp "$CALIB" "$PI:$DEST/"
    echo "[deploy] sent stereo_calib.yaml"
else
    echo "[deploy] WARN: $CALIB not found — nodes will lack calibration"
fi
echo "[deploy] done. On the Pi (source /opt/ros/jazzy/setup.bash first):"
echo "    bash $DEST/run_vision.sh nearest        # camera + depth"
echo "    python3 $DEST/hand_eye_calibrate.py --ros-args -p calib:=$DEST/stereo_calib.yaml -p ee_frame:=<link>"
