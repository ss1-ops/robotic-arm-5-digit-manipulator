#!/usr/bin/env bash
# Copy the vision nodes + scripts + calibration to the Pi.
# Usage: bash deploy_to_pi.sh   (override host with: PI=armpi@192.168.1.142 bash deploy_to_pi.sh)
set -e

PI="${PI:-armpi@armpi.local}"
DEST="${DEST:-~/vision}"
CALIB="../calibration/stereo_calib.yaml"

echo "[deploy] target: $PI:$DEST"
ssh "$PI" "mkdir -p $DEST"
scp stereo_camera_node.py hand_eye_calibrate.py \
    mjpeg_stream.py \
    run_stream.sh README.md "$PI:$DEST/"
if [ -f "$CALIB" ]; then
    scp "$CALIB" "$PI:$DEST/"
    echo "[deploy] sent stereo_calib.yaml"
else
    echo "[deploy] WARN: $CALIB not found — nodes will lack calibration"
fi
echo "[deploy] done."
echo ""
echo "For maximum Pi performance (recommended): run only the camera + stream + motors on the Pi."
echo "All heavy vision (SGBM depth, blob detection, servoing) now runs on your Mac."
echo "See the updated README.md in this directory for offload instructions and the new"
echo "mac_stereo_grabber.py + mac_processor.py in software/vision/."
