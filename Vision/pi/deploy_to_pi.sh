#!/usr/bin/env bash
# Copy the stereo camera node + calibration to the Pi.
# Usage: bash deploy_to_pi.sh
set -e

PI="${PI:-armpi@armpi.local}"
DEST="${DEST:-~/vision}"
CALIB="../calibration/stereo_calib.yaml"

echo "[deploy] target: $PI:$DEST"
ssh "$PI" "mkdir -p $DEST"
scp stereo_camera_node.py "$PI:$DEST/"
if [ -f "$CALIB" ]; then
    scp "$CALIB" "$PI:$DEST/"
    echo "[deploy] sent stereo_calib.yaml"
else
    echo "[deploy] WARN: $CALIB not found — node will stream without CameraInfo"
fi
echo "[deploy] done. On the Pi:"
echo "    cd $DEST && python3 stereo_camera_node.py --ros-args -p calib:=\$PWD/stereo_calib.yaml"
