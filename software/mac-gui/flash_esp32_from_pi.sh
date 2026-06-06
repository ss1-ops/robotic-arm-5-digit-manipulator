#!/bin/bash
# Flash the esp32s3_arm_controller sketch to the ESP32-S3 from the Pi.
# The sketch directory must be at ~/sketches/esp32s3_arm_controller/
# Usage: bash ~/flash_esp32.sh
#
# To push a new sketch from Mac first:
#   scp -r /path/to/esp32s3_arm_controller armpi@armpi.local:~/sketches/
#
# REQUIRES: Pi GPIO17→ESP32 EN and GPIO27→ESP32 IO0 wired (see ~/esp32_control.sh)

set -e
export PATH="$HOME/.local/bin:$PATH"

SKETCH_DIR="$HOME/sketches/esp32s3_arm_controller"
FQBN="esp32:esp32:esp32s3"
PORT="${1:-/dev/moveo_arduino}"

if [ ! -f "$SKETCH_DIR/esp32s3_arm_controller.ino" ]; then
  echo "ERROR: sketch not found at $SKETCH_DIR"
  echo "Copy it first: scp -r <mac_sketch_dir> armpi@armpi.local:~/sketches/"
  exit 1
fi

echo "=== Stopping micro-ROS agent (releases serial port) ==="
pkill -f micro_ros_agent 2>/dev/null && sleep 1 || true

echo "=== Entering ESP32 bootloader mode ==="
bash ~/esp32_control.sh bootloader
sleep 0.5

echo "=== Compiling $SKETCH_DIR ==="
arduino-cli compile \
  --fqbn "$FQBN" \
  --libraries "$HOME/Arduino/libraries" \
  "$SKETCH_DIR"

echo "=== Uploading to $PORT ==="
arduino-cli upload \
  --fqbn "$FQBN" \
  --port "$PORT" \
  "$SKETCH_DIR"

echo "=== Resetting ESP32 into normal run mode ==="
bash ~/esp32_control.sh reset
sleep 2

echo "=== Restarting micro-ROS agent ==="
cd ~/microros_ws
source install/setup.bash
nohup ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/moveo_arduino -b 1000000 \
  > /tmp/micro_ros_agent.log 2>&1 &
echo "Agent PID $! — log at /tmp/micro_ros_agent.log"

echo "=== Done ==="
