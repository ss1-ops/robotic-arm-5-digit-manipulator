#!/bin/bash
set -e
export PATH="$HOME/.local/bin:$PATH"

SKETCH_DIR="$HOME/sketches/esp32s3_arm_controller"
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc"
PORT="${1:-/dev/moveo_arduino}"

if [ ! -f "$SKETCH_DIR/esp32s3_arm_controller.ino" ]; then
  echo "ERROR: sketch not found at $SKETCH_DIR"; exit 1
fi

echo "=== Stopping micro-ROS agent ==="
pkill -9 micro_ros_agent 2>/dev/null || true
sleep 1

echo "=== Entering ESP32 bootloader mode ==="
bash ~/esp32_control.sh bootloader
sleep 0.5

echo "=== Compiling $SKETCH_DIR ==="
arduino-cli compile --fqbn "$FQBN" --libraries "$HOME/Arduino/libraries" "$SKETCH_DIR"

echo "=== Uploading to $PORT ==="
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo "=== Resetting ESP32 into normal run mode ==="
bash ~/esp32_control.sh reset
sleep 2

echo "=== Restarting micro-ROS agent ==="
pkill -9 micro_ros_agent 2>/dev/null || true
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
cd ~/microros_ws
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 > /tmp/micro_ros_agent.log 2>&1 </dev/null &
echo "Agent PID $! — log at /tmp/micro_ros_agent.log"

echo "=== Done ==="
