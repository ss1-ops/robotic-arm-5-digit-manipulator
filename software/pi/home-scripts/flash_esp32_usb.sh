#!/usr/bin/env bash
# Minimal ESP32-S3 flash that avoids the flaky GPIO bootloader entry: esptool
# self-resets the S3 into download mode over its USB-Serial-JTAG. Logs to
# ~/flash2.log (home dir survives /tmp cleaning). Detach with: setsid -f.
export PATH="$HOME/.local/bin:$PATH"
LOG="$HOME/flash2.log"
exec > "$LOG" 2>&1
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc"
SKETCH="$HOME/sketches/esp32s3_arm_controller"
PORT="/dev/ttyACM0"

echo "=== $(date) starting ==="
echo "=== stop agent (release port) ==="
pkill -9 -f micro_ros_agent 2>/dev/null; sleep 2

echo "=== compile (CDCOnBoot=cdc) ==="
arduino-cli compile --fqbn "$FQBN" --libraries "$HOME/Arduino/libraries" "$SKETCH" || { echo "COMPILE_FAILED"; exit 1; }

echo "=== upload (esptool USB auto-reset, no GPIO) ==="
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH" || { echo "UPLOAD_FAILED"; exit 1; }

echo "=== restart micro-ROS agent ==="
sleep 2
cd "$HOME/microros_ws" && source install/setup.bash 2>/dev/null
setsid -f ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b 115200 -v4 >/tmp/mra.log 2>&1 </dev/null
echo "=== FLASH_COMPLETE $(date) ==="
