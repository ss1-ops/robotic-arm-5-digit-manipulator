#!/usr/bin/env bash
# Recover the micro-ROS link after a flash: one fresh agent, then hard-reset the
# ESP32 so its micro-ROS client re-handshakes. Log: /tmp/micro_ros_agent.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
# 1) release the serial port FIRST so the chip reset re-enumerates cleanly
echo "=== stopping agent (release port) ==="
pkill -9 -f micro_ros_agent 2>/dev/null
sleep 2
# 2) hard-reset the ESP into run mode with NOTHING holding the port
echo "=== hard-resetting ESP32 ==="
bash "$HOME/esp32_control.sh" reset
sleep 6
echo "=== ttyACM after reset ==="; ls -l /dev/ttyACM* 2>/dev/null || echo "NO ttyACM!"
# 3) now start ONE fresh agent; it will catch the ESP's handshake
cd "$HOME/microros_ws" && source install/setup.bash 2>/dev/null
setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 \
  >/tmp/micro_ros_agent.log 2>&1 </dev/null &
sleep 8
echo "=== session check ==="
if grep -qiE "session established|create_participant|create_client|Client connected" /tmp/micro_ros_agent.log; then
  echo "ESP CONNECTED — micro-ROS session established"
else
  echo "no session yet — may need a physical USB power-cycle of the ESP"
fi
grep -iE "session|participant|client|topic|subscriber" /tmp/micro_ros_agent.log | tail -6
