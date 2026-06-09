#!/usr/bin/env bash
# Bring the ESP32 micro-ROS client online in the correct order for native USB CDC:
# stop agent -> reset ESP (boot firmware) -> wait for the CDC port -> start agent.
export PATH="$HOME/.local/bin:$PATH"
LOG=/tmp/agent_connect.log
echo "=== stop any agent ==="
pkill -9 -f micro_ros_agent 2>/dev/null; sleep 2
echo "=== reset ESP32 into run mode ==="
bash "$HOME/esp32_control.sh" reset
echo "=== wait for /dev/ttyACM0 to re-enumerate ==="
for i in $(seq 1 30); do [ -e /dev/ttyACM0 ] && { echo "port up after ${i}x0.5s"; break; }; sleep 0.5; done
sleep 2  # let CDC settle + firmware reach its micro-ROS init
echo "=== start agent on /dev/ttyACM0 (detached) ==="
source /opt/ros/jazzy/setup.bash 2>/dev/null
[ -f "$HOME/microros_ws/install/setup.bash" ] && source "$HOME/microros_ws/install/setup.bash"
setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 >"$LOG" 2>&1 </dev/null &
echo "agent launched, logging to $LOG"
