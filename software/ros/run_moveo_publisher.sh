#!/usr/bin/env bash
# Restart the moveo_publisher TCP/IK server (port 9000) detached on the Pi.
# Log: /tmp/moveo_publisher.log
source /opt/ros/jazzy/setup.bash 2>/dev/null
pkill -9 -f "ros_nodes/moveo_publisher.py" 2>/dev/null
sleep 2
setsid python3 /home/armpi/ros_nodes/moveo_publisher.py >/tmp/moveo_publisher.log 2>&1 </dev/null &
# ROS init + node setup takes several seconds before the TCP server binds
for _ in $(seq 1 10); do ss -tlnp 2>/dev/null | grep -q :9000 && break; sleep 1; done
if ss -tlnp 2>/dev/null | grep -q :9000; then
  echo "moveo_publisher up, :9000 listening"
else
  echo "FAILED to bind :9000 — see /tmp/moveo_publisher.log"
fi
