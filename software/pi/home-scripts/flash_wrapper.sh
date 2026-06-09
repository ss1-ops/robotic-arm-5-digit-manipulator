#!/usr/bin/env bash
exec >/tmp/flash.log 2>&1
echo "=== flash wrapper start $(date) ==="
pkill -9 -f micro_ros_agent 2>/dev/null
sleep 2
bash "$HOME/flash_esp32.sh"
echo "=== flash wrapper done rc=$? $(date) ==="
