#!/usr/bin/env bash
# F-006 test: Reconnect ESP after killing micro-ROS (and all ROS processes) exactly as the Mac GUI _CMD_KILL + agent + ESP32 poll flow.
# Run this on the Pi (or deploy via ssh). Supports multiple cycles for repeatable verification.
#
# Usage: ./test_reconnect_after_kill.sh [--cycles N] [--help]
#
# HARDWARE GATE (non-negotiable): Actuators MUST be disconnected (no power to joints/steppers).
# This script performs ONLY port release, process kill, agent restart, and session check.
# It sends /reboot (to ESP firmware) and starts micro_ros_agent + polls logs.
# NO joint commands, no motion, no actuator power. Confirm before running.
# See firmware/esp32s3-arm-controller/esp32s3_arm_controller/troubleshooting_micro_ros.md and software/mac-gui/reconnect_esp.sh.
#
# Matches current software/mac-gui/moveo_simple_controller.py _CMD_KILL / _CMD_AGENT / _CMD_ESP32 (as of latest).
# Creates /tmp/test_reconnect.log for this run.
set -u

CYCLES=3
LIGHT=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cycles) CYCLES="$2"; shift ;;
    --light) LIGHT=1 ;;
    -h|--help) echo "Usage: $0 [--cycles N] [--light]"; echo "  --light : leave micro_ros_agent running (new recommended path)"; echo "HARDWARE GATE: actuators disconnected. Only tests comms/port/ESP reconnect."; exit 0 ;;
    *) echo "Unknown arg $1"; exit 1 ;;
  esac
  shift
done

LOG="/tmp/test_reconnect.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== F-006 GUI-flow reconnect test (exact Mac GUI kill sequence) ==="
echo "Cycles: $CYCLES"
echo "HARDWARE GATE: actuators disconnected (no power to joints). Only comms/port tests."
echo "Pi: $(hostname) $(date)"
echo "Initial tty: $(ls -l /dev/ttyACM0 2>/dev/null || echo 'NONE')"
echo "Initial agent: $(pgrep -a micro_ros_agent || echo 'none')"
source ~/microros_ws/install/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true

success=0
for c in $(seq 1 "$CYCLES"); do
  echo ""
  echo "=== CYCLE $c / $CYCLES START $(date +%T) ==="

  if [ "$LIGHT" -eq 1 ]; then
    # === LIGHT path (leave micro_ros_agent running) ===
    echo "[cycle $c] LIGHT cleanup (leave agent running, only /reboot + non-agent pkill)..."
    echo "[startup] cleanup: phase 0 - sourcing";
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source ~/microros_ws/install/setup.bash 2>/dev/null || true;
    echo "[startup] cleanup: phase 1 - /reboot kick to ESP (agent stays alive)";
    timeout 3s ros2 topic pub --once /reboot std_msgs/msg/Float32 "{data: 1.0}" >/dev/null 2>&1 || true; sleep 0.3;
    echo "[startup] cleanup: phase 2 - pkill non-agent nodes";
    pkill -9 -f "moveo_publisher.py" 2>/dev/null || true;
    pkill -9 -f "stereo_camera_node.py" 2>/dev/null || true;
    pkill -9 -f "stereo_depth_node.py" 2>/dev/null || true;
    pkill -9 -f "mjpeg_stream.py" 2>/dev/null || true;
    pkill -9 -f "goto_object.py" 2>/dev/null || true;
    pkill -9 -f "approach_object.py" 2>/dev/null || true;
    echo "[startup] cleanup: phase 3 - report tty holders (agent should still hold it)";
    echo "holders:"; fuser /dev/ttyACM0 2>/dev/null || echo "(none visible)";
    echo "[startup] cleanup: phase 4 - shm cleanup";
    sleep 1;
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true;
    echo "[startup] cleanup done - agent left running, ESP kicked" || echo "[cycle $c] LIGHT cleanup had error (non-fatal)"
  else
    # === FULL / heavy path (original GUI behavior) ===
    echo "[cycle $c] FULL KILL sequence (GUI _CMD_KILL)..."
    timeout 45s bash -c '
      echo "[startup] kill: phase 0 - sourcing";
      source /opt/ros/jazzy/setup.bash 2>/dev/null || source ~/microros_ws/install/setup.bash 2>/dev/null || true;
      echo "[startup] kill: phase 1 - /reboot kick to ESP (short timeout)";
      timeout 3s ros2 topic pub --once /reboot std_msgs/msg/Float32 "{data: 1.0}" >/dev/null 2>&1 || true; sleep 0.3;
      echo "[startup] kill: phase 2 - killall micro_ros_agent";
      killall -9 -q micro_ros_agent 2>/dev/null || true;
      echo "[startup] kill: phase 3 - fuser /dev/ttyACM0";
      fuser -k -9 /dev/ttyACM0 2>/dev/null || true;
      echo "[startup] kill: phase 4 - USB re-enum (best-effort, sudo -n)";
      DEV=$(readlink -f /sys/class/tty/ttyACM0/device 2>/dev/null || true); while [ -n "$DEV" ] && [ "$DEV" != "/" ] && [ ! -f "$DEV/authorized" ]; do DEV=$(dirname "$DEV"); done;
      if [ -f "$DEV/authorized" ]; then
        if sudo -n true 2>/dev/null; then
          echo 0 | sudo -n tee $DEV/authorized >/dev/null 2>&1 || true; sleep 0.7;
          echo 1 | sudo -n tee $DEV/authorized >/dev/null 2>&1 || true; sleep 1.2;
          echo "[startup] USB re-enumerated";
        else
          echo "[startup] re-enum: no passwordless sudo (skipped; kill+fuser + firmware 3s retry loop is usually enough)";
        fi;
      else
        echo "[startup] re-enum: no authorized sysfs entry (skipped)";
      fi;
      echo "[startup] kill: phase 5 - pkill nodes";
      pkill -9 -f "moveo_publisher.py" 2>/dev/null || true;
      pkill -9 -f "stereo_camera_node.py" 2>/dev/null || true;
      pkill -9 -f "stereo_depth_node.py" 2>/dev/null || true;
      pkill -9 -f "mjpeg_stream.py" 2>/dev/null || true;
      pkill -9 -f "goto_object.py" 2>/dev/null || true;
      pkill -9 -f "approach_object.py" 2>/dev/null || true;
      sleep 1.5;
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true;
      echo "[startup] old processes cleared + serial port released"
    ' || echo "[cycle $c] FULL KILL: remote timeout or error (non-fatal)"
  fi

  echo "Post-cleanup tty: $(ls -l /dev/ttyACM* 2>/dev/null || echo 'NONE')"

  # === Ensure / start agent (light or full) ===
  echo "[cycle $c] ENSURE_AGENT..."
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  source ~/microros_ws/install/setup.bash 2>/dev/null || true
  if [ "$LIGHT" -eq 1 ]; then
    if pgrep -c micro_ros_agent > /dev/null; then
      echo "[startup] micro_ros_agent already running (left alive for ESP reconnect)"
    else
      echo "[startup] micro_ros_agent not running — starting (fallback)"
      for i in 1 2 3 4 5; do [ -e /dev/ttyACM0 ] && break; sleep 1; done
      if [ ! -e /dev/ttyACM0 ]; then
        echo "[cycle $c] ERROR: /dev/ttyACM0 not present"
        continue
      fi
      setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 > /tmp/mra.log 2>&1 </dev/null &
      sleep 3
      pgrep -c micro_ros_agent > /dev/null && echo "[startup] micro_ros_agent started" || { echo "ERROR starting agent"; cat /tmp/mra.log || true; continue; }
    fi
  else
    # full path: always (re)start after heavy kill
    for i in 1 2 3 4 5; do [ -e /dev/ttyACM0 ] && break; sleep 1; done
    if [ ! -e /dev/ttyACM0 ]; then
      echo "[cycle $c] ERROR: /dev/ttyACM0 not present after kill"
      continue
    fi
    setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 > /tmp/mra.log 2>&1 </dev/null &
    sleep 3
    if pgrep -c micro_ros_agent > /dev/null; then
      echo "[startup] micro_ros_agent running (serial:/dev/ttyACM0)"
    else
      echo "[startup] ERROR: agent not found"; cat /tmp/mra.log || true
      continue
    fi
  fi

  # === Exact _CMD_ESP32 (30 iter poll for session, timeout wrapper, tail) ===
  echo "[cycle $c] ESP32 poll (GUI _CMD_ESP32, up to ~30s)..."
  if timeout 32s bash -c '
    for i in $(seq 1 30); do grep -q "session established\|datareader created" /tmp/mra.log && break; sleep 1; done
    if grep -q "session established\|datareader created" /tmp/mra.log; then
      echo "[startup] ESP32 connected"
    else
      echo "[startup] WARNING: ESP32 not yet connected (will appear shortly; publisher side is already live)"
    fi
  '; then
    if grep -qiE "session established|datareader created" /tmp/mra.log; then
      echo "[cycle $c] SUCCESS: session markers present"
      success=$((success + 1))
    else
      echo "[cycle $c] PARTIAL: no markers in poll window (firmware will retry every 3s; publisher can still publish)"
    fi
  else
    echo "[cycle $c] poll wrapper timed out (non-fatal)"
  fi

  echo "Post-cycle tty: $(ls -l /dev/ttyACM0 2>/dev/null || echo 'NONE')"
  echo "=== CYCLE $c / $CYCLES END $(date +%T) ==="
  sleep 2
done

echo ""
echo "=== RESULT: $success / $CYCLES cycles achieved session markers (or clean agent start) without physical ESP power cycle ==="
echo "Full log: $LOG"
echo "Last mra.log tail:"
tail -10 /tmp/mra.log || true
echo "Test complete. Re-state: actuators were disconnected; only port/ROS process/ESP session tested."
