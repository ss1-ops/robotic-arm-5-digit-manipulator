#!/usr/bin/env python3
"""
serial_test.py  —  Diagnostic test for moveo_serial_test.ino
Run on armpi after killing micro_ros_agent and flashing the diagnostic sketch.

Usage:
    sudo systemctl stop micro-ros-agent 2>/dev/null; pkill -f micro_ros_agent; sleep 1
    python3 ~/ros_nodes/serial_test.py [port] [baud]

Defaults: port=/dev/moveo_arduino  baud=115200
"""

import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/moveo_arduino'
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

print(f"Opening {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)

time.sleep(2)  # wait for ESP32 reset after serial open

# Drain startup messages
print("--- ESP32 startup ---")
deadline = time.time() + 3
while time.time() < deadline:
    line = ser.readline()
    if line:
        print(line.decode(errors='replace').rstrip())

# --- Test sequence ---
tests = [
    ("[1] Joint_1 +0.5 rad",      "0.5,0.0,0.0,0.0,0.0"),
    ("[2] Joint_1 -0.5 rad (home)", "0.0,0.0,0.0,0.0,0.0"),
    ("[3] All joints spread",      "0.3,-0.4,0.5,-0.3,0.2"),
    ("[4] Home",                   "0.0,0.0,0.0,0.0,0.0"),
]

for label, cmd in tests:
    print(f"\n=== {label} ===")
    print(f">>> {cmd}")
    ser.write((cmd + '\n').encode())
    ser.flush()

    # Collect responses for 4 seconds, print status lines
    deadline = time.time() + 4
    while time.time() < deadline:
        line = ser.readline()
        if line:
            text = line.decode(errors='replace').rstrip()
            print(text)

print("\n--- Done ---")
ser.close()
