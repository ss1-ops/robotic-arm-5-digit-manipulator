# micro-ROS Troubleshooting Log

## System Overview

```
Mac GUI (Python/Tkinter)
  └─ /manual_joint_command (String JSON) ──► manual_joint_controller_node.py (Pi)
                                                  └─ /joint_commands (JointState, RELIABLE)
                                                       └─ micro-ROS agent (Pi, udp4:8888)
                                                            └─ WiFi UDP ──► ESP32-S3
                                                                               └─ 5x stepper motors
```

**Key versions:**
- ROS2: Jazzy (Pi, Ubuntu 24.04 aarch64)
- micro-ROS agent: `5.0.2` (Jazzy)
- micro_ros_arduino library: `3.0.0-iron` (Iron distro) ← **version mismatch with agent**
- ESP32-S3, Arduino CLI v1.4.1, esp32:esp32 core 3.3.8
- FQBN: `esp32:esp32:esp32s3:CDCOnBoot=cdc`

---

## Issues Encountered and Resolutions

---

### ✅ FIXED — USB CDC: Serial routed to UART0 instead of USB CDC

**Symptom:** ESP32 never connected to micro-ROS agent. Agent log showed no session.

**Cause:** Default FQBN `esp32:esp32:esp32s3` compiles with `CDCOnBoot=default`, which routes `Serial` to UART0 (the hardware UART, not the USB CDC port). `set_microros_transports()` uses `Serial`, so micro-ROS traffic was going nowhere.

**Fix:** Compile and upload with `CDCOnBoot=cdc`:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc ...
arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc ...
```

---

### ✅ FIXED — `pkill -f micro_ros_agent` kills SSH session

**Symptom:** SSH session dropped when trying to kill the agent before upload.

**Cause:** `pkill -f` pattern-matches command arguments. The SSH process command contains the command string it's running, which includes "micro_ros_agent", so pkill kills the SSH session itself.

**Fix:** Use `killall -q micro_ros_agent` instead — matches exact process name only.

---

### ✅ FIXED — `position.size` stays 0 after deserialization (micro-ROS Iron bug)

**Symptom:** Callback called but `target_pos` never updated. `msg->position.size` was always 0 despite data in the array.

**Cause:** The `micro_ros_arduino 3.0.0-iron` library fills `position.data[]` on receive but does not update `position.size`. This is a known bug in the Iron-era library.

**Fix:** Use `capacity` as fallback in the callback:
```cpp
size_t n = msg->position.size > 0 ? msg->position.size : msg->position.capacity;
```

---

### ✅ FIXED — ESP32 subscribed to wrong topic (`/joint_states` instead of `/joint_commands`)

**Symptom:** Data flowing to ESP32 at 10Hz (confirmed via v6 agent log), but target angles were slowly drifting simulation values — not GUI commands.

**Cause:** Firmware initially subscribed to `/joint_states`, which is published by `joint_state_broadcaster` at 50Hz and reflects the ROS2 simulation state (not the GUI commands).

**Context:**
- `/joint_states` — published by `joint_state_broadcaster`, contains simulation feedback
- `/joint_commands` — published by `manual_joint_controller_node.py`, contains GUI commands

**Fix:** Changed subscription topic in `uros_init()`:
```cpp
"/joint_commands"  // was "/joint_states"
```

---

### ✅ FIXED — UDP: ESP32 doesn't detect agent restart / silent disconnect

**Symptom:** After killing and restarting the agent, ESP32 never reconnected. Agent log showed only 2 lines (init + verbose_level). `rclc_executor_spin_some` returns OK even when the UDP agent is gone.

**Cause:** Unlike serial (where a disconnect produces read errors), UDP is stateless at the OS level. `spin_some` never errors out. The ESP32 kept "thinking" it was connected to the old session.

**Fix:** Added periodic ping in `loop()`:
```cpp
if (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
    uros_cleanup();
    uros_ok = false;
}
```
Also call `set_microros_wifi_transports()` again on each reconnect attempt (UDP requires re-init of the transport socket).

**Note:** The correct function name in Iron is `rmw_uros_ping_agent` — NOT `rmw_uros_ping_agent_ms` (which is Jazzy+).

---

### ✅ SWITCHED — Moved from USB CDC serial to WiFi UDP transport

**Reason:** USB CDC had persistent connection issues — timing-dependent reconnect, serial port reenumeration after upload, and the CDCOnBoot fuse requirement. WiFi UDP is cleaner: no cable dependency, ElegantOTA already requires WiFi.

**Agent command changed from:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/moveo_arduino -b 1000000
```
**To:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Firmware change:** `set_microros_transports()` → `set_microros_wifi_transports(ssid, password, agent_ip, port)`

---

### ❌ UNRESOLVED — Motors do not move despite confirmed connection

**Symptom:** Agent log confirms session established, participant/topic/subscriber/datareader created successfully. `write_topic` count = 0 in verbose (-v6) agent log even after publishing to `/joint_commands`.

**What is confirmed working:**
- ESP32 connects to agent (session established, datareader created)
- Hardware confirmed OK via step-test sketch (direct GPIO pulsing moved motors)
- ESP32 WiFi IP: `192.168.1.252` connecting to Pi `192.168.1.142:8888`

**Root cause identified: QoS mismatch**

The ESP32 subscribes using `rclc_subscription_init_default`, which uses **RELIABLE** QoS. All test commands used `--qos-reliability best_effort`:

```bash
ros2 topic pub --qos-reliability best_effort /joint_commands ...
```

In DDS, a **BEST_EFFORT publisher cannot satisfy a RELIABLE subscriber**. They don't match → the agent's datareader never receives the messages → `write_topic = 0`.

**The GUI path (`manual_joint_controller_node.py`) uses RELIABLE** (default queue depth=10). So GUI-driven commands should actually match the RELIABLE subscription — but the GUI was not tested while both fixes (capacity fallback + correct topic) were simultaneously active.

**What to try next:**

1. **Test with GUI running** — ensure `manual_joint_controller_node.py` is running on Pi and send a command via the Mac GUI. This is RELIABLE→RELIABLE and should match.

2. **OR change test command** — remove `--qos-reliability best_effort`:
   ```bash
   ros2 topic pub -r 10 -t 30 /joint_commands sensor_msgs/msg/JointState \
     "{name: ['j1','j2','j3','j4','j5'], position: [0.0,0.5,0.0,0.0,0.0], velocity: [], effort: []}"
   ```

3. **OR switch to BEST_EFFORT subscription** in firmware (matches both GUI and test pubs if GUI is also changed):
   ```cpp
   RCCHECK(rclc_subscription_init_best_effort(
     &subscriber, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
     "/joint_commands"));
   ```

4. **Verify GUI is publishing** — confirm `manual_joint_controller_node.py` is actually running and publishing:
   ```bash
   ssh armpi@armpi.local 'source ~/microros_ws/install/setup.bash && \
     ros2 topic hz /joint_commands'
   ```

5. **Check step interval math** — if `steps_per_rad` is very large, `step_interval_us` is very small and `micros()` wraparound or timing jitter could cause steps to be skipped. Verify with Serial debug output.

---

## Agent Quick-Start Reference

```bash
# Start UDP agent (keep running — don't kill between tests)
ssh armpi@armpi.local 'source ~/microros_ws/install/setup.bash && \
  setsid ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 \
  > /tmp/agent.log 2>&1 &'

# Check agent log
ssh armpi@armpi.local 'cat /tmp/agent.log'

# Verify ESP32 connected (look for "session established" + "datareader created")
# Successful connection shows 4 lines: participant, topic, subscriber, datareader

# Send test command (RELIABLE — no --qos-reliability flag)
ssh armpi@armpi.local 'source ~/microros_ws/install/setup.bash && \
  ros2 topic pub -r 10 -t 30 /joint_commands sensor_msgs/msg/JointState \
  "{name: [\"j1\",\"j2\",\"j3\",\"j4\",\"j5\"], \
    position: [0.0, 0.5, 0.0, 0.0, 0.0], velocity: [], effort: []}"'

# Compile + flash (kill agent first, restart after)
ssh armpi@armpi.local 'export PATH="$HOME/.local/bin:$PATH" && \
  killall -q micro_ros_agent 2>/dev/null; sleep 5 && \
  arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc \
    --libraries ~/Arduino/libraries ~/sketches/esp32s3_arm_controller && \
  arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc \
    --port /dev/ttyACM0 ~/sketches/esp32s3_arm_controller'
```

---

## Known Limitations / Risks

- **Version mismatch:** micro_ros_arduino 3.0.0-iron client vs 5.0.2-Jazzy agent. Connection works but some API differences exist (e.g. `rmw_uros_ping_agent` vs `rmw_uros_ping_agent_ms`). If further issues arise, consider building a Jazzy-compatible micro_ros_arduino library.
- **Iron `position.size` bug:** Capacity fallback in callback is permanent workaround — do not rely on `size` field.
- **UDP fragmentation:** Large JointState messages (5 joints, with names) may exceed MTU on some networks. If intermittent drops occur, shorten joint names or switch to a custom message type with just a float array.
