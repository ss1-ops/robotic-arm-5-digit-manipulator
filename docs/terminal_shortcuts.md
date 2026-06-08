# Terminal Shortcuts — Robotic Arm

## Push & Flash Sketch to ESP32 via Pi (no USB required)
Copies the sketch from Mac to Pi, then compiles and flashes over the Pi's USB connection to the ESP32. Automatically stops the micro-ROS agent, triggers bootloader mode via GPIO, flashes, resets, and restarts the agent.
```bash
scp -r '/Users/sam/Library/CloudStorage/Dropbox/SAM SNYDER/SAM/BUSINESS/Clients/Robotic Arm/Code/esp32s3_arm_controller' armpi@armpi.local:~/sketches/ && ssh armpi@armpi.local 'bash ~/flash_esp32.sh'
```

## Reset ESP32 via Pi GPIO (without flashing)
Pulses the EN (RST) pin low via GPIO17 to hard-reset the ESP32 into normal run mode.
Requires Pi GPIO17→ESP32 EN and GPIO27→ESP32 IO0 wired with 330Ω resistors.
```bash
ssh armpi@armpi.local 'bash ~/esp32_control.sh reset'
```

## Watch UDP Debug Output from ESP32
Listens for debug messages sent by the ESP32 over WiFi UDP. Run on Pi before/during a flash.
```bash
nc -ul 9999
```

## SSH to Pi
```bash
ssh armpi@armpi.local
```

## Start micro-ROS Agent (Pi) — WiFi UDP
```bash
cd ~/microros_ws && source install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Check /joint_commands Topic
```bash
source /opt/ros/jazzy/setup.bash && ros2 topic info /joint_commands
```

## Send Test Joint Command (Joint 1 to 0.5 rad ≈ 28.6°)
```bash
source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /manual_joint_command std_msgs/msg/String "data: '[0.5, 0.0, 0.0, 0.0, 0.0]'"
```

## Foxglove (visualization)
Foxglove bridge is running via systemd on port 8765 and exposing all topics (/joint_states, /joint_commands, stereo cameras, /ee_target, etc).

On your Mac (Foxglove Studio desktop app from foxglove.dev):
- Open connection → WebSocket → `ws://armpi.local:8765` (or `ws://192.168.1.142:8765`)
- Add **3D** panel: set "URDF" to topic `/robot_description`; it will drive from `/joint_states` (Joint_1..Joint_5 names).
- Add **Image** panels for `/stereo/left/image_raw` and right.
- Add **Raw Messages** or **Plot** for joints/commands.

Live `/joint_states` (for viz) is published by `moveo_publisher` (timer + mirrors external `/joint_commands` pubs and TCP cmds). The `foxglove_ee_to_joint_states` node (if `/ee_target` published) uses MoveIt IK for alternative EE-driven state.

Optional (for IK/EE target path in Foxglove):
```bash
ssh armpi@armpi.local 'source /opt/ros/jazzy/setup.bash && source ~/moveo_ws/install/setup.bash && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && ros2 launch moveo_moveit_config moveit.launch.py'
```
(Starts `/move_group` providing `/compute_ik`; the ee node will then react to `/ee_target`.)

To start rsp (for /tf if other nodes need it):
```bash
ssh armpi@armpi.local 'source /opt/ros/jazzy/setup.bash && source ~/moveo_ws/install/setup.bash && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && ros2 launch moveo_description moveo_description.launch.py'
```
(Note: also starts controller_manager bits; harmless for viz but check logs.)

See also: `~/ros_nodes/foxglove_ee_to_joint_states.py`, moveo_publisher.py (now publishes + mirrors state), and the moveo_moveit_config / moveo_description packages in ~/moveo_ws/src.
