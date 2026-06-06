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
