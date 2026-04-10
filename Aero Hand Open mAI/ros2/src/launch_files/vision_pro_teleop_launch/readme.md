## Apple Vision Pro teleoperation

This launch file uses hand pose captured by an Apple Vision Pro (via the Tracking Streamer app) to control the Aero hand.


### Hardware requirements

- An Apple Vision Pro running the [Tracking Streamer app](https://apps.apple.com/us/app/tracking-streamer/id6478969032)
- An Aero hand connected to the computer


### Software requirements

- `avp-stream`





### Run

1. Make sure the Apple Vision Pro and the computer are connected to the same Wi‑Fi network.


2. Find the IPv4 address of the Apple Vision Pro:
   - On Vision Pro: Settings → Wi‑Fi → (your network) → details → look for **IPv4 / IP Address**
   - Example: `192.168.1.101`



3. Launch the Tracking Streamer app on the Apple Vision Pro.
   - Note: if the headset is removed, the app may stop and tracking will be lost.

4. Connect the Aero hand to the computer.
   - If only one right hand is connected, the port can usually be auto-detected.
   - Otherwise, set the port explicitly (see overrides below).

5. Build and source:

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

6. Launch:

```bash
ros2 launch src/launch_files/vision_pro_teleop_launch/vision_pro_teleop.launch.py vision_pro_ip:=192.168.1.101
```

### Common overrides

```bash
# Disable visualization
ros2 launch src/launch_files/vision_pro_teleop_launch/vision_pro_teleop.launch.py vision_pro_viz:=false

# Set right hand port explicitly (and disable left hand)
ros2 launch src/launch_files/vision_pro_teleop_launch/vision_pro_teleop.launch.py right_hand_port:=/dev/ttyUSB0 left_hand_port:=""
```