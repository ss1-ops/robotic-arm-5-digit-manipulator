## Manus glove teleoperation

This launch file uses hand pose captured by a [Manus glove](https://www.manus-meta.com/) to control the Aero hand.


### Hardware requirements

- A [Manus glove](https://www.manus-meta.com/) connected to the computer
- The Manus license USB (required for raw data access)
- An Aero hand connected to the computer


### Run

1. Connect the Manus glove to the computer (USB cable or Bluetooth connector).
   - Refer to the Manus documentation for setup details.
   - Make sure the license USB is present so raw data can be accessed.

2. Grant USB permissions for the Manus glove by installing the provided `udev` rule:
   ```bash
   sudo cp <path/to/manus_glove_pkg>/70-manus-hid.rules /etc/udev/rules.d/
   ```

3. Reload the udev rules:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

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
ros2 launch src/launch_files/manus_teleop_launch/manus_teleop.launch.py
```

### Common overrides

```bash
# Set right hand port explicitly (and disable left hand)
ros2 launch src/launch_files/manus_teleop_launch/manus_teleop.launch.py right_hand_port:=/dev/ttyUSB0 left_hand_port:=""
```