# Moveo Mac GUI

Desktop GUI for manual joint testing during the Moveo validation phase.

## Files

- `moveo_joint_controller.py`: Mac GUI application
- `manual_joint_controller_node.py`: ROS 2 helper node to run on `armpi`
- `install_dependencies.sh`: installs `PyQt5` and `paramiko`
- `launch_joint_controller.sh`: launches the GUI and installs missing packages if needed
- `setup_manual_controller_on_pi.sh`: creates the `manual-joint-controller.service` systemd unit on `armpi`
- `requirements.txt`: Python dependencies for the Mac GUI

## Mac Setup

From this directory:

```bash
bash install_dependencies.sh
```

Then launch the GUI:

```bash
bash launch_joint_controller.sh
```

Or directly:

```bash
python3 moveo_joint_controller.py
```

## Pi Setup

Copy the helper node and setup script to `armpi`:

```bash
scp manual_joint_controller_node.py setup_manual_controller_on_pi.sh armpi@armpi.local:~
```

SSH in and run:

```bash
mkdir -p ~/ros_nodes
mv ~/manual_joint_controller_node.py ~/ros_nodes/
bash ~/setup_manual_controller_on_pi.sh
```

The setup script creates and enables `manual-joint-controller.service`.

## Using the GUI

1. Click `Connect SSH`
2. Enter the SSH password for `armpi`
3. Use sliders for live joint updates
4. Use text boxes plus `Set Joint_X` for manual exact values
5. Use `E-STOP` to hold the arm at its current measured position

## Notes

- Joint commands are published to `/manual_joint_command`
- The helper node republishes those as `/joint_states`
- This is intended for manual testing, not autonomous CV control
- Generated Arduino build artifacts are intentionally ignored from git