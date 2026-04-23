# Robotic Arm

This repository contains the hardware files, firmware, and control tooling for a BCN3D Moveo-style robotic arm build.

## Repository Layout

- `Code/`: active software and firmware work
- `Code/Mac GUI/`: macOS desktop GUI for manual joint testing over SSH and ROS 2
- `Code/esp32s3_arm_controller/`: ESP32-S3 Arduino sketch for the arm controller
- `CAD Enclosure/`: enclosure design files
- `Aero Hand Open mAI/`: hand/end-effector related assets
- `BCN3D MOVEO - A fully OpenSource 3D printed Robot Arm - 1693444 - part 1 of 2/`: source mechanical files
- `BCN3D MOVEO - A fully OpenSource 3D printed Robot Arm - 1693444 - part 2 of 2/`: source mechanical files

## Current Software Pieces

### Mac GUI

The Mac GUI is intended for manual validation before moving further into autonomy or CV-driven pickup.

Key files:

- `Code/Mac GUI/moveo_joint_controller.py`: desktop GUI for joint control
- `Code/Mac GUI/manual_joint_controller_node.py`: ROS 2 helper node for manual joint commands
- `Code/Mac GUI/setup_manual_controller_on_pi.sh`: installs the helper as a service on the Raspberry Pi

See `Code/Mac GUI/README.md` for setup and usage details.

### ESP32-S3 Controller

The active Arduino sketch is:

- `Code/esp32s3_arm_controller/esp32s3_arm_controller.ino`

This is the ESP32-S3 control-side firmware for the arm hardware.

## Recommended Workflow

1. Bring up the Raspberry Pi ROS 2 stack.
2. Use the Mac GUI for manual joint testing and validation.
3. Verify motion limits, repeatability, homing behavior, and emergency stop behavior.
4. Iterate on firmware and ROS integration before adding higher-level perception or autonomy.

## Notes

- Generated build artifacts are ignored via `.gitignore` and should not be committed.
- Some top-level folders contain source CAD, reference assets, or print files rather than active code.
- The repo currently includes both software and mechanical assets, so the root is intentionally broader than a code-only project.