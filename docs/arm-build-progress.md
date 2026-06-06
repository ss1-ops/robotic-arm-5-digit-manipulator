# Arm Build Progress & Live Inventory

**Status (as of 2026):** Transitioning primary control from Arduino Uno R4 to ESP32-S3 micro-ROS. Basic joint control, IK, and manual GUI working. Vision (stereo + eye-in-hand) bring-up and full integration in progress. Aero Hand partially assembled (multiple fingers functional).

This document captures pinouts, ROS 2 launch commands, known issues, and the current state of the armpi (Raspberry Pi running the ROS 2 Jazzy stack). For the absolute latest, SSH to armpi or consult the linked Obsidian notes.

## High-Level Architecture
- Raspberry Pi (armpi.local) runs ROS 2 Jazzy + MoveIt2 for planning.
- ESP32-S3 (micro-ROS) handles low-level stepper/servo control, quadrature feedback, OTA.
- Vision on Pi (planned eye-in-hand stereo).
- Mac GUI for manual testing and sequences.

See the root README for overall project goals and sub-projects (especially the full Aero Hand Open under gripper/).

## Current Capabilities
- ROS 2 Jazzy on armpi
- Joint control via forward_position_controller + serial bridge
- Smooth IK + coordinated motion via MoveIt2
- Manual Mac GUI (joint sliders, sequences)
- Stereo vision pipeline in development (calibration, depth, object approach)
- Aero Hand gripper (tendon-driven, partial assembly + control)

## Key References
- Main arm controller: firmware/esp32s3-arm-controller/
- Mac GUI / ROS nodes: software/mac-gui/
- Aero Hand full project: gripper/aero-hand-open/ (own ROS2 packages, firmware, hardware, SDK, RL tools)
- Original BCN3D reference: reference/bcn3d-moveo/

## Live Armpi Notes (example from inventory)
- SSH: armpi@armpi.local (key auth)
- Workspaces: ~/moveo_ws, ~/microros_ws
- Source: /opt/ros/jazzy/setup.bash
- ELP stereo cam: /dev/video0
- micro-ROS agent typically on serial to ESP32-S3

For exact current topics, launch files, pin table, and troubleshooting, see the Obsidian vault notes (ROS2 Setup (armpi), notes/micro-ros-esp32, daily logs) or contact for live access.

## Roadmap Items (in progress)
- CV bring-up: ELP dual-lens stereo driver, checkerboard calibration, hand-eye calibration for eye-in-hand mount
- "Point at object" demo using detection + distance-gated approach
- Full integration of Aero Hand (7 DoF tendon) with main arm
- Homing, OTA improvements, more I/O (PCA9685)

## Decision Log Highlights
- Migrated to ESP32-S3 for native micro-ROS, more GPIO, OTA.
- URDF fixes for shoulder, firmware DIR inversion.
- Eye-in-hand mount preferred over fixed overhead for the target pick-and-place accuracy (5–10 mm).

For the most up-to-date wiring, commands, and issues, run exploration on the live armpi or refer to the full project notes.
