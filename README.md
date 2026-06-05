# BCN3D Moveo Robotic Arm

Client-commissioned 6-DOF robotic arm (BCN3D Moveo base) with custom Aero Hand tendon-driven gripper. End goal: autonomous pick-and-place using computer vision on a Raspberry Pi running ROS 2 Jazzy, with low-level control on ESP32-S3 (micro-ROS).

**Current status (2026):** Transitioning from Arduino Uno R4 to ESP32-S3. Basic joint control, IK, and manual GUI working. Vision bring-up and full integration in progress. Aero Hand partially assembled (3 fingers functional).

## Quick Links
- [Current Build Status & Architecture](docs/arm-build-progress.md)
- [ROS 2 Setup on armpi (live inventory)](https://github.com/ss1-ops/robotic-arm-5-digit-manipulator/blob/main/docs/arm-build-progress.md) (see also Obsidian notes)
- [Aero Hand Gripper](gripper/aero-hand-open/) — full open-source tendon-driven hand (7 DoF, 3D-printed, ROS 2 + SDK + firmware)
- [Firmware (ESP32-S3)](firmware/esp32s3-arm-controller/)
- [Software / ROS 2 + GUI + Vision](software/)
- [Hardware / CAD](hardware/)
- [Original BCN3D Reference](reference/bcn3d-moveo/) (attribution + source files)

## Project Structure (Cleaned for Maintainability)
```
.
├── README.md
├── LICENSE
├── .gitattributes
├── docs/
│   └── arm-build-progress.md          # Detailed pinouts, commands, issues, future plans
├── hardware/
│   ├── cad/                           # Custom + reference CAD (STEP, STL, F3D, 3MF)
│   └── enclosure/                     # Custom platform/enclosure designs
├── firmware/
│   ├── esp32s3-arm-controller/        # Active micro-ROS target firmware
│   └── legacy-tests/                  # Earlier Arduino sketches
├── software/
│   ├── mac-gui/                       # macOS desktop GUI + ROS 2 nodes for manual control
│   ├── ros/                           # ROS 2 nodes, publishers
│   └── vision/                        # Stereo, hand-eye, IBVS, object approach
├── gripper/
│   └── aero-hand-open/                # Full Aero Hand Open (tendon-driven EE)
├── reference/
│   └── bcn3d-moveo/                   # Original open-source BCN3D Moveo mechanical files (attribution)
├── urdf/                              # Robot description
└── assets/                            # Footage, images
```

## Current Capabilities
- ROS 2 Jazzy on Raspberry Pi (armpi.local)
- Joint control via forward_position_controller + serial bridge to stepper drivers
- Smooth IK + coordinated motion
- Manual Mac GUI for testing (joint sliders, sequences)
- Stereo vision pipeline (calibration, depth, object approach)
- Aero Hand gripper (tendon-driven, 7 servos, partial assembly + control)

See `docs/arm-build-progress.md` for pinouts, exact launch commands, known issues, and detailed roadmap (ESP32-S3 migration, PCA9685 for gripper, eye-in-hand CV, homing, etc.).

## Getting Started (High Level)
1. On the Pi: source ROS 2, launch description + controller.
2. Run serial bridge or micro-ROS agent.
3. Use Mac GUI or `ros2 topic pub` for commands.
4. For gripper: see `gripper/aero-hand-open/` (has its own SDK, ROS2 packages, firmware examples).

Full live setup details are in the build progress doc and the armpi SSH inventory.

## Sub-Projects & Highlights
- **Aero Hand Open** (`gripper/aero-hand-open/`): Professional open-source 7-DoF tendon-driven hand. Includes hardware (CAD, PCB, BOM), firmware, Python SDK, ROS 2 integration, simulation, RL tools. See its README for features, assembly, and shop links.
- **Vision & CV**: Hand-eye calibration, IBVS servo, stereo depth for pick-and-place. Eye-in-hand mount planned.
- **Firmware Evolution**: Moving to ESP32-S3 for native micro-ROS, OTA, more I/O (PCA9685 for servos, IMUs).

## License
MIT (see LICENSE). Sub-components may have their own licenses (e.g., Aero Hand, original BCN3D).

## Attribution
Base arm mechanics from the excellent open-source [BCN3D Moveo](https://github.com/BCN3D/BCN3D-Moveo) project. Custom work by Sam Snyder (automation, controls, integration, Aero Hand adaptation, vision).

For the absolute latest status, wiring, or Pi inventory, see the linked Obsidian notes or run live exploration on `armpi`.

---

**Active project** — contributions and questions welcome via issues or the linked Discord/community channels in the Aero Hand sub-project.