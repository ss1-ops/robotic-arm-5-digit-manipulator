# BCN3D Moveo Robotic Arm + Aero Hand

Client-commissioned 6-DOF robotic arm (BCN3D Moveo base) with custom Aero Hand tendon-driven gripper. End goal: autonomous pick-and-place using computer vision on a Raspberry Pi running ROS 2 Jazzy, with low-level control on ESP32-S3 (micro-ROS).

**Status:** ACTIVE — transitioning from Arduino Uno R4 to ESP32-S3. Basic joint control, IK, and manual GUI working. Vision bring-up and full integration in progress. Aero Hand partially assembled (multiple fingers functional).

**Featured on [Sam Snyder's portfolio](https://ss1-ops.github.io#projects).**

## Quick Links
- [Current Build Status & Architecture](docs/arm-build-progress.md)
- [Aero Hand Gripper](gripper/aero-hand-open/) — full open-source tendon-driven hand (7 DoF, 3D-printed, ROS 2 + SDK + firmware + RL)
- [Firmware (ESP32-S3)](firmware/esp32s3-arm-controller/)
- [Software / ROS 2 + GUI + Vision](software/)
- [Hardware / CAD](hardware/)
- [Original BCN3D Reference](reference/bcn3d-moveo/) (attribution + source files)

## Project Structure
```
.
├── README.md
├── LICENSE
├── docs/
│   └── arm-build-progress.md          # Pinouts, commands, issues, roadmap, live inventory
├── hardware/
│   ├── cad/                           # Custom + reference CAD (STEP, STL, etc.)
│   └── enclosure/                     # Custom platform/enclosure designs
├── firmware/
│   ├── esp32s3-arm-controller/        # Active micro-ROS target firmware
│   └── legacy-tests/                  # Earlier Arduino sketches
├── software/
│   ├── mac-gui/                       # macOS desktop GUI + ROS 2 nodes for manual control
│   ├── ros/                           # ROS 2 nodes, publishers
│   └── vision/                        # Stereo, hand-eye, IBVS, object approach (in progress)
├── gripper/
│   └── aero-hand-open/                # Full Aero Hand Open (tendon-driven EE) — complete sub-project with hardware, firmware, ROS2, SDK, RL
├── reference/
│   └── bcn3d-moveo/                   # Original open-source BCN3D Moveo mechanical files (attribution)
├── urdf/                              # Robot description
└── assets/                            # Footage, images, demos (to be populated)
```

## Current Capabilities
- ROS 2 Jazzy on Raspberry Pi (armpi.local)
- Joint control via forward_position_controller + serial bridge to stepper drivers
- Smooth IK + coordinated motion
- Manual Mac GUI for testing (joint sliders, sequences)
- Stereo vision pipeline (calibration, depth, object approach) — in active bring-up
- Aero Hand gripper (tendon-driven, partial assembly + control)

See `docs/arm-build-progress.md` for pinouts, exact launch commands, known issues, and detailed roadmap (ESP32-S3 migration, PCA9685 for gripper, eye-in-hand CV, homing, etc.).

## Getting Started (High Level)
1. On the Pi: source ROS 2, launch description + controller.
2. Run serial bridge or micro-ROS agent.
3. Use Mac GUI or `ros2 topic pub` for commands.
4. For gripper: see `gripper/aero-hand-open/` (has its own SDK, ROS2 packages, firmware examples, CAD, PCB).

Full live setup details are in the build progress doc and the armpi SSH inventory.

## Sub-Projects & Highlights
- **Aero Hand Open** (`gripper/aero-hand-open/`): Professional open-source 7-DoF tendon-driven hand. Includes hardware (CAD, PCB, BOM, molds), firmware (ESP32), Python SDK, ROS 2 integration (multiple packages for retargeting, RL, description), simulation. See its internal README for features, assembly, and shop links.
- **Vision & CV**: Hand-eye calibration, IBVS servo, stereo depth for pick-and-place. Eye-in-hand mount planned (5–10 mm target accuracy).
- **Firmware Evolution**: ESP32-S3 for native micro-ROS, OTA, more I/O (PCA9685 for servos, IMUs, quadrature feedback).

## License
MIT (see LICENSE). Sub-components (Aero Hand, original BCN3D) have their own licenses.

## Attribution
Base arm mechanics from the excellent open-source [BCN3D Moveo](https://github.com/BCN3D/BCN3D-Moveo) project. Custom work by Sam Snyder (automation, controls, integration, Aero Hand adaptation, vision).

For the absolute latest status, wiring, or Pi inventory, see the linked Obsidian notes or run live exploration on `armpi`.

---

**Active project** — contributions and questions welcome via issues. Deeper demos, live arm access, and private materials available on request.
