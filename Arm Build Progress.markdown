# BCN3D MOVEO ROS 2 Project - Current State (April 2026)

## Hardware
- **Raspberry Pi**: Ubuntu 24.04.4 LTS (arm64), ROS 2 Jazzy
  - Stable IP: `192.168.10.172` (static via Netplan)
  - mDNS: `armpi.local`
- **Arduino Uno R4**: Connected via USB (`/dev/ttyACM0` or `/dev/ttyACM1`)
  - Current sketch: Relative displacement stepper control
- **Future**: ESP32-S3 (castellated module) – will replace Uno R4 with micro-ROS + OTA + PCA9685 for gripper

## ROS 2 Stack on Pi

**Workspace**: `~/moveo_ws`

### Core Packages
- `moveo_description` – URDF + robot model + ros2_control config
  - Clean URDF: `moveo_clean.urdf`
  - Hardware interface: `mock_components/GenericSystem`
  - Controllers: `joint_state_broadcaster` + `forward_position_controller`

- `moveo_serial_bridge` – Python node that forwards joint commands to Arduino via serial

## Pinout Table (Current Arduino Uno R4)

| Axis              | Joint Name       | Step Pin | Dir Pin | Status      | Notes |
|-------------------|------------------|----------|---------|-------------|-------|
| Shoulder          | Joint_1          | 5        | 4       | Wired       | Active – **running in reverse** |
| Elbow             | Joint_3          | 7        | 6       | Wired       | Active |
| Wrist             | Joint_5          | 9        | 8       | Wired       | Active |
| Waist (future)    | Joint_0          | -        | -       | Not wired   | Planned |
| Wrist Roll (future) | Joint_4        | -        | -       | Not wired   | Planned |
| Wrist Pitch (future) | Joint_?        | -        | -       | Not wired   | Planned |

## Command Reference Table

| Purpose                        | Command |
|--------------------------------|---------|
| Launch robot description + controller manager | `ros2 launch moveo_description moveo_description.launch.py` |
| Run serial bridge to Arduino   | `ros2 run moveo_serial_bridge serial_bridge` |
| Send single relative movement (one-shot) | `ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.5, 0.0, 0.0]}"` |
| Run movement sequence script   | `./move_sequence.sh` |
| Rebuild workspace              | `colcon build --symlink-install` |
| Source workspace               | `source install/local_setup.bash` |
| Monitor Arduino serial output  | `screen /dev/ttyACM0 57600` (exit: Ctrl+A then K then Y) |

## Live Terminal Visualization

While the system is running, use these commands in separate terminals for real-time joint state monitoring:

```bash
# Live updating joint states (recommended)
watch -n 0.5 "ros2 topic echo /joint_states --once"

# Simple one-time echo
ros2 topic echo /joint_states --once
```

## Current Functionality

- ROS 2 Jazzy running on Ubuntu 24.04 arm64
- Robot description loads correctly
- Controller manager + hardware interface active
- Serial bridge communicates with Arduino Uno R4
- Arduino receives **relative joint displacements** (cumulative)
- Basic multi-joint movement sequence script available (`move_sequence.sh`)

## Known Issues

- Shoulder joint is currently running in **reverse** → sign of movement commands for Joint_1 needs to be multiplied by -1 in the ESP32 code
- No limit switches or encoders on the arm → calibration / homing must be done manually
- Homing (setting “0” reference points for all joints) will be handled on the **Pi side** (ROS 2 level)
- Arduino Uno R4 has RAM limitations with full micro-ROS → migrating to ESP32-S3
- Movement is currently **relative** (cumulative displacement)
- No absolute positioning or advanced trajectory smoothing yet

## Aero Hand Integration (Planned Manipulator)

**Decision**: Use **PCA9685 I2C 16-channel servo driver** board to control the 7 servos of the Aero Hand.

**Notes**:
- The Aero Hand uses **7 servos** (not 5 independent fingers — it is a more advanced but still underactuated design).
- PCA9685 will be mounted **inside the palm housing** (Palm Rear Frame / Servo Frame) to keep wiring clean and minimize cable length along the arm.
- Power the servos from a **separate 5V–6V supply** (do not power from ESP32 or Pi).
- I2C bus (SDA + SCL) will run from the ESP32-S3 up the arm to the PCA9685.
- Short servo signal wires will connect from the PCA9685 directly to each servo.
- In ROS 2, the gripper will be added as additional joint(s) in the URDF and ros2_control configuration.

**Size & Weight**: PCA9685 module is approximately 62×25×15 mm and weighs only 4–6 grams — easily fits inside the Aero Hand housing.

## IMU for Vibration Sensing (Future Addition)

**Purpose**: Add one or more IMUs to detect vibrations caused by the stepper motors. This data can be used for adaptive speed control, fault detection, or tuning stepper parameters.

**Recommended Models** (available on Amazon):
- **MPU6050** — Cheapest and most popular (~$2–4). Good enough for basic vibration detection (accelerometer + gyroscope).
- **BMI270** or **ICM-20948** — Slightly higher quality and better noise performance (~$5–8). Recommended if you want cleaner data.

**Placement suggestion**: One near the base and one near the wrist/end-effector for best coverage.

**Integration Notes**:
- IMUs use I2C → can share the same bus as the PCA9685 (only 2 wires needed).
- Data will be published as `sensor_msgs/Imu` or a custom vibration topic from the ESP32-S3.
- Processing and monitoring will happen on the Raspberry Pi.
- Difficulty: Easy to Moderate (adds minimal overhead).

**Additional Capability (Future)**: IMUs could also assist with:
- Detecting skipped steps (by comparing commanded vs. measured motion)
- Coarse automatic homing (as a backup to limit switches)
- Soft closed-loop angle correction

## Camera Setup for Computer Vision (CV)

**Recommended Strategy: Hybrid Approach**

- **Primary Camera: Fixed (Overhead or Side-mounted)**
  - Mount a good USB camera (e.g. Logitech C920/C922) or Pi Camera on a tripod/stand ~50–80 cm above the workspace, pointing down.
  - Best for: Object detection, global positioning, coarse arm planning.
  - Advantages: Stable calibration, wide field of view, less occlusion.

- **Secondary Camera (Optional): Eye-in-Hand**
  - Small lightweight camera mounted near the gripper (on the Aero Hand).
  - Best for: Fine alignment just before grasping, grasp verification.
  - Note: Requires hand-eye calibration and is more complex.

**Vision Pipeline Location**:  
The entire CV stack (camera driver, object detection, pose estimation, pick-and-place logic) will run on the **Raspberry Pi**, not on the ESP32-S3.

## Future Architecture

### Planned Final System (ESP32-S3 + Pi)

- **ESP32-S3** (Low-level real-time layer)
  - Runs micro-ROS firmware over USB (initially) for lowest latency
  - Direct GPIO control of 5 stepper motors (STEP + DIR)
  - I2C connection to PCA9685 for the 7 Aero Hand servos
  - I2C support for IMU(s) for vibration sensing
  - ArduinoOTA for wireless code updates
  - Possible future WiFi transport

- **Raspberry Pi** (High-level intelligence layer)
  - Runs full ROS 2 Jazzy stack
  - MoveIt 2 for motion planning and trajectories
  - **Camera node + OpenCV / vision pipeline** for pick-and-place
  - High-level task planning and sequencing
  - Sends target joint + gripper commands to ESP32-S3 via micro-ROS
  - Handles manual homing / zero-point calibration
  - Monitors IMU vibration data

**Communication**:
- Initial: USB Serial (micro-ROS) for lowest latency
- Future: WiFi (micro-ROS over UDP) for cable-free operation

**Data Flow**:
Pi (MoveIt 2 + Vision) → micro-ROS commands → ESP32-S3 → Stepper drivers + PCA9685 + IMU(s) → Physical MOVEO arm + Aero Hand

## Future Additions (Optional)

- IMU(s) for vibration sensing, skipped step detection, and coarse angle feedback
- Limit switches or endstops for reliable homing
- Encoders on joints for true closed-loop control
- Eye-in-hand camera for fine grasping
- Wireless (WiFi) micro-ROS transport instead of USB
- Improved non-blocking stepper control with acceleration profiles
- **Conversion of URDF to Xacro** — recommended for easier maintenance and adding the Aero Hand / IMUs later
