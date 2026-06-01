# BCN3D MOVEO ROS 2 Project - Current State (May 11 2026)

## Hardware
- **Raspberry Pi**: Ubuntu 24.04.4 LTS (arm64), ROS 2 Jazzy
  - Stable IP: `192.168.10.172` (static via Netplan)
  - mDNS: `armpi.local`
- **Arduino Uno R4**: Connected via USB (`/dev/ttyACM0` or `/dev/ttyACM1`)
  - Current sketch: Relative displacement stepper control
- **Aero Hand**: partially constructed; 3 fingers attached and functioning with the Arduino Uno R4 servo test script
  - Remaining hardware: set IDs for 4 remaining servos, attach pinky and thumb
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
- Shoulder direction is now correct
- IK is functional and all movement is smooth
- Aero Hand currently has 3 fingers attached and tested; the hand is partially constructed and working with the Uno R4 servo test flow

## Known Issues

- No limit switches or encoders on the arm → calibration / homing must be done manually
- Homing (setting “0” reference points for all joints) will be handled on the **Pi side** (ROS 2 level)
- Arduino Uno R4 has RAM limitations with full micro-ROS → migrating to ESP32-S3
- Movement is currently **relative** (cumulative displacement)
- No absolute positioning or advanced trajectory smoothing yet
- Hand build currently needs 4 remaining servo IDs assigned and attachments for the pinky and thumb

## Aero Hand Integration (Current Build)

- The Aero Hand is partially assembled and three fingers are mounted and functioning with the current Arduino Uno R4 servo test script.
- The remaining work is to set unique IDs for the four unassigned servos and attach the pinky and thumb.
- The long-term plan remains to move gripper control to a **PCA9685 I2C 16-channel servo driver** board for the 7 Aero Hand servos.

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

**Camera**: ELP Dual Lens USB Camera Module (Mini UVC USB2.0, 960p 60fps synchronized
binocular). Enumerates as a **single UVC `/dev/video*` device** outputting one
**side-by-side synchronized frame** (~2×1280×960); software splits each frame into
left/right halves. At 960p60 over USB2 the stream must be **MJPEG** (YUYV won't fit
USB2 bandwidth). No onboard depth — disparity is computed on the Pi in OpenCV.

**Mount decision (updated): EYE-IN-HAND.**
- Stereo pair mounts on the arm, **just above the hand**. Moves with the wrist.
- Calibration: **checkerboard** for stereo intrinsics/extrinsics, then **hand-eye
  calibration** (`cv2.calibrateHandEye`) to get camera→end-effector transform `T_ee_cam`.
  The arm's known forward kinematics then place the camera in base frame.
- This replaces the earlier fixed-overhead hybrid plan.

**Depth strategy: distance-gated visual approach (no need for precise far-field depth).**
- Small baseline (~6 cm) → stereo 3D is **noisy at distance, that's accepted**.
- Set a **confidence/distance threshold**: beyond it, the 3D estimate is disregarded
  for fine targeting and the arm only uses bearing (point-at-object direction).
- Approach at a **capped, safe speed bounded by the vision processing rate** (arm never
  moves faster than perception can update), and **decelerate as the target enters
  reliable perception range** — depth accuracy improves as the camera closes in, so the
  final approach/grasp uses the now-trustworthy near-field 3D.

**Near-term milestone: "point at object."**
- Detect target → compute bearing (and coarse 3D when in range) → command the arm to
  aim the hand/camera at it via the existing cartesian/trajectory interface. Grasping
  sequence comes after point-at-object is solid.

**Vision Pipeline Location**:
The entire CV stack (camera driver, stereo rectification, detection, depth, hand-eye
transform, approach logic) runs on the **Raspberry Pi**, not the ESP32-S3. It feeds the
existing `moveo_publisher` TCP interface (`armpi.local:9000`, `{"cartesian":[x,y,z]}` /
`{"trajectory":[...]}`) → Pi-side ikpy → `/joint_commands` → micro-ROS → ESP32-S3.

## Hand / Gripper Status (updated)

- **Aero Hand is fully actuated.** All servos wired and driven, **write-only (open-loop,
  no position feedback)**. Adequate for point-at-object and basic open/close grasping now.
- **Planned closed-loop migration:** move servo control to the **ESP32-S3 over a serial
  servo bus** (read/write) for position/load feedback → true closed-loop grasping. Not
  required for the current CV milestones.

## Next Steps

- **CV bring-up (unblocked, current focus):**
  1. Camera driver node on the Pi: MJPEG capture, split L/R, publish stereo images.
     **Built + tested on Pi (2026-06-01):** `Vision/pi/stereo_camera_node.py`
     (rclpy, V4L2/MJPG, publishes /stereo/{left,right}/image_raw + camera_info from
     calib; no cv_bridge dep). Deploy via `Vision/pi/deploy_to_pi.sh`. Camera at
     `/dev/video0`, opens full 2560x960, all 4 topics live, camera_info populated.
     **Perf:** node 8 fps; raw cv2 capture+decode 10 fps — bottleneck is Pi
     software MJPEG decode (Mac did 30). Workable for the vision-paced gated
     approach (SGBM will likely be the slower stage). If higher fps needed: OpenCV
     on the Pi has GStreamer 1.24.1, so a HW-JPEG-decode pipeline is available.
     CLI note: node uses sensor-data QoS, so `ros2 topic echo` needs
     `--qos-reliability best_effort`; `ros2 topic hz` has no QoS flag (use the
     node's own fps log).
  2. Checkerboard stereo calibration → rectification maps + `Q`; save to YAML.
     **Tooling built (runs on Mac, Pi is headless):** `Vision/calibration/`
     (`find_camera.py`, `capture_stereo.py`, `stereo_calibrate.py`). Output
     `stereo_calib.yaml` scp'd to the Pi. Camera confirmed enumerating on the Mac
     as "3D USB Camera" (VendorID 13028 = ELP), index 0, native 2560x960. Needs
     terminal Camera permission. macOS doesn't deliver keys to OpenCV windows, so
     capture is auto (board still + new pose). Added `view_stereo.py` (live preview)
     and `depth_overlay.py` (live distance heatmap; metric once calib exists).
     **First calibration done (2026-06-01):** 8x6 board, 23.25mm; 30 pairs; per-lens
     RMS 0.39/0.42px, baseline 60.5mm (matches physical), rectified epipolar error
     0.71px mean. Usable for the distance-gated approach; recapture sharper + with
     edge/corner coverage to tighten (<0.5px) for precise close-range grasping.
  4. **Detection + depth DONE + tested on Pi (2026-06-01):** `Vision/pi/stereo_depth_node.py`
     subscribes to the stereo topics, rectifies (maps+Q recomputed from calib at
     --scale 0.5), runs StereoSGBM → 3D, detects target (`nearest` blob, or `color`
     HSV), samples robust median depth, applies the near/far distance gate, and
     publishes `/stereo/target/{point,distance,state,debug}`. State machine:
     TRACK | FAR | BEARING_ONLY | NONE. Verified live: stable ~44cm, 77% valid
     depth, ~processing pace at 8-10fps cam rate. Launch: `Vision/pi/run_vision.sh`
     [nearest|color] / `stop_vision.sh`. NOTE: `ros2 topic echo` is unreliable over
     non-interactive ssh here (DDS discovery doesn't complete) — read the node's own
     logs (`/tmp/depth.log`) instead.
  5. **Hand-eye calibration (NEXT, needs arm live):** `cv2.calibrateHandEye` →
     `T_ee_cam`, to map `/stereo/target/point` (camera frame) into base frame.
  6. Distance-gated approach controller feeding `moveo_publisher` cartesian/trajectory.
  7. "Point at object" demo end-to-end.
- Then: grasp sequencing (approach/close/retreat) using the fully-actuated hand.
- Later: migrate hand servos to ESP32-S3 + serial bus for closed-loop feedback.
- Later: migrate stepper control fully to ESP32-S3 micro-ROS (in progress separately).

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
