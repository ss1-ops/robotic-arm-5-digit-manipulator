# Pi Stereo Camera Driver

ROS 2 (Jazzy) driver for the ELP dual-lens USB stereo camera, running on the
Raspberry Pi. Captures the side-by-side UVC stream over **V4L2**, splits it L/R,
and publishes left/right `image_raw` + `camera_info`.

```
/stereo/left/image_raw     sensor_msgs/Image  (bgr8)
/stereo/right/image_raw    sensor_msgs/Image  (bgr8)
/stereo/left/camera_info   sensor_msgs/CameraInfo   (K/D/R/P from calibration)
/stereo/right/camera_info  sensor_msgs/CameraInfo
```

It's a loose `rclpy` script (run with `python3`, like `moveo_publisher.py`) — no
colcon build required. Image messages are built by hand, so **no cv_bridge
dependency** (just `rclpy`, `sensor_msgs`, `opencv`, `numpy`, all already on the
ROS perception Pi).

## Deploy from the Mac

```bash
cd Vision/pi
bash deploy_to_pi.sh        # scp node + ../calibration/stereo_calib.yaml to armpi:~/vision
```

## On the Pi: confirm the camera + V4L2 mode

```bash
v4l2-ctl --list-devices                 # find the ELP's /dev/videoN
v4l2-ctl -d /dev/video0 --list-formats-ext | grep -A3 MJPG   # confirm 2560x960 MJPG exists
groups | grep -q video || sudo usermod -aG video $USER       # then re-login
```

The ELP usually presents two `/dev/video` nodes (capture + metadata); use the
capture one. Pass it via `device:=N` (integer index) below.

## Run

```bash
cd ~/vision
python3 stereo_camera_node.py --ros-args \
    -p device:=0 \
    -p calib:=$PWD/stereo_calib.yaml
```

Parameters: `device` (index or path), `width` (2560), `height` (960), `fps` (30),
`calib` (YAML path), `left_frame` / `right_frame` (TF frame ids).

## Verify

From the Pi, or from the Mac if ROS_DOMAIN_ID matches and discovery works:

```bash
ros2 topic hz   /stereo/left/image_raw          # ~30 Hz expected
ros2 topic echo /stereo/left/camera_info --once  # K/D/R/P populated from calib
```

For a visual check, on a machine with a display:
```bash
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw
```

## Hand-eye calibration (`hand_eye_calibrate.py`)

Finds `T_ee_cam` so camera-frame targets map into the arm base frame
(`p_base = T_base_ee @ T_ee_cam @ p_cam`). Eye-in-hand; uses tf2 for `T_base_ee`
(from `robot_state_publisher`) and solvePnP on the rectified left image.

Prereqs running: `stereo_camera_node` (for `/stereo/left/image_raw`) **and** the
robot description / `robot_state_publisher` (so `base_frame -> ee_frame` is in TF).

1. Find the end-effector link name (the link the camera is rigidly bolted to):
   ```bash
   ros2 run tf2_tools view_frames     # or read the warn output listing TF frames
   ```
2. Clamp the checkerboard somewhere **fixed** in the workspace.
3. Run, then jog the arm (GUI / moveo_publisher) to ~18 varied poses that all keep
   the board in the left lens. It auto-captures when the arm is still and the pose
   is new (watch the log; `ros2 topic echo` is unreliable over ssh here).
   ```bash
   python3 hand_eye_calibrate.py --ros-args \
       -p calib:=$PWD/stereo_calib.yaml \
       -p base_frame:=base_link -p ee_frame:=<link> \
       -p inner_cols:=8 -p inner_rows:=6 -p square_mm:=23.25
   ```
4. After `--count` poses (or publish `std_msgs/Empty` to `/hand_eye/finish`) it
   writes `hand_eye.yaml` and prints a consistency residual (board-position spread;
   aim for <~5 mm / <~1°). Recapture with more varied poses if it's high.

## Notes

- **MJPEG is mandatory** for 2560x960 over USB2; the node forces it. If you see a
  width < 2560 warning, the device fell back to a single-lens mode (bandwidth or
  cabling) — check the USB port (use a USB2/USB3 port directly, avoid hubs).
- `CAP_PROP_BUFFERSIZE=1` keeps latency low for the approach controller.
- Without `calib:=`, it still streams images but publishes empty CameraInfo.
- **Next node:** a depth/detection node subscribes to these topics, rectifies via
  the camera_info, runs StereoSGBM, and emits the distance-gated target to the
  existing `moveo_publisher` cartesian interface.
