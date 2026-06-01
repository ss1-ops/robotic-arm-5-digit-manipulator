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

## Notes

- **MJPEG is mandatory** for 2560x960 over USB2; the node forces it. If you see a
  width < 2560 warning, the device fell back to a single-lens mode (bandwidth or
  cabling) — check the USB port (use a USB2/USB3 port directly, avoid hubs).
- `CAP_PROP_BUFFERSIZE=1` keeps latency low for the approach controller.
- Without `calib:=`, it still streams images but publishes empty CameraInfo.
- **Next node:** a depth/detection node subscribes to these topics, rectifies via
  the camera_info, runs StereoSGBM, and emits the distance-gated target to the
  existing `moveo_publisher` cartesian interface.
