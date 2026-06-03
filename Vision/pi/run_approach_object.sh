#!/usr/bin/env bash
# Start depth(color) + approach_object. Args: H_LO S_LO V_LO H_HI S_HI V_HI [DIST_m]
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
CALIB="$HOME/vision/stereo_calib.yaml"
HLO=$1; SLO=$2; VLO=$3; HHI=$4; SHI=$5; VHI=$6; DIST=${7:-0.22}
# Match .py so this doesn't kill itself (run_approach_object.sh contains "approach_object").
pkill -9 -f "ibvs_servo.py|approach_servo.py|approach_object.py|joint_probe.py|scale_probe.py|stereo_depth_node.py" 2>/dev/null
sleep 1
setsid python3 stereo_depth_node.py --ros-args -p calib:="$CALIB" -p detector:=color \
  -p h_lo:=$HLO -p s_lo:=$SLO -p v_lo:=$VLO -p h_hi:=$HHI -p s_hi:=$SHI -p v_hi:=$VHI \
  >/tmp/depth.log 2>&1 </dev/null &
sleep 3
setsid python3 approach_object.py --ros-args -p target_dist_m:=$DIST >/tmp/approach.log 2>&1 </dev/null &
sleep 1
echo "approach_object: HSV [$HLO,$SLO,$VLO]-[$HHI,$SHI,$VHI] dist ${DIST}m"
