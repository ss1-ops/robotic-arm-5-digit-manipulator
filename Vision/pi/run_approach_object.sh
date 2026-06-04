#!/usr/bin/env bash
# Launch goto_object. Args: H_LO S_LO V_LO H_HI S_HI V_HI [STANDOFF_m]
# goto_object handles its own color detection + one-shot depth (no depth node needed).
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd "$HOME/vision" || exit 1
CALIB="$HOME/vision/stereo_calib.yaml"
HLO=$1; SLO=$2; VLO=$3; HHI=$4; SHI=$5; VHI=$6; DIST=${7:-0.22}
# Match .py so this doesn't kill itself.
pkill -9 -f "ibvs_servo.py|approach_servo.py|approach_object.py|goto_object.py|stereo_depth_node.py" 2>/dev/null
sleep 1
setsid python3 goto_object.py --ros-args \
  -p calib:="$CALIB" \
  -p h_lo:=$HLO -p s_lo:=$SLO -p v_lo:=$VLO \
  -p h_hi:=$HHI -p s_hi:=$SHI -p v_hi:=$VHI \
  -p standoff_m:=$DIST \
  >$HOME/approach.log 2>&1 </dev/null &
sleep 1
echo "goto_object: HSV [$HLO,$SLO,$VLO]-[$HHI,$SHI,$VHI] standoff ${DIST}m"
