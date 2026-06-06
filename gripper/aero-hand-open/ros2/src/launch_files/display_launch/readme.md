## URDF visualization (RViz)


This launch file visualizes the left and right Aero hands in RViz and provides a GUI to manipulate finger joints.


### Software requirements

- rviz2


### Run

Build and source:

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch:

```bash
ros2 launch src/launch_files/display_launch/display.launch.py
```


Move sliders in the **joint_state_publisher_gui** window to manipulate finger joints and observe the live visualization in RViz.