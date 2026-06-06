std_msgs/Header header

## Available Control Modes ##
# 0 - POSITION_CONTROL
# 1 - TORQUE_CONTROL
# 2 - VELOCITY_CONTROL

int8 control_mode

float32[] actuation_positions
float32[] actuation_torques
float32[] actuation_velocities