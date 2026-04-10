std_msgs/Header header

string side  # "right" or "left"

float32[] actuations                    # degrees
float32[] actuator_speeds               # RPM (signed)
float32[] actuator_currents             # mA (signed)
float32[] actuator_temperatures         # Degree Celsius (unsigned)

###### Actuation Indices ######
# 0 - thumb_cmc_abd_act
# 1 - thumb_cmc_flex_act
# 2 - thumb_tendon_act
# 3 - index_tendon_act
# 4 - middle_tendon_act
# 5 - ring_tendon_act
# 6 - pinky_tendon_act