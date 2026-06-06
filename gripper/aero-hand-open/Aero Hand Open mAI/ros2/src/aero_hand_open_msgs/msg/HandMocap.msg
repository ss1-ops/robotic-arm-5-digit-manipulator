std_msgs/Header header

string side  # "right" or "left"

geometry_msgs/Pose[] keypoints  # Array of hand keypoint poses

## keypoint indices ##
# 0 - wrist
# 1 - thumb_cmc
# 2 - thumb_mcp
# 3 - thumb_ip
# 4 - thumb_tip
# 5 - index_cmc
# 6 - index_mcp
# 7 - index_pip
# 8 - index_dip
# 9 - index_tip
# 10 - middle_cmc
# 11 - middle_mcp
# 12 - middle_pip
# 13 - middle_dip
# 14 - middle_tip
# 15 - ring_cmc
# 16 - ring_mcp
# 17 - ring_pip
# 18 - ring_dip
# 19 - ring_tip
# 20 - pinky_cmc
# 21 - pinky_mcp
# 22 - pinky_pip
# 23 - pinky_dip
# 24 - pinky_tip