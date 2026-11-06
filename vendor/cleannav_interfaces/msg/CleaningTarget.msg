# Perception -> Mission Manager
# 清扫目标环境观测，不直接成为 Goal 或 Costmap

uint8 TYPE_UNKNOWN=0
uint8 TYPE_LEAF=1
uint8 TYPE_LEAF_PILE=2
uint8 TYPE_PUDDLE=3
uint8 TYPE_BOTTLE_CAN=4
uint8 TYPE_PAPER_TRASH=5

uint8 OBSERVATION_UNKNOWN=0
uint8 OBSERVATION_NEW=1
uint8 OBSERVATION_CONFIRMED=2
uint8 OBSERVATION_LOST=3
uint8 OBSERVATION_EXPIRED=4
uint8 OBSERVATION_INVALID=5

# header.frame_id 必须为 "map"
# header.stamp 为目标位置估计时间
std_msgs/Header header
string<=16 interface_version
string<=128 target_id           # 目标生命周期内稳定
string<=64 source               # 感知来源
uint8 target_type
string<=64 class_name           # 可选文字标签
float32 confidence
bool projection_valid           # false 时不可用于导航
geometry_msgs/Point centroid    # map frame
geometry_msgs/Polygon footprint # map frame
float32 area_m2
float32 position_uncertainty_m
uint8 observation_state
builtin_interfaces/Duration valid_for

# 说明:
# - 不直接写 Costmap
# - 不直接成为 /goal_pose
# - 普通位置更新不自动重发 Goal
