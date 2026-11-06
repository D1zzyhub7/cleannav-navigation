# Mission Manager / Status Aggregator -> HMI
# 机器人综合状态，不等同于 TaskStatus.state

uint8 SYSTEM_UNKNOWN=0
uint8 SYSTEM_READY=1
uint8 SYSTEM_BUSY=2
uint8 SYSTEM_PAUSED=3
uint8 SYSTEM_BLOCKED=4
uint8 SYSTEM_ERROR=5
uint8 SYSTEM_EMERGENCY_STOP=6

# header.frame_id 在 pose 有效时必须为 "map"
std_msgs/Header header
string<=16 interface_version
uint8 system_state
bool localization_ok            # false 时 pose 不得被消费者使用
geometry_msgs/Pose pose         # map 位姿
bool navigation_active          # 存在活动或清理中的 navigation generation
                                # 发出 cancel 请求不等于 false
bool emergency_stop_active
bool autonomous_enabled         # Safety 实际授权，不得与 navigation_active 混同
float32 linear_velocity_mps     # 测量速度，优先来自 odometry
float32 angular_velocity_rps
string<=256 message             # 人类可读，不用于程序控制
