# Perception -> Mission Manager / HMI
# 感知子系统健康状态

uint8 STATE_UNKNOWN=0
uint8 STATE_OK=1
uint8 STATE_DEGRADED=2
uint8 STATE_ERROR=3

# header.frame_id 为空
# header.stamp 为健康状态采样时间
std_msgs/Header header
string<=16 interface_version
string<=64 source
uint8 state
bool camera_ok
bool inference_ok
bool tf_ok
bool projection_ok
float32 inference_fps
float32 average_latency_ms
uint32 dropped_frames
string<=256 message

# 说明:
# - 消息 stale 后不再视为健康，即使最后一条为 STATE_OK
# - stale timeout 是节点参数，不写入消息
