# Mission Manager -> HMI/Agent 的序列级状态；TaskStatus V1 保持不变。
uint8 STATE_UNKNOWN=0
uint8 STATE_QUEUED=1
uint8 STATE_ACTIVE=2
uint8 STATE_SUCCEEDED=3
uint8 STATE_FAILED=4
uint8 STATE_CANCELED=5
uint8 STATE_EMERGENCY_STOPPED=6

std_msgs/Header header
string<=16 interface_version
string<=64 sequence_id
string<=128 command_id
uint8 state
string<=64 current_execution_id
string<=32 current_step_id
uint16 current_step_index       # 从1开始；0表示尚无活动步骤
uint16 step_count
uint16 completed_steps
uint8 operation
string<=64 region_id
string<=64 charger_id
string<=64 route_id
uint8 current_lap               # 从1开始；0表示非巡航或尚未开始
uint8 total_laps
uint16 current_segment          # 从1开始；0表示无路线段
uint16 total_segments
int32 reason_code
string<=512 message
