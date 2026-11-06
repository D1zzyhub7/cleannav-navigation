# CleanNav HMI -> Mission Manager
# v1.0 只携带预定义任务 ID，不携带任意坐标或速度命令。

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_VOICE=1
uint8 SOURCE_APP=2
uint8 SOURCE_MOCK=3

# header.frame_id 必须为空
# header.stamp 使用 ROS Clock（由 Bridge 生成）
std_msgs/Header header
string<=16 interface_version
string<=128 command_id          # 幂等键，格式: ^[A-Za-z0-9._:-]+$
uint8 source
uint16 task_id                   # Task Catalog 编号
float32 confidence               # APP 固定 1.0，语音填离线识别置信度
string<=512 raw_text             # 审计信息，不参与执行语义
builtin_interfaces/Duration valid_for  # 自 header.stamp 起计算的有效期
bool user_confirmed                    # HMI 已完成本次命令所要求的人机确认
                                         # requires_confirmation=false 的任务可为 false
                                         # 该字段只代表 HMI 用户确认，不代表 Safety 条件已经满足

# 说明:
# - TaskCommand 不携带任意坐标和速度
# - 控制命令 (task_kind=CONTROL) 不进入普通 Mission Queue
# - 内部状态机事件不得通过公共 HMI Topic 发布
