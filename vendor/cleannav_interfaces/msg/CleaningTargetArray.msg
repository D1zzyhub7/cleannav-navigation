# Perception -> Mission Manager
# 清扫目标批次观测，header.frame_id 必须为 "map"

std_msgs/Header header
string<=16 interface_version
string<=64 source
uint32 sequence              # 批次排序编号，不用于 command 幂等
cleannav_interfaces/CleaningTarget[] targets

# 说明:
# - 空数组只表示本批次没有观测目标，不立即清空 Target Registry
# - 同批所有目标必须使用 map frame
# - sequence 仅用于批次排序，不用于 command 幂等
