# 一个有序业务序列中的任务步骤。该接口不接受任意导航坐标或速度。
uint8 OPERATION_UNSPECIFIED=0
uint8 GOTO_REGION=1
uint8 PATROL_REGION=2
uint8 RETURN_CHARGER=3

string<=32 step_id
uint8 operation
string<=64 region_id
string<=64 route_id
uint8 laps
string<=64 charger_id
