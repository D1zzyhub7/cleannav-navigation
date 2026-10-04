# EXP001：局部代价地图更新频率实验

日期：2026-10-04
分支：ljy
修改前提交编号：c441c54d271a69f7a12f5f655883941b941d1f5f

## 目的
尝试减少动态障碍物进入局部代价地图的刷新延迟。

## 修改
文件：cleannav_navigation/config/nav2_params_ackermann.yaml
位置：local_costmap -> local_costmap -> ros__parameters
参数：update_frequency
原值：5.0
候选值：10.0

publish_frequency 保持 2.0。
本轮不修改 MPPI、车辆尺寸、速度上限和安全门控。

## 已有验证
规划器：active
控制器：active
/odom、/scan：已有数据
/rtabmap/map：已有有效地图
map -> base_link：可查询
控制话题连接：已确认

## 修改前测量
运行时 update_frequency：5.0 Hz
配置 publish_frequency：2.0 Hz
/local_costmap/costmap 实际平均频率：约 1.56 Hz
/local_costmap/costmap 最小发布间隔：0.594 秒
/local_costmap/costmap 最大发布间隔：2.742 秒
/local_costmap/costmap 标准差：约 0.25 秒
/scan 频率：7.945 Hz
是否出现更新超时：观察到发布间隔最长达到 2.742 秒
是否完成动态障碍导航任务：未测
说明：局部代价地图实际频率低于配置的 2.0 Hz，并存在发布间隔抖动。

## 修改后测量
修改后 update_frequency：10.0 Hz
publish_frequency：2.0 Hz
/local_costmap/costmap 实际平均频率：约 1.55 Hz
/local_costmap/costmap 最小发布间隔：0.588 秒
/local_costmap/costmap 最大发布间隔：2.667 秒
/local_costmap/costmap 标准差：约 0.274 秒
/scan 频率：本轮未重新测量
规划器状态：active [3]
控制器状态：active [3]
是否出现更新超时：出现，最大间隔 2.667 秒
CPU 开销观察：未测

## 动态避障对比
是否完成动态障碍导航任务：未测
是否碰撞：未测
是否到达：未测
任务耗时：未测
最小障碍物距离：未测
## 结论
将局部代价地图 update_frequency 从 5.0 Hz 改为 10.0 Hz 后，
实际平均发布频率没有明显提高，仍约为 1.55 Hz。
最大发布间隔略有下降，但标准差增加，发布抖动没有改善。
仅从本轮频率测试看，该参数调整没有显示出明确收益。

动态障碍测试条件
地图话题：/rtabmap/map
地图分辨率：0.05 m
地图宽度：372
地图高度：165
地图原点：x=-8.313，y=-4.075
地图范围：x=-8.313~10.287，y=-4.075~4.175
固定目标点：x=5.0，y=0.0，yaw=0

## 动态导航测试补充（自动运行记录）

### 10Hz-1
参数：update_frequency = 10.0 Hz
Safety 测试配置：safety_block_all:=false（仅用于允许候选速度进行实验）
固定目标：x=5.0，y=0.0，yaw=0
结果：目标被接受，但最终 ABORTED。
控制器日志：Failed to make progress。
结论：未形成有效的动态避障性能结果，不能用于判定 10 Hz 优于或劣于 5 Hz。

### 10Hz-可达性复测
固定目标：x=2.0，y=0.0，yaw=0
结果：目标被接受，但最终 ABORTED。
控制器日志：Failed to make progress。
结论：更近目标仍未完成，当前问题不能归因于目标距离；需要继续检查定位/控制反馈或测试场景。

备注：本轮曾发现 cleannav_interfaces 未在工作区，已补齐并重新构建。此依赖问题已修复，但导航目标仍未完成。

## 测试诊断补充

本轮自动化复测未形成有效的 5 Hz/10 Hz 对比，原因如下：

1. Safety Supervisor 日志出现 utonomous timeout — back to block_all，导致速度授权不能覆盖完整导航任务。
2. 固定目标 (2.0, 0.0) 的 SmacPlannerHybrid 日志出现 ailed to create plan, exceeded maximum iterations，目标未生成有效路径。
3. 因此本轮目标中止不能归因于 local_costmap update_frequency，5 Hz 与 10 Hz 的导航性能结论暂不成立。

下一轮开始前必须先解决 Safety Lease 生命周期和可达目标选择，再进行参数对比。
