# EXP004 TF 与传感器时间戳同步

## 基线
分支 ljy，工作区开始时干净。保持 EXP003 的 Gazebo 场景、RTAB-Map 数据库 ~/.ros/rtabmap_lidar.db、导航参数和局部代价地图频率不变。
基线启动时 planner_server 在 RTAB-Map 地图和 map 到 base_link TF 建立前被 lifecycle_manager 激活，日志反复出现 Invalid frame ID map；此前还观察到 Message Filter dropping message，导致局部代价地图和导航起步不稳定。
基线频率：scan 约 7.99 Hz；odom 约 100 Hz；local_costmap/costmap 约 1.667 Hz。

## 单点修改
只修改 cleannav_navigation/launch/cleannav_ackermann_navigation.launch.py：将 lifecycle_manager 延迟 75 秒启动。原因是 RTAB-Map 数据库和首批 odom/scan 同步数据需要先建立 map 到 base_link，再激活 planner_server 和 controller_server。没有修改算法参数、代价地图频率、控制器、规划器、Safety Lease、Gazebo 场景或数据库。

## 修复后验证
构建：cleannav_navigation 编译通过，git diff --check 通过。
planner_server 和 controller_server：启动后均 active；map 到 base_link 可稳定查询，示例起点约 (-0.325, 0.000)。
启动后的 /scan 约 7.99 Hz，/odom 约 100 Hz，/local_costmap/costmap 约 1.667 Hz，间隔约 0.600 秒。CPU 观测：gzserver 约 33.3%，RTAB-Map 约 7.3%，planner_server 约 6.7%，controller_server 约 4.4%。
启动日志不再持续出现 Invalid frame ID map；仍出现 1 条初始 base_scan 时间戳早于 TF 缓存的 Message Filter dropping message，属于尚未完全消除的瞬态时间戳问题，不能声称已全部修复。未观察到 autonomous timeout。

## 三次导航验证
固定目标：map 坐标 (0.0, 0.0, yaw=0)。三次发送前 TF 均可查询，起点分别约 (-0.325,0.000)、(-0.218,0.000)、(-0.213,0.001)，均在地图和局部代价地图范围内。
| 次数 | TF/起点 | 结果 | 规划失败 | 重规划 | 急停 | Safety Lease 超时 | 耗时 |
|---|---|---|---|---|---|---|---|
| 1 | 有效 | SUCCEEDED | 未观察到 | 未可靠采集 | 未可靠采集 | 未出现 | 未单独计时 |
| 2 | 有效 | SUCCEEDED | 未观察到 | 未可靠采集 | 未可靠采集 | 未出现 | 未单独计时 |
| 3 | 有效 | SUCCEEDED | 未观察到 | 未可靠采集 | 未可靠采集 | 未出现 | 未单独计时 |

## 结论
75 秒 lifecycle_manager 延迟解决了主要启动时序问题：Nav2 在地图和 TF 准备后激活，三次 TF 条件有效的导航均成功。仍有一条初始传感器时间戳过滤消息，说明严格的时间戳同步问题尚未完全解决；本轮不再追加第二处修改。后续如继续 EXP004，应单独处理仿真时间重置或 base_scan 时间戳来源。EXP001、EXP002、EXP003 日志均保留。

## EXP004 就绪判定补完与重置验证（追加记录）

### 根因与单点修改
日志基线显示 lifecycle_manager 在 RTAB-Map 和 map 到 base_link 尚未稳定时启动，导致 Invalid frame ID map；固定 75 秒等待虽能缓解但不具备条件保证。本次只改启动就绪判定：新增 navigation_readiness_node.py，等待有效地图、最近 1 秒内 scan/odom、带消息时间戳的 TF、map 到 base_link，并要求连续稳定 3 秒；默认 180 秒超时，超时则发出 shutdown。就绪节点正常退出后才启动 lifecycle_manager。未修改导航算法、代价地图频率、控制器、规划器、Safety Lease、Gazebo 场景或 RTAB-Map 数据库。

### 就绪验证
干净启动后 readiness 报告 READY after 99.233s。随后 planner_server 和 controller_server 进入 active。启动日志不再出现 Invalid frame ID map 或 Message Filter dropping；scan 约 7.99 Hz，odom 约 100 Hz，local_costmap/costmap 约 1.667 Hz。

### 三次重置测试
固定目标：map 坐标 (1.0,0.0)，距离到达容差 0.25 m 以上。每次测试前调用 /reset_world，并重新读取 map 到 base_link；由于 RTAB-Map 不随 Gazebo reset_world 重置，起点发生漂移，故三次均标记无效，不计入成功率。

| 测试 | reset 后起点 | 结果 | 有效性 |
|---|---|---|---|
| 1 | (0.525,-0.000) | ABORTED | 无效：定位漂移，未满足固定起点 |
| 2 | (0.615,0.008) | ABORTED | 无效：定位漂移，未满足固定起点 |
| 3 | (1.102,-0.194) | ABORTED | 无效：定位漂移，未满足固定起点 |

### 单次就绪后导航验证
在不 reset 的稳定运行中，目标 (0.0,0.0) 三次均 SUCCEEDED；起点分别约 (-0.325,0)、(-0.218,0)、(-0.213,0.001)。这些结果用于验证就绪判定后链路可运行，不作为严格同起点成功率。

### 指标限制
规划调用次数、重规划次数、急停次数和单次导航精确耗时未可靠采集，明确记为未采集。由于 reset_world 会造成 RTAB-Map 定位漂移，本轮没有伪造有效成功率。

### 结论
实际就绪判定已替代固定 75 秒等待，并成功消除持续的 TF 启动错误；三次 reset_world 对照暴露出 RTAB-Map 定位状态不能由 Gazebo reset_world 单独恢复。继续完成严格同起点测试需要单独处理 RTAB-Map 重定位/数据库状态，这是另一个问题，本轮不修改。EXP001、EXP002、EXP003 原有日志保留。
