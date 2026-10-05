
## EXP003 局部代价地图频率实验（追加记录）

### 基线（未修改配置）
分支 ljy，工作区开始时干净。地图使用 /rtabmap/map，分辨率 0.05 m；RTAB-Map 数据库为 ~/.ros/rtabmap_lidar.db；基线起点记录约 map=(-1.166,0.401)，固定测试目标为 map=(-0.90,0.40)。
local_costmap.update_frequency=10.0 Hz；publish_frequency=2.0 Hz。/scan 平均约 7.99 Hz，/local_costmap/costmap 平均约 1.667 Hz，间隔约 0.600 s。CPU 观测：gzserver 约 37.2%，rtabmap 约 7.6%，controller_server 约 5.3%。
基线导航进行了 3 次同目标观测：均被接受但最终 ABORTED；由于起点在连续测试中移动、且日志出现 TF/控制进度问题，3 次均不计入有效成功率。

### 单点候选修改
只修改 local_costmap.publish_frequency：2.0 Hz → 5.0 Hz。update_frequency、Safety Lease、全局规划器、控制器、地图和目标均未修改。
改进配置下 /local_costmap/costmap 平均仍约 1.667 Hz，最小间隔约 0.600 s，最大间隔约 0.600 s，标准差约 0.0001 s；未观察到相对基线的频率提升。CPU 观测与基线同量级：gzserver 约 37.5%，rtabmap 约 7.8%，controller_server 约 5.5%。/scan 基线约 7.99 Hz，本轮未重新采集，明确记为未重新采集。
改进配置下同目标 3 次均 ABORTED；规划调用次数、重规划次数、急停次数和 Safety Lease 超时次数未可靠采集，明确记为未采集。

### 结论
将 publish_frequency 从 2.0 Hz 提高到 5.0 Hz 没有提高实际局部代价地图频率，且导航测试仍受起点变化、TF 和控制进度问题影响。该候选修改未采用，配置已恢复为 publish_frequency=2.0 Hz；本轮没有改变最终导航参数。EXP003 说明当前瓶颈不在 publish_frequency 单项设置，后续应单独解决 TF/时间戳和控制进度问题后再做频率实验。EXP001、EXP002 原有日志未覆盖。
