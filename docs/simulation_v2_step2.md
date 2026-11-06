# PC Simulation V2 Step 2

## 构建与启动

在工作区根目录，使用独立 overlay，不 source 旧 filtered install：

```bash
cd /home/d1zzy/code/cleannav_modularization
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/cleannav-step2/log build \
  --base-paths cleannav-system/src/cleannav-interfaces \
    cleannav-system/src/cleannav-navigation \
    cleannav-system/src/cleannav-mission-manager \
  --build-base /tmp/cleannav-step2/build \
  --install-base /tmp/cleannav-step2/install \
  --packages-up-to cleannav_simulation
source /tmp/cleannav-step2/install/local_setup.bash
ros2 launch cleannav_simulation cleannav_full_sim.launch.py
```

默认 `enable_motion=false`、`auto_start_task=false`、`start_rviz=true`。
无 GUI 时加 `gui:=false start_rviz:=false`。显式允许仿真运动时加
`enable_motion:=true`，然后等待主终端 SYSTEM READY。

总 launch 的子进程强制 `ROS_LOCALHOST_ONLY=1`，默认 `ROS_DOMAIN_ID=229`，
Gazebo master 为 `http://127.0.0.1:11459`。可覆盖 `ros_domain_id`（非零）
与 `gazebo_port`，务必选择未被其他系统使用的域和端口。
该入口不启动任何车辆 gateway，也不向实车网络发送命令。

## 启动阶段与 readiness

```text
Safety (BLOCKED) + MCity demo + persistent status monitor
  → simulation probe 成功 → RTAB-Map wrapper
  → SLAM probe 成功 → Ackermann navigation (Facade，复用已启动的唯一 Safety)
  → navigation probe 成功 → standard real-adapter Mission Manager
  → mission probe 成功 → 等待完整地图与依赖 → SYSTEM READY
```

复用已有 MCity launch 的 60 秒 spawn 时序；它只是生成调度，不是 readiness。
simulation probe 还检查持续变化的 /clock、有效 /scan /odom、机器人 joint_states、
两个 ACTIVE ros2_control controller、Gazebo spawn service、odom → base_link TF。
SLAM 检查实际 RTAB 节点和 map → odom / base_link TF；没有可用 /rtabmap/map 时
明确 PARTIAL_READY，不能授权运动。Nav2 检查 planner/controller lifecycle ACTIVE、
三个 Action Server 与真实 SafetyStatus。MM 检查 TaskCommand 订阅、实际 TaskStatus
以及标准 RealNavigation / RealSafety 的接口心跳。

阶段超时默认 210 / 90 / 120 / 30 wall-clock 秒，可通过四个 `*_timeout` 参数覆盖。
探针 deadline、状态轮询和授权超时使用 monotonic / STEADY_TIME，不依赖 /clock 推进。
阶段失败打印原因与日志目录并关闭本次启动链；没有无限等待后自动宣称 READY。
关键进程退出会关闭本次子进程；RViz 退出只降级可视化。Ctrl+C 由一个 launch 统一回收。

## 运动授权与任务

Safety 在生成车辆之前就以 block_all=true 启动并持续输出零速度；后续 Navigation
include 关闭自己的 Safety 实例，整套系统只有一个最终 /cmd_vel 发布者。
只有显式 `enable_motion=true` 且监控确认全部
依赖和地图就绪后，才调用新 SetBool 服务
`/cleannav/safety/set_simulation_motion_enabled` 解除 simulation interlock。
该服务要求启动时 `simulation_motion_authorization=true`、use_sim_time=true、无 ESTOP。
解除 interlock 不获取 Lease、不设置 autonomous_enabled；MM 仍必须正常 acquire。
运行时有效 interlock 以 SafetyStatus.block_all 为准，启动参数仍保留默认安全值。
已 READY 后丢失关键依赖时监控发送 ESTOP，不自动恢复。

独立 pc_demo.launch.py、Task 30 DemoPhysicalCompletionNavigationAdapter 保持原样。
总 launch 使用已有视觉目标 resolver + 标准 RealNavigationAdapter，不自动发 Task 30，
普通 FollowPath ABORT 仍是失败。`auto_start_task=true` 必须同时 enable_motion=true，
并等待 SYSTEM READY 才启动下面的 Task 31 client；其默认目标来自 task_goal_x/y 参数。

### 手动点到点任务

另开一个终端，source 同一 ROS / overlay，并使用相同隔离域：

```bash
source /opt/ros/humble/setup.bash
source /tmp/cleannav-step2/install/local_setup.bash
export ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1
ros2 run cleannav_simulation simulation_task_client --ros-args \
  -p use_sim_time:=true -p goal_x:=0.8 -p goal_y:=0.0
```

这会显式创建 map frame 的模拟 LEAF_PILE 观测，再通过现有
`/cleannav/hmi/task_command` 发布 Task 31 CLEAN_NEAREST_LEAF_PILE。
目标坐标应先在地图中确认可达。TaskCommand 本身仍不携带坐标，没有修改 TaskCatalog。
该目标是操作者提供的仿真观测，不代表真实感知识别。结果必须来自标准导航 Action；
不会用清扫演示映射伪造完成，也不证明覆盖式清扫。
客户端等待 readiness，执行超时或中断会请求 ESTOP，不能静默遗弃在途任务。

通过正规任务接口停止任务或紧停：

```bash
ros2 run cleannav_simulation simulation_task_client --ros-args \
  -p use_sim_time:=true -p control_task_id:=4
# 紧停用 control_task_id:=6；不需要运动授权或 READY。
```

确认任务停止、Safety 授权释放后，在启动终端 Ctrl+C 退出整套仿真。
无需 pkill；不会杀死其他 ROS 域的节点。

## 终端、日志和边界

主终端只转发状态变化、关键 ERROR/FATAL 与异常退出。
完整 stdout、stderr 和 ROS 日志保存在启动时打印的时间戳日志目录，包含每进程
`*-stdout.log`、`*-stderr.log`、合并日志和 launch.log。未覆盖已有日志。
默认 RTAB 数据库是该次日志目录中的新 rtabmap.db；也可显式指定 database_path。
不删除或改写原有 ~/.ros/rtabmap_lidar.db。
RTAB 的可写 INI 配置也复制到每次日志目录，避免退出时回写包内配置。

RViz 显示 RobotModel、TF、scan、RTAB 地图、global/local costmap、/plan；
不提供 2D Goal 适配器。Facade 默认不消费 /goal_pose。
本启动链保持 Step 1 单执行者约束；DDS 全局访问隔离、Lease TTL/heartbeat 和
监控进程死亡时的独立硬件安全保障仍未实现。
UNIT / mock Action 测试不能替代 Gazebo runtime 验收。

## 本次运行结论：PARTIAL

2026-10-09 的隔离域 229 验证中，完整启动、有效传感器/TF、RTAB 地图、
Nav2 ACTIVE、标准 MM 和默认 Safety BLOCKED 均得到真实接口证据。
默认模式采样 257 条最终速度全部为零、没有自动 TaskCommand；但轮式里程计
25 秒变化约 5.4 cm，两次 Gazebo 真值快照相差约 7 mm，物理静止尚未验收。

显式 enable_motion=true 后，Task 31 的真实 Smac 路径生成成功、MPPI 开始执行，
最终 controller_server 报 Failed to make progress，FollowPath ABORT，
MM 返回 STATE_FAILED=15。失败后 Lease 释放，额外 10 秒采样的 93 条最终速度全零。
没有点到点成功证据，没有改变 MPPI/Smac 参数或 Task 30 语义。

默认与运动测试的 Ctrl+C 均正常回收全部子进程。另一次 SLAM 首帧 base_scan TF
失败导致阶段超时，该失败分支的 RTAB 退出需要 launch 升级到 SIGKILL，Gazebo
以 -6 退出；正常退出成功不能覆盖这一异常分支缺口。RViz 在 WSL 中启动但报 GLSL
link error，显示尚未验证通过。运行中发现的 stderr / rosout 同一错误重复转发，
已补充终端去重和回归测试；这项最终日志改动尚未再次启动完整 Gazebo 复核。

完整证据保留在 /tmp/cleannav-step2：runtime-default.out、default-evidence.json、
runtime-motion.out（SLAM 失败）、runtime-motion-retry.out、task-client.out、
motion-evidence.json、motion-end-evidence.json、post-task-evidence.json。
后续应先定位物理静止漂移与控制无进展，再恢复点到点验收；本轮未实施 Step 3。
