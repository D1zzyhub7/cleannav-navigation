# CleanNav Navigation

CleanNav Navigation 是 CleanNav 无人清扫车系统的 ROS 2 导航、定位、路径执行与运动安全门控仓库。

当前主要开发环境：

- Ubuntu 22.04 / WSL2
- ROS 2 Humble
- Nav2
- RTAB-Map
- Gazebo
- Ackermann vehicle model

当前比赛冻结标签：

`competition-hil-baseline-20260921`

对应比赛导航基线：

`2e6b3225d9d6c571f04aab90c7bb8d94d7901709`

---

## 1. 仓库职责

本仓库负责：

- RTAB-Map LiDAR 定位包装；
- Nav2 bringup；
- 全局规划；
- Ackermann Hybrid-A*；
- FollowPath 路径执行；
- MPPI Ackermann 局部控制；
- Navigation Facade；
- Safety Supervisor；
- Safety Lease；
- Gazebo / MCity 仿真；
- PC Showcase；
- PC ↔ J6M Navigation HIL Bridge。

本仓库不负责：

- APP；
- Voice ASR；
- Mission Manager 任务状态机；
- 感知模型；
- VLM；
- 真实车辆 CAN Gateway。

APP、Voice 和 Mission Manager 不允许直接发布最终 `/cmd_vel`。

---

## 2. 当前主要 ROS 2 Package

当前比赛分支包含：

- `cleannav_simulation`
- `cleannav_navigation`
- `cleannav_global_planner`
- `cleannav_path_executor`
- `cleannav_safety_supervisor`
- `cleannav_rtabmap`

主要职责：

`cleannav_simulation`

- Gazebo / MCity 仿真；
- Ackermann 底盘；
- Showcase 场景；
- 障碍物与清扫目标演示。

`cleannav_navigation`

- Nav2 参数；
- Ackermann bringup；
- Navigation Facade；
- HIL Navigation Client。

`cleannav_global_planner`

- CleanNav 全局规划包装；
- Hybrid Planner Bridge；
- SmacPlannerHybrid 接入。

`cleannav_path_executor`

- 将路径提交给 Nav2 FollowPath。

`cleannav_safety_supervisor`

- 候选控制门控；
- Safety Lease；
- 急停与授权；
- 最终 `/cmd_vel` 输出。

`cleannav_rtabmap`

- RTAB-Map LiDAR 定位与建图包装。

---

## 3. Git 分支关系

当前协作时必须区分以下代码线。

### main

当前：

`3c223ca5c201fb0c672695b2b9d9841fd50df896`

用途：

- 基础稳定主线；
- GitHub 仓库治理；
- 不代表当前比赛 Showcase 最新代码。

### feature/ackermann-hybrid-mppi-v1

当前：

`d7fa56c5414b6138cdb1f8a57d415f154104c65e`

主要内容：

- Smac Hybrid-A*；
- MPPI Ackermann；
- Ackermann production bringup。

### feature/navigation-facade-v1

当前：

`6027010443535771ed1ff66035d4367adb58c24e`

主要内容：

- NavigateToPose Facade；
- execution-owned Safety Lease；
- structured SafetyStatus。

### feature/pc-demo-mcity

当前：

`2e6b3225d9d6c571f04aab90c7bb8d94d7901709`

这是当前决赛阶段 Navigation 推荐开发基线。

它已经包含：

- Ackermann；
- Hybrid-A*；
- MPPI；
- Navigation Facade；
- Safety Lease；
- MCity；
- RTAB-Map 修复；
- replanning / recovery；
- HIL Bridge；
- Competition Leaf Cleaning Showcase。

决赛 Navigation 组继续开发时，优先从该分支建立新的 feature branch，而不是从旧 `main` 直接开始。

---

## 4. 当前比赛导航链路

当前主要闭环：

    RTAB-Map / Localization
        ↓
    Nav2 Global Costmap
        ↓
    SmacPlannerHybrid
        ↓
    FollowPath
        ↓
    MPPI Ackermann
        ↓
    /cleannav/cmd_vel_candidate
        ↓
    Safety Supervisor
        ↓
    /cmd_vel

最终 `/cmd_vel` 的运动安全门控属于 Safety Supervisor。

Navigation Controller 产生的是候选控制，不允许绕过 Safety Supervisor。

---

## 5. 当前 Ackermann 冻结参数

比赛导航基线中的主要生产参数包括：

- `reverse_penalty = 2.0`
- `vx_min = -0.15 m/s`
- `vx_max = 0.50 m/s`
- `vx_std = 0.02`
- `min_turning_r = 1.12 m`
- Smac minimum turning radius = `1.12 m`
- reverse safety speed limit = `0.10 m/s`
- controller frequency = `10 Hz`
- MPPI time_steps = `50`
- MPPI batch_size = `750`
- `wz_max = 0.60`

这些参数属于当前仿真 / PC 基线。

它们不是最终真实车辆标定参数。

真实车辆上线前必须重新确认：

- wheelbase；
- steering zero；
- steering sign；
- steering limit；
- steering rate；
- velocity scaling；
- acceleration scaling；
- gear semantics；
- control latency。

---

## 6. Competition PC Showcase

当前 canonical Showcase 入口：

`ros2 launch cleannav_simulation ackermann_mcity_showcase_v2.launch.py`

当前比赛 PC Showcase 已验证：

- MCity 场景启动；
- RTAB-Map；
- Smac Hybrid-A*；
- FollowPath；
- MPPI Ackermann；
- Safety Supervisor；
- Task 30 Navigation 链路；
- Leaf cleaning 场景；
- Safety Lease；
- Navigation completion；
- PC HIL Bridge。

比赛冻结基线：

`competition-hil-baseline-20260921`

对应：

`2e6b3225d9d6c571f04aab90c7bb8d94d7901709`

---

## 7. Task 30 Showcase

当前演示重点任务：

`CLEAN_NEAREST_LEAF`

Mission Manager 负责选择目标和生成导航请求。

Navigation 只负责：

- 接收有效 Navigation Goal；
- 规划；
- 跟踪；
- 控制；
- 返回 Navigation Result。

Perception Target 不应直接成为 `/goal_pose`。

---

## 8. Safety 边界

固定控制合同：

    controller_server
        ↓
    /cleannav/cmd_vel_candidate
        ↓
    Safety Supervisor
        ↓
    /cmd_vel

Safety Supervisor 负责：

- 最终速度门控；
- Safety Lease；
- emergency stop；
- forward / reverse limit；
- candidate freshness；
- autonomous authorization。

Safety Supervisor 不是全局规划器，也不是局部路径规划器。

---

## 9. HIL

当前比赛 HIL 使用 PC 与 J6M 的 HTTP/TCP Bridge。

原因是当前 WSL2 ↔ J6M 跨机器 DDS/UDP 链路不作为比赛冻结依赖。

Navigation 侧已有：

`cleannav_navigation/demo_hil_navigation_client.py`

相关 HIL 提交已经进入当前比赛分支。

HIL 的职责是传递 Navigation 请求和结果，不应创建第二条车辆运动控制链。

---

## 10. 当前已验证状态

当前已经完成并冻结：

- Ackermann Gazebo 闭环；
- Smac Hybrid-A*；
- MPPI Ackermann；
- forward / reverse 基础能力；
- Safety Supervisor；
- Safety Lease；
- RTAB-Map MCity；
- 周期重规划；
- FollowPath path replacement；
- PC Competition Showcase；
- Task 30 leaf cleaning；
- PC ↔ J6M Navigation HIL。

PC Showcase 已作为比赛演示基线冻结。

---

## 11. 尚未宣称完成的内容

当前不要把以下能力描述为已经正式完成：

- 真实车辆 Ackermann 标定；
- J6M 最终车辆控制闭环；
- 动态障碍专项完整 Runtime 验证；
- Coverage Cleaning 最终方案；
- 真实车辆长期稳定性；
- 最终 Vehicle Adapter / Arbiter；
- 最终异常工况全集。

MCity Showcase 中存在动态行人等可视化元素，但这本身不能作为动态障碍算法已经完整验证的证据。

---

## 12. 决赛下一阶段

Navigation 组后续重点建议：

1. Coverage Planner；
2. Structured Cleaning Path；
3. Dynamic Obstacle Behavior；
4. wait / slow / bypass / rejoin；
5. zone transition / U-turn / recovery；
6. Vehicle Adapter；
7. 实车 Ackermann 标定；
8. J6M Shadow / HIL；
9. 低速实车闭环；
10. 最终异常工况测试。

Hybrid-A* 可以继续承担：

- 区域间转移；
- U-turn；
- recovery；
- topology transition。

清扫区域内部可进一步评估 Coverage Planner + Local Behavior Manager，而不是只依赖点到点 Hybrid-A*。

---

## 13. 地图与运行时数据

地图资源位于：

`cleannav_navigation/maps/`

RTAB-Map 数据库通常位于：

`~/.ros/rtabmap_lidar.db`

以下运行时文件不得提交：

- RTAB 数据库；
- `build/`
- `install/`
- `log/`
- Gazebo 临时数据；
- ROS runtime logs。

---

## 14. 构建原则

在 ROS 2 Humble workspace 中构建。

推荐：

    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash

如果只构建 Navigation 相关 package，可以通过 `--packages-select` 进行选择。

---

## 15. 决赛协作方式

当前 Navigation 决赛推荐基线：

`feature/pc-demo-mcity`

组员首次获取：

    git clone https://github.com/D1zzyhub7/cleannav-navigation.git
    cd cleannav-navigation
    git fetch --all --tags
    git switch feature/pc-demo-mcity
    git pull --ff-only

开始新功能时：

    git switch -c feature/<your-feature>

不要直接重写：

- `competition-hil-baseline-20260921`
- `feature/pc-demo-mcity` 的冻结历史。

大功能应使用独立 feature branch。

---

## 16. 相关仓库

CleanNav 当前主要仓库：

- `cleannav-interfaces`
- `cleannav-navigation`
- `cleannav-mission-manager`
- `cleannav-hmi`
- `cleannav-system`

Voice 已整合到 `cleannav-hmi/voice/`，不再单独维护新的 GitHub 仓库。

系统级版本与比赛基线由 `cleannav-system` 负责记录。

---

## 17. 当前比赛冻结点

Navigation：

`2e6b3225d9d6c571f04aab90c7bb8d94d7901709`

Tag：

`competition-hil-baseline-20260921`

该冻结点用于回滚和比赛演示复现。

后续算法优化必须新建 commit / branch，不得重写该历史。
