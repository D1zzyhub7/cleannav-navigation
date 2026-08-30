# CleanNav Navigation

CleanNav Navigation 是 CleanNav 的独立 ROS 2 导航仓库，负责定位包装、全局规划、路径执行、Nav2 控制集成，以及最终速度安全门控。

## 仓库内容

仓库代码线合计包含 6 个 ROS 2 package：`main` 稳定基线包含 5 个，Ackermann V1 特性分支新增仿真包后包含 6 个：

- `cleannav_simulation`：Gazebo 仿真、底盘与控制器。
- `cleannav_navigation`：Nav2 参数、bringup 与地图资源。
- `cleannav_global_planner`：CleanNav A* 与 Ackermann Hybrid Planner Bridge。
- `cleannav_path_executor`：将路径提交给 Nav2 `FollowPath`。
- `cleannav_safety_supervisor`：候选速度安全门控与最终速度仲裁。
- `cleannav_rtabmap`：RTAB-Map LiDAR 定位包装。

其中 `cleannav_simulation` 属于 `feature/ackermann-hybrid-mppi-v1` 的 Ackermann Gazebo 仿真代码；它不代表已经合并到 `main`。

## Stable main baseline：`main`（稳定主线）

`main` 保持已验证的差速底盘导航基线，当前基线 HEAD 为
`f38568842998a2c71c272e9b5d5a804f86d6fcf7`。

```text
RTAB-Map / localization
  → CleanNav A* global planner
  → /cleannav/global_path
  → Path Executor
  → Nav2 FollowPath
  → DWB
  → Safety Supervisor
  → /cmd_vel
```

Safety Supervisor 是最终 `/cmd_vel` 的唯一发布者。Path Executor、DWB 和其他上游组件只能参与候选控制链路，不能绕过 Safety Supervisor 直接发布最终速度。

## Ackermann feature status：V1 特性分支

`feature/ackermann-hybrid-mppi-v1` 是 Ackermann V1 开发分支，不表示已合并到 `main`，其已验证代码 HEAD 为
`e1446e35b2dbb983272a0cd587024c88e1347268`。该分支的闭环链路为：

```text
/goal_pose
  → Hybrid Planner Bridge
  → ComputePathToPose
  → SmacPlannerHybrid
  → /cleannav/global_path
  → Path Executor / FollowPath
  → MPPI Ackermann
  → /cleannav/cmd_vel_candidate
  → Safety Supervisor
  → /cmd_vel
  → Ackermann controller
```

已验证的 Ackermann 仿真要点包括：`gazebo_ros2_control`、`ackermann_steering_controller`、`/odom`、`/scan`，以及 `odom → base_footprint → base_link → base_scan` TF 链。MPPI 运行时运动模型为 Ackermann；Smac 与 MPPI 的最小转弯半径均为 `1.12 m`。

### N-A5C 运行证据

- Nav2 `FollowPath`：`SUCCEEDED`。
- MPPI Ackermann：运行时运动模型确认。
- Gazebo `/odom`：观察到位移 `0.141 m`。
- 最终 `/cmd_vel`：由 Safety Supervisor 发布，控制器不是最终速度所有者。
- `1.12 m` 是由当前仿真几何推导的 `SIMULATION_PLACEHOLDER`，不是最终真实车辆参数。

上述结果仅代表 Ackermann Gazebo 仿真闭环验证，不代表真实硬件验证。仓库当前不包含 Mission Manager，也不允许 Mission Manager 或 APP/语音等外部入口直接发布 `/cmd_vel` 或绕过导航链路。

## Safety 边界

控制合同固定为：

```text
controller_server → /cleannav/cmd_vel_candidate → Safety Supervisor → /cmd_vel
```

Safety Supervisor 负责最终速度门控、限幅、急停与授权仲裁；它不是局部规划器。动态障碍响应由局部规划器与 costmap 负责。

## 地图与运行时数据

- 地图：`cleannav_navigation/maps/cleannav_first_map.yaml` 与对应 PGM。
- RTAB-Map 默认数据库：`~/.ros/rtabmap_lidar.db`。
- 运行时数据库、colcon 产物和日志不得提交到 Git。

## 仓库边界

本仓库不包含 `cleannav_interfaces`、Mission Manager、Perception 或 HMI。跨组件 ROS 接口由独立接口仓库维护。

详细状态与验证记录见 [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md)。
