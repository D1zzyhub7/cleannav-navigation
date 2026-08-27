# CleanNav Navigation

`cleannav-navigation` 是 CleanNav 的独立导航组件仓库，负责建图包装、全局规划、路径执行、Nav2 配置以及最终速度安全门控。

## 1. 当前状态

当前关键 Git 基线：

- 历史提取基线：`00bd8455035cff4c515c727bcd4e3758fef20974`
- 独立运行适配：`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`
- 主分支：`main`

当前 active Nav2 controller 为 `dwb_core::DWBLocalPlanner`。

TEB 当前未激活，属于后续导航技术路线。

## 2. 仓库内容

包含五个 ROS 2 package：

- `cleannav_global_planner`：A* 全局规划。
- `cleannav_navigation`：Nav2 参数与 bringup。
- `cleannav_path_executor`：Path Bridge，将 Path 提交给 Nav2 `FollowPath`。
- `cleannav_rtabmap`：RTAB-Map LiDAR 2D 包装。
- `cleannav_safety_supervisor`：候选速度安全门控与最终速度仲裁。

Safety Supervisor 不替代局部规划器；Path Bridge 不直接发布最终 Twist。

## 3. 当前导航链路

RTAB-Map / 地图与定位输入
→ A* Global Planner
→ Path Bridge
→ Nav2 FollowPath
→ DWB Controller
→ Safety Supervisor
→ 最终速度输出边界

## 4. 地图与运行时数据

默认静态地图：

- `cleannav_navigation/maps/cleannav_first_map.yaml`
- `cleannav_navigation/maps/cleannav_first_map.pgm`

地图安装到 `cleannav_navigation` package share，不再依赖原 monorepo 的绝对路径。

RTAB-Map 默认数据库：

`~/.ros/rtabmap_lidar.db`

数据库属于运行时状态，不应提交到 Git。

## 5. 独立验证

当前 standalone validation：

- ROS 2 Humble
- 5 packages
- build PASS
- 25 tests
- 0 errors
- 0 failures
- 0 skipped

同时已验证地图安装、package prefix、monorepo 路径消除，以及 repo-local `build/`、`install/`、`log/` 不产生污染。

## 6. 仓库边界

本仓库不包含：

- `cleannav_interfaces`
- `cleannav_mission_manager`
- Perception
- HMI

跨组件 ROS 合同由独立 `cleannav-interfaces` 仓库维护。

## 7. Git 历史

原 monorepo Navigation 源提交：

`0408167e69819422e90d74f236339a39cb5fe9fb`

过滤后的独立历史基线：

`00bd8455035cff4c515c727bcd4e3758fef20974`

独立运行适配：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

详细迁移记录见 `docs/MONOREPO_MIGRATION.md`。

## 8. 开发原则

- 功能修改与仓库治理分开提交。
- 当前 DWB 状态必须如实记录。
- TEB 切换必须作为独立功能变更验证。
- Safety Supervisor 保持最终安全门控职责。
- 运行时数据库和 colcon 产物不得进入 Git。
- 系统级版本最终由 `cleannav-system` 固定各组件精确 SHA。
