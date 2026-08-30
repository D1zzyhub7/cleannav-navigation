# CleanNav Navigation 项目状态

## 1. 模块定位

`cleannav-navigation` 是 CleanNav 的独立导航组件仓库。

当前包含：

- `cleannav_global_planner`
- `cleannav_navigation`
- `cleannav_path_executor`
- `cleannav_rtabmap`
- `cleannav_safety_supervisor`

当前 `main` 稳定基线包含 5 个 package；Ackermann V1 特性分支新增 `cleannav_simulation` 后，特性代码树包含 6 个 package。

## 2. Stable main baseline：`main`（稳定主线基线）

`main` 是已验证的差速底盘导航稳定基线，当前基线 HEAD：

`f38568842998a2c71c272e9b5d5a804f86d6fcf7`

主线链路：

`RTAB-Map / localization → CleanNav A* → /cleannav/global_path → Path Executor → Nav2 FollowPath → DWB → Safety Supervisor → /cmd_vel`

历史提取基线：

`00bd8455035cff4c515c727bcd4e3758fef20974`

独立运行适配：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

## 3. Ackermann feature status：V1 特性状态

`feature/ackermann-hybrid-mppi-v1` 是未合并到 `main` 的 Ackermann V1 开发分支。已验证代码 HEAD：

`e1446e35b2dbb983272a0cd587024c88e1347268`

该特性包含以下 4 个增量提交：

- `af16d83`：simulation baseline
- `9040eeb`：lidar + sensor TF
- `902cbf3`：Smac Hybrid-A*
- `e1446e3`：MPPI Ackermann

Ackermann 仿真链路：

`/goal_pose → Hybrid Planner Bridge → ComputePathToPose → SmacPlannerHybrid → /cleannav/global_path → Path Executor / FollowPath → MPPI Ackermann → /cleannav/cmd_vel_candidate → Safety Supervisor → /cmd_vel → Ackermann controller`

已确认的运行时事实：

- `FollowPath`：`SUCCEEDED`。
- MPPI 运行时运动模型：`Ackermann`。
- MPPI `min_turning_r`：`1.12 m`。
- Smac `minimum_turning_radius`：`1.12 m`。
- Gazebo `/odom` 观察到位移：`0.141 m`。
- 最终 `/cmd_vel` 发布者：Safety Supervisor。
- N-A5C Safety closed-loop runtime：`PASS`。

`1.12 m` 是当前仿真几何推导的 `SIMULATION_PLACEHOLDER`，不是最终真实车辆参数；上述证据不代表真实硬件验证。不得将该特性描述为已合并主线。

## 4. 当前 Git 基线记录

历史提取基线：

`00bd8455035cff4c515c727bcd4e3758fef20974`

独立运行适配：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

治理分支：

`chore/github-governance`

主线稳定基线使用 DWB；Ackermann V1 使用 Smac Hybrid-A* 与 MPPI Ackermann。两者属于不同分支状态，不应混写为单一“当前 controller”事实。

## 5. 地图状态

静态地图：

- `cleannav_navigation/maps/cleannav_first_map.yaml`
- `cleannav_navigation/maps/cleannav_first_map.pgm`

地图由 `cleannav_navigation` 安装到 package share。

## 6. RTAB-Map 数据库

默认数据库：

`~/.ros/rtabmap_lidar.db`

该数据库属于运行时状态，不应提交到 Git。

## 7. Safety Supervisor 边界

Safety Supervisor：

- 负责最终速度安全门控。
- 不替代局部规划器。
- 不承担 Mission Manager 状态机。
- 保持在 Navigation 仓库中。

## 8. Path Bridge 边界

`cleannav_path_executor` 将 CleanNav Path 提交给 Nav2 `FollowPath`。

它不直接发布最终 Twist。

## 9. 当前验证状态

历史提取基线：

- 5 packages
- build PASS
- 25 tests
- 0 errors
- 0 failures
- 0 skipped

独立运行适配：

- 5 packages
- build PASS
- 25 tests
- 0 errors
- 0 failures
- 0 skipped

额外验证：

- 地图安装 PASS
- package prefix PASS
- monorepo 路径消除 PASS
- repo-local build/install/log absence PASS

## 10. 历史映射

原 monorepo：

`0408167e69819422e90d74f236339a39cb5fe9fb`

独立仓库：

`00bd8455035cff4c515c727bcd4e3758fef20974`

映射：

`0408167e69819422e90d74f236339a39cb5fe9fb -> 00bd8455035cff4c515c727bcd4e3758fef20974`

## 11. 当前治理状态

已经完成：

- G5.1 Navigation 边界审计
- G5.2 历史与运行资产边界冻结
- G5.3 历史过滤
- G5.4 独立基线验证
- G5.5 branch/tag/replace-ref 治理
- G5.6 独立运行适配与验证
- G5.7 治理文件生成
- N-A5C Ackermann MPPI Safety closed-loop runtime 验证

尚需完成：

- 治理文件总审查
- governance commit
- postcommit validation
- Navigation baseline tag
- G5 最终闭环

## 12. 当前禁止事项

Navigation 闭环前：

- 不建立 GitHub remote
- 不 push
- 不切换 TEB
- 不修改导航算法行为
- 不改变 Safety Supervisor 职责
- 不混入 Mission Manager / Interfaces / Perception / HMI

## 13. 后续顺序

治理文件审查
→ governance commit
→ postcommit validation
→ baseline tag
→ Navigation G5 完整闭环
→ 新对话接力
→ 继续下一个组件模块化

所有组件完成后，再建立 `cleannav-system`，使用 Git submodule 固定各组件精确 SHA。
