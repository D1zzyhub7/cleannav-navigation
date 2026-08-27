# CleanNav Navigation 项目状态

## 1. 模块定位

`cleannav-navigation` 是 CleanNav 的独立导航组件仓库。

当前包含：

- `cleannav_global_planner`
- `cleannav_navigation`
- `cleannav_path_executor`
- `cleannav_rtabmap`
- `cleannav_safety_supervisor`

## 2. 当前 Git 基线

历史提取基线：

`00bd8455035cff4c515c727bcd4e3758fef20974`

独立运行适配：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

当前分支：

`main`

## 3. 当前导航链路

地图 / RTAB-Map
→ A* Global Planner
→ Path Bridge
→ Nav2 FollowPath
→ DWB Controller
→ Safety Supervisor
→ 最终速度输出边界

## 4. 当前 Controller

当前 active controller：

`dwb_core::DWBLocalPlanner`

当前事实：

`DWB = 当前事实`

后续目标：

`TEB = 后续目标`

TEB 当前未激活，未来切换必须作为独立导航功能变更验证。

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
