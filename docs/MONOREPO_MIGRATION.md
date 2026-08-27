# CleanNav Navigation Monorepo Migration Record

## 1. 目的

本文记录 `cleannav-navigation` 从原 CleanNav monorepo 提取为独立仓库的历史映射、范围、验证和回退依据。

## 2. 原始 Monorepo

原仓库：

`~/code/cleannav`

拆仓冻结状态：

- branch：`feature/mission-manager-m1`
- HEAD：`0830c301084df41fd2e39501c6d52e10c5465892`

Navigation 拆仓过程中原仓库保持不变。

## 3. 安全锚点

原 monorepo 已建立：

- branch：`backup/pre-modularization-20260825`
- tag：`pre-modularization-20260825`

二者均指向：

`0830c301084df41fd2e39501c6d52e10c5465892`

完整 bundle：

`~/code/cleannav_backups/cleannav-pre-modularization-20260825.bundle`

SHA256：

`ed19c0268b4ed995bd86d1123a3e56913b28cfe7230593e2963fcfbbd4444427`

## 4. 提取范围

保留五个 ROS 2 package：

- `cleannav_global_planner`
- `cleannav_navigation`
- `cleannav_path_executor`
- `cleannav_rtabmap`
- `cleannav_safety_supervisor`

同时保留两份静态地图资产。

过滤前边界共 39 个 tracked files。

明确排除 Interfaces、Mission Manager、monorepo docs、reference、scripts 和 tools。

## 5. 历史映射

原 Navigation 内容来自：

`0408167e69819422e90d74f236339a39cb5fe9fb`

主题：

`建立 CleanNav 导航执行基线`

过滤后映射为：

`00bd8455035cff4c515c727bcd4e3758fef20974`

关键 commit-map：

`0408167e69819422e90d74f236339a39cb5fe9fb 00bd8455035cff4c515c727bcd4e3758fef20974`

过滤结果：

- commit count：1
- tracked files：39
- byte mismatches：0

## 6. Refs 治理

过滤后曾继承：

- `feature/mission-manager-m1`
- `pre-modularization-20260825`
- 一条 filter-repo replace ref

replace ref 来源确认后已删除。

随后：

`feature/mission-manager-m1 -> main`

继承 tag 已删除。

当前：

- branch：`main`
- tags：none
- replace refs：none
- remotes：none

## 7. 历史基线验证

`00bd8455035cff4c515c727bcd4e3758fef20974` 已在独立 clean shell 验证：

- ROS 2 Humble
- 5 packages
- build PASS
- 25 tests
- 0 errors
- 0 failures
- 0 skipped

## 8. 独立运行适配

适配提交：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

父提交：

`00bd8455035cff4c515c727bcd4e3758fef20974`

主要适配：

- 地图迁入 `cleannav_navigation/maps/`
- 地图安装到 package share
- Nav2 默认地图取消 monorepo 路径依赖
- RTAB-Map DB 默认改为 `~/.ros/rtabmap_lidar.db`
- 补充 `ament_index_python` 与 `launch` 运行依赖

两份地图均为 100% rename，内容未改变。

## 9. 当前 Controller

当前 active controller：

`dwb_core::DWBLocalPlanner`

TEB 当前未激活。

未来 TEB 切换必须作为独立导航功能变更验证。

## 10. Safety Supervisor 边界

Safety Supervisor 保留在 Navigation 仓库，负责最终速度安全门控和仲裁。

它不替代局部规划器，也不承担 Mission Manager 状态机职责。

## 11. 独立适配验证

适配后再次验证：

- 5 packages
- build PASS
- 25 tests
- 0 errors
- 0 failures
- 0 skipped
- 地图安装 PASS
- package prefix PASS
- monorepo map path absence PASS
- repo-local build/install/log absence PASS

## 12. 回退原则

组件级回退使用 `cleannav-navigation` 自身 Git 历史。

未来系统级回退由 `cleannav-system` 通过 Git submodule 固定各组件精确 SHA。

拆仓前完整系统仍可由原 monorepo bundle 恢复。
