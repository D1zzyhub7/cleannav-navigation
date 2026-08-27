# Changelog

本文件记录 `cleannav-navigation` 独立仓库的重要变化。

## [Unreleased]

### Repository modularization

- 从原 CleanNav monorepo 提取 Navigation。
- 当前独立仓库包含 5 个 ROS 2 package。
- 历史过滤基线：
  `00bd8455035cff4c515c727bcd4e3758fef20974`
- 主分支治理为 `main`。
- monorepo 继承 tag 与 replace ref 已清理。
- 当前尚未建立 GitHub remote。

### Standalone runtime adaptation

独立运行适配提交：

`54a3beb6d8ffd6bf6a476ea910d057aa9c8481ae`

主要变化：

- 地图迁入 `cleannav_navigation/maps/`。
- 地图安装到 `cleannav_navigation` package share。
- Nav2 默认地图不再依赖原 monorepo 路径。
- RTAB-Map 默认数据库改为 `~/.ros/rtabmap_lidar.db`。
- 补充 `ament_index_python` 和 `launch` 运行依赖。
- 未修改导航算法和 Safety Supervisor 控制逻辑。

### Independent validation

已验证：

- ROS 2 Humble。
- 5 packages。
- build PASS。
- 25 tests。
- 0 errors。
- 0 failures。
- 0 skipped。
- 地图安装 PASS。
- package prefix PASS。
- repo-local `build/`、`install/`、`log/` 保持不存在。

### Controller boundary

当前 active controller：

`dwb_core::DWBLocalPlanner`

TEB 当前未激活，属于后续独立导航功能演进。

## Historical baseline

原 monorepo Navigation 源提交：

`0408167e69819422e90d74f236339a39cb5fe9fb`

过滤后对应：

`00bd8455035cff4c515c727bcd4e3758fef20974`

原始提交主题：

`建立 CleanNav 导航执行基线`
