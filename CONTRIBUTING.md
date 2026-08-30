# 贡献 CleanNav Navigation

- `main` 只接受稳定且通过对应 Gate 验证的代码；不要直接在 `main` 开发。
- 新功能使用 `feature/*`，缺陷修复使用 `fix/*`。
- 一个 PR 只处理一个清晰范围；提交 PR 前完成适用的 build/test。
- 禁止对 `main` force push；禁止改写、reset 或 rebase 共享历史。
- 不要提交 `build/`、`install/`、`log/`，也不要提交 token、password、private key、本地数据库或 rosbag。
- 同一 feature 分支避免多人同时直接编辑；合并前必须完成 review。
- Navigation、Mission Manager、Interfaces 的跨仓库接口变化必须显式协调并记录兼容性影响。
