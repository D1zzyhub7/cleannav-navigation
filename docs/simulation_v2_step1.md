# PC Simulation V2 Step 1：执行权与安全边界

## 默认执行链

```text
TaskCommand → Mission Manager Core → RealNavigationAdapter
  → /cleannav/navigate_to_pose (Navigation Facade)
  → /compute_path_to_pose → /follow_path → Nav2 controller
  → /cleannav/cmd_vel_candidate → Safety Supervisor → /cmd_vel
```

`cleannav_ackermann_navigation.launch.py` 的 `execution_mode` 默认 `facade`。
此模式只启动 Facade，不启动 Hybrid Planner Bridge / Path Executor；
`/goal_pose` 不会触发该启动链的执行。`execution_mode:=legacy` 则只启动
Bridge / Executor，关闭 Facade；两种模式始终保留 Safety Supervisor。
Legacy 模式不能服务 Mission Manager 的 Facade Action，不能用于任务主链。
Legacy `/goal_pose` 自身不获得 Lease，默认仍被 Safety 阻断。

这保证本启动链的单执行客户端，不保证 ROS 图的全局访问隔离。
外部节点仍可直接调用底层 Nav2 Action，或运行另一套启动链；
未配置 DDS 权限控制，原始 Twist 也不携带 execution_id。

## Ownership 与后续恢复扩展点

| 资源或操作 | Owner |
| --- | --- |
| 原始目标 Pose 的深拷贝 | Facade `_Session.original_goal` |
| ComputePathToPose、FollowPath 的提交与取消 | Facade runtime |
| 内层 goal handle、result future、阶段 | 同一 Facade session |
| 向 Mission Manager 返回最终导航结果 | Facade 外层 Action → RealNavigationAdapter |
| task / execution / generation、任务控制意图 | Mission Manager Core |
| Lease acquire / release | Core → RealSafetyAdapter → Safety Supervisor |
| 速度授权、ESTOP、freshness、最终限幅 | Safety Supervisor |

Facade 只接受一个在途外层目标，不自动抢占。外层 abort 并不证明内层已停；
取消被拒绝或结果异常时保持 busy，直到同一内层 handle 的真实 terminal。
目标提交回复不明确或结果永久丢失时会保持关闭，需要诊断后重启；
当前未实现自动恢复超时策略。
旧 session 回调不得释放新 session 的 busy。普通 FollowPath ABORT 保持失败。

后续 TRACKING → STOP/WAIT → REPLAN → RESTART FOLLOW_PATH 应放在同一
Facade session 中：保留原始 Pose，先确认旧 FollowPath terminal，再重新规划
和提交；不得重新启用并行 Executor。Core 保留任务控制和 Lease 生命周期，
中间重规划不向 Core 伪造成功。本阶段没有实现周期重规划、WAIT 或恢复行为树。

## 终态与 Lease 清理

LEASE_ACQUIRING 阶段缓存第一次导航终态；Lease 成功后释放一次，再报告任务
终态；Lease 拒绝时不能将提前 success 作为授权完成。CANCELING 阶段的自然
success / failure 作为动作已停的证据，保留 PAUSE / STOP / ESTOP 意图，等待
Lease 清理。重复导航终态及重复释放回调不会重复执行清理副作用。

RealSafetyAdapter 对已发出的 service 请求使用默认 2 秒 monotonic deadline，
STEADY_TIME 定时器检查。超时发送 ESTOP、通知 Core 锁定紧停，再报告操作失败；
operation token 丢弃迟到回复。Lease release 失败还会独立发送 ESTOP，保留
SAFETY_BLOCKED / cleanup_failed，不能报告任务成功或自动恢复运动。
此状态仍需人工诊断清理，不承诺每一种通信故障都会产生正常任务终态。

## Safety 默认和限制

`safety_block_all` 默认仍为 `true`；block_all 或 ESTOP 下拒绝 acquire，
Lease 成功不等于解除 block_all。仿真运行必须由操作者另行显式配置授权；
测试中的解除只用于验证授权路径，不改变启动默认值。

Safety 检查 Twist 六个分量均有限，且只允许 linear.x / angular.z。
非法输入立即清空缓存并输出零。释放、reset、ESTOP、新 owner 授权和 legacy
授权超时清空旧速度；新 owner 必须收到新 candidate 才能运动。
输出定时器使用 STEADY_TIME，candidate 同时检查 ROS age 和 monotonic age，
即使 /clock 暂停也会在 candidate timeout 后归零。
降低 vx 时同比缩放 wz，再限幅 wz，避免限速导致曲率收紧；没有修改 MPPI 参数。

Lease 尚无 TTL / heartbeat。Mission Manager 死亡、Nav2 仍持续输出新 candidate
时，Lease 可继续有效，属于下一阶段必须处理的失效风险。ESTOP 通过 ROS topic
发送，仍依赖 DDS 交付和 Safety 进程存活，不能代替车辆硬件急停。
Task 30 的 DemoPhysicalCompletionNavigationAdapter 演示专用映射保持不变；
其演示完成不能视为标准导航或覆盖清扫的完成证明。
