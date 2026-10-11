# Jetson Autoware 现场操作与状态索引

本页是 2026-10-11 现场知识索引。它描述 Jetson 上 ROS 1 Melodic / Autoware 的证据，不表示 CleanNav ROS 2 / Nav2 已完成平台集成。

详细记录见 [2026-10-11 工作日志](../worklogs/2026-10-11-jetson-autoware.md)。

## 已确认状态

- AGX Xavier，Ubuntu 18.04.3 / glibc 2.27 / L4T R32.3.1 / ROS Melodic；使用 SSH 与文件传输。
- 所选 Autoware 本地备份 11,990 文件、1,333,318,857 bytes，checksum 复核 0 差异；保存在 `C:\CleanNav\autoware_backup`。
- 两颗 UB960 均 RX2/3 锁定并有图像输入；V4L2 video2 单帧试验仍超时且 0 bytes，相机未修复。
- 离线建图时 `/points_raw` 不能同时混入实时雷达和 bag 输入。
- XYZI 数据缺少 ring，不能直接交给依赖 ring field 的过滤器。外部 voxel 与去地面是不同步骤。
- 用户最终报告地图正常并采用 ray 水平分组去地面；当前 ray 持续运行未复核。
- 已保存 PCD：`/data/datalog/mapping_test_261011_zhang/autoware-261011.pcd`，ASCII XYZIntensity，338,850 点，12,099,619 bytes。

## 相机排查顺序

1. 记录物理视角与 RX 输入；目前用户从左至右标注后视、前视、右前视、左前视，尚无设备映射。
2. 实测输入 VC、UB960 输出 VC、CSI 接收通道和 `/dev/video*` 的对应。
3. 核对 1920×1080 输入、1824×940 默认 V4L2 设置与 DTS RGB888 / UYVY 不一致。
4. 先证明能取到非空完整帧，再添加 V4L2 ROS 发布节点。

不能盲跑厂家示例中的 i2c1；现场总线为 bus30 / bus32。已有 video2 试验没有改变 UB 转发、VC、boot、DTS；回滚时 colorspace 被驱动从 DEFAULT 规范化为 SRGB。

## 点选目标准备

用户选用地图点选目标。无 vector map 的当前条件下，可评估 Autoware `astar_navi` 的栅格、当前 pose 与 `/move_base_simple/goal` 路线。默认 24×12 m 局部 costmap 不足以自动代表整幅地图，需要全局可通行栅格。

先验证 NDT 定位、`/current_pose`、frame 和栅格范围，再检查静态路径输出。现场尚无定位与车辆运动闭环证据。ROS 1 Autoware 与 CleanNav ROS 2 Nav2 的控制桥接须另行验证。

## 证据与保密边界

详细日志保留相机试验目录、地图文件元数据和最终截图。私钥、凭据、rosbag、PCD、厂家手册及本地备份不上传。截图只证明可视化状态，不作为车辆运行通过的证据。
