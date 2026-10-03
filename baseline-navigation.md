# CleanNav Navigation 基线记录

## 基本信息

- 日期：2026-10-04
- 分支：ljy
- 仿真基线：feature/pc-demo-mcity
- ROS：ROS 2 Humble
- 平台：Ubuntu 22.04 / WSL2
- 项目路径：/home/lenovo/workspace/cleannav-navigation

## 启动命令

### Gazebo 仿真

```bash
ros2 launch cleannav_simulation ackermann_mcity_showcase_v2.launch.py

CleanNav Navigation 基线记录

日期：2026-10-04
分支：ljy
系统：Ubuntu 22.04 / WSL2
ROS：ROS 2 Humble
项目路径：/home/lenovo/workspace/cleannav-navigation

仿真状态：
1. Gazebo 物理服务器：正常
2. Gazebo 图形界面：gzclient 崩溃
3. /odom：有数据
4. /scan：有数据
5. Ackermann 控制器：正常

地图和定位：
1. RTAB-Map：正常
2. 地图话题：/rtabmap/map
3. 地图分辨率：0.05
4. 地图宽度：223
5. 地图高度：151
6. map 到 base_link：可以查询

导航节点：
1. planner_server：active [3]
2. controller_server：active [3]
3. global_costmap：已启动
4. local_costmap：已启动
5. cleannav_hybrid_planner_bridge：已启动
6. cleannav_path_executor：已启动
7. cleannav_navigation_facade：已启动

导航接口：
1. /cleannav/navigate_to_pose：可用
2. /compute_path_to_pose：可用
3. /follow_path：可用

控制链：
controller_server
到 /cleannav/cmd_vel_candidate
到 cleannav_safety_supervisor
到 /cmd_vel
到 ackermann_controller

当前结论：
仿真、定位、规划、控制和安全节点已经启动。
下一步可以开始算法优化。
