# CleanNav RTAB-Map LiDAR 2D 建图包装 launch
#
# 包装官方 rtabmap_launch/rtabmap.launch.py，预设 LiDAR 2D 模式：
#   - depth=false（关键：禁用 Depth/RGB-D 回调）
#   - approx_sync=true（/odom 与 /scan 使用近似时间同步）
#   - 不订阅 RGB / Depth / Stereo / RGB-D
#   - 不使用视觉里程计或 ICP 里程计（使用轮式 /odom topic）
#   - 不启动 rtabmap_viz、rviz2
#   - 不输出 /cmd_vel
#   - 不集成 Nav2 / AMCL / slam_toolbox
#
# 修正记录：
#   第 2 次运行：LiDAR-only 输入链已验证（subscribe_depth=false,
#   subscribe_rgb=false, subscribe_scan=true）；精确同步 exact sync
#   导致 /odom 与 /scan 无法稳定配对，追加 approx_sync=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')

    cleannav_rtabmap_dir = get_package_share_directory('cleannav_rtabmap')
    default_cfg = os.path.join(cleannav_rtabmap_dir, 'config', 'rtabmap_lidar.ini')
    default_db = os.path.expanduser('~/code/cleannav/maps/rtabmap_lidar.db')

    # --- 顶层可覆盖参数 ---
    database_path = LaunchConfiguration('database_path')
    cfg_file = LaunchConfiguration('cfg')
    log_level = LaunchConfiguration('log_level')
    queue_size = LaunchConfiguration('queue_size')
    approx_sync = LaunchConfiguration('approx_sync')
    approx_sync_max_interval = LaunchConfiguration('approx_sync_max_interval')

    # --- 参数传递策略 ---
    # cfg: 指向 rtabmap_lidar.ini（RTAB-Map INI 格式，存放 Grid/Reg 参数）
    # args: 保持为空，不重复设置 cfg 中已有的参数
    # depth: false（关键！覆盖官方默认 stereo=false→depth=true 的行为）

    return LaunchDescription([
        DeclareLaunchArgument('database_path',
            default_value=default_db,
            description='RTAB-Map database path (default: ~/code/cleannav/maps/rtabmap_lidar.db)'),

        DeclareLaunchArgument('cfg', default_value=default_cfg,
            description='RTAB-Map INI config file (RTAB-Map native format, not ROS2 YAML)'),

        DeclareLaunchArgument('log_level', default_value='info',
            description='ROS logging level'),

        DeclareLaunchArgument('queue_size', default_value='20',
            description='Topic synchronizer queue size'),

        DeclareLaunchArgument('approx_sync', default_value='true',
            description='If timestamps of the input topics should be synchronized using approximate or exact time policy'),

        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.0',
            description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                # === 关键：禁用 Depth/RGB-D ===
                # depth 官方默认值：stereo=false → true（行 45）
                # 必须显式设为 false 以阻止 Depth 回调
                'depth':                     'false',

                # --- 传感器模式 ---
                'subscribe_scan':            'true',
                'subscribe_scan_cloud':      'false',
                'subscribe_rgb':             'false',
                'subscribe_rgbd':            'false',
                'stereo':                    'false',

                # --- 里程计（使用 /odom topic，非 odom_frame_id TF 模式） ---
                'visual_odometry':           'false',
                'icp_odometry':              'false',

                # --- UI ---
                'rtabmap_viz':               'false',
                'rviz':                      'false',

                # --- 仿真与坐标系 ---
                'use_sim_time':              'true',
                'frame_id':                  'base_footprint',
                'map_frame_id':              'map',

                # --- 话题 ---
                'odom_topic':                '/odom',
                'scan_topic':                '/scan',

                # --- 地图输出 ---
                'map_topic':                 'map',
                'publish_tf_map':            'true',

                # --- 同步 ---
                'approx_sync':               approx_sync,
                'approx_sync_max_interval':  approx_sync_max_interval,

                # --- 数据库 ---
                'database_path':             database_path,

                # --- 日志与队列 ---
                'log_level':                 log_level,
                'queue_size':                queue_size,

                # --- RTAB-Map CLI 参数（保持为空，参数在 .ini 中管理） ---
                'args':                      '',
                'rtabmap_args':              '',

                # --- 配置 ---
                'cfg':                       cfg_file,
                'namespace':                 'rtabmap',
            }.items(),
        ),
    ])
