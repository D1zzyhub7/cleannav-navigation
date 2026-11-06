# Localization RTAB-Map launch. The selected map profile supplies its scan and
# odometry mode. The roof-map profile uses odom_frame_id=odom (TF mode); the
# historical MCity profile uses odom_frame_id='' and odom_topic=/odom.
#
# Installed rtabmap_launch declares odom_frame_id as: if set, TF is used to get
# odometry instead of the topic. rtabmap_slam 0.22.1 then looks up
# odom → frame_id (base_footprint) and drops the frame when that TF is missing.
# It does not also consume the /odom pose, so the static
# ackermann_rear_axle → base_footprint offset is applied by the existing TF
# publisher in TF mode. Topic mode consumes the saved map's historical /odom
# input contract. Neither mode changes the controller's odometry calculation.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch_path = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
    cleannav_rtabmap_dir = get_package_share_directory('cleannav_rtabmap')
    default_cfg = os.path.join(cleannav_rtabmap_dir, 'config', 'rtabmap_lidar.ini')
    default_db = os.path.expanduser('~/.ros/rtabmap_lidar.db')

    return LaunchDescription([
        DeclareLaunchArgument('core_log_args', default_value='', choices=['', '--uinfo', '--udebug']),
        DeclareLaunchArgument('localization', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_pose', default_value=''),
        DeclareLaunchArgument('scan_topic', description='Explicit scan stream bound to the saved map contract'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom',
            description='Non-empty selects TF odometry; empty preserves the /odom topic input'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
            description='Wheel odometry topic used when odom_frame_id is empty'),
        DeclareLaunchArgument('database_path', default_value=default_db,
            description='RTAB-Map database path (default: ~/.ros/rtabmap_lidar.db)'),
        DeclareLaunchArgument('cfg', default_value=default_cfg,
            description='RTAB-Map INI config file (RTAB-Map native format, not ROS2 YAML)'),
        DeclareLaunchArgument('log_level', default_value='info', description='ROS logging level'),
        DeclareLaunchArgument('queue_size', default_value='20',
            description='Topic synchronizer queue size'),
        DeclareLaunchArgument('approx_sync', default_value='true',
            description='Approximate time policy for the remaining sensor inputs'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.0',
            description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                'depth': 'false',
                'localization': LaunchConfiguration('localization'),
                'initial_pose': LaunchConfiguration('initial_pose'),
                'subscribe_scan': 'true',
                'subscribe_scan_cloud': 'false',
                'subscribe_rgb': 'false',
                'subscribe_rgbd': 'false',
                'stereo': 'false',
                'visual_odometry': 'false',
                'icp_odometry': 'false',
                'rtabmap_viz': 'false',
                'rviz': 'false',
                'use_sim_time': 'true',
                'frame_id': 'base_footprint',
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'map_frame_id': 'map',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'map_topic': 'map',
                'publish_tf_map': 'true',
                'approx_sync': LaunchConfiguration('approx_sync'),
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                'database_path': LaunchConfiguration('database_path'),
                'log_level': LaunchConfiguration('log_level'),
                'queue_size': LaunchConfiguration('queue_size'),
                'args': LaunchConfiguration('core_log_args'),
                'rtabmap_args': '',
                'cfg': LaunchConfiguration('cfg'),
                'namespace': 'rtabmap',
            }.items(),
        ),
    ])
