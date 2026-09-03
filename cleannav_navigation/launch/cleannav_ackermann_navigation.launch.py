"""启动 CleanNav Ackermann V1 的 Nav2 控制闭环。

This launch DOES NOT start:
  - Gazebo / simulation
  - robot_state_publisher
  - ros2_control
  - Ackermann chassis controller
  - RTAB-Map
  - AMCL
  - map_server
  - Nav2 localization
  - velocity_smoother
  - Mission Manager
  - HMI
  - perception

外部必须提供 map、TF、odom、scan，或等价的 Navigation 所需环境。
本 launch 不使用标准 Nav2 bringup，也不声明 full CleanNav system runtime integration。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory('cleannav_navigation')
    global_planner_share = get_package_share_directory('cleannav_global_planner')
    path_executor_share = get_package_share_directory('cleannav_path_executor')
    safety_share = get_package_share_directory('cleannav_safety_supervisor')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    planner_id = LaunchConfiguration('planner_id')
    map_frame = LaunchConfiguration('map_frame')
    log_level = LaunchConfiguration('log_level')
    safety_block_all = LaunchConfiguration('safety_block_all')
    safety_autonomous_timeout_sec = LaunchConfiguration(
        'safety_autonomous_timeout_sec')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    planner_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            global_planner_share,
            'launch',
            'cleannav_hybrid_planner_bridge.launch.py',
        )),
        launch_arguments={
            'planner_id': planner_id,
            'map_frame': map_frame,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    path_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            path_executor_share,
            'launch',
            'cleannav_path_executor.launch.py',
        )),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    safety_supervisor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            safety_share,
            'launch',
            'safety_supervisor.launch.py',
        )),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'block_all': safety_block_all,
            'autonomous_timeout_sec': safety_autonomous_timeout_sec,
        }.items(),
    )

    node_log_args = ['--ros-args', '--log-level', log_level]

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=node_log_args,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', '/cleannav/cmd_vel_candidate')],
        arguments=node_log_args,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['planner_server', 'controller_server'],
        }],
        arguments=node_log_args,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_share, 'config', 'nav2_params_ackermann.yaml'),
            description='Full path to the Ackermann Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo clock when explicitly enabled',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate managed nodes',
        ),
        DeclareLaunchArgument(
            'planner_id',
            default_value='GridBased',
            description='Nav2 planner plugin id',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Frame used by the Hybrid Planner Bridge',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for direct Nav2 nodes',
        ),
        DeclareLaunchArgument(
            'safety_block_all',
            default_value='true',
            description='Block candidate velocities by default',
        ),
        DeclareLaunchArgument(
            'safety_autonomous_timeout_sec',
            default_value='5.0',
            description='Safety autonomous authorization timeout',
        ),
        planner_server,
        controller_server,
        lifecycle_manager,
        planner_bridge,
        path_executor,
        safety_supervisor,
    ])
