# cleannav_path_executor Launch
# 仅启动 Path Bridge 节点，不启动 controller_server、Gazebo、RTAB-Map、A*
# 不 publish Twist，不 remap 到 /cmd_vel

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    completion_window_xy_tolerance = LaunchConfiguration(
        'completion_window_xy_tolerance')
    robot_base_frame = LaunchConfiguration('robot_base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'completion_window_xy_tolerance',
            default_value='0.25',
            description='Nav2 goal-checker xy tolerance used for completion window',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_link',
            description='Robot base frame used for completion-window TF lookup',
        ),

        Node(
            package='cleannav_path_executor',
            executable='path_executor_node',
            name='cleannav_path_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'completion_window_xy_tolerance': completion_window_xy_tolerance,
                'robot_base_frame': robot_base_frame,
            }],
        ),
    ])
