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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),

        Node(
            package='cleannav_path_executor',
            executable='path_executor_node',
            name='cleannav_path_executor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
