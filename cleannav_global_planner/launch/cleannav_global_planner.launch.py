"""Boot A* planner only — no Gazebo, RTAB-Map, Nav2, AMCL, TEB, /cmd_vel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        Node(
            package='cleannav_global_planner',
            executable='global_planner_node',
            name='cleannav_global_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
