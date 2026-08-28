"""只启动 CleanNav Hybrid Planner Bridge。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_id = LaunchConfiguration('planner_id')
    map_frame = LaunchConfiguration('map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('planner_id', default_value='GridBased'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='cleannav_global_planner',
            executable='hybrid_planner_bridge_node',
            name='cleannav_hybrid_planner_bridge',
            output='screen',
            parameters=[{
                'planner_id': planner_id,
                'map_frame': map_frame,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
