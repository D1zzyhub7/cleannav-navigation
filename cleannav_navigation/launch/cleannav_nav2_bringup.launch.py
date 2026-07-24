import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    cleannav_dir = get_package_share_directory('cleannav_navigation')

    map_yaml_file = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'code/cleannav/maps/cleannav_first_map.yaml',
        ]))

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(cleannav_dir, 'config', 'nav2_params.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    slam = LaunchConfiguration('slam', default='False')
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='False')
    log_level = LaunchConfiguration('log_level', default='info')

    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether run a SLAM'),

        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed bringup'),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='log level'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'slam': slam,
                'autostart': autostart,
                'use_composition': use_composition,
                'log_level': log_level,
            }.items()),
    ])
