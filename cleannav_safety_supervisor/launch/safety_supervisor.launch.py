# cleannav_safety_supervisor Launch
# 仅启动 Safety Supervisor 节点，不启动任何其他系统

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    publish_freq = LaunchConfiguration('publish_frequency', default='10.0')
    candidate_timeout = LaunchConfiguration(
        'candidate_timeout_sec', default='0.5')
    status_freq = LaunchConfiguration(
        'status_publish_frequency', default='2.0')
    block_all = LaunchConfiguration('block_all', default='true')
    max_forward = LaunchConfiguration(
        'max_forward_linear_x', default='0.05')
    max_reverse = LaunchConfiguration(
        'max_reverse_linear_x', default='0.0')
    max_angular = LaunchConfiguration(
        'max_angular_z', default='0.3')
    auto_timeout = LaunchConfiguration(
        'autonomous_timeout_sec', default='5.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'publish_frequency', default_value='10.0',
            description='/cmd_vel publish frequency in Hz'),
        DeclareLaunchArgument(
            'candidate_timeout_sec', default_value='0.5',
            description='Candidate timeout in seconds'),
        DeclareLaunchArgument(
            'status_publish_frequency', default_value='2.0',
            description='Status publish frequency in Hz'),
        DeclareLaunchArgument(
            'block_all', default_value='true',
            description='Block all candidate velocities'),
        DeclareLaunchArgument(
            'max_forward_linear_x', default_value='0.05',
            description='Maximum forward linear x in m/s'),
        DeclareLaunchArgument(
            'max_reverse_linear_x', default_value='0.0',
            description='Maximum reverse linear x in m/s'),
        DeclareLaunchArgument(
            'max_angular_z', default_value='0.3',
            description='Maximum angular z in rad/s'),
        DeclareLaunchArgument(
            'autonomous_timeout_sec', default_value='5.0',
            description='Autonomous mode timeout in seconds'),

        Node(
            package='cleannav_safety_supervisor',
            executable='safety_supervisor_node',
            name='cleannav_safety_supervisor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': publish_freq,
                'candidate_timeout_sec': candidate_timeout,
                'status_publish_frequency': status_freq,
                'block_all': block_all,
                'max_forward_linear_x': max_forward,
                'max_reverse_linear_x': max_reverse,
                'max_angular_z': max_angular,
                'autonomous_timeout_sec': auto_timeout,
            }],
        ),
    ])
