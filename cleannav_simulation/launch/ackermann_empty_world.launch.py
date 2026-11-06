"""启动 CleanNav Ackermann V1 的最小 Gazebo Classic 空世界。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cleannav_simulation")
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [package_share, "urdf", "cleannav_ackermann.urdf.xacro"]
                ),
                " controllers_file:=",
                PathJoinSubstitution(
                    [package_share, "config", "ackermann_controllers.yaml"]
                ),
                " cmd_vel_topic:=/cmd_vel",
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "cleannav_ackermann",
            "-topic",
            "robot_description",
            "-z",
            "0.02",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30.0",
        ],
    )

    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "ackermann_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30.0",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="是否启动 Gazebo GUI"
            ),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster_spawner,
            ackermann_controller_spawner,
        ]
    )
