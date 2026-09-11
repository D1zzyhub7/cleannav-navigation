"""Launch CleanNav Ackermann in the external mcity Gazebo Classic world.

DEMO-FIRST STARTUP POLICY

The mcity world is substantially heavier than the original empty world.
Gazebo's /spawn_entity service may become visible before the world is fully
ready to accept and confirm a complex robot model.

For demo stability this launcher therefore:

1. starts Gazebo and robot_state_publisher;
2. waits 60 seconds for mcity to settle;
3. spawns the CleanNav Ackermann robot;
4. starts ros2_control controller spawners after the spawn process exits.

The long waits are intentional for the competition demo.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_WORLD = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/worlds/mcity.world"
)

DEFAULT_MODEL_PATH = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/models"
)

# Demo-first: mcity needs significantly more startup time than empty_world.
SPAWN_DELAY_SEC = 60.0
SPAWN_SERVICE_TIMEOUT_SEC = "120.0"
CONTROLLER_MANAGER_TIMEOUT_SEC = "120.0"


def generate_launch_description():
    package_share = FindPackageShare("cleannav_simulation")

    world = LaunchConfiguration("world")
    model_path = LaunchConfiguration("model_path")
    gui = LaunchConfiguration("gui")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [
                        package_share,
                        "urdf",
                        "cleannav_ackermann.urdf.xacro",
                    ]
                ),
                " controllers_file:=",
                PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "ackermann_controllers.yaml",
                    ]
                ),
                " cmd_vel_topic:=/cmd_vel",
            ]
        ),
        value_type=str,
    )

    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            model_path,
            ":",
            EnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                default_value="",
            ),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
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
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
            "-timeout",
            SPAWN_SERVICE_TIMEOUT_SEC,
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
            CONTROLLER_MANAGER_TIMEOUT_SEC,
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
            CONTROLLER_MANAGER_TIMEOUT_SEC,
        ],
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                ackermann_controller_spawner,
            ],
        )
    )

    delayed_robot_spawn = TimerAction(
        period=SPAWN_DELAY_SEC,
        actions=[spawn_entity],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=DEFAULT_WORLD,
                description="Gazebo Classic world file",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=DEFAULT_MODEL_PATH,
                description="External mcity Gazebo model directory",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Robot initial X coordinate",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-5.0",
                description="Robot initial Y coordinate",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.02",
                description="Robot initial Z coordinate",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot initial yaw in radians",
            ),
            gazebo_model_path,
            gazebo,
            robot_state_publisher,
            controllers_after_spawn,
            delayed_robot_spawn,
        ]
    )
