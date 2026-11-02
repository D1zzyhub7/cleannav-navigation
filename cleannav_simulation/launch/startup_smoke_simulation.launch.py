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
    OpaqueFunction,
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
import importlib.util
import json
import os
import time
from pathlib import Path


# Demo-first: mcity needs significantly more startup time than empty_world.
SPAWN_SERVICE_TIMEOUT_SEC = "120.0"
CONTROLLER_MANAGER_TIMEOUT_SEC = "120.0"


class DiagnosticSpawnTimer(TimerAction):
    """Observe the real timer without changing its period or child actions."""
    def _record(self, context, event):
        from launch.utilities import perform_substitutions
        row = {"event": event, "run_id": os.environ["CLEANNAV_RVIZ_RUN_ID"],
               "monotonic_ns": time.monotonic_ns(), "wall_ns": time.time_ns(),
               "pid": os.getpid(),
               "spawn": {key: LaunchConfiguration(key).perform(context) for key in
                         ("spawn_x", "spawn_y", "spawn_z", "spawn_yaw", "spawn_delay_sec")},
               "resolved_timer_period_s": float(perform_substitutions(context, self.period))}
        with (Path(os.environ["ROS_LOG_DIR"]) / "spawn-timing.jsonl").open("a") as stream:
            stream.write(json.dumps(row) + "\n")

    async def _wait_to_fire_event(self, context):
        self._record(context, "wait_started")
        await super()._wait_to_fire_event(context)

    def handle(self, context):
        actions = super().handle(context)
        self._record(context, "wait_fired")
        return actions


def _rear_axle_tf_actions(context):
    """Publish the one URDF-derived rear-axle to body-centre transform."""
    if LaunchConfiguration("publish_rear_axle_tf").perform(context) != "true":
        return []
    share = Path(FindPackageShare("cleannav_simulation").find("cleannav_simulation"))
    helper_path = share / "launch" / "rear_axle_frames.py"
    spec = importlib.util.spec_from_file_location("cleannav_rear_axle_frames", helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    offset = helper.derive_rear_axle_offset(share / "urdf" / "cleannav_ackermann.urdf.xacro")
    return [Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="ackermann_rear_axle_to_base_footprint", output="screen",
        arguments=["--x", str(offset), "--y", "0", "--z", "0", "--yaw", "0",
                   "--pitch", "0", "--roll", "0", "--frame-id", "ackermann_rear_axle",
                   "--child-frame-id", "base_footprint"],
        parameters=[{"use_sim_time": True}],
    )]


def generate_launch_description():
    package_share = FindPackageShare("cleannav_simulation")

    world = LaunchConfiguration("world")
    model_path = LaunchConfiguration("model_path")
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    gui = LaunchConfiguration("gui")
    extra_gazebo_args = LaunchConfiguration("extra_gazebo_args")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                robot_xacro_file,
                " controllers_file:=",
                LaunchConfiguration("controllers_file"),
                " cmd_vel_topic:=/cmd_vel",
                " sim_lidar_samples:=", LaunchConfiguration("sim_lidar_samples"),
                " sim_lidar_rate:=", LaunchConfiguration("sim_lidar_rate"),
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
            "extra_gazebo_args": extra_gazebo_args,
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
            ],
        )
    )
    # Do not race two spawner clients through controller-manager startup.
    ackermann_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=lambda event, context: (
                [ackermann_controller_spawner]
                if event.returncode == 0 and not context.is_shutdown else []
            ),
        )
    )

    timer_type = (DiagnosticSpawnTimer if
        os.environ.get("CLEANNAV_RVIZ_TIMING_PROFILE") == "localization-diagnostic"
        and os.environ.get("CLEANNAV_RVIZ_RUN_ID") and os.environ.get("ROS_LOG_DIR")
        else TimerAction)
    delayed_robot_spawn = timer_type(
        period=LaunchConfiguration("spawn_delay_sec"),
        actions=[spawn_entity],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                description="Gazebo Classic world file",
            ),
            DeclareLaunchArgument(
                "model_path",
                description="External mcity Gazebo model directory",
            ),
            DeclareLaunchArgument(
                "robot_xacro_file",
                default_value=PathJoinSubstitution(
                    [package_share, "urdf", "cleannav_ackermann.urdf.xacro"]
                ),
                description="Explicit robot Xacro; defaults to the installed package share",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "extra_gazebo_args",
                default_value="",
                description=(
                    "Additional gzserver arguments; demo-only scene overlays "
                    "use this to load gazebo_ros_state"
                ),
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
                "spawn_delay_sec",
                default_value="60.0",
                description="Wait for the selected Gazebo world before spawning the robot",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot initial yaw in radians",
            ),
            DeclareLaunchArgument("controllers_file", default_value=PathJoinSubstitution(
                [package_share, "config", "ackermann_controllers.yaml"]),
                description="Simulation controller YAML; default uses rear-axle odometry"),
            DeclareLaunchArgument("publish_rear_axle_tf", default_value="true",
                choices=["true", "false"],
                description="Publish the matching rear axle to body-centre static transform"),
            DeclareLaunchArgument("sim_lidar_samples", default_value="181",
                description="Simulation-only LiDAR beam count; frozen default 181"),
            DeclareLaunchArgument("sim_lidar_rate", default_value="8",
                description="Simulation-only LiDAR Hz; frozen default 8"),
            gazebo_model_path,
            gazebo,
            robot_state_publisher,
            # RSP owns all URDF joints; this is the only publisher of this
            # experiment/controller reference transform.
            OpaqueFunction(function=_rear_axle_tf_actions),
            controllers_after_spawn,
            ackermann_after_joint_states,
            delayed_robot_spawn,
        ]
    )
