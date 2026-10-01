"""Launch the mcity Ackermann demo with demo-only obstacle entities."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import xml.etree.ElementTree as ElementTree


SCENE_SPAWN_DELAY_SEC = 65.0
DEFAULT_WORLD = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/worlds/mcity.world"
)
DEFAULT_MODEL_PATH = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/models"
)
OVERLAY_WORLD_PATH = Path("/tmp/cleannav_mcity_obstacles_demo.world")
STATE_PLUGIN_FILENAME = "libgazebo_ros_state.so"
STATIC_BLOCK_NAME = "cleannav_demo_static_block"
STATIC_CYLINDER_NAME = "cleannav_demo_static_cylinder"
DYNAMIC_OBSTACLE_NAME = "cleannav_demo_dynamic_obstacle"


def _state_plugin_count(world):
    """Count state world plugins by library basename."""
    return sum(
        Path(plugin.attrib.get("filename", "")).name
        == STATE_PLUGIN_FILENAME
        for plugin in world.findall("plugin")
    )


def generate_mcity_overlay_world(
    base_world_path,
    output_world_path=OVERLAY_WORLD_PATH,
):
    """Write a deterministic state-plugin overlay without changing the base."""
    base_path = Path(base_world_path)
    output_path = Path(output_world_path)
    if base_path.resolve() == output_path.resolve():
        raise ValueError("overlay world must not overwrite the base world")

    parser = ElementTree.XMLParser(
        target=ElementTree.TreeBuilder(insert_comments=True)
    )
    tree = ElementTree.parse(base_path, parser=parser)
    root = tree.getroot()
    worlds = root.findall("world")
    if len(worlds) != 1:
        raise ValueError("base world must contain exactly one <world>")

    world = worlds[0]
    plugin_count = _state_plugin_count(world)
    if plugin_count > 1:
        raise ValueError(
            f"base world contains {plugin_count} state plugins"
        )
    if plugin_count == 0:
        world.append(
            ElementTree.Element(
                "plugin",
                {
                    "name": "cleannav_gazebo_ros_state",
                    "filename": STATE_PLUGIN_FILENAME,
                },
            )
        )

    ElementTree.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )
    return output_path


def _include_base_with_overlay(context):
    """Generate the overlay before visiting the base Gazebo launch."""
    base_world_path = LaunchConfiguration("world").perform(context)
    overlay_world_path = generate_mcity_overlay_world(base_world_path)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("cleannav_simulation"),
                        "launch",
                        "ackermann_mcity_demo.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "world": str(overlay_world_path),
                "model_path": LaunchConfiguration("model_path"),
                "gui": LaunchConfiguration("gui"),
                "extra_gazebo_args": "",
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        )
    ]


def _spawn_file(entity_name, file_name, x, y, z):
    """Create one deterministic Gazebo SDF spawn action."""
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            entity_name,
            "-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("cleannav_simulation"),
                    "models",
                    file_name,
                ]
            ),
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-timeout",
            "120.0",
        ],
    )


def generate_launch_description():
    """Create the base demo plus optional static and dynamic obstacles."""
    enable_static = LaunchConfiguration("enable_static_obstacles")
    enable_dynamic = LaunchConfiguration("enable_dynamic_obstacle")
    dynamic_speed = LaunchConfiguration("dynamic_obstacle_speed")

    static_block_x = LaunchConfiguration("static_block_x")
    static_block_y = LaunchConfiguration("static_block_y")
    static_block_z = LaunchConfiguration("static_block_z")
    static_cylinder_x = LaunchConfiguration("static_cylinder_x")
    static_cylinder_y = LaunchConfiguration("static_cylinder_y")
    static_cylinder_z = LaunchConfiguration("static_cylinder_z")

    dynamic_a_x = LaunchConfiguration("dynamic_obstacle_a_x")
    dynamic_a_y = LaunchConfiguration("dynamic_obstacle_a_y")
    dynamic_a_z = LaunchConfiguration("dynamic_obstacle_a_z")
    dynamic_a_yaw = LaunchConfiguration("dynamic_obstacle_a_yaw")
    dynamic_b_x = LaunchConfiguration("dynamic_obstacle_b_x")
    dynamic_b_y = LaunchConfiguration("dynamic_obstacle_b_y")
    dynamic_b_z = LaunchConfiguration("dynamic_obstacle_b_z")
    dynamic_b_yaw = LaunchConfiguration("dynamic_obstacle_b_yaw")

    static_block = _spawn_file(
        STATIC_BLOCK_NAME,
        "cleannav_demo_static_block.sdf",
        static_block_x,
        static_block_y,
        static_block_z,
    )
    static_cylinder = _spawn_file(
        STATIC_CYLINDER_NAME,
        "cleannav_demo_static_cylinder.sdf",
        static_cylinder_x,
        static_cylinder_y,
        static_cylinder_z,
    )
    dynamic_spawn = _spawn_file(
        DYNAMIC_OBSTACLE_NAME,
        "cleannav_demo_dynamic_obstacle.sdf",
        dynamic_a_x,
        dynamic_a_y,
        dynamic_a_z,
    )

    dynamic_controller = Node(
        package="cleannav_simulation",
        executable="dynamic_obstacle_controller",
        name="cleannav_dynamic_obstacle_controller",
        output="screen",
        condition=IfCondition(enable_dynamic),
        parameters=[
            {
                "obstacle_name": DYNAMIC_OBSTACLE_NAME,
                "service_name": "/set_entity_state",
                "a_x": ParameterValue(dynamic_a_x, value_type=float),
                "a_y": ParameterValue(dynamic_a_y, value_type=float),
                "a_z": ParameterValue(dynamic_a_z, value_type=float),
                "a_yaw": ParameterValue(dynamic_a_yaw, value_type=float),
                "b_x": ParameterValue(dynamic_b_x, value_type=float),
                "b_y": ParameterValue(dynamic_b_y, value_type=float),
                "b_z": ParameterValue(dynamic_b_z, value_type=float),
                "b_yaw": ParameterValue(dynamic_b_yaw, value_type=float),
                "speed": ParameterValue(dynamic_speed, value_type=float),
                "update_rate": 10.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=DEFAULT_WORLD,
                description="Base mcity world used to create the demo overlay",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=DEFAULT_MODEL_PATH,
                description="Gazebo model directory",
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
            DeclareLaunchArgument(
                "static_block_x",
                default_value="2.20",
            ),
            DeclareLaunchArgument(
                "static_block_y",
                default_value="-5.00",
            ),
            DeclareLaunchArgument(
                "static_block_z",
                default_value="0.40",
            ),
            DeclareLaunchArgument(
                "static_cylinder_x",
                default_value="2.20",
            ),
            DeclareLaunchArgument(
                "static_cylinder_y",
                default_value="-2.50",
            ),
            DeclareLaunchArgument(
                "static_cylinder_z",
                default_value="0.40",
            ),
            DeclareLaunchArgument(
                "enable_static_obstacles",
                default_value="true",
                description="Spawn the two demo-only static obstacles",
            ),
            DeclareLaunchArgument(
                "enable_dynamic_obstacle",
                default_value="true",
                description="Spawn and control the demo-only dynamic obstacle",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_speed",
                default_value="0.25",
                description="Dynamic obstacle speed in m/s",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_a_x",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_a_y",
                default_value="-6.25",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_a_z",
                default_value="0.40",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_a_yaw",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_b_x",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_b_y",
                default_value="-3.75",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_b_z",
                default_value="0.40",
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_b_yaw",
                default_value="0.0",
            ),
            OpaqueFunction(function=_include_base_with_overlay),
            GroupAction(
                condition=IfCondition(enable_static),
                actions=[
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC,
                        actions=[static_block, static_cylinder],
                    )
                ],
            ),
            GroupAction(
                condition=IfCondition(enable_dynamic),
                actions=[
                    # Spawn first, then start the controller after a short
                    # grace period. Starting both in the same TimerAction
                    # caused repeated SetEntityState errors before Gazebo
                    # had created the dynamic entity.
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC,
                        actions=[dynamic_spawn],
                    ),
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC + 5.0,
                        actions=[dynamic_controller],
                    ),
                ],
            ),
        ]
    )
