"""Independent CleanNav PC Demo Scene v2 recording launch."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SCENE_SPAWN_DELAY_SEC = 65.0
DEFAULT_WORLD = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/worlds/mcity.world"
)
DEFAULT_MODEL_PATH = (
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/models"
)
SHOWCASE_PERSON_NAME = "cleannav_demo_showcase_person"
LEAF_ENTITY_NAMES = tuple(
    f"cleannav_demo_leaf_pile_{index}" for index in range(5)
)
LEAF_FIELD_POSITIONS = tuple(
    (10.4, y, 0.01)
    for y in (-5.40, -5.20, -5.00, -4.80, -4.60)
)


def _spawn_file(entity_name, file_name, x, y, z):
    """Create one deterministic SDF file spawn action."""
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
    """Create the independent, recording-oriented Scene v2 composition."""
    obstacles_launch = PathJoinSubstitution(
        [
            FindPackageShare("cleannav_simulation"),
            "launch",
            "ackermann_mcity_obstacles_demo.launch.py",
        ]
    )

    enable_static = LaunchConfiguration("enable_static_obstacles")
    enable_person = LaunchConfiguration("enable_dynamic_person")
    enable_bicycle = LaunchConfiguration("enable_bicycle")
    enable_leaf = LaunchConfiguration("enable_leaf_goal")

    person_spawn = _spawn_file(
        SHOWCASE_PERSON_NAME,
        "cleannav_demo_showcase_person.sdf",
        LaunchConfiguration("showcase_person_a_x"),
        LaunchConfiguration("showcase_person_a_y"),
        LaunchConfiguration("showcase_person_a_z"),
    )
    person_controller = Node(
        package="cleannav_simulation",
        executable="showcase_person_controller",
        name="cleannav_showcase_person_controller",
        output="screen",
        condition=IfCondition(enable_person),
        parameters=[
            {
                "person_name": SHOWCASE_PERSON_NAME,
                "service_name": "/set_entity_state",
                "a_x": ParameterValue(
                    LaunchConfiguration("showcase_person_a_x"),
                    value_type=float,
                ),
                "a_y": ParameterValue(
                    LaunchConfiguration("showcase_person_a_y"),
                    value_type=float,
                ),
                "a_z": ParameterValue(
                    LaunchConfiguration("showcase_person_a_z"),
                    value_type=float,
                ),
                "b_x": ParameterValue(
                    LaunchConfiguration("showcase_person_b_x"),
                    value_type=float,
                ),
                "b_y": ParameterValue(
                    LaunchConfiguration("showcase_person_b_y"),
                    value_type=float,
                ),
                "b_z": ParameterValue(
                    LaunchConfiguration("showcase_person_b_z"),
                    value_type=float,
                ),
                "speed": ParameterValue(
                    LaunchConfiguration("showcase_person_speed"),
                    value_type=float,
                ),
                "update_rate": ParameterValue(
                    LaunchConfiguration("showcase_person_update_rate"),
                    value_type=float,
                ),
            }
        ],
    )

    bicycle_spawn = _spawn_file(
        "cleannav_demo_bicycle",
        "cleannav_demo_bicycle.sdf",
        LaunchConfiguration("bicycle_a_x"),
        LaunchConfiguration("bicycle_a_y"),
        LaunchConfiguration("bicycle_a_z"),
    )
    bicycle_controller = Node(
        package="cleannav_simulation",
        executable="bicycle_patrol_controller",
        name="cleannav_bicycle_patrol_controller",
        output="screen",
        condition=IfCondition(enable_bicycle),
        parameters=[
            {
                "bicycle_name": "cleannav_demo_bicycle",
                "service_name": "/set_entity_state",
                "a_x": ParameterValue(
                    LaunchConfiguration("bicycle_a_x"), value_type=float
                ),
                "a_y": ParameterValue(
                    LaunchConfiguration("bicycle_a_y"), value_type=float
                ),
                "a_z": ParameterValue(
                    LaunchConfiguration("bicycle_a_z"), value_type=float
                ),
                "b_x": ParameterValue(
                    LaunchConfiguration("bicycle_b_x"), value_type=float
                ),
                "b_y": ParameterValue(
                    LaunchConfiguration("bicycle_b_y"), value_type=float
                ),
                "b_z": ParameterValue(
                    LaunchConfiguration("bicycle_b_z"), value_type=float
                ),
                "speed": ParameterValue(
                    LaunchConfiguration("bicycle_speed"), value_type=float
                ),
                "update_rate": ParameterValue(
                    LaunchConfiguration("bicycle_update_rate"),
                    value_type=float,
                ),
            }
        ],
    )

    leaf_spawns = [
        _spawn_file(
            entity_name,
            "cleannav_demo_leaf_pile.sdf",
            str(x),
            str(y),
            str(z),
        )
        for entity_name, (x, y, z) in zip(
            LEAF_ENTITY_NAMES,
            LEAF_FIELD_POSITIONS,
        )
    ]
    leaf_controller = Node(
        package="cleannav_simulation",
        executable="leaf_cleanup_demo_controller",
        name="cleannav_leaf_cleanup_demo_controller",
        output="screen",
        condition=IfCondition(enable_leaf),
        parameters=[
            {
                "robot_entity": "cleannav_ackermann",
                "leaf_entities": list(LEAF_ENTITY_NAMES),
                "distance_threshold": ParameterValue(
                    LaunchConfiguration("leaf_collect_distance"),
                    value_type=float,
                ),
                "cleaning_point_offset_x": 0.37,
                "cleaning_point_offset_y": 0.0,
                "update_rate": 10.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=DEFAULT_WORLD,
                description="Base mcity world file",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=DEFAULT_MODEL_PATH,
                description="Gazebo model directory",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "spawn_x", default_value="0.0", description="Robot world X"
            ),
            DeclareLaunchArgument(
                "spawn_y", default_value="-5.0", description="Robot world Y"
            ),
            DeclareLaunchArgument("spawn_z", default_value="0.02"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "enable_static_obstacles", default_value="true"
            ),
            DeclareLaunchArgument(
                "enable_dynamic_person", default_value="true"
            ),
            DeclareLaunchArgument("enable_bicycle", default_value="true"),
            DeclareLaunchArgument("enable_leaf_goal", default_value="true"),
            DeclareLaunchArgument("static_block_x", default_value="4.0"),
            DeclareLaunchArgument("static_block_y", default_value="-4.4"),
            DeclareLaunchArgument("static_block_z", default_value="0.40"),
            DeclareLaunchArgument("static_cylinder_x", default_value="8.0"),
            DeclareLaunchArgument("static_cylinder_y", default_value="-5.8"),
            DeclareLaunchArgument("static_cylinder_z", default_value="0.40"),
            DeclareLaunchArgument("showcase_person_a_x", default_value="5.8"),
            DeclareLaunchArgument(
                "showcase_person_a_y", default_value="-3.85"
            ),
            DeclareLaunchArgument(
                "showcase_person_a_z", default_value="0.40"
            ),
            DeclareLaunchArgument("showcase_person_b_x", default_value="5.8"),
            DeclareLaunchArgument(
                "showcase_person_b_y", default_value="-3.35"
            ),
            DeclareLaunchArgument(
                "showcase_person_b_z", default_value="0.40"
            ),
            DeclareLaunchArgument(
                "showcase_person_speed", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "showcase_person_update_rate", default_value="10.0"
            ),
            DeclareLaunchArgument("bicycle_a_x", default_value="1.0"),
            DeclareLaunchArgument("bicycle_a_y", default_value="-3.0"),
            DeclareLaunchArgument("bicycle_a_z", default_value="0.0"),
            DeclareLaunchArgument("bicycle_b_x", default_value="10.0"),
            DeclareLaunchArgument("bicycle_b_y", default_value="-3.0"),
            DeclareLaunchArgument("bicycle_b_z", default_value="0.0"),
            DeclareLaunchArgument("bicycle_speed", default_value="0.80"),
            DeclareLaunchArgument("bicycle_update_rate", default_value="10.0"),
            DeclareLaunchArgument(
                "leaf_collect_distance", default_value="0.35"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(obstacles_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "model_path": LaunchConfiguration("model_path"),
                    "gui": LaunchConfiguration("gui"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "enable_static_obstacles": enable_static,
                    # The obstacle-test launch retains the real collision
                    # person, but Scene v2 deliberately disables it.
                    "enable_dynamic_obstacle": "false",
                    "static_block_x": LaunchConfiguration("static_block_x"),
                    "static_block_y": LaunchConfiguration("static_block_y"),
                    "static_block_z": LaunchConfiguration("static_block_z"),
                    "static_cylinder_x": LaunchConfiguration(
                        "static_cylinder_x"
                    ),
                    "static_cylinder_y": LaunchConfiguration(
                        "static_cylinder_y"
                    ),
                    "static_cylinder_z": LaunchConfiguration(
                        "static_cylinder_z"
                    ),
                }.items(),
            ),
            GroupAction(
                condition=IfCondition(enable_person),
                actions=[
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC,
                        actions=[person_spawn, person_controller],
                    )
                ],
            ),
            GroupAction(
                condition=IfCondition(enable_bicycle),
                actions=[
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC,
                        actions=[bicycle_spawn, bicycle_controller],
                    )
                ],
            ),
            GroupAction(
                condition=IfCondition(enable_leaf),
                actions=[
                    TimerAction(
                        period=SCENE_SPAWN_DELAY_SEC,
                        actions=[*leaf_spawns, leaf_controller],
                    )
                ],
            ),
        ]
    )
