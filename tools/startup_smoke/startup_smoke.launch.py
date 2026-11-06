"""Fixed no-motion adapter of historical B's successful mapping composition."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

DEFAULTS = dict(odometry_mode='wheel', localization_mode='none', map_yaml='', sensor_only='false', mode='startup_smoke', motion_profile='patrol_030_candidate',
    gui='false', start_rviz='false', enable_rviz_goals='false', auto_start_task='false',
    enable_motion='false', ros_domain_id='204', gazebo_port='11704',
    spawn_x='0.0', spawn_y='-8.0', spawn_z='0.02', spawn_yaw='0.0', spawn_delay_sec='10.0',
    simulation_timeout='70.0', database_path='', continue_from_map='', continuation_world_sha256='')

def compose(context):
    path = Path(__file__).with_name('mapping_mode_retry.py')
    spec = importlib.util.spec_from_file_location('startup_mapping_composition', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.compose_mapping(context, lambda name: LaunchConfiguration(name).perform(context))

def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name,value in DEFAULTS.items()],
        *[DeclareLaunchArgument(name) for name in ('world','model_path','maps_root')],
        OpaqueFunction(function=compose)])
