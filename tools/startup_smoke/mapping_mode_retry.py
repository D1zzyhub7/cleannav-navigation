"""Independent mapping composition, reusing the single rear-axle base launch."""
import json
import hashlib
import os
import copy
from pathlib import Path
import shutil
import sys
import uuid

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import (EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch_ros.actions import Node, SetParameter

# Publish safe zero commands through the existing candidate path for the
# bounded mapping session. Collision Monitor action decisions remain unchanged.
COLLISION_MONITOR_ZERO_HEARTBEAT_TIMEOUT_S = 900.0
NAV_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = NAV_ROOT
RECOVERY_SIMULATION_LAUNCH = (
    SOURCE_ROOT / 'cleannav_simulation' / 'launch' / 'startup_smoke_simulation.launch.py')
RECOVERY_ROOF_XACRO = (
    SOURCE_ROOT / 'cleannav_simulation' / 'urdf' / 'cleannav_ackermann_startup_smoke.urdf.xacro')
RECOVERY_RTAB_LAUNCH = (
    SOURCE_ROOT / 'cleannav_rtabmap' / 'launch' / 'startup_smoke_rtabmap.launch.py')
try:
    from mapping_support import validate_motion_profile
    from mapping_assets import prepare_database_continuation
except ImportError:
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, str(
        Path(get_package_prefix('cleannav_simulation'))
        / 'lib' / 'cleannav_simulation'))
    from mapping_support import validate_motion_profile
    from mapping_assets import prepare_database_continuation


def load_motion_profile(profile_id, config_directory=None):
    """Return the named mapping profile without changing its source asset."""
    if profile_id in ('baseline', 'mapping_conservative'):
        profile = {
            'max_forward_linear_x': 0.20,
            'max_reverse_linear_x': 0.10,
            'min_turning_radius_m': 1.12,
            'max_curvature_inv_m': 0.80,
            'max_angular_z': 0.16,
            'acceleration_mps2': 0.25,
            'curvature_rate_inv_mps': 1.0,
            'reverse_allowed': True,
            'safety': {
                'max_forward_linear_x': 0.20,
                'max_reverse_linear_x': 0.10,
                'max_angular_z': 0.16,
            },
        }
        profile.update(validate_motion_profile(profile))
        return profile, None
    if profile_id not in ('patrol_030_candidate', 'patrol_050_mapping_candidate'):
        raise ValueError(f'unknown mapping motion_profile={profile_id!r}')
    if config_directory is None:
        config_directory = (
            Path(get_package_share_directory('cleannav_simulation'))
            / 'config' / 'motion_profiles')
    profile_path = Path(config_directory) / f'{profile_id}.json'
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    if profile.get('profile_id') != profile_id:
        raise ValueError('mapping motion profile identity mismatch')
    # Normalize validated vehicle values (including the nested radius) into
    # the mapping broker's flat runtime parameter contract.
    profile.update(validate_motion_profile(profile))
    return profile, hashlib.sha256(profile_path.read_bytes()).hexdigest()


def validate_mapping_collision_monitor(config_path):
    """Validate the mapping-only scan stop filter and its command-chain topics."""
    path = Path(config_path)
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    params = config.get('collision_monitor', {}).get('ros__parameters', {})
    if params.get('cmd_vel_in_topic') != '/cleannav/mapping/cmd_vel_raw':
        raise ValueError('Collision Monitor input must be the private mapping command topic')
    if params.get('cmd_vel_out_topic') != '/cleannav/cmd_vel_candidate':
        raise ValueError('Collision Monitor output must feed the formal Safety candidate topic')
    if float(params.get('source_timeout', 0.)) > 0.35:
        raise ValueError('Collision Monitor scan timeout is too permissive for manual mapping')
    sources = params.get('observation_sources', [])
    if 'scan' not in sources or params.get('scan', {}).get('topic') != '/scan':
        raise ValueError('mapping Collision Monitor must consume the filtered /scan topic')
    polygon_names = params.get('polygons', [])
    if set(polygon_names) != {'MappingForwardStop', 'MappingSweptApproach'}:
        raise ValueError('mapping requires the forward stop zone and full-footprint swept approach filter')
    polygon = params.get('MappingForwardStop', {})
    points = polygon.get('points', [])
    if (polygon.get('action_type') != 'stop' or polygon.get('type') != 'polygon'
            or polygon.get('max_points') != 0 or polygon.get('enabled') is not True
            or len(points) < 8 or len(points) % 2):
        raise ValueError('mapping stop zone must stop on any scan point inside its polygon')
    xs, ys = points[0::2], points[1::2]
    if min(xs) < 0.25 or max(xs) < 1.20 or max(abs(y) for y in ys) < 0.65:
        raise ValueError('mapping stop zone does not cover the front sensor/vehicle envelope')
    approach = params.get('MappingSweptApproach', {})
    if (approach.get('type') != 'polygon' or approach.get('action_type') != 'approach'
            or approach.get('footprint_topic') != '/cleannav/mapping/robot_footprint'
            or approach.get('enabled') is not True
            or int(approach.get('max_points', 0)) < 1
            or float(approach.get('time_before_collision', 0.)) < 1.5
            or float(approach.get('simulation_time_step', 1.)) > 0.10):
        raise ValueError('mapping swept approach must use a fresh full vehicle footprint and bounded prediction')
    return path


def prepare_collision_monitor_runtime_config(source_path, runtime_path):
    """Create a per-session YAML changing only the zero-output heartbeat."""
    source = validate_mapping_collision_monitor(source_path)
    original = yaml.safe_load(source.read_text(encoding='utf-8'))
    runtime = copy.deepcopy(original)
    runtime_params = runtime['collision_monitor']['ros__parameters']
    runtime_params['stop_pub_timeout'] = COLLISION_MONITOR_ZERO_HEARTBEAT_TIMEOUT_S
    target = Path(runtime_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(runtime, sort_keys=False), encoding='utf-8')
    written = yaml.safe_load(target.read_text(encoding='utf-8'))
    source_params = copy.deepcopy(original['collision_monitor']['ros__parameters'])
    written_params = copy.deepcopy(written['collision_monitor']['ros__parameters'])
    source_params.pop('stop_pub_timeout', None)
    written_params.pop('stop_pub_timeout', None)
    if source_params != written_params:
        raise ValueError('candidate Collision Monitor config changed fields beyond its zero heartbeat')
    if float(written['collision_monitor']['ros__parameters']['stop_pub_timeout']) != COLLISION_MONITOR_ZERO_HEARTBEAT_TIMEOUT_S:
        raise ValueError('candidate Collision Monitor heartbeat timeout was not written')
    validate_mapping_collision_monitor(target)
    return target


def enable_full_graph_proximity_candidates(config_path):
    """Remove RTAB-Map's 50-node proximity-loop candidate depth cap.

    This only edits the per-session copy of the INI. The one-lap survey creates
    about 250 graph nodes before it returns near its starting location, so the
    default depth of 50 excludes that non-neighbor return candidate. Value 0
    disables this depth cap; all matching thresholds and ICP settings stay put.
    """
    path = Path(config_path)
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    matches = []
    for index, line in enumerate(lines):
        left, separator, _ = line.partition('=')
        if separator and left.strip().replace('\\', '/') == 'RGBD/ProximityMaxGraphDepth':
            matches.append(index)
    if len(matches) != 1:
        raise ValueError(
            'expected one RGBD/ProximityMaxGraphDepth entry in the private RTAB config; '
            f'found {len(matches)}')
    index = matches[0]
    line = lines[index]
    ending = '\n' if line.endswith('\n') else ''
    lines[index] = line.partition('=')[0].rstrip() + ' = 0' + ending
    path.write_text(''.join(lines), encoding='utf-8')
    checked = [line for line in path.read_text(encoding='utf-8').splitlines()
               if line.partition('=')[0].strip().replace('\\', '/')
               == 'RGBD/ProximityMaxGraphDepth']
    if len(checked) != 1 or checked[0].partition('=')[2].strip() != '0':
        raise ValueError('private RTAB config did not retain the proximity graph depth override')
    return path


def increase_mapping_detection_rate(config_path):
    """Process two scan updates per second in the private mapping session."""
    path = Path(config_path)
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    matches = []
    for index, line in enumerate(lines):
        left, separator, value = line.partition('=')
        if separator and left.strip().replace('\\', '/') == 'Rtabmap/DetectionRate':
            matches.append((index, value.strip()))
    if len(matches) != 1:
        raise ValueError(
            'expected one Rtabmap/DetectionRate entry in the private RTAB config; '
            f'found {len(matches)}')
    index, current = matches[0]
    if current != '1':
        raise ValueError(f'expected baseline Rtabmap/DetectionRate=1, found {current!r}')
    line = lines[index]
    ending = '\n' if line.endswith('\n') else ''
    lines[index] = line.partition('=')[0].rstrip() + ' = 2' + ending
    path.write_text(''.join(lines), encoding='utf-8')
    checked = [line for line in path.read_text(encoding='utf-8').splitlines()
               if line.partition('=')[0].strip().replace('\\', '/')
               == 'Rtabmap/DetectionRate']
    if len(checked) != 1 or checked[0].partition('=')[2].strip() != '2':
        raise ValueError('private RTAB config did not retain DetectionRate=2')
    return path


def compose_mapping(context, arg):
    if arg('mode') != 'startup_smoke':
        raise ValueError('this workcopy only permits startup_smoke')
    if any(arg(k) != 'false' for k in ('enable_motion', 'gui', 'start_rviz')):
        raise ValueError('startup_smoke refuses motion and GUI entrypoints')
    if arg('continue_from_map') or arg('continuation_world_sha256'):
        raise ValueError('startup_smoke cannot alter or continue existing maps')
    if arg('auto_start_task') == 'true' or arg('enable_rviz_goals') == 'true':
        raise ValueError('mapping mode refuses automatic tasks and RViz navigation goals')
    if arg('database_path'):
        raise ValueError('mapping owns a new persistent session database; database_path must be empty')
    profile, profile_sha256 = load_motion_profile(arg('motion_profile'))
    collision_config = (Path(get_package_share_directory('cleannav_simulation'))
                        / 'config' / 'mapping_collision_monitor.yaml')
    validate_mapping_collision_monitor(collision_config)
    if not Path(arg('world')).is_file():
        raise FileNotFoundError(arg('world'))
    root = Path(arg('maps_root')).expanduser().resolve()
    session = root / '.sessions' / uuid.uuid4().hex
    session.mkdir(parents=True, exist_ok=False)
    runtime_collision_config = prepare_collision_monitor_runtime_config(
        collision_config, session / 'mapping_collision_monitor_candidate.yaml')
    runtime_collision_config_sha = hashlib.sha256(
        runtime_collision_config.read_bytes()).hexdigest()
    world_source = Path(arg('world')).resolve(strict=True)
    source_world_sha = hashlib.sha256(world_source.read_bytes()).hexdigest()
    runtime_world = session / 'mapping_diagnostic.world'
    world_text = world_source.read_text(encoding='utf-8')
    plugin_name = 'cleannav_mapping_diagnostic_state'
    if plugin_name not in world_text:
        state_plugin = (
            f'<plugin name="{plugin_name}" filename="libgazebo_ros_state.so">'
            '<ros><namespace>/gazebo</namespace></ros>'
            '<update_rate>20</update_rate></plugin>'
        )
        if '</world>' not in world_text:
            raise ValueError('mapping world has no closing </world> element')
        world_text = world_text.replace('</world>', state_plugin + '</world>')
    runtime_world.write_text(world_text, encoding='utf-8')
    runtime_world_sha = hashlib.sha256(runtime_world.read_bytes()).hexdigest()
    rtab_share = Path(get_package_share_directory('cleannav_rtabmap'))
    cfg = session / 'rtabmap_config.ini'
    continuation_name = arg('continue_from_map').strip()
    continuation_sha = arg('continuation_world_sha256').strip().lower()
    continuation = None
    if continuation_name:
        continuation = prepare_database_continuation(
            root, continuation_name, arg('world'), continuation_sha,
            session/'rtabmap.db')
        shutil.copyfile(root/continuation_name/'rtabmap_config.ini', cfg)
    else:
        if continuation_sha:
            raise ValueError('continuation_world_sha256 requires continue_from_map')
        shutil.copyfile(rtab_share / 'config' / 'rtabmap_lidar_mapping_sim.ini', cfg)
        enable_full_graph_proximity_candidates(cfg)
        increase_mapping_detection_rate(cfg)
    session_file = session / 'session.json'
    session_file.write_text(json.dumps(dict(schema_version=1, operation_mode='mapping',
        startup_smoke=True, simulation_motion_authorization=False, session_id=session.name, maps_root=str(root), database_path=str(session/'rtabmap.db'),
        config_path=str(cfg), state='starting', ros_domain_id=int(arg('ros_domain_id')),
        rtabmap_node='/rtabmap/rtabmap', world_path=arg('world'), simulation_only=True,
        world_sha256=source_world_sha, runtime_world_path=str(runtime_world),
        runtime_world_sha256=runtime_world_sha,
        collision_monitor_source_config=str(collision_config),
        collision_monitor_runtime_config=str(runtime_collision_config),
        collision_monitor_runtime_config_sha256=runtime_collision_config_sha,
        collision_monitor_stop_pub_timeout_s=COLLISION_MONITOR_ZERO_HEARTBEAT_TIMEOUT_S,
        rtabmap_proximity_max_graph_depth=0 if not continuation_name else None,
        rtabmap_detection_rate=2 if not continuation_name else None,
        database_continuation=continuation,
        motion_profile=arg('motion_profile'), motion_profile_sha256=profile_sha256), indent=2))
    logs = launch_config.log_dir
    os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'own_log'

    def update_session(**values):
        state = json.loads(session_file.read_text())
        state.update(values)
        pending = session / 'session.launch.tmp'
        pending.write_text(json.dumps(state, indent=2))
        pending.replace(session_file)

    def include(package, filename, **arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/filename)),
            launch_arguments=arguments.items())])

    def include_recovery(path, **arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=arguments.items())])

    simulation = include_recovery(RECOVERY_SIMULATION_LAUNCH,
        world=str(runtime_world), model_path=arg('model_path'), gui=arg('gui'),
        spawn_x=arg('spawn_x'), spawn_y=arg('spawn_y'), spawn_z=arg('spawn_z'),
        spawn_yaw=arg('spawn_yaw'), spawn_delay_sec=arg('spawn_delay_sec'),
        robot_xacro_file=str(RECOVERY_ROOF_XACRO),
        sim_lidar_samples='181', sim_lidar_rate='8')
    safety = GroupAction(actions=[
        SetParameter(name='simulation_motion_authorization', value=False),
        SetParameter(name='control_mode', value='mapping_manual'),
        include('cleannav_safety_supervisor', 'safety_supervisor.launch.py',
                use_sim_time='true', block_all='true',
                max_forward_linear_x=str(profile['safety']['max_forward_linear_x']),
                max_reverse_linear_x=str(profile['safety']['max_reverse_linear_x']),
                max_angular_z=str(profile['safety']['max_angular_z']))])
    collision_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='mapping_collision_lifecycle_manager', output='log',
        parameters=[{'use_sim_time': True}, {'autostart': True},
                    {'node_names': ['collision_monitor']}])
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='log',
        parameters=[str(runtime_collision_config), {'use_sim_time': True}])
    slam = GroupAction(actions=[
        SetParameter(name='is_rtabmap_paused', value=True),
        include_recovery(RECOVERY_RTAB_LAUNCH,
                cfg=str(cfg), database_path=str(session/'rtabmap.db'),
                scan_topic='/mapping/scan')])
    input_guard = Node(package='cleannav_simulation', executable='mapping_status_monitor',
        name='mapping_rtab_input_guard', output='log',
        arguments=['--session', str(session_file), '--resume-rtab', '--timeout', '30'],
        parameters=[{'use_sim_time': True}])
    coverage = Node(package='cleannav_simulation', executable='mapping_coverage',
                    output='log', parameters=[{'use_sim_time': True}])
    monitor = Node(package='cleannav_simulation', executable='mapping_status_monitor',
                   output='log', arguments=['--session', str(session_file)],
                   parameters=[{'use_sim_time': True}])
    probe = Node(package='cleannav_simulation', executable='mapping_status_monitor',
                 name='mapping_simulation_probe', output='log',
                 arguments=['--session', str(session_file), '--probe-simulation',
                            '--timeout', arg('simulation_timeout')],
                 parameters=[{'use_sim_time': True}])
    def simulated(event, context):
        if context.is_shutdown:
            return []
        if event.returncode:
            return [LogInfo(msg=f'[MAPPING] FAILED simulation inputs; logs={logs}'),
                    EmitEvent(event=Shutdown(reason='mapping simulation inputs failed'))]
        # Start lifecycle management only after Gazebo clock, sensor topics and
        # rear-axle TF have passed the simulation probe. The monitor process is
        # already present but stays inactive until its inputs are usable.
        return [collision_lifecycle, slam, input_guard]

    def started(event, context):
        if event.process_name.startswith('rtabmap-'):
            # PID+kernel start time avoids signalling a later process reusing this PID.
            stat = Path(f'/proc/{event.pid}/stat').read_text().rsplit(')', 1)[1].split()
            update_session(state='running', rtabmap_pid=event.pid, process_start_ticks=stat[19])
        return []

    critical = ('gzserver', 'robot_state_publisher', 'rtabmap-', 'safety_supervisor_node',
                'collision_monitor', 'mapping_collision_lifecycle_manager',
                'mapping_status_monitor')
    def exited(event, context):
        if event.action is probe:
            return []
        if event.action is input_guard and event.returncode == 0:
            return []
        if event.process_name.startswith('rtabmap-'):
            requested = json.loads(session_file.read_text()).get('state') == 'save_requested'
            update_session(state='rtabmap_closed', rtabmap_returncode=event.returncode)
            if requested and event.returncode == 0 and not context.is_shutdown:
                return [LogInfo(msg='[MAPPING] SAVING RTAB closed normally; session is finalized')]
        if context.is_shutdown:
            return []
        if (any(name in event.process_name for name in critical)
                or (event.returncode != 0 and any(name in event.process_name for name in ('spawn_entity', 'spawner')))):
            return [LogInfo(msg=f'[MAPPING] FAILED critical process {event.process_name} rc={event.returncode}'),
                    EmitEvent(event=Shutdown(reason='mapping critical process exit'))]
        return []

    def output(event):
        text = event.text.decode(errors='replace')
        return [LogInfo(msg=line) for line in text.splitlines()
                if line.startswith(('[MAPPING]', '[TELEOP]', '[SAFETY_SHUTDOWN]'))
                or 'Traceback' in line]

    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', f'http://127.0.0.1:{arg("gazebo_port")}'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('ROS_LOG_DIR', logs),
        SetParameter(name='use_sim_time', value=True),
        LogInfo(msg=f'[MAPPING] STARTING session={session_file}; startup_smoke; motion authorization disabled; no broker/route/Lease'),
        RegisterEventHandler(OnProcessExit(target_action=probe, on_exit=simulated)),
        RegisterEventHandler(OnProcessStart(on_start=started)),
        RegisterEventHandler(OnProcessExit(on_exit=exited)),
        RegisterEventHandler(OnProcessIO(on_stdout=output, on_stderr=output)),
        safety, collision_monitor, coverage, monitor,
        simulation, probe,
    ]
