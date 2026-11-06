"""One launch owns the isolated Gazebo / SLAM / Nav2 / mission lifecycle."""

import os
from pathlib import Path
import re
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


DEFAULT_ASSETS = '/home/d1zzy/code/cleannav_demo_assets/osrf_car_demo/car_demo'


def _error_identity(line):
    """Match the same ROS error forwarded by process stderr and /rosout."""
    clean = re.sub(r'\x1b\[[0-9;]*m', '', line).strip()
    for pattern in (
        r'\[(?:ERROR|FATAL)\]\s+\[[^\]]+\]\s+\[([^\]]+)\]:\s*(.*)',
        r'^\[ERROR\]\s+(?:ERROR|FATAL)\s+([^:]+):\s*(.*)',
    ):
        match = re.search(pattern, clean)
        if match:
            return match.group(1), match.group(2)
    return None


def readiness_failure_actions(reason, *, diagnostic_startup_wait=False):
    """Only ordinary no-motion initialization failures wait for the global bound.

    Critical process exits use their separate unchanged Shutdown handler.
    A failed probe never advances a stage and never manufactures readiness.
    """
    if diagnostic_startup_wait:
        return [LogInfo(msg='[WAITING] '+reason+'; global startup window remains authoritative')]
    return [LogInfo(msg='[ERROR] FAILED '+reason), EmitEvent(event=Shutdown(reason=reason))]


def _compose(context, integration=None):
    # The repository RViz adapter supplies explicit source/asset bindings.
    # The historical online-demo composition keeps its existing defaults.
    integration = integration or {}
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    domain = int(arg('ros_domain_id'))
    if not 1 <= domain <= 232:
        raise ValueError('full simulation requires a nonzero isolated ROS_DOMAIN_ID (1..232)')
    motion, automatic = arg('enable_motion') == 'true', arg('auto_start_task') == 'true'
    if automatic and not motion:
        raise ValueError('auto_start_task=true requires explicit enable_motion=true')
    if not Path(arg('world')).is_file():
        raise FileNotFoundError(f'MCity world missing: {arg("world")}')
    # Humble's ExecuteLocal supports this documented override, including
    # nested third-party launches. Complete stdout/stderr stays in launch logs.
    os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'own_log'
    logs = launch_config.log_dir
    simulation_world = arg('world')
    if arg('enable_diagnostics') == 'true' or integration.get('oracle_odometry'):
        # Read-only ground truth service; leave the original MCity asset intact.
        world_text = Path(simulation_world).read_text()
        state_plugin = (
            '<plugin name="cleannav_diagnostic_state" filename="libgazebo_ros_state.so">'
            '<ros><namespace>/gazebo</namespace></ros><update_rate>10</update_rate></plugin>'
        )
        simulation_world = str(Path(logs) / 'diagnostic.world')
        Path(simulation_world).write_text(world_text.replace('</world>', state_plugin+'</world>'))
    database = integration.get('database') or arg('database_path') or str(Path(logs) / 'rtabmap.db')
    # RTAB-Map writes its cfg on shutdown. Give each run a private copy.
    rtab_cfg = str(Path(logs) / 'rtabmap_lidar.ini')
    shutil.copyfile(integration.get('cfg') or (
        Path(get_package_share_directory('cleannav_rtabmap')) / 'config' / 'rtabmap_lidar.ini'), rtab_cfg)

    def include(package, filename, **arguments):
        key = package + '/' + filename
        arguments.update(integration.get('arguments', {}).get(key, {}))
        source = integration.get('includes', {}).get(key) or (
            Path(get_package_share_directory(package)) / 'launch' / filename)
        child = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments=arguments.items(),
        )
        # Included launch defaults (notably Gazebo's empty params_file)
        # must not leak into subsequent SLAM / Navigation stages.
        return GroupAction(scoped=True, actions=[child])

    def probe(stage):
        if integration.get('localization_monitor_source'):
            return ExecuteProcess(name='cleannav_probe_'+stage, output='log', cmd=[
                sys.executable, str(integration['localization_monitor_source']),
                '--session', integration['session'], '--probe', stage,
                '--timeout', arg(f'{stage}_timeout'), '--ros-args', '-p', 'use_sim_time:=true'])
        localization = bool(integration) and stage == 'slam'
        return Node(
            package='cleannav_simulation', executable=(
                'localization_status_monitor' if localization else 'simulation_status_monitor'),
            name=f'cleannav_probe_{stage}', output='log',
            arguments=['--probe', stage, '--timeout', arg(f'{stage}_timeout')] + (
                ['--session', integration['session']] if localization else []),
            parameters=[{'use_sim_time': True}],
        )

    simulation = include(
        'cleannav_simulation', 'ackermann_mcity_demo.launch.py',
        world=simulation_world, model_path=arg('model_path'), gui=arg('gui'),
        spawn_x=arg('spawn_x'), spawn_y=arg('spawn_y'),
        spawn_z=arg('spawn_z'), spawn_yaw=arg('spawn_yaw'),
    )
    if integration.get('localization_kind') == 'amcl':
        slam = GroupAction(actions=integration['localization_node_actions'])
    else:
        slam = include('cleannav_rtabmap', 'cleannav_rtabmap_lidar.launch.py',
                       database_path=database, cfg=rtab_cfg)
        if integration:
            slam = GroupAction(actions=[SetParameter(name='is_rtabmap_paused', value=True), slam])
    safety = GroupAction(actions=[
        *([SetEnvironmentVariable('PYTHONPATH', integration['mission_environment']['PYTHONPATH'])]
          if integration.get('mission_environment') else []),
        SetParameter(name='simulation_motion_authorization', value=motion),
        *([SetParameter(name='require_localization', value=True)] if integration else []),
        include('cleannav_safety_supervisor', 'safety_supervisor.launch.py',
                use_sim_time='true', block_all='true',
                # The integrated guard requires status younger than 0.5 wall seconds.
                status_publish_frequency='10.0' if integration else '2.0',
                max_forward_linear_x=integration.get('forward_limit', '0.35'),
                max_reverse_linear_x=integration.get('reverse_limit', '0.10'),
                max_angular_z=integration.get('angular_limit', '0.60')),
    ])
    navigation = GroupAction(actions=[
        include('cleannav_navigation', 'cleannav_ackermann_navigation.launch.py',
                execution_mode='facade', use_sim_time='true', safety_block_all='true',
                start_safety_supervisor='false',
                params_file=(arg('nav2_params_file') or str(
                    Path(get_package_share_directory('cleannav_navigation'))
                    / 'config' / 'nav2_params_ackermann.yaml'))),
    ])
    manager = Node(
        package='cleannav_mission_manager', executable='cleannav_simulation_mission_manager',
        name='mission_manager_node', output='log',
        additional_env=integration.get('mission_environment', {}), parameters=[{
            'use_sim_time': True, 'enable_rviz_goals': arg('enable_rviz_goals') == 'true',
            'pose_motion_enabled': motion, **integration.get('mission_parameters', {})}],
    )
    monitor = Node(
        package='cleannav_simulation', executable=(
            'localization_status_monitor' if integration else 'simulation_status_monitor'),
        name='cleannav_simulation_status_monitor', output='log',
        # The unified RViz supervisor owns the one-shot Motion Admission.
        # In that composition, keep Safety.block_all asserted until the
        # outer supervisor has checked every readiness fact and explicitly
        # opens the existing simulation interlock. Legacy full-sim behavior
        # remains unchanged when this integration flag is absent.
        arguments=(['--enable-motion'] if motion and not integration.get(
            'defer_motion_authorization', False) else []) + (
            ['--session', integration['session']] if integration else []),
        parameters=[{'use_sim_time': True}],
    )
    if integration.get('localization_monitor_source'):
        monitor = ExecuteProcess(name='localization_status_monitor', output='log', cmd=[
            sys.executable, str(integration['localization_monitor_source']),
            '--session', integration['session']] + (
            ['--enable-motion'] if motion and not integration.get(
                'defer_motion_authorization', False) else []) + [
            '--ros-args', '-p', 'use_sim_time:=true'])
    probes = {stage: probe(stage) for stage in ('simulation', 'slam', 'navigation', 'mission')}
    next_actions = {
        'simulation': [slam, *integration.get('localization_actions', []), probes['slam']],
        'slam': [navigation, probes['navigation']],
        'navigation': [manager, probes['mission']],
        'mission': (integration.get('mission_goal_actions') if integration and
                    integration.get('mission_goal_actions') is not None else
                    ([Node(
                        package='cleannav_simulation', executable='rviz_goal_adapter', output='screen',
                        parameters=[{'use_sim_time': True}],
                    )] if arg('enable_rviz_goals') == 'true' else [])),
    }

    def stage_exit(stage):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                return readiness_failure_actions(f'{stage} readiness failed; diagnostics={logs}',
                    diagnostic_startup_wait=integration.get('diagnostic_startup_wait') is True)
            return next_actions[stage]
        return callback

    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=action, on_exit=stage_exit(stage))) for stage, action in probes.items()]
    if integration:
        # The existing CM coordinator is the transition owner. Only its
        # successful ACTIVE receipt admits localization, then Nav2/Mission.
        admitted = next_actions['simulation']
        next_actions['simulation'] = integration['collision_actions']
        def collision_exit(event, context):
            if context.is_shutdown:
                return []
            if event.returncode:
                return readiness_failure_actions('Collision Monitor startup failed',
                    diagnostic_startup_wait=integration.get('diagnostic_startup_wait') is True)
            return admitted
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=integration['collision_gate'], on_exit=collision_exit)))
        mission_admitted = next_actions['navigation']
        next_actions['navigation'] = [integration['guard_gate']]
        def guard_exit(event, context):
            if context.is_shutdown:
                return []
            if event.returncode:
                return readiness_failure_actions('navigation safety guard admission failed',
                    diagnostic_startup_wait=integration.get('diagnostic_startup_wait') is True)
            return mission_admitted
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=integration['guard_gate'], on_exit=guard_exit)))
    critical = ('gzserver', 'robot_state_publisher', 'rtabmap', 'planner_server',
                'controller_server', 'navigation_facade_node', 'safety_supervisor_node',
                'cleannav_simulation_mission_manager', 'simulation_status_monitor',
                'localization_status_monitor', 'collision_monitor', 'navigation_safety_guard', 'map_server',
                'amcl', 'amcl_lifecycle_manager') + (
                    ('gazebo_oracle_odometry',) if integration.get('oracle_odometry') else ())

    def process_exit(event, context):
        if context.is_shutdown or event.action in probes.values():
            return []
        if event.action in integration.get('oneshot_actions', []):
            return ([] if event.returncode == 0 else
                    readiness_failure_actions('navigation one-shot dependency failed',
                        diagnostic_startup_wait=integration.get('diagnostic_startup_wait') is True))
        name = event.process_name
        if (any(component in name for component in critical)
                or (event.returncode != 0 and any(
                    component in name for component in ('spawn_entity', 'spawner')))):
            return [LogInfo(msg=f'[ERROR] FAILED {name} exited '
                                f'rc={event.returncode}; logs={logs}'),
                    EmitEvent(event=Shutdown(reason=f'critical process exited: {name}'))]
        if 'rviz' in name and event.returncode != 0:
            return [LogInfo(msg=f'[RVIZ] PARTIAL_READY visualization exited; logs={logs}')]
        return []

    partial_lines = {}
    last_errors = {}
    auto_started = False

    def process_io(event):
        nonlocal auto_started
        key = (id(event.action), event.from_stdout)
        text = partial_lines.get(key, '') + event.text.decode(errors='replace')
        lines = text.split('\n')
        partial_lines[key] = lines.pop()[-8192:]
        actions = []
        for line in lines:
            identity = _error_identity(line)
            if identity:
                source, message = identity
                if last_errors.get(source) == message:
                    continue
                last_errors[source] = message
            if re.match(
                r'^\[(CLEANNAV|SIMULATION|SLAM|LOCALIZATION|NAVIGATION|MISSION|SAFETY|ERROR|NAV|MM|RVIZ_GOAL)\]', line,
            ):
                actions.append(LogInfo(msg=line))
            elif re.search(r'\b(ERROR|FATAL|Exception|Traceback)\b', line):
                actions.append(LogInfo(msg=f'[ERROR] {event.process_name}: {line}'))
            if (automatic and not auto_started
                    and line.startswith('[CLEANNAV] READY SYSTEM READY;')):
                auto_started = True
                actions.append(Node(
                    package='cleannav_simulation', executable='simulation_task_client',
                    output='log', parameters=[{
                        'use_sim_time': True, 'goal_x': float(arg('task_goal_x')),
                        'goal_y': float(arg('task_goal_y')),
                    }],
                ))
        return actions

    rviz = [LogInfo(msg='[RVIZ] DISABLED start_rviz=false')]
    if arg('start_rviz') == 'true':
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            rviz = [Node(
                package='rviz2', executable='rviz2', name='rviz', output='log',
                arguments=['-d', str(integration.get('rviz_config') or (Path(get_package_share_directory('cleannav_simulation'))
                                     / 'config' / ('simulation_v2_goals.rviz'
                                                  if arg('enable_rviz_goals') == 'true'
                                                  else 'simulation_v2.rviz')))],
                parameters=[{'use_sim_time': True}],
            ), LogInfo(msg='[RVIZ] STARTING visualization; verify displays in GUI')]
        else:
            rviz = [LogInfo(msg='[RVIZ] PARTIAL_READY no DISPLAY; running headless')]
    # Localization may require RViz's real initial-pose input. Start the GUI
    # alongside disk localization after CM admission, not after Mission READY.
    # The goal adapter remains behind Mission admission; this grants no motion.
    root_rviz = rviz
    if integration:
        admitted.extend(rviz)
        root_rviz = []
    diagnostics = []
    if arg('enable_diagnostics') == 'true':
        diagnostics = [Node(
            package='cleannav_simulation', executable='simulation_motion_diagnostics',
            output='log', arguments=['--output-dir', str(Path(logs) / 'motion'),
                                     '--duration', arg('diagnostics_duration')],
            parameters=[{'use_sim_time': True}],
        )]
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', f'http://127.0.0.1:{arg("gazebo_port")}'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('ROS_LOG_DIR', logs),
        # Avoid Gazebo attempting remote model downloads during startup.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        LogInfo(msg=f'[CLEANNAV] Launching simulation; domain={domain}; diagnostics={logs}'),
        *handlers,
        RegisterEventHandler(OnProcessExit(on_exit=process_exit)),
        RegisterEventHandler(OnProcessIO(on_stdout=process_io, on_stderr=process_io)),
        SetParameter(name='use_sim_time', value=True),
        *integration.get('environment_actions', []),
        monitor, safety, *integration.get('initial_actions', []),
        simulation, probes['simulation'], *root_rviz, *diagnostics,
    ]


def generate_launch_description():
    defaults = {
        'enable_rviz_goals': 'false',
        'enable_motion': 'false', 'auto_start_task': 'false', 'start_rviz': 'true',
        'gui': 'true', 'ros_domain_id': '229', 'gazebo_port': '11459',
        'world': DEFAULT_ASSETS + '/worlds/mcity.world',
        'model_path': DEFAULT_ASSETS + '/models', 'database_path': '',
        # Retain spawn substitutions in the outer context: the reused base
        # launch evaluates them later from its TimerAction callback.
        'spawn_x': '0.0', 'spawn_y': '-5.0', 'spawn_z': '0.02', 'spawn_yaw': '0.0',
        'simulation_timeout': '210.0', 'slam_timeout': '90.0',
        'navigation_timeout': '120.0', 'mission_timeout': '30.0',
        'task_goal_x': '0.8', 'task_goal_y': '0.0',
        'enable_diagnostics': 'false', 'diagnostics_duration': '600',
        'nav2_params_file': '',
    }
    boolean = {'enable_rviz_goals', 'enable_motion', 'auto_start_task', 'start_rviz', 'gui', 'enable_diagnostics'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value,
                                **({'choices': ['true', 'false']} if name in boolean else {}))
          for name, value in defaults.items()],
        OpaqueFunction(function=_compose),
    ])
