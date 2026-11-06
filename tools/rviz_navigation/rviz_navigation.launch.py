"""Unified source entry; static-integrated, dynamic acceptance still NOT RUN.

Reuse the full-sim business chain. Select the proven dual-lidar model explicitly,
reuse disk-map localization, and insert CM before the existing Safety input.
Run only under a separately approved resource/lock/authorization/cleanup supervisor.
"""
from pathlib import Path
import importlib.util
import json
import os
import sys
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, TimerAction, UnsetEnvironmentVariable)
from launch.events import Shutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from supervisor_policy import (EXTENDED_STARTUP, LOCALIZATION_DIAGNOSTIC,
                               MAX_LAUNCH_RUNTIME_S, SHORT_GOAL_MOTION)
spec = importlib.util.spec_from_file_location('rviz_integration_contract', HERE/'integration.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)
NAV = contract.NAV
base = contract.load_module(NAV/'cleannav_simulation/launch/cleannav_full_sim.launch.py', 'rviz_base_full_sim')


def _compose(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    if arg('auto_start_task') != 'false' or arg('database_path'):
        raise ValueError('RViz entry forbids auto tasks and arbitrary database_path')
    motion_profile = arg('motion_profile')
    odometry_mode = arg('odometry_mode')
    if odometry_mode not in ('wheel', 'oracle'):
        raise ValueError('unsupported odometry mode')
    motion_enabled = arg('enable_motion') == 'true'
    if motion_profile not in ('none', 'short-goal', 'historical', 'patrol'):
        raise ValueError('unsupported motion profile')
    if motion_enabled != (motion_profile in ('short-goal', 'historical', 'patrol')):
        raise ValueError('enable_motion requires the explicit short-goal profile, and vice versa')
    if arg('ros_domain_id') != '204' or arg('gazebo_port') != '11704':
        raise ValueError('RViz entry retains audited Domain 204 / Gazebo port 11704')
    contract.verify_source_manifest()
    prefixes = contract.verify_packages(get_package_prefix)
    runtime, session, profile = contract.prepare_inputs(arg, Path(launch_config.log_dir), prefixes)
    amcl_mode = session.get('localization_kind') == 'amcl'
    if amcl_mode and motion_profile not in ('none','short-goal','historical'):
        raise ValueError('Oracle-AMCL profile supports only none, short-goal, or historical motion')
    if motion_profile=='short-goal' and session['map_profile']!='simple-four-wall-nohit-v1-oracle-short-goal':
        raise ValueError('retired short-goal map/profile combination')
    oracle = None
    oracle_action = None
    model_xacro = session['simulation_model_xacro']
    controller_yaml = str(Path(prefixes['cleannav_simulation']) /
        'share/cleannav_simulation/config/ackermann_controllers.yaml')
    if odometry_mode == 'oracle':
        oracle = contract.prepare_oracle_odometry_inputs(
            model_xacro, controller_yaml, runtime)
        model_xacro = oracle['model_xacro']
        controller_yaml = oracle['controller_yaml']
        oracle_action = ExecuteProcess(
            name='gazebo_oracle_odometry', output='screen', cmd=[
                sys.executable, str(HERE/'oracle_odometry.py'),
                '--ros-args', '-p', 'use_sim_time:=true',
                '-p', 'truth_output:='+str(runtime/'oracle-truth.jsonl')])
    if (motion_enabled
            and session.get('map_profile') in ('mcity-localization07-canonical-debug', 'patrol-a-v4')):
        # The last accepted match in the 2026-10-23 navigation run was followed
        # by ~20 mm / 0.01 rad of final motion. The saved 0.1 m / 0.1 rad update
        # thresholds suppressed the next proximity scan search. Tune only this
        # run's private INI so RTAB-Map can register that final displacement;
        # immutable map assets and the health monitor's acceptance rules remain
        # unchanged.
        localization_mode = contract.load_module(
            Path(prefixes['cleannav_simulation']) / 'share' / 'cleannav_simulation'
            / 'launch' / 'localization_mode.py',
            'rviz_localization_mode')
        updates = localization_mode.configure_motion_updates(
            runtime / 'rtabmap_config.ini', '0.005', '0.005')
        if updates != {'RGBD/LinearUpdate': '0.005', 'RGBD/AngularUpdate': '0.005'}:
            raise ValueError('private RTAB-Map motion registration settings were not applied')
        if motion_profile == 'historical':
            # 0.22.1 doubles this ratio for merged proximity scans. The final
            # scans in run163521 had 20/122..24/126 correspondences, below .20.
            cfg = runtime / 'rtabmap_config.ini'
            lines = [line for line in cfg.read_text().splitlines()
                     if line.split('=', 1)[0].strip().replace('\\', '/') != 'Icp/CorrespondenceRatio']
            lines.insert(lines.index('[Core]') + 1, 'Icp\\CorrespondenceRatio=0.075')
            cfg.write_text('\n'.join(lines) + '\n')
    spawn = session.get('spawn') or {
        'x': arg('spawn_x'), 'y': arg('spawn_y'), 'z': arg('spawn_z'), 'yaw': arg('spawn_yaw')}
    spawn_delay = session.get('spawn_delay_sec') or arg('spawn_delay_sec')
    cm = yaml.safe_load((HERE/'collision_monitor.source.yaml').read_text())
    cm['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = '/cleannav/navigation/cmd_vel_raw'
    cm_path = runtime/'collision_monitor.yaml'
    cm_path.write_text(yaml.safe_dump(cm, sort_keys=False))
    cm_node = Node(package='nav2_collision_monitor', executable='collision_monitor',
                   name='collision_monitor', output='log', parameters=[str(cm_path)])
    cm_manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='nav2_collision_lifecycle_manager', output='log', parameters=[{
                          'use_sim_time': True, 'autostart': False, 'node_names': ['collision_monitor']}])
    cm_gate = ExecuteProcess(name='rviz_collision_startup', output='screen', cmd=[
        sys.executable, str(HERE/'collision_monitor_lifecycle_startup.py'),
        '--node', '/collision_monitor', '--manager', '/nav2_collision_lifecycle_manager',
        '--timeout', '180' if EXTENDED_STARTUP else '45', '--discovery-settle', '2', '--startup-response-timeout', '12',
        '--output', str(runtime/'collision-lifecycle.json')])
    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server',
                      output='log', parameters=[{'use_sim_time': True,
                        'yaml_filename': session['map_yaml_path'],
                        'topic_name': '/cleannav/localization/map', 'frame_id': 'map'}])
    map_gate = Node(package='cleannav_simulation', executable='campus_map_server_lifecycle_coordinator',
                    name='rviz_map_lifecycle', output='log', arguments=[
                        '--output-dir', str(runtime), '--service-timeout', '180' if EXTENDED_STARTUP else '20',
                        '--response-timeout', '2', '--state-timeout', '2'])
    if odometry_mode == 'oracle':
        # Oracle publishes /odom at base_footprint and disables the rear-axle
        # static TF; the installed guard only accepts the wheel contract and can
        # never resume RTAB-Map. Wheel mode keeps the installed executable.
        input_guard = ExecuteProcess(name='rviz_rtab_input_guard', output='log', cmd=[
            sys.executable, str(HERE/'mapping_status_monitor.py'),
            '--session', str(runtime/'session.json'), '--resume-rtab',
            '--timeout', '180' if EXTENDED_STARTUP else '30',
            '--ros-args', '-p', 'use_sim_time:=true'])
    else:
        input_guard = Node(package='cleannav_simulation', executable='mapping_status_monitor',
                           name='rviz_rtab_input_guard', output='log', arguments=[
                               '--session', str(runtime/'session.json'), '--resume-rtab', '--timeout', '180' if EXTENDED_STARTUP else '30'],
                           parameters=[{'use_sim_time': True}])
    guard = ExecuteProcess(name='navigation_safety_guard', output='screen', cmd=[
        sys.executable, str(HERE/'navigation_safety_guard.py'),
        '--output', str(runtime/'safety-guard.json'),
        '--motion-enabled', arg('enable_motion'),
        '--motion-profile', arg('motion_profile'),
        '--odometry-mode', odometry_mode,
        '--run-id', os.environ.get('CLEANNAV_RVIZ_RUN_ID', ''),
        '--localization-scan-topic', session['scan_topic'],
        *(['--diagnostic-continue-on-readiness-loss'] if LOCALIZATION_DIAGNOSTIC else []),
        '--timeout', '195' if EXTENDED_STARTUP else '110',
        '--ros-args', '-p', 'use_sim_time:=true'])
    guard_gate = ExecuteProcess(name='rviz_guard_admission', output='screen', cmd=[
        sys.executable, str(HERE/'navigation_safety_guard.py'),
        '--wait-ready', str(runtime/'safety-guard.json'), '--timeout', '180' if EXTENDED_STARTUP else '40'])
    sim_key='cleannav_simulation/ackermann_mcity_demo.launch.py'
    rtab_key='cleannav_rtabmap/cleannav_rtabmap_lidar.launch.py'
    nav_key='cleannav_navigation/cleannav_ackermann_navigation.launch.py'
    integration = dict(diagnostic_startup_wait=EXTENDED_STARTUP, database=session['database_path'], cfg=str(runtime/'rtabmap_config.ini'),
        session=str(runtime/'session.json'), forward_limit=str(profile['safety']['max_forward_linear_x']),
        # Both localizers use the source monitor: it is the only copy that
        # selects the /odom contract from the odometry source (the installed
        # localization_status_monitor only accepts the wheel contract).
        localization_monitor_source=HERE/'localization_status_monitor.py',
        defer_motion_authorization=(motion_profile == 'short-goal'),
        reverse_limit='0.0', angular_limit=str(profile['safety']['max_angular_z']),
        includes={sim_key: NAV/'cleannav_simulation/launch/startup_smoke_simulation.launch.py',
                  rtab_key: NAV/'cleannav_rtabmap/launch/cleannav_rtabmap_localization.launch.py',
                  nav_key: NAV/'cleannav_navigation/launch/cleannav_ackermann_navigation.launch.py'},
        arguments={sim_key:dict(robot_xacro_file=model_xacro,
                                # Wheel mode retains the selected installed
                                # controller's rear-axle odometry frame. Oracle
                                # mode passes private copies with that wheel TF
                                # authority disabled and publishes at footprint.
                                controllers_file=controller_yaml,
                                **({'publish_rear_axle_tf': 'false'} if oracle else {}),
                                sim_lidar_samples='181', sim_lidar_rate='8',
                                spawn_x=spawn['x'], spawn_y=spawn['y'], spawn_z=spawn['z'],
                                spawn_yaw=spawn['yaw'], spawn_delay_sec=spawn_delay),
                   rtab_key:dict(localization='true',
                                 initial_pose=session['localization_initial_pose'],
                                 scan_topic=session['scan_topic'],
                                 odom_frame_id=session['odom_frame_id'],
                                 odom_topic=session['odom_topic'],
                                 core_log_args='--uinfo' if EXTENDED_STARTUP else ''),
                   nav_key:dict(controller_cmd_vel_topic='/cleannav/navigation/cmd_vel_raw',
                                params_file=str(runtime/'nav2_params.yaml'))},
        localization_actions=[input_guard, map_server, map_gate],
        collision_actions=[cm_node,cm_manager,cm_gate], collision_gate=cm_gate,
        mission_parameters=({'region_registry_file': str(contract.WORKSPACE/session['patrol']['registry_relative'])}
                            if session.get('patrol') else {}),
        initial_actions=[guard, *([oracle_action] if oracle_action else [])], guard_gate=guard_gate,
        oneshot_actions=[cm_gate,guard_gate,input_guard,map_gate], rviz_config=session['rviz_config'])
    if arg('motion_profile') == 'short-goal':
        integration['mission_environment']={'PYTHONPATH':str(HERE)+os.pathsep+os.environ.get('PYTHONPATH','')}
    integration['odometry_mode'] = odometry_mode
    if amcl_mode:
        identity = session['amcl_runtime_identity']
        amcl = Node(package='nav2_amcl', executable='amcl', name='amcl', namespace='/',
            output='log', parameters=[identity['localization_runtime_identity']['params'],
                                      session['amcl_overrides']],
            remappings=[('map', '/cleannav/localization/map')],
            additional_env=identity['amcl_node_environment'])
        manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='amcl_lifecycle_manager', output='log', parameters=[{
                'use_sim_time':True, 'autostart':True, 'node_names':['amcl']}])
        integration.update(localization_kind='amcl', localization_node_actions=[amcl, manager],
            localization_actions=[map_server,map_gate], oneshot_actions=[cm_gate,guard_gate,map_gate])
        # The verified loader environment belongs only to the AMCL process.
        os.environ.pop('CLEANNAV_TF_PRIVATE_DIAGNOSTICS', None)
        integration['environment_actions']=[UnsetEnvironmentVariable('CLEANNAV_TF_PRIVATE_DIAGNOSTICS')]
    if oracle:
        integration['oracle_odometry'] = oracle
    if motion_profile == 'short-goal':
        run_id = os.environ.get('CLEANNAV_RVIZ_RUN_ID', '')
        run_dir = Path(os.environ.get('CLEANNAV_RVIZ_RUN_DIR', '')).resolve()
        if (not run_id.startswith('rviz-nav-') or not run_dir.is_dir()
                or not (run_dir/'wrapper.json').is_file()):
            raise ValueError('short-goal adapter requires the current wrapper-bound run directory')
        integration['mission_goal_actions'] = [ExecuteProcess(
            name='cleannav_short_goal_rviz_adapter', output='screen', cmd=[
                sys.executable, str(HERE/'short_goal_rviz_adapter.py'),
                '--run-id', run_id,
                '--authorization', str(run_dir/'motion-authorization.json'),
                '--scope', str(run_dir/'motion-scope.json'),
                '--admission', str(run_dir/'motion-admission.json'),
                '--lifecycle', str(run_dir/'motion-lifecycle.json'),
                '--events-path', str(run_dir/'motion-events.jsonl'),
                '--execution-id', 'ros-execution-1',
                '--ros-args', '-p', 'use_sim_time:=true'])]
    else:
        integration['mission_goal_actions'] = [] if motion_profile == 'patrol' else None
    # Scoped launch shutdown is bounded by a timer; external process-group
    # cleanup and preflight time are owned by the supervisor, not this timer.
    timeout=float(arg('runtime_timeout_sec'))
    if not 0 < timeout <= MAX_LAUNCH_RUNTIME_S:
        raise ValueError('runtime_timeout_sec exceeds the selected timing profile bound')
    return [*base._compose(context, integration=integration),
            TimerAction(period=timeout, actions=[EmitEvent(event=Shutdown(reason='RViz session timeout'))])]


def generate_launch_description():
    # Retain base argument definitions to avoid a second parser with stale names.
    description = base.generate_launch_description()
    declarations = [x for x in description.entities if isinstance(x, DeclareLaunchArgument)]
    overrides = dict(enable_rviz_goals='true', enable_motion='false', auto_start_task='false',
                     motion_profile='none',
                     gui='false', start_rviz='true', ros_domain_id='204', gazebo_port='11704',
                     world=None, spawn_y='-8.0', simulation_timeout='180' if EXTENDED_STARTUP else '90',
                     slam_timeout='180' if EXTENDED_STARTUP else '40',
                     navigation_timeout='180' if EXTENDED_STARTUP else '40', mission_timeout='180' if EXTENDED_STARTUP else '20')
    required = {'world'}
    rebuilt=[]
    for old in declarations:
        if old.name in overrides:
            rebuilt.append(DeclareLaunchArgument(old.name, **(
                {} if old.name in required else {'default_value':overrides[old.name]}), choices=old.choices))
        else:
            rebuilt.append(old)
    return LaunchDescription([*rebuilt,
        DeclareLaunchArgument('odometry_mode', default_value='wheel', choices=['wheel', 'oracle'],
            description='Select wheel odometry or the Gazebo truth diagnostic source'),
        DeclareLaunchArgument('map_directory', description='Immutable persisted map directory; audited contract must match'),
        DeclareLaunchArgument('map_profile', default_value='roof-lidar-360',
            choices=sorted(contract.PROFILE_CONTRACTS),
            description='Explicit allow-listed disk map and localization profile'),
        DeclareLaunchArgument('localization_initial_pose', default_value=''),
        DeclareLaunchArgument('spawn_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('runtime_timeout_sec', default_value='145'),
        DeclareLaunchArgument('motion_profile', default_value='none', choices=['none', 'short-goal', 'historical', 'patrol']),
        OpaqueFunction(function=_compose)])
