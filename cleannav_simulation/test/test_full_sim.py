"""Readiness and composition regressions; these are not Gazebo runtime tests."""

import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('source', sorted(ROOT.glob('launch/*.launch.py')))
def test_launch_python_parses(source):
    ast.parse(source.read_text())


def test_full_launch_defaults_and_no_auto_task():
    module = load('full_sim_launch_test', 'launch/cleannav_full_sim.launch.py')
    description = module.generate_launch_description()
    defaults = {item.name: item.default_value[0].text for item in description.entities
                if hasattr(item, 'default_value')}
    assert defaults['enable_motion'] == 'false'
    assert defaults['auto_start_task'] == 'false'
    assert defaults['ros_domain_id'] != '0'
    source = (ROOT / 'launch/cleannav_full_sim.launch.py').read_text()
    assert "execution_mode='facade'" in source
    assert "safety_block_all='true'" in source
    assert 'pc_demo.launch.py' not in source
    assert not any(isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                   and item.func.id == 'TimerAction' for item in ast.walk(ast.parse(source)))


def test_error_identity_matches_stderr_and_rosout():
    module = load('error_identity_test', 'launch/cleannav_full_sim.launch.py')
    stderr = '\x1b[31m[ERROR] [123.456] [controller_server]: Failed to make progress\x1b[0m\r'
    rosout = '[ERROR] ERROR controller_server: Failed to make progress'
    assert module._error_identity(stderr) == module._error_identity(rosout)
    assert module._error_identity(stderr) == ('controller_server', 'Failed to make progress')
    assert module._error_identity('[MISSION] FAILED task=31') is None


@pytest.mark.parametrize('setting', ['automatic_without_motion', 'domain_zero', 'missing_world'])
def test_invalid_full_launch_fails_before_starting_processes(monkeypatch, setting):
    module = load('invalid_full_launch_test', 'launch/cleannav_full_sim.launch.py')
    description = module.generate_launch_description()
    context = LaunchContext()
    for item in description.entities:
        if hasattr(item, 'default_value'):
            context.launch_configurations[item.name] = item.default_value[0].text
    if setting == 'automatic_without_motion':
        context.launch_configurations['auto_start_task'] = 'true'
    elif setting == 'domain_zero':
        context.launch_configurations['ros_domain_id'] = '0'
    else:
        context.launch_configurations['world'] = '/missing/world'
    with pytest.raises((ValueError, FileNotFoundError)):
        module._compose(context)


@pytest.mark.parametrize('missing', ['clock', 'scan', 'odom'])
def test_missing_sensor_cannot_pass_simulation_readiness(missing):
    module = load('readiness_policy_test', 'scripts/simulation_status_monitor.py')
    facts = dict.fromkeys(('clock', 'spawn_service', 'vehicle', 'scan', 'odom', 'odom_tf',
                          'scan_tf'), True)
    facts[missing] = False
    assert module.missing_requirements('simulation', facts) == [missing]


@pytest.mark.parametrize('missing', [
    'planner_action', 'follow_action', 'facade_action',
    'planner_active', 'controller_active', 'safety',
])
def test_missing_navigation_interface_cannot_pass_readiness(missing):
    module = load('navigation_policy_test', 'scripts/simulation_status_monitor.py')
    facts = dict.fromkeys(('planner_active', 'controller_active', 'planner_action',
                          'follow_action', 'facade_action', 'safety'), True)
    facts[missing] = False
    assert module.missing_requirements('navigation', facts) == [missing]


def test_scan_requires_actual_valid_message(capsys):
    module = load('actual_sensor_validation_test', 'scripts/simulation_status_monitor.py')
    rclpy.init()
    node = module.SimulationStatusMonitor()
    try:
        assert all(callable(subscription.callback) for subscription in node.subscriptions)
        from rosgraph_msgs.msg import Clock
        clock = Clock()
        clock.clock.sec = 1
        node._on_clock(clock)
        clock.clock.sec = 2
        node._on_clock(clock)
        assert node.clock_samples == 2
        from rcl_interfaces.msg import Log
        node._rosout(Log(level=20, name='test', msg='ordinary info'))
        assert not capsys.readouterr().out
        node._rosout(Log(level=40, name='test', msg='actual error'))
        assert '[ERROR]' in capsys.readouterr().out
        node._rosout(Log(level=40, name='test', msg='actual error'))
        assert not capsys.readouterr().out
        node._scan(LaserScan())
        assert not node._fresh('scan')
        scan = LaserScan()
        scan.angle_increment = 0.01
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [float('nan')]
        node._scan(scan)
        assert not node._fresh('scan')
        scan.ranges = [2.0]
        node._scan(scan)
        assert node._fresh('scan')
        node.received['scan'] -= 10.0
        assert not node._fresh('scan')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failed_motion_authorization_never_reports_ready(monkeypatch, capsys):
    from types import SimpleNamespace
    import json
    module = load('failed_authorization_test', 'scripts/simulation_status_monitor.py')
    rclpy.init()
    node = module.SimulationStatusMonitor(enable_motion=True)
    try:
        facts = dict.fromkeys((
            'clock', 'spawn_service', 'vehicle', 'scan', 'odom', 'odom_tf', 'scan_tf',
            'slam_node', 'map_odom_tf', 'map_base_tf', 'map', 'planner_active',
            'controller_active', 'planner_action', 'follow_action', 'facade_action',
            'safety', 'task_subscriber', 'task_status', 'real_adapters',
        ), True)
        monkeypatch.setattr(node, 'facts', lambda: facts)
        node.safety = SimpleNamespace(emergency_stop_active=False)
        node.authorization_failed = True
        messages = []
        node.readiness_pub = SimpleNamespace(publish=messages.append)
        node.panel()
        assert '[CLEANNAV] BLOCKED' in capsys.readouterr().out
        assert json.loads(messages[-1].data)['ready'] is False
        assert not node.ever_ready
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_scan_tf_requires_actual_static_and_timestamped_odom_transforms():
    from geometry_msgs.msg import TransformStamped
    module = load('scan_time_tf_test', 'scripts/simulation_status_monitor.py')
    rclpy.init()
    node = module.SimulationStatusMonitor(quiet=True)
    try:
        scan = LaserScan()
        scan.header.frame_id = 'base_scan'
        scan.header.stamp.sec = 10
        scan.angle_increment = 0.01
        scan.range_min, scan.range_max, scan.ranges = 0.1, 10.0, [2.0]
        node._scan(scan)
        assert not node._scan_tf()
        static = TransformStamped()
        static.header.frame_id, static.child_frame_id = 'base_footprint', 'base_scan'
        static.transform.rotation.w = 1.0
        node.buffer.set_transform_static(static, 'test')
        assert not node._scan_tf()
        odom = TransformStamped()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.header.stamp.sec = 10
        odom.transform.rotation.w = 1.0
        node.buffer.set_transform(odom, 'test')
        assert node._scan_tf()
        # A latest scan awaiting its TF must not invalidate a recent valid scan.
        import copy
        scan = copy.deepcopy(scan)
        scan.header.stamp.sec = 11
        node._scan(scan)
        assert node._scan_tf()
        node.scan_headers.clear()
        node._scan(scan)
        assert not node._scan_tf()
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('component', [
    'gzserver', 'rtabmap', 'planner_server', 'controller_server', 'simulation_status_monitor',
])
@pytest.mark.parametrize('shutting_down', [False, True])
def test_critical_process_exit_is_diagnosable_and_shutdown_does_not_restart(
    monkeypatch, tmp_path, component, shutting_down,
):
    module = load('process_exit_policy_test', 'launch/cleannav_full_sim.launch.py')
    monkeypatch.setenv('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'log')
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(ROOT.parent / name))
    context = LaunchContext()
    for item in module.generate_launch_description().entities:
        if hasattr(item, 'default_value'):
            context.launch_configurations[item.name] = item.default_value[0].text
    world = tmp_path / 'world.sdf'
    world.write_text('<sdf version="1.6"><world name="test"/></sdf>')
    context.launch_configurations['world'] = str(world)
    actions = module._compose(context)
    handlers = [action.event_handler for action in actions
                if isinstance(action, RegisterEventHandler)
                and isinstance(action.event_handler, OnProcessExit)]
    handler = handlers[-1]
    context._set_is_shutdown(shutting_down)
    process = ExecuteProcess(cmd=['/bin/false'])
    event = ProcessExited(action=process, name=component + '-1', cmd=['/bin/false'],
                          cwd=None, env=None, pid=123, returncode=1)
    result = handler.handle(event, context)
    if shutting_down:
        assert result == []
    else:
        assert any(isinstance(action, EmitEvent) for action in result)


def test_gazebo_empty_params_file_cannot_replace_navigation_yaml(monkeypatch, tmp_path):
    module = load('navigation_parameter_scope_test', 'launch/cleannav_full_sim.launch.py')
    monkeypatch.setenv('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log')
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(ROOT.parent / name))
    context = LaunchContext()
    for item in module.generate_launch_description().entities:
        if hasattr(item, 'default_value'):
            context.launch_configurations[item.name] = item.default_value[0].text
    world = tmp_path / 'world.sdf'
    world.write_text('<sdf version="1.6"><world name="test"/></sdf>')
    context.launch_configurations['world'] = str(world)
    context.launch_configurations['params_file'] = ''  # Gazebo's real default.
    actions = module._compose(context)
    handlers = [action.event_handler for action in actions
                if isinstance(action, RegisterEventHandler)
                and isinstance(action.event_handler, OnProcessExit)]
    event = ProcessExited(action=ExecuteProcess(cmd=['true']), name='slam-probe',
                          cmd=['true'], cwd=None, env=None, pid=123, returncode=0)
    navigation = handlers[1].handle(event, context)[0]

    def includes(action):
        if isinstance(action, IncludeLaunchDescription):
            yield action
        elif isinstance(action, GroupAction):
            for child in action.get_sub_entities():
                yield from includes(child)

    child = next(includes(navigation))
    value = dict(child.launch_arguments)['params_file']
    filename = perform_substitutions(context, normalize_to_list_of_substitutions(value))
    parameters = yaml.safe_load(Path(filename).read_text())
    assert parameters['planner_server']['ros__parameters']['GridBased']['plugin'] == (
        'nav2_smac_planner/SmacPlannerHybrid'
    )
    assert context.launch_configurations['params_file'] == ''
