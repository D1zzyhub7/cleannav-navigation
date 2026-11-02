"""Resource and typed sequence contracts; no ROS init, nodes or service calls."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace as N

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(HERE), str(HERE.parent/'startup_smoke')]
import integration
import roof_lidar_runtime as runtime
import supervisor_policy as policy


def test_low_available_is_auditable_warning_but_other_resources_still_block(monkeypatch):
    values = dict(MemAvailable=5067984, SwapTotal=1000000, SwapFree=999368, SwapCached=0)
    monkeypatch.setattr(runtime, 'read_meminfo', lambda: values)
    monkeypatch.setattr(runtime, 'read_psi_avg10', lambda: 0.)
    monkeypatch.setattr(runtime, 'cgroup_memory_snapshot', lambda: dict(status='UNKNOWN'))
    waiver = dict(eligible=True, status='WAIVER_ELIGIBLE', cgroup_memory_disposition='WAIVED')
    def sample():
        return runtime.memory_snapshot(cgroup_waiver=waiver,
            min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,
            memavailable_warning_only=True)
    result = sample()
    assert result['ok'] and result['MemAvailable_warning']
    assert result['cgroup_memory_status'] == 'UNKNOWN'
    assert result['cgroup_memory_disposition'] == 'WAIVED'
    assert not runtime.memory_snapshot(cgroup_waiver=waiver,
        min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB)['ok']
    monkeypatch.setattr(runtime, 'read_psi_avg10', lambda: 1.)
    assert not sample()['ok']
    monkeypatch.setattr(runtime, 'read_psi_avg10', lambda: 0.)
    values.update(SwapTotal=2*1024*1024, SwapFree=0)
    assert not sample()['ok']
    values.update(SwapFree=values['SwapTotal'])
    monkeypatch.setattr(runtime, 'cgroup_memory_snapshot', lambda: dict(status='FINITE', headroom_bytes=0))
    assert not sample()['ok']


def test_formal_memory_has_no_replacement_floor_and_still_stops_on_oom():
    result = policy.runtime_memory_decision(dict(ok=True, reasons=[], MemAvailable_kib=4000000),
        timing_profile='historical-motion', no_motion_contract=False)
    assert result['ok'] and result['soft_warning'] and result['hard_stop_threshold_kib'] is None
    assert not policy.runtime_memory_decision(dict(ok=True, reasons=[], MemAvailable_kib=5067984, oom_kill_delta=1),
        timing_profile='historical-motion', no_motion_contract=False)['ok']


def test_a_profile_matches_immutable_map_world_and_closed_registry():
    profile = json.loads(integration.contract_path_for_profile('patrol-a-v4').read_text())
    result = integration.verify_map(integration.WORKSPACE/profile['map_directory_relative'],
        integration.WORKSPACE/profile['world_relative'], profile='patrol-a-v4')
    assert result['patrol']['laps'] == 1 and result['patrol']['route_segments'] == 5
    assert result['patrol']['map_identity']['sha256'] == result['files']['occupancy_grid.json']


def test_actual_submit_method_sends_one_typed_atomic_sequence():
    from cleannav_interfaces.srv import SubmitTaskSequence
    from cleannav_interfaces.msg import TaskSequenceStep
    from builtin_interfaces.msg import Time
    tree = ast.parse((HERE/'navigation_supervisor.py').read_text())
    method = next(x for x in ast.walk(tree) if isinstance(x, ast.FunctionDef) and x.name == 'submit_patrol')
    spec = json.loads((HERE/'map_contract_patrol_a_v4.json').read_text())['patrol']
    sent = []
    response = SubmitTaskSequence.Response(accepted=True, sequence_id='rviz-nav-test', current_execution_id='ros-execution-1')
    future = N(result=lambda: response, add_done_callback=lambda fn: fn(future))
    client = N(service_is_ready=lambda: True, call_async=lambda request: sent.append(request) or future)
    owner = N(patrol_started=None, patrol_client=client, patrol_owners=set(),
        get_clock=lambda: N(now=lambda: N(to_msg=lambda: Time(sec=42))),
        get_publishers_info_by_topic=lambda _: [N()])
    scope = dict(SubmitTaskSequence=SubmitTaskSequence, TaskSequenceStep=TaskSequenceStep,
        args=N(run_id='rviz-nav-test'), patrol_spec=spec, time=N(monotonic=lambda: 1.),
        event=lambda *a, **kw: None, one_expected_publisher=lambda *a, **kw: (True, []),
        ContractError=policy.ContractError, fail=lambda *a: (_ for _ in ()).throw(AssertionError(a)))
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<actual submit_patrol>', 'exec'), scope)
    scope['submit_patrol'](owner)
    request = sent[0]
    assert owner.patrol_accepted and len(sent) == 1
    assert request.header.stamp.sec == 42 and request.command_id == 'rviz-nav-test-patrol'
    assert [s.operation for s in request.steps] == [TaskSequenceStep.GOTO_REGION, TaskSequenceStep.PATROL_REGION]
    assert request.steps[1].laps == 1 and request.steps[1].route_id == spec['route_id']
    assert request.map_sha256 == spec['map_identity']['sha256']
    with pytest.raises(policy.ContractError, match='duplicate'):
        scope['submit_patrol'](owner)


def test_runtime_guard_accepts_only_the_bound_execution():
    facts = {key: True for key in ('/scan','/mapping/scan','/scan_tf','/mapping/scan_tf',
        'clock','odom','safety','cmd','cm','safety_no_estop','cm_active','cmd_publisher',
        '/cleannav/navigation/cmd_vel_raw_publisher','/cleannav/cmd_vel_candidate_publisher',
        '/cmd_vel_authorized_publisher','localization')}
    safety = dict(block_all=False, estop=False, enabled=True, owner='ros-execution-5')
    state = dict(monotonic=10., facts=facts, safety=safety, ready=True, missing=[], distance_m=5.)
    policy.validate_motion_guard_runtime_state(state, now_monotonic=10., motion_admitted=True,
        mission_lease_active=True, expected_owner='ros-execution-5', maximum_distance_m=14.)
    with pytest.raises(policy.ContractError, match='Lease'):
        policy.validate_motion_guard_runtime_state(state, now_monotonic=10., motion_admitted=True,
            mission_lease_active=True, expected_owner='ros-execution-6', maximum_distance_m=14.)


def test_actual_a_composition_prepares_registry_and_existing_checkers(tmp_path, monkeypatch):
    import yaml
    from test_integration import adapter, defaults
    module = adapter()
    contract = json.loads((HERE/'map_contract_patrol_a_v4.json').read_text())
    monkeypatch.setattr(module.launch_config, 'log_dir', str(tmp_path))
    captured = []
    monkeypatch.setattr(module.base, '_compose', lambda context, integration: captured.append(integration) or [])
    context = defaults(module, dict(map_profile='patrol-a-v4',
        map_directory=str(integration.WORKSPACE/contract['map_directory_relative']),
        world=str(integration.WORKSPACE/contract['world_relative']),
        motion_profile='patrol', enable_motion='true', enable_rviz_goals='false',
        odometry_mode='wheel', runtime_timeout_sec='100',
        spawn_x='-15.0', spawn_y='2.4', spawn_z='0.02', spawn_yaw='0.0'))
    module._compose(context)
    config = captured[0]
    assert config['mission_parameters']['region_registry_file'] == str(integration.WORKSPACE/contract['patrol']['registry_relative'])
    assert config['mission_goal_actions'] == []
    session = json.loads(Path(config['session']).read_text())
    nav = yaml.safe_load(Path(session['nav2_yaml_path']).read_text())['controller_server']['ros__parameters']
    assert nav['FollowPath']['vx_max'] == .3
    assert {'patrol_goal_checker','patrol_position_checker'}.issubset(nav['goal_checker_plugins'])
    assert nav['patrol_position_checker']['plugin'] == 'cleannav_navigation::PositionGoalChecker'
    facade = yaml.safe_load(Path(session['nav2_yaml_path']).read_text())['cleannav_navigation_facade']['ros__parameters']
    assert facade['constraint_map_sha256'] == contract['patrol']['map_identity']['sha256']
    assert facade['patrol_route_id'] == contract['patrol']['route_id']
    assert facade['intermediate_patrol_goal_checker_id'] == 'patrol_position_checker'
