"""Pure contract checks of restored historical business-path selection."""
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest
HERE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(HERE),str(HERE.parents[1]/'tools/startup_smoke')]
import navigation_supervisor as supervisor
import supervisor_policy as policy


def test_historical_argv_binds_true_motion_and_default_business_adapter():
    argv=supervisor.bounded_launch_argv(world=Path('/w'),map_directory=Path('/m'),
        map_profile='mcity-localization07-canonical-debug',output_dir=Path('/r'),
        runtime_timeout_s=140,motion_profile='historical')
    assert 'enable_motion:=true' in argv
    assert 'enable_rviz_goals:=true' in argv
    assert 'auto_start_task:=false' in argv
    assert 'motion_profile:=historical' in argv
    assert 'spawn_y:=-5.0' in argv and 'spawn_delay_sec:=60.0' in argv
    assert policy.verify_short_goal_launch_argv(argv,expected_profile='historical')['enable_motion']
    with pytest.raises(policy.ContractError):
        policy.verify_short_goal_launch_argv(argv)


@pytest.mark.parametrize('available,ok,warning',[
    (5*1024*1024+1,True,False),(5*1024*1024-10000,True,True),
    (4*1024*1024,True,True),(4*1024*1024-1,True,True)])
def test_user_scoped_pc_memory_policy(available,ok,warning):
    snapshot=dict(ok=False,reasons=['MemAvailable_below_5GiB'],MemAvailable_kib=available)
    decision=policy.runtime_memory_decision(snapshot,timing_profile='historical-motion',no_motion_contract=False)
    assert decision['ok'] is ok and decision['soft_warning'] is warning
    assert decision['hard_stop_threshold_kib'] is None
    assert policy.runtime_memory_decision(snapshot,timing_profile='standard',no_motion_contract=False)['ok'] is False


def test_historical_memory_keeps_emergency_rejections():
    for reason in ('memory_full_psi_high','SwapUsed_abnormal','cgroup_headroom_insufficient'):
        snapshot=dict(ok=False,reasons=['MemAvailable_below_5GiB',reason],MemAvailable_kib=5000000)
        assert policy.runtime_memory_decision(snapshot,timing_profile='historical-motion',no_motion_contract=False)['ok'] is False
    snapshot=dict(ok=True,reasons=[],MemAvailable_kib=6000000,oom_kill_delta=1)
    assert policy.runtime_memory_decision(snapshot,timing_profile='historical-motion',no_motion_contract=False)['ok'] is False


def test_historical_timing_has_same_bounded_cleanup_reserve_as_reviewed_motion_timer():
    env=dict(os.environ,PYTHONPATH=str(HERE),CLEANNAV_RVIZ_TIMING_PROFILE='historical-motion')
    result=json.loads(subprocess.check_output([sys.executable,'-c',
        'import json,supervisor_policy as p;print(json.dumps(p.timing_identity()))'],env=env,text=True))
    assert result['preflight_latest_launch_s']==55
    assert result['gui_goal_wait_s']==300
    assert result['readiness_latest_t0_s']==205
    assert result['historical_task_timeout_s']==120
    assert result['soft_stop_t0_s']==1045
    assert result['hard_process_cutoff_t0_s']==1075
    assert result['evidence_cutoff_t0_s']==1080
    assert result['max_launch_runtime_s']==1045


@pytest.mark.parametrize('value', ['600', '3600'])
def test_configured_gui_wait_fits_latest_readiness_and_task(value):
    env=dict(os.environ, PYTHONPATH=str(HERE), CLEANNAV_RVIZ_TIMING_PROFILE='historical-motion',
             CLEANNAV_RVIZ_GUI_GOAL_WAIT_S=value)
    identity=json.loads(subprocess.check_output([sys.executable, '-c',
        'import json,supervisor_policy as p;print(json.dumps(p.timing_identity()))'],env=env,text=True))
    wait=float(value)
    assert identity['gui_goal_wait_s']==wait
    assert identity['historical_task_timeout_s']==120
    assert identity['soft_stop_t0_s']==205+wait+530+10
    assert identity['max_launch_runtime_s']==identity['soft_stop_t0_s']
    assert identity['evidence_cutoff_t0_s']-identity['soft_stop_t0_s']==35
    assert identity['sigterm_t0_s']-identity['soft_stop_t0_s']==14
    assert identity['sigkill_t0_s']-identity['soft_stop_t0_s']==22


def test_historical_task_timeout_does_not_change_retired_short_goal_budget():
    historical=dict(os.environ,PYTHONPATH=str(HERE),CLEANNAV_RVIZ_TIMING_PROFILE='historical-motion')
    retired=dict(os.environ,PYTHONPATH=str(HERE),CLEANNAV_RVIZ_TIMING_PROFILE='short-goal-motion')
    historical_identity=json.loads(subprocess.check_output([sys.executable,'-c',
        'import json,supervisor_policy as p;print(json.dumps(p.timing_identity()))'],
        env=historical,text=True))
    retired_identity=json.loads(subprocess.check_output([sys.executable,'-c',
        'import json,supervisor_policy as p;print(json.dumps(p.timing_identity()))'],
        env=retired,text=True))
    assert historical_identity['historical_task_timeout_s']==120
    assert historical_identity['max_launch_runtime_s']==1045
    assert retired_identity['historical_task_timeout_s'] is None
    assert retired_identity['max_launch_runtime_s']==250


@pytest.mark.parametrize('value', ['', 'no', 'nan', 'inf', '-1', '0', '.5', '3601'])
def test_invalid_gui_wait_is_rejected(value):
    env=dict(os.environ, PYTHONPATH=str(HERE), CLEANNAV_RVIZ_GUI_GOAL_WAIT_S=value)
    r=subprocess.run([sys.executable,'-c','import supervisor_policy'],env=env,capture_output=True,text=True)
    assert r.returncode!=0
    assert 'must be finite seconds in [1, 3600]' in r.stderr


def _gui_scope(now=10., ready=True, started=None, clicks=()):
    # Execute the actual existing historical branch with a fake clock/observer;
    # no ROS, process launch, authorization, publisher, or sleep is used.
    import ast
    from types import SimpleNamespace as N
    tree=ast.parse((HERE/'navigation_supervisor.py').read_text())
    block=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and isinstance(n.test,ast.Name) and n.test.id=='historical_motion'
        and n.body and isinstance(n.body[0],ast.If)
        and any(isinstance(b,ast.If) and 'historical_gui_ready_at is None' in ast.unparse(b.test) for b in n.body))
    nodes=[]
    for n in block.body:
        if isinstance(n,ast.If) and ast.unparse(n.test)=='motion_terminal_payload':
            break
        nodes.append(n)
    events=[]; stops=[]; notices=[]
    announced={'ready': started is not None}
    def announce_navigation_ready():
        if announced['ready']:
            return
        announced['ready']=True
        notices.append(policy.format_navigation_ready(
            run_id='rviz-nav-test', odometry_mode='oracle', motion_profile='historical',
            map_profile='simple-four-wall-nohit-v1-oracle-short-goal'))
    def announce_navigation_not_ready(stage, reason):
        if announced['ready']:
            return
        notices.append(policy.format_navigation_not_ready(stage=stage, reason=reason))
    ledger=policy.PhaseLedger()
    observer=N(ground_truth_samples=[],motion_events=list(clicks),safety_owner_history=[],
        stop_publisher=N(publish=lambda msg:stops.append('estop')),
        cancel_client=N(service_is_ready=lambda:True,call_async=lambda req:stops.append('cancel')),
        safety_state={'owner':''})
    scope=dict(time=N(monotonic=lambda:now,monotonic_ns=lambda:int(now*1e9)),
        historical_gui_ready_at=started,all_phases=ready,historical_initial_truth=None,
        startup_verification_complete=started is not None, motion_admission_wait_started=None,
        read_json=lambda p:{},run_dir=Path("/unused"),atomic_json=lambda *a:None,guard=None,
        MAX_HISTORICAL_DISPLACEMENT_M=policy.MAX_HISTORICAL_DISPLACEMENT_M,
        HISTORICAL_TASK_TIMEOUT_S=policy.HISTORICAL_TASK_TIMEOUT_S,
        HISTORICAL_GUI_GOAL_WAIT_S=300.,result={} if started is None else {'historical_gui_wait':{'status':'WAITING'}},
        elapsed=lambda:now,observer=observer,event=lambda kind,**kw:events.append((kind,kw)),
        motion_event_index=0,motion_click_command_id=None,motion_goal_seen_at=None,
        motion_task_deadline=None,motion_task_id=None,motion_terminal_payload=None,
        historical_lease_seen=False,failures=[],any_motion=True,historical_motion=True,
        motion_interlock_open=False,motion_interlock_stage=None,PHASES=policy.PHASES,
        ledger=ledger,ContractError=policy.ContractError,Bool=lambda **kw:kw,
        CancelGoal=N(Request=lambda:None),notices=notices,
        announce_navigation_ready=announce_navigation_ready,
        announce_navigation_not_ready=announce_navigation_not_ready)
    fail=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='fail')
    exec(compile(ast.Module(body=[fail],type_ignores=[]),'<actual-fail>','exec'),scope)
    return scope,events,stops,compile(ast.Module(body=nodes,type_ignores=[]),'<actual-gui>','exec')


def test_wait_starts_only_after_all_existing_readiness_phases():
    scope,events,stops,code=_gui_scope(ready=False)
    exec(code,scope)
    assert scope['historical_gui_ready_at'] is None and not events and not stops
    scope['all_phases']=True
    exec(code,scope)
    assert scope['historical_gui_ready_at']==10.
    assert events[-1][0]=='historical_motion_ready_for_gui'
    assert events[-1][1]['goal_wait_limit_s']==300.


def test_received_goal_is_distinct_from_gui_timeout_and_keeps_task_limit():
    scope,events,stops,code=_gui_scope(now=301.,started=1.,clicks=[{'event':'CLICK','command_id':'one'}])
    exec(code,scope)
    assert scope['result']['historical_gui_wait']['status']=='RECEIVED'
    assert scope['motion_task_deadline']==421.
    assert events[-1][0]=='historical_gui_goal_received'
    assert events[-1][1]['task_limit_s']==120
    assert not stops


def test_historical_goal_task_deadline_is_bounded_at_120s_and_fail_closed():
    import ast
    tree=ast.parse((HERE/'navigation_supervisor.py').read_text())
    branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and ast.unparse(n.test).startswith('motion_task_deadline and'))
    code=compile(ast.Module(body=[branch],type_ignores=[]),'<actual-historical-task-deadline>','exec')
    scope,events,stops,gui_code=_gui_scope(now=100.,started=1.,
        clicks=[{'event':'CLICK','command_id':'one'}])
    exec(gui_code,scope)
    assert scope['motion_task_deadline']==220.
    scope['time'].monotonic=lambda:219.999
    exec(code,scope)
    assert not stops
    scope['time'].monotonic=lambda:220.001
    with pytest.raises(policy.ContractError,match='historical Mission task exceeded 120s'):
        exec(code,scope)
    assert scope['result']['failure_stage']=='mission_manager'
    assert stops==['estop','cancel']


def test_gui_timeout_uses_actual_existing_fail_closed_stop():
    scope,events,stops,code=_gui_scope(now=301.1,started=1.)
    with pytest.raises(policy.ContractError,match='GUI goal wait expired'):
        exec(code,scope)
    assert scope['result']['historical_gui_wait']['status']=='TIMED_OUT'
    assert scope['result']['failure_stage']=='rviz'
    assert stops==['estop','cancel']
    assert any(e[0]=='historical_gui_goal_wait_timeout' for e in events)
    assert scope['ledger'].failed_at=='rviz'


@pytest.mark.parametrize('wait,now', [(300.,121.), (300.,301.), (600.,451.)])
def test_valid_gui_wait_remains_open_beyond_old_sixty_second_limit(wait,now):
    scope,events,stops,code=_gui_scope(now=now,started=1.)
    scope['HISTORICAL_GUI_GOAL_WAIT_S']=wait
    exec(code,scope)
    assert scope['result']['historical_gui_wait']['status']=='WAITING'
    assert not events and not stops


@pytest.mark.parametrize('distance,stopped', [(2.01,False),(2.51,False),(8.,False),(11.99,False),(12.,True),(12.01,True)])
def test_actual_historical_guard_odometry_limit(distance, stopped):
    import ast
    from types import SimpleNamespace as N
    tree=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    block=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and 'MAX_HISTORICAL_ODOMETRY_M' in ast.unparse(n.test))
    failures=[]
    scope=dict(self=N(armed=True,distance=distance,fail=failures.append,route_budget=None,goal_distance_base=0.),short_goal=False,
        historical=True,MAX_HISTORICAL_ODOMETRY_M=policy.MAX_HISTORICAL_ODOMETRY_M)
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<actual-distance-stop>','exec'),scope)
    assert bool(failures) is stopped


@pytest.mark.parametrize('distance,stopped', [(2.51,False),(8.,False),(9.99,False),
    (10.,False),(10.99,False),(11.,True),(11.01,True)])
def test_actual_historical_truth_displacement_limit(distance, stopped):
    import ast
    tree=ast.parse((HERE/'navigation_supervisor.py').read_text())
    block=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and ast.unparse(n.test).startswith('displacement >='))
    failures=[]
    scope=dict(displacement=distance,budget=None,MAX_HISTORICAL_DISPLACEMENT_M=policy.MAX_HISTORICAL_DISPLACEMENT_M,
        fail=lambda reason,stage:failures.append((reason,stage)))
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<actual-truth-stop>','exec'),scope)
    assert bool(failures) is stopped


def _operator_guard(now=10., **facts):
    required = (
        "/scan", "/mapping/scan", "/scan_odom_tf", "/mapping/scan_odom_tf",
        "/mapping/scan_map_tf", "clock", "odom",
        "safety", "safety_no_estop", "cm", "cm_active", "cmd_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "no_motion_authority",
        "/cleannav/navigation/cmd_vel_raw_publisher", "localization",
    )
    values = {key: True for key in required}
    values.update(facts)
    return {"ready": True, "monotonic": now, "facts": values,
            "safety": {"block_all": True, "estop": False, "enabled": False, "owner": ""}}


def _operator_readiness(log, guard, **flags):
    values = dict(rviz_node=True, rviz_process=True, gazebo_process=True,
                  mission_node=True, now_monotonic=10.)
    values.update(flags)
    return policy.phase_readiness(
        log, {"passed": True, "decision": "PASS"}, guard, **values)


def _assert_branch_stays_silent():
    scope, events, stops, code = _gui_scope(ready=False)
    exec(code, scope)
    assert scope['notices'] == []
    assert scope['historical_gui_ready_at'] is None
    assert not events and not stops


def test_rviz_without_map_localization_does_not_print_ready():
    log = "[SIMULATION] READY\n[NAVIGATION] READY\n[MISSION] READY"
    readiness = _operator_readiness(log, _operator_guard())
    assert readiness['rviz'][0] is True
    assert readiness['map_localization'][0] is False
    assert not all(ok for ok, _evidence in readiness.values())
    _assert_branch_stays_silent()


def test_map_server_without_amcl_does_not_print_ready():
    log = ("[SIMULATION] READY\n[NAVIGATION] READY\n[MISSION] READY\n"
           "map_server lifecycle node launched\nCreating")
    readiness = _operator_readiness(log, _operator_guard(localization=False))
    assert readiness['map_localization'][0] is False
    assert not all(ok for ok, _evidence in readiness.values())
    _assert_branch_stays_silent()


def test_goal_adapter_not_ready_does_not_print_ready():
    log = "[SIMULATION] READY\n[LOCALIZATION] READY\n[NAVIGATION] READY"
    readiness = _operator_readiness(log, _operator_guard(), mission_node=False)
    assert readiness['map_localization'][0] is True
    assert readiness['mission_manager'][0] is False
    assert not all(ok for ok, _evidence in readiness.values())
    _assert_branch_stays_silent()


def test_all_existing_phases_print_ready_once():
    log = "\n".join(f"[{name}] READY" for name in
                    ("SIMULATION", "LOCALIZATION", "NAVIGATION", "MISSION"))
    readiness = _operator_readiness(log, _operator_guard())
    assert all(ok for ok, _evidence in readiness.values())
    scope, events, stops, code = _gui_scope(ready=True)
    exec(code, scope)
    assert len(scope['notices']) == 1
    text = scope['notices'][0]
    assert text.startswith("[CLEANNAV] NAVIGATION READY - YOU CAN SEND GOAL NOW\n")
    assert "run_id=rviz-nav-test" in text
    assert "odometry_mode=oracle" in text
    assert "motion_profile=historical" in text
    assert "map_profile=simple-four-wall-nohit-v1-oracle-short-goal" in text
    assert "Click 2D Goal Pose in RViz now." in text
    assert events[-1][0] == 'historical_motion_ready_for_gui'
    exec(code, scope)
    assert len(scope['notices']) == 1
    assert not stops


def test_ready_notice_leaves_goal_receipt_unchanged():
    scope, events, stops, code = _gui_scope(
        now=301., started=1., clicks=[{'event': 'CLICK', 'command_id': 'one'}])
    exec(code, scope)
    assert scope['notices'] == []
    assert scope['result']['historical_gui_wait']['status'] == 'RECEIVED'
    assert events[-1][0] == 'historical_gui_goal_received'
    assert not stops


def test_failure_before_ready_prints_not_ready_stage_and_reason():
    scope, events, stops, code = _gui_scope(ready=False)
    with pytest.raises(policy.ContractError, match='unknown: None'):
        scope['fail']('initial lifecycle state is unknown: None', 'map_localization')
    assert len(scope['notices']) == 1
    text = scope['notices'][0]
    assert text == (
        "[CLEANNAV] NOT READY\n"
        "[CLEANNAV] stage=map_localization\n"
        "[CLEANNAV] reason=initial lifecycle state is unknown: None\n")
    assert events[-1][0] == 'failure'
    assert stops == ['estop', 'cancel']


def test_operator_notice_flushes_the_given_stream():
    from io import StringIO
    stream = StringIO()
    text = policy.format_navigation_ready(
        run_id='rviz-nav-test', odometry_mode='oracle', motion_profile='historical',
        map_profile='simple-four-wall-nohit-v1-oracle-short-goal')
    supervisor.write_operator_notice(text, streams=[stream])
    assert stream.getvalue() == text
    assert stream.tell() == len(text)


def test_gui_recovery_uses_own_bound_and_does_not_fake_readiness():
    scope,events,stops,code=_gui_scope(now=10.,ready=False,started=1.)
    scope['motion_admission_wait_started']=9.
    exec(code,scope)
    assert not stops and not scope['all_phases']
    scope['all_phases']=True
    exec(code,scope)
    assert scope['motion_admission_wait_started'] is None
    scope['all_phases']=False
    scope['motion_admission_wait_started']=8.
    with pytest.raises(policy.ContractError,match='did not recover within 2s'):
        exec(code,scope)
    assert stops==['estop','cancel']
