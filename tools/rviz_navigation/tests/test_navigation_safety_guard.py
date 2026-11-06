"""Pure contract tests for the installed localization health helper call."""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from navigation_safety_guard import (
    COSTMAP_STAMP_LEAD_S, costmap_stamp_current,
    localization_bootstrap_ready, publish_shutdown_estop,
    readiness_transition_records,
)


def valid_health():
    return {
        "localized": True,
        "native_parameters_verified": True,
        "assets_unchanged": True,
        "sensors_fresh": True,
        "frozen_map": True,
        "map_server_active": True,
        "info": {"matches": 1},
    }


def test_localization_helper_receives_state_then_freshness_age():
    state = valid_health()
    calls = []

    def installed_contract(received_state, age):
        calls.append((received_state, age))
        return (0 <= age <= 1.5
                and all(received_state.get(key) is True for key in (
                    "localized", "native_parameters_verified", "assets_unchanged",
                    "sensors_fresh", "frozen_map", "map_server_active"))
                and received_state.get("info", {}).get("matches", 0) >= 1)

    assert localization_bootstrap_ready(installed_contract, state, 10.0, 11.0)
    assert calls == [(state, 1.0)]


@pytest.mark.parametrize("received_at,now", [(10.0, 12.0), (12.0, 10.0), (float("nan"), 12.0)])
def test_stale_future_or_nonfinite_health_timestamp_is_not_ready(received_at, now):
    calls = []
    result = localization_bootstrap_ready(
        lambda state, age: calls.append((state, age)) or 0 <= age <= 1.5,
        valid_health(), received_at, now)
    assert result is False
    if now < received_at or received_at != received_at:
        assert calls == []


@pytest.mark.parametrize("state", [
    None,
    [],
    {"info": []},
    {"info": {"matches": "one"}},
    {"info": {"matches": float("nan")}},
])
def test_malformed_health_payload_fails_closed_without_calling_contract(state):
    calls = []
    result = localization_bootstrap_ready(
        lambda *args: calls.append(args) or True, state, 10.0, 10.1)
    assert result is False
    assert calls == []


def test_health_helper_false_is_not_upgraded_to_ready():
    assert not localization_bootstrap_ready(
        lambda state, age: False, valid_health(), 10.0, 10.1)


def test_shutdown_skips_estop_publish_after_ros_context_is_invalid():
    calls = []
    assert publish_shutdown_estop(lambda: False, lambda: calls.append("publish")) is False
    assert calls == []


def test_shutdown_publishes_estop_while_context_is_valid():
    calls = []
    assert publish_shutdown_estop(lambda: True, lambda: calls.append("publish")) is True
    assert calls == ["publish"]


def test_shutdown_only_swallows_publish_error_if_context_closed():
    states = iter((True, False))
    assert publish_shutdown_estop(lambda: next(states),
                                  lambda: (_ for _ in ()).throw(RuntimeError("closed"))) is False

    states = iter((True, True))
    with pytest.raises(RuntimeError, match="closed"):
        publish_shutdown_estop(lambda: next(states),
                               lambda: (_ for _ in ()).throw(RuntimeError("closed")))


def test_readiness_true_to_false_transition_persists_run_time_age_and_reason():
    previous={'/scan':True}
    observations={'/scan':{
        'ready':False,'reason':'receive_age_exceeded','receive_age_s':.39561,
        'stamp_age_s':.081,'freshness_limit_s':.3,
    }}
    rows=readiness_transition_records('rviz-nav-test',previous,{'/scan':False},
        observations,monotonic_ns=1234,wall_time_ns=5678)
    assert rows==[{
        'run_id':'rviz-nav-test','stage':'no_motion_guard','monotonic_ns':1234,
        'wall_time_ns':5678,'check':'/scan','previous':True,'value':False,
        'reason':'receive_age_exceeded','receive_age_s':.39561,'stamp_age_s':.081,
        'freshness_limit_s':.3,'evidence':observations['/scan'],
    }]
    assert readiness_transition_records('rviz-nav-test',{'/scan':False},{'/scan':False},
        observations,monotonic_ns=2345,wall_time_ns=6789)==[]



def test_receipt_writer_bounded_history_sequence_and_actual_write_audit(tmp_path):
    import json,time
    from navigation_safety_guard import ReceiptWriter
    writer=ReceiptWriter(tmp_path/'safety-guard.json','test-run')
    state=dict(monotonic=time.monotonic(),failed=False,
               graph_history=[{'sample_index':i,'payload':'x'*1000} for i in range(1000)])
    first=writer.write(state);writer.flush();second=writer.write(state);writer.flush()
    assert len(state['graph_history'])==1000  # no mutation of caller evidence
    assert len(first['graph_history'])==5
    assert (first['receipt_sequence'],second['receipt_sequence'])==(1,2)
    assert (tmp_path/'safety-guard.json').stat().st_size<8000
    writes=[json.loads(s) for s in (tmp_path/'safety-receipt-writes.jsonl').read_text().splitlines()]
    for x in writes:
        assert x['write_started_monotonic_ns']<=x['serialization_finished_monotonic_ns']
        assert x['serialization_finished_monotonic_ns']<=x['replace_started_monotonic_ns']
        assert x['replace_started_monotonic_ns']<=x['replace_finished_monotonic_ns']
        assert x['replace_finished_monotonic_ns']<=x['history_finished_monotonic_ns']
    assert [x['receipt_sequence'] for x in writes]==[1,2]
    writer.close()


def _sync_guard(monkeypatch):
    import ast
    import types
    import guard_policy
    source=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    cls=next(x for x in ast.walk(source) if isinstance(x,ast.ClassDef) and x.name=='Guard')
    clock=types.SimpleNamespace(value=200_000_000,monotonic_ns=lambda:clock.value,
                                monotonic=lambda:clock.value/1e9)
    scope=dict(time=clock,historical=True,patrol=False,short_goal=False,motion=True,
               EXPECTED_EXECUTION_ID='ros-execution-1',command_error=guard_policy.command_error,
               args=types.SimpleNamespace(diagnostic_continue_on_readiness_loss=False,run_id="unit-sync",motion_profile="historical"),json=__import__('json'))
    methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('on_command','on_safety','tick')]
    exec(compile(ast.Module(body=methods,type_ignores=[]),'<actual-guard-methods>','exec'),scope)
    g=types.SimpleNamespace(expected_owner='ros-execution-1',safety=dict(block_all=False,estop=False,enabled=False,owner='',monotonic_ns=50_000_000),
        goal_received_ns=100_000_000,safety_sync_deadline_ns=None,rviz_goal_seen=True,
        mission_goal_accepted=True,mission_terminal=False,last_state={},
        command_samples=__import__('collections').defaultdict(int),command_nonzero=__import__('collections').defaultdict(int),
        command_last={},mark=lambda _:None,failed=None)
    g.receipt_writer=types.SimpleNamespace(safety_sequence=0,timing=lambda *a,**k:None)
    g.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=clock.value))
    g.fail=lambda why:setattr(g,'failed',why)
    g.get_logger=lambda:types.SimpleNamespace(info=lambda _:None)
    for name in ('on_command','on_safety','tick'):setattr(g,name,types.MethodType(scope[name],g))
    return g,clock


def _sync_twist(x):
    from types import SimpleNamespace as N
    return N(linear=N(x=x,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))


@pytest.mark.parametrize("speed", [.05,.1935612,.5])
def test_prelease_raw_waits_zero_final_then_matching_state_resumes(monkeypatch,speed):
    from types import SimpleNamespace as N
    g,clock=_sync_guard(monkeypatch)
    g.on_command(_sync_twist(speed),'/cleannav/navigation/cmd_vel_raw')
    assert g.failed is None and g.safety_sync_deadline_ns==700_000_000
    g.on_command(_sync_twist(0.),'/cmd_vel')
    assert g.failed is None and g.command_nonzero['/cmd_vel']==0
    clock.value=250_000_000
    g.on_safety(N(header=N(stamp=N(sec=1,nanosec=0)),block_all=False,emergency_stop_active=False,autonomous_enabled=True,
                  lease_owner_execution_id='ros-execution-1'))
    assert g.safety_sync_deadline_ns is None
    g.on_command(_sync_twist(speed),'/cmd_vel')
    assert g.failed is None


def test_prelease_wait_is_bounded(monkeypatch):
    g,clock=_sync_guard(monkeypatch)
    g.on_command(_sync_twist(.05),'/cleannav/navigation/cmd_vel_raw')
    clock.value=700_000_000;g.tick()
    assert 'synchronization timeout' in g.failed


@pytest.mark.parametrize('change',[
    {'owner':'wrong-owner','enabled':True},
    {'owner':'ros-execution-1','enabled':True}, # pre-goal, old Lease status
    {'estop':True},
    {'monotonic_ns':-1_000_000_000,'owner':'ros-execution-1','enabled':True},
])
def test_invalid_lease_state_is_never_waited_or_admitted(monkeypatch,change):
    g,_=_sync_guard(monkeypatch);g.safety.update(change)
    g.on_command(_sync_twist(.05),'/cleannav/navigation/cmd_vel_raw')
    assert g.failed and g.safety_sync_deadline_ns is None


def test_nonzero_final_during_prelease_wait_still_rejected(monkeypatch):
    g,_=_sync_guard(monkeypatch)
    g.on_command(_sync_twist(.05),'/cleannav/navigation/cmd_vel_raw')
    g.on_command(_sync_twist(.05),'/cmd_vel')
    assert g.failed


def test_matching_state_after_deadline_cannot_resume(monkeypatch):
    from types import SimpleNamespace as N
    g,clock=_sync_guard(monkeypatch)
    g.on_command(_sync_twist(.05),'/cleannav/navigation/cmd_vel_raw')
    clock.value=700_000_000
    g.on_safety(N(header=N(stamp=N(sec=1,nanosec=0)),block_all=False,emergency_stop_active=False,autonomous_enabled=True,
                  lease_owner_execution_id='ros-execution-1'))
    assert 'synchronization timeout' in g.failed


def test_idle_status_received_after_click_still_waits_for_real_lease(monkeypatch):
    g,_=_sync_guard(monkeypatch);g.safety['monotonic_ns']=150_000_000
    g.on_command(_sync_twist(.05),'/cleannav/navigation/cmd_vel_raw')
    assert g.failed is None and g.safety_sync_deadline_ns==700_000_000


def test_timing_records_source_receipt_and_check_without_changing_policy(tmp_path):
    import json
    import time
    from navigation_safety_guard import ReceiptWriter
    writer = ReceiptWriter(tmp_path / 'safety-guard.json', 'timing-test')
    writer.safety_sequence = 7
    state = dict(monotonic_ns=time.monotonic_ns()-505_000_000, stamp_ns=123,
                 owner='ros-execution-1', enabled=True, estop=False)
    writer.timing('command_checked', state, error='Safety state stale or future dated')
    item = json.loads((tmp_path/'safety-timing.jsonl').read_text())
    assert item['receiver_sequence'] == 7
    assert item['safety']['stamp_ns'] == 123
    assert item['age_ns'] >= 505_000_000
    assert item['error'] == 'Safety state stale or future dated'
    assert item['check_monotonic_ns'] > state['monotonic_ns']


@pytest.mark.parametrize('vx,wz,mode,rejected', [
    (.1935612,.0367766,'historical',False),
    (.5,.5/1.12,'historical',False),
    (.5001,0.,'historical',True),
    (0.,.5/1.12+.0001,'historical',True),
    (.1935612,.0367766,'short-goal',True),
    (.15,.13392857142857142,'short-goal',False),
])
def test_command_limits_follow_existing_mode_profile(vx,wz,mode,rejected):
    from guard_policy import command_error
    safety=dict(block_all=False,estop=False,enabled=True,owner='ros-execution-1',monotonic_ns=100)
    error=command_error([vx,0,0,0,0,wz],True,safety,
        motion_profile=mode,expected_owner='ros-execution-1',now_ns=200)
    assert bool(error) is rejected


@pytest.mark.parametrize('change', ['owner','estop','enabled','block_all','stale','reverse','nan','nonplanar'])
def test_historical_profile_preserves_command_safety_rejections(change):
    from guard_policy import command_error
    safety=dict(block_all=False,estop=False,enabled=True,owner='ros-execution-1',monotonic_ns=100)
    values=[.1935612,0.,0.,0.,0.,.0367766];now=200
    if change=='owner':safety['owner']='old-lease'
    elif change=='estop':safety['estop']=True
    elif change=='enabled':safety['enabled']=False
    elif change=='block_all':safety['block_all']=True
    elif change=='stale':now=500_000_101
    elif change=='reverse':values[0]=-.01
    elif change=='nan':values[0]=float('nan')
    elif change=='nonplanar':values[1]=.01
    assert command_error(values,True,safety,motion_profile='historical',
        expected_owner='ros-execution-1',now_ns=now)


def test_guard_supervisor_and_pending_commands_supply_the_same_mode():
    import ast
    for filename in ('navigation_safety_guard.py','navigation_supervisor.py'):
        tree=ast.parse((HERE/filename).read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
               and isinstance(n.func,ast.Name) and n.func.id=='command_error']
        assert len(calls)==2
        for call in calls:
            mode=next(k.value for k in call.keywords if k.arg=='motion_profile')
            assert ast.unparse(mode)=='args.motion_profile'


def test_costmap_stamp_accepts_one_clock_step_lead_and_keeps_stale_bound():
    # Round 1 rviz-nav-20261030T154528Z-76d5825d: clock 72.9 s, global costmap
    # stamp 73.0 s. The 3.0 s stale bound was not exceeded.
    clock_ns = 72_900_000_000
    assert costmap_stamp_current(clock_ns, 73_000_000_000, 3.0)
    assert costmap_stamp_current(clock_ns, 71_000_000_000, 3.0)
    assert costmap_stamp_current(clock_ns, clock_ns, 3.0)
    assert costmap_stamp_current(clock_ns, clock_ns + int(COSTMAP_STAMP_LEAD_S * 1e9), 3.0)
    assert costmap_stamp_current(clock_ns, clock_ns - int(3.0 * 1e9), 3.0)
    assert not costmap_stamp_current(clock_ns, clock_ns + int(COSTMAP_STAMP_LEAD_S * 1e9) + 1, 3.0)
    assert not costmap_stamp_current(clock_ns, clock_ns - int(3.0 * 1e9) - 1, 3.0)
    assert not costmap_stamp_current(clock_ns, 69_000_000_000, 3.0)
    assert not costmap_stamp_current(True, 73_000_000_000, 3.0)
    assert not costmap_stamp_current(clock_ns, True, 3.0)
    assert not costmap_stamp_current(None, 73_000_000_000, 3.0)
    assert not costmap_stamp_current(clock_ns, None, 3.0)


def _replace_original():
    return Path.replace


def test_blocked_replace_does_not_stall_scan_updates(tmp_path, monkeypatch):
    import json
    import threading
    import time
    from navigation_safety_guard import ReceiptWriter
    from readiness_inputs import InputEvidence
    original = _replace_original()
    entered = threading.Event()
    calls = {'n': 0}

    def slow(self, target):
        calls['n'] += 1
        if calls['n'] == 1:
            entered.set()
            time.sleep(1.25)
        return original(self, target)

    monkeypatch.setattr(Path, 'replace', slow)
    writer = ReceiptWriter(tmp_path / 'safety-guard.json', 'slow-replace')
    try:
        writer.write(dict(monotonic=time.monotonic(), failed=False, ready=True))
        assert entered.wait(1)
        started = time.monotonic()
        latest = writer.write(dict(monotonic=time.monotonic(), failed=False, ready=True,
                                    marker='latest'))
        assert time.monotonic() - started < 0.05
        inputs = InputEvidence()
        base = time.monotonic()
        last = base + 7 * 0.125
        inputs.observe('odom', 10_000_000_000, base, frame='odom')
        inputs.observe('/mapping/scan', 10_000_000_000, base, frame='mapping_lidar_link')
        for index in range(8):
            inputs.observe('/scan', 10_000_000_000 + index * 125_000_000,
                           base + index * 0.125, frame='base_scan')
        inputs.observe('clock', 10_000_000_000 + 7 * 125_000_000, last)
        now = last + 0.05
        facts, observations = inputs.snapshot(now, lambda *args: {'ok': True})
        assert facts['/scan'] is True
        assert observations['/scan']['freshness_limit_s'] == .3
        assert InputEvidence.LIMITS['clock'] == 1.
        writer.flush(timeout=3)
        saved = json.loads((tmp_path / 'safety-guard.json').read_text())
        assert saved['receipt_sequence'] == latest['receipt_sequence']
        assert saved['marker'] == 'latest'
    finally:
        writer.close()


def test_scan_silence_beyond_point_three_seconds_is_still_missing(tmp_path):
    from guard_policy import missing_inputs
    from readiness_inputs import InputEvidence
    inputs = InputEvidence()
    now = 1.0
    inputs.observe('clock', 10_000_000_000, now)
    inputs.observe('odom', 10_000_000_000, now, frame='odom')
    inputs.observe('/scan', 10_000_000_000, now, frame='base_scan')
    inputs.observe('/mapping/scan', 10_000_000_000, now, frame='mapping_lidar_link')
    facts, observations = inputs.snapshot(now + 0.301, lambda *args: {'ok': True})
    assert facts['/scan'] is False
    assert observations['/scan']['reason'] == 'receive_age_exceeded'
    assert observations['/scan']['freshness_limit_s'] == .3
    assert '/scan' in missing_inputs(facts)


def test_receipt_writer_keeps_failure_and_closes_with_latest_snapshot(tmp_path, monkeypatch):
    import json
    import threading
    from navigation_safety_guard import ReceiptWriter
    original = _replace_original()
    started = threading.Event()
    release = threading.Event()

    def blocked_once(self, target):
        if not started.is_set():
            started.set()
            assert release.wait(2)
        return original(self, target)

    monkeypatch.setattr(Path, 'replace', blocked_once)
    writer = ReceiptWriter(tmp_path / 'safety-guard.json', 'close-latest')
    try:
        writer.write(dict(monotonic=1.0, failed=False, ready=True))
        assert started.wait(1)
        writer.write(dict(monotonic=2.0, failed='lost safety admission: /scan', ready=False))
        latest = writer.write(dict(monotonic=3.0, failed=False, ready=True))
        release.set()
        writer.flush()
        history = [json.loads(line) for line in
                   (tmp_path / 'safety-input-observations.jsonl').read_text().splitlines()]
        assert any(item['failed'] == 'lost safety admission: /scan' for item in history)
        saved = json.loads((tmp_path / 'safety-guard.json').read_text())
        assert saved['receipt_sequence'] == latest['receipt_sequence']
        assert saved['monotonic'] == 3.0
    finally:
        writer.close()


def test_receipt_replace_is_single_threaded(tmp_path, monkeypatch):
    import threading
    import time
    from navigation_safety_guard import ReceiptWriter
    original = _replace_original()
    depth = {'now': 0, 'max': 0}
    gate = threading.Lock()

    def tracked(self, target):
        with gate:
            depth['now'] += 1
            depth['max'] = max(depth['max'], depth['now'])
        try:
            time.sleep(0.02)
            return original(self, target)
        finally:
            with gate:
                depth['now'] -= 1

    monkeypatch.setattr(Path, 'replace', tracked)
    writer = ReceiptWriter(tmp_path / 'safety-guard.json', 'one-writer')
    try:
        threads = []
        for index in range(6):
            threads.append(threading.Thread(
                target=lambda i=index: writer.write(dict(monotonic=float(i), failed=False, ready=True))))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        writer.flush()
        assert depth['max'] == 1
        assert depth['now'] == 0
    finally:
        writer.close()


def test_receipt_write_error_is_recorded_and_not_reported_as_saved(tmp_path, monkeypatch, capsys):
    from navigation_safety_guard import ReceiptWriteError, ReceiptWriter

    def broken(self, target):
        raise OSError('disk full')

    monkeypatch.setattr(Path, 'replace', broken)
    writer = ReceiptWriter(tmp_path / 'safety-guard.json', 'disk-error')
    writer.write(dict(monotonic=1.0, failed=False, ready=True))
    with pytest.raises(ReceiptWriteError, match='disk full'):
        writer.flush()
    with pytest.raises(ReceiptWriteError, match='disk full'):
        writer.write(dict(monotonic=2.0, failed=False, ready=True))
    assert 'disk full' in capsys.readouterr().err
    error = (tmp_path / 'safety-receipt-error.json').read_text()
    assert 'disk full' in error
    assert not (tmp_path / 'safety-guard.json').exists()
    with pytest.raises(ReceiptWriteError, match='disk full'):
        writer.close()
