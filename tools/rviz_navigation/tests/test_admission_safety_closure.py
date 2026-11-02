"""Actual methods and explicitly scheduled futures; no nodes, timers or sleep."""
import ast
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace as S

import pytest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import integration
import short_goal_policy as policy
from supervisor_policy import ContractError

HERE=Path(__file__).resolve().parents[1]
ROOT=integration.WORKSPACE
SRC=ROOT/'cleannav-experiments/motion-admission-safety-closure-20261031/src'
MM=SRC/'cleannav-mission-manager'
PKG=MM/'cleannav_mission_manager'
SAFETY=SRC/'cleannav_safety_supervisor/cleannav_safety_supervisor/safety_supervisor_node.py'


@pytest.fixture
def admitted(tmp_path,monkeypatch):
    """Synthetic, fixed-name JSON fixtures outside every operational runs tree."""
    run='rviz-nav-static-closure-fixture';now=100_100_000_000
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_ID',run)
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_DIR',str(tmp_path))
    monkeypatch.setenv('CLEANNAV_RVIZ_MOTION_PROFILE','short-goal')
    monkeypatch.setattr(policy,'time',S(monotonic_ns=lambda:now))
    import deadline_watchdog
    monkeypatch.setattr(deadline_watchdog,'process_identity',lambda pid:dict(start_ticks=2,state='S'))
    auth,scope,_=policy.issue_motion_receipts(run_id=run,run_dir=tmp_path,
        issued_monotonic_ns=100_000_000_000,expires_monotonic_ns=185_000_000_000,
        source_manifest_sha256=policy.sha256(HERE/'SOURCE_MANIFEST.json'),
        map_contract_sha256='b'*64,world_sha256='c'*64)
    safety=dict(block_all=False,estop=False,enabled=False,owner='',
        stamp_ns=20,generation_stamp_ns=10,monotonic_ns=100_000_000_000)
    facts={k:True for k in ('/scan','/mapping/scan','/scan_tf','/mapping/scan_tf','clock',
        'odom','safety','cmd','cm','safety_no_estop','cm_active','cmd_publisher',
        '/cleannav/navigation/cmd_vel_raw_publisher','/cleannav/cmd_vel_candidate_publisher',
        '/cmd_vel_authorized_publisher','localization')}
    guard=dict(run_id=run,writer_pid=44,monotonic=100.,safety=safety,
        facts=facts,ready=True,missing=[],distance_m=0.)
    proof=dict(schema_version=1,run_id=run,profile_id=policy.PROFILE_ID,status='ADMITTED',
        authorization_sha256=policy.sha256(tmp_path/'motion-authorization.json'),
        scope_sha256=policy.sha256(tmp_path/'motion-scope.json'),
        interlock_response_success=True,safety_unblocked_unleased=True,all_phases_passed=True,
        goal_seen_before_admission=False,issued_monotonic_ns=auth['issued_monotonic_ns'],
        expires_monotonic_ns=auth['expires_monotonic_ns'],guard_path=str(tmp_path/'guard.json'),
        guard_writer_pid=44,safety_status=safety)
    def write(name,data):
        (tmp_path/name).write_text(json.dumps(data))
    write('guard.json',guard);write('motion-admission.json',proof)
    write('supervisor_state.json',dict(run_id=run,supervisor_pid=1,supervisor_start_ticks=2))
    return S(root=tmp_path,proof=proof,guard=guard,write=write,run=run,monkeypatch=monkeypatch)


def test_confirmed_receipt_accepts_original_generation_while_heartbeat_advances(admitted):
    assert policy.validate_runtime_admission()['status']=='ADMITTED'


@pytest.mark.parametrize('failure',['opening','revoked','stale','generation','run','writer','expired','lifecycle','supervisor'])
def test_proof_rejects_missing_or_changed_authority(admitted,failure):
    p=admitted
    if failure in ('opening','revoked'):
        p.proof['status']=failure.upper();p.write('motion-admission.json',p.proof)
    elif failure=='stale':p.guard['monotonic']=99.;p.write('guard.json',p.guard)
    elif failure=='generation':
        p.guard['safety']['generation_stamp_ns']=15;p.write('guard.json',p.guard)
    elif failure=='writer':p.guard['writer_pid']=45;p.write('guard.json',p.guard)
    elif failure=='run':p.proof['run_id']='wrong';p.write('motion-admission.json',p.proof)
    elif failure=='expired':p.monkeypatch.setattr(policy,'time',S(monotonic_ns=lambda:186_000_000_000))
    elif failure=='lifecycle':
        value=json.loads((p.root/'motion-lifecycle.json').read_text());value['status']='CLOSED';p.write('motion-lifecycle.json',value)
    else:
        import deadline_watchdog
        p.monkeypatch.setattr(deadline_watchdog,'process_identity',lambda pid:None)
    with pytest.raises(ValueError):policy.validate_runtime_admission()


def load_test_module(path,name):
    import importlib.util
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def nav_adapter():
    doubles=load_test_module(MM/'test/unit/test_real_navigation_adapter.py','closure_nav_doubles')
    patched=load_test_module(PKG/'adapters/real_navigation.py','closure_nav_source')
    client=doubles.FakeActionClient();events=[]
    adapter=patched.RealNavigationAdapter(doubles.FakeNode(),events.append,action_client=client)
    return doubles,adapter,client,events


def test_interleaving_T1_T6_never_sends_goal_or_lease_before_confirmation(admitted):
    from cleannav_mission_manager.domain.generation_gate import GenerationHandle
    p=admitted;doubles,adapter,client,events=nav_adapter()
    safety_module=load_test_module(PKG/'adapters/real_safety.py','closure_safety_transport')
    sd=load_test_module(MM/'test/unit/test_real_safety_adapter.py','closure_safety_doubles')
    node=sd.FakeNode();lease=safety_module.RealSafetyAdapter(node,events.append)
    # T1 interlock open, T2 observed Safety heartbeat, but proof is OPENING.
    p.proof['status']='OPENING';p.write('motion-admission.json',p.proof)
    # T3 is a real transport call (also used by TaskCommand/TaskSequence).
    adapter.submit_goal(GenerationHandle('ros-execution-1',1),doubles._pose())
    assert client.sent_goals==[]  # T4 cannot occur.
    lease.acquire_lease('ros-execution-1')
    assert not lease._acquire_client.requests  # T5 cannot occur.
    # T6 confirmation; a fresh transport generation may now proceed normally.
    p.proof['status']='ADMITTED';p.write('motion-admission.json',p.proof)
    adapter.submit_goal(GenerationHandle('ros-execution-1',2),doubles._pose())
    assert len(client.sent_goals)==1
    lease.acquire_lease('ros-execution-1');assert len(lease._acquire_client.requests)==1


def test_delayed_action_server_rechecks_after_revocation(admitted):
    from cleannav_mission_manager.domain.generation_gate import GenerationHandle
    doubles,adapter,client,events=nav_adapter();client.ready=False
    adapter.submit_goal(GenerationHandle('ros-execution-1',1),doubles._pose())
    admitted.proof['status']='REVOKED';admitted.write('motion-admission.json',admitted.proof)
    client.ready=True
    for timer in adapter._node.timers:timer.fire()
    assert not client.sent_goals


def safety_methods():
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    tree=ast.parse(SAFETY.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_acquire_lease_cb','_estop_cb')]
    scope=dict(time=time,os=os,Twist=Twist,Bool=Bool)
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(SAFETY),'exec'),scope)
    return scope


def test_safety_service_checks_proof_at_grant_not_only_at_request_send(admitted):
    scope=safety_methods();node=S(_localization_ok=lambda:True,
        _valid_execution_id=lambda x:bool(x),_control_mode='autonomous',_emergency_stop=False,
        _block_all=False,_lease_owner_execution_id=None,_invalidate_candidate=lambda:None,
        _status_timer_cb=lambda:None,get_logger=lambda:S(info=lambda m:None))
    admitted.proof['status']='OPENING';admitted.write('motion-admission.json',admitted.proof)
    response=scope['_acquire_lease_cb'](node,S(execution_id='ros-execution-1'),S())
    assert not response.success and node._lease_owner_execution_id is None
    admitted.proof['status']='ADMITTED';admitted.write('motion-admission.json',admitted.proof)
    assert scope['_acquire_lease_cb'](node,S(execution_id='ros-execution-1'),S()).success


def test_lease_failure_cancels_already_accepted_navigation(admitted):
    from cleannav_mission_manager.domain.state_machine import (
        StateMachineContext,StateMachineEvent as E,TransitionEffect as F,transition)
    from cleannav_mission_manager.domain.models import InternalExecutionState as I
    from cleannav_mission_manager.domain.generation_gate import GenerationHandle
    doubles,adapter,client,events=nav_adapter();handle=GenerationHandle('ros-execution-1',1)
    adapter.submit_goal(handle,doubles._pose());goal=doubles.FakeClientGoalHandle()
    doubles._accept(client,goal)
    context=transition(StateMachineContext(state=I.PREPARING_GOAL),E.GOAL_PREPARED).next_context
    context=transition(context,E.NAV_GOAL_ACCEPTED).next_context
    decision=transition(context,E.LEASE_ACQUIRE_FAILED)
    assert F.CANCEL_NAVIGATION in decision.effects
    adapter.cancel_goal(handle);assert goal.cancel_calls==1
    goal.cancel_future.set_result(doubles._cancel_response(goal))
    # This is a controlled transport ACK, not a claim about a live Nav2 server.
    assert goal.cancel_future.done()


def test_mission_sigkill_guard_reuses_existing_estop_and_cancel(admitted):
    from std_msgs.msg import Bool
    from action_msgs.srv import CancelGoal
    tree=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Guard')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('tick','fail')]
    scope=dict(short_goal=True,time=time,args=S(run_id=admitted.run),Bool=Bool,CancelGoal=CancelGoal)
    exec(compile(ast.Module(body=methods,type_ignores=[]),'actual Guard methods','exec'),scope)
    safety=S(_emergency_stop=False,_autonomous_enabled=True,_autonomous_start_time=None,
        _lease_owner_execution_id='ros-execution-1',_invalidate_candidate=lambda:None,
        _cmd_vel_pub=S(publish=lambda msg:None))
    stop=safety_methods()['_estop_cb']
    cancelled=[];guard=S(safety={'owner':'ros-execution-1'},mission_runtime={},
        fresh=lambda name,age:False,failed=None,last_state={},distance=0.,
        estop=S(publish=lambda m:stop(safety,m)),cancel=S(service_is_ready=lambda:True,
            call_async=cancelled.append),write_state=lambda state:None)
    guard.fail=scope['fail'].__get__(guard)
    scope['tick'](guard)
    assert guard.failed and len(cancelled)==1
    assert safety._emergency_stop and not safety._autonomous_enabled
    assert safety._lease_owner_execution_id is None


def test_legacy_wheel_transport_does_not_consume_scoped_proof(monkeypatch):
    monkeypatch.delenv('CLEANNAV_RVIZ_MOTION_PROFILE',raising=False)
    from cleannav_mission_manager.domain.generation_gate import GenerationHandle
    doubles,adapter,client,_=nav_adapter()
    adapter.submit_goal(GenerationHandle('legacy-execution',1),doubles._pose())
    assert len(client.sent_goals)==1


def test_mission_service_requires_confirmed_proof_even_when_readiness_is_true(admitted):
    module=load_test_module(PKG/'navigation_goal_service.py','closure_mission_service')
    gate=object.__new__(module.NavigationGoalService)
    gate.motion_enabled=True;gate.safety=S(emergency_stop_active=False,block_all=False)
    gate.safety_time=time.monotonic();gate.readiness_time=time.monotonic()
    gate.readiness={'ready':True,'motion_enabled':True}
    clients=S(service_is_ready=lambda:True)
    gate.node=S(runtime=S(navigation=S(_action_client=S(server_is_ready=lambda:True)),
        safety=S(_acquire_client=clients,_release_client=clients)))
    admitted.proof['status']='OPENING';admitted.write('motion-admission.json',admitted.proof)
    assert gate.admission() is not None
    admitted.proof['status']='ADMITTED';admitted.write('motion-admission.json',admitted.proof)
    assert gate.admission() is None


def test_formal_package_verifier_rejects_old_installs_for_short_goal(admitted):
    from ament_index_python.packages import get_package_prefix
    with pytest.raises(ValueError,match='requires the bound cleannav_mission_manager installation'):
        integration.verify_packages(get_package_prefix)
