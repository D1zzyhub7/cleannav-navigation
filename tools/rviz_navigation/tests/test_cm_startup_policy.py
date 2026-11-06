"""Pure fake-client tests: do not import rclpy or start any ROS process."""
from types import SimpleNamespace
from concurrent.futures import Future
import pytest
from cm_state_poll import CMStatePoll
from guard_policy import motion_admission, MOTION_REQUIRED_FACTS, diagnostic_continuation_decision

class Client:
    available = False
    def __init__(self): self.calls=[];self.removed=[]
    def service_is_ready(self): return self.available
    def call_async(self, request):
        f=Future();self.calls.append(f);return f
    def remove_pending_request(self,f):self.removed.append(f)

def response(f,state):f.set_result(SimpleNamespace(current_state=SimpleNamespace(id=state)))

def test_discovery_delayed_and_real_fresh_active_then_inactive_and_unavailable():
    c=Client();p=CMStatePoll()
    assert p.step(c,object,0)['state_id'] is None
    assert not c.calls
    c.available=True
    assert p.step(c,object,10)['events'][-1]['event']=='dispatch'
    response(c.calls[-1],3)
    s=p.step(c,object,10.1)
    assert s['active'] and s['response_monotonic']==10.1
    p.step(c,object,10.4);response(c.calls[-1],2)
    assert not p.step(c,object,10.5)['active']
    p.step(c,object,10.8);response(c.calls[-1],3)
    assert p.step(c,object,10.9)['active']
    c.available=False
    assert not p.step(c,object,11)['active']
    assert p.state_id is None

def test_pending_bounded_cancel_retry_success_and_stale_not_latched():
    c=Client();c.available=True;p=CMStatePoll()
    p.step(c,object,0)
    assert p.step(c,object,1.9)['pending_age_s']==1.9
    s=p.step(c,object,2)
    assert c.calls[0].cancelled() and len(c.removed)==1
    assert s['events'][0]['event']=='cancel'
    p.step(c,object,2.3);response(c.calls[-1],3)
    assert p.step(c,object,2.4)['active']
    assert not p.step(c,object,3.5)['active']

def test_forever_pending_has_finite_attempts():
    c=Client();c.available=True;p=CMStatePoll(max_attempts=3)
    for t in range(30):s=p.step(c,object,t)
    assert len(c.calls)==3 and len(c.removed)==3
    assert s['attempts_exhausted'] and not s['active']

def test_exception_invalid_response_and_cancel_exception_fail_closed():
    c=Client();c.available=True;p=CMStatePoll()
    p.step(c,object,0);c.calls[-1].set_exception(RuntimeError('injected'))
    assert 'injected' in p.step(c,object,.1)['exception']
    p.step(c,object,.4);c.calls[-1].set_result(None)
    assert not p.step(c,object,.5)['active']
    p.step(c,object,.8)
    def broken(f):raise RuntimeError('remove failed')
    c.remove_pending_request=broken
    s=p.step(c,object,3)
    assert any(e['event']=='remove_pending_exception' for e in s['events']) and not s['active']

def test_startup_cm_unknown_is_waiting_but_motion_blocked():
    facts=dict(safety=True,safety_no_estop=True,cmd_publisher=True,no_motion_authority=True,
               cm=False,cm_active=False,localization=False)
    d=diagnostic_continuation_decision(facts,{},nonzero_command_count=0,distance_m=0)
    assert d['allowed'] and 'cm_active' in d['ordinary_readiness_missing']
    assert not motion_admission(facts)['allowed']
    for key in ('safety_no_estop','no_motion_authority'):
        assert not diagnostic_continuation_decision(dict(facts,**{key:False}),{},
             nonzero_command_count=0,distance_m=0)['allowed']

def test_motion_each_missing_fact_denies_and_only_all_true_admits():
    facts=dict.fromkeys(MOTION_REQUIRED_FACTS,True)
    assert motion_admission(facts)['allowed']
    for key in facts:
        for missing in (False,None,'true'):
            assert not motion_admission(dict(facts,**{key:missing}))['allowed']

def test_cancel_still_executes_if_client_request_removal_raises():
    c=Client();c.available=True;p=CMStatePoll();p.step(c,object,0)
    def broken(f):raise RuntimeError('remove failed')
    c.remove_pending_request=broken
    s=p.step(c,object,2)
    assert c.calls[0].cancelled()
    assert any(e['event']=='remove_pending_exception' for e in s['events'])
    assert not s['active']


def test_launch_readiness_failure_actions_wait_only_for_explicit_diagnostic():
    # Construct real launch actions, without executing LaunchService or a node.
    import importlib.util
    from pathlib import Path
    from launch.actions import EmitEvent, LogInfo
    path=Path(__file__).resolve().parents[3]/'cleannav_simulation/launch/cleannav_full_sim.launch.py'
    spec=importlib.util.spec_from_file_location('startup_wait_fullsim',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for diagnostic in (False,True):
        actions=module.readiness_failure_actions('service pending',diagnostic_startup_wait=diagnostic)
        assert isinstance(actions[0],LogInfo)
        assert any(isinstance(a,EmitEvent) for a in actions) is (not diagnostic)
