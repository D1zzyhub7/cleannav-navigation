"""Route and phase boundary regression without ROS or motion authority."""
import ast
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as N
import pytest

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import supervisor_policy as policy

@pytest.mark.parametrize('direct,length',[(8,10),(10,13),(15,20),(30,45),(60,95)])
def test_route_covers_detour_and_travel_time(direct,length):
    b=policy.historical_route_budget(direct,length)
    assert b['odometry_m']>=length*1.5+2
    assert b['displacement_m']>=direct+2
    assert b['task_s']>=length/.2+30
    # Execute the production Guard hard-stop branch at path completion and limit.
    tree=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and 'MAX_HISTORICAL_ODOMETRY_M' in ast.unparse(n.test))
    for travelled,should_stop in [(length,False),(b['odometry_m'],True)]:
        failures=[]
        scope=dict(self=N(armed=True,distance=travelled,route_budget=b,goal_distance_base=0.,fail=failures.append),
                   short_goal=False,historical=True,MAX_HISTORICAL_ODOMETRY_M=12.)
        exec(compile(ast.Module(body=[branch],type_ignores=[]),'guard-distance','exec'),scope)
        assert bool(failures)==should_stop

@pytest.mark.parametrize('direct,length',[(1,float('nan')),(1,float('inf')),(-1,2),(1,101)])
def test_bad_or_outside_envelope_path_is_rejected(direct,length):
    with pytest.raises(ValueError):policy.historical_route_budget(direct,length)

@pytest.mark.parametrize('value',['nan','inf','0','-1','1001','no'])
def test_invalid_route_envelope_rejected(value):
    r=subprocess.run([sys.executable,'-c','import supervisor_policy'],
        env=dict(os.environ,PYTHONPATH=str(HERE),CLEANNAV_RVIZ_MAX_ROUTE_M=value),capture_output=True)
    assert r.returncode!=0

def test_historical_startup_deadline_does_not_restart_in_gui():
    code='''import supervisor_policy as p
assert p.deadline_stop_reason(224,all_phases_pass=False,observation_elapsed_s=None,startup_verification_complete=True) is None
assert 'readiness_deadline' in p.deadline_stop_reason(224,all_phases_pass=False,observation_elapsed_s=None,startup_verification_complete=False)
assert 'soft_stop' in p.deadline_stop_reason(p.SOFT_STOP_FROM_T0_S,all_phases_pass=False,observation_elapsed_s=None,startup_verification_complete=True)
from deadline_watchdog import readiness_complete_for_deadline
assert readiness_complete_for_deadline(dict(timing_profile='historical-motion',startup_verification_complete=True,readiness_all_passed=False))
'''
    subprocess.run([sys.executable,'-c',code],env=dict(os.environ,PYTHONPATH=str(HERE),CLEANNAV_RVIZ_TIMING_PROFILE='historical-motion'),check=True)

def test_actual_exact_lookup_has_one_shared_bounded_wait_and_keeps_scan_stamp():
    tree=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='exact_lookup')
    # Substitute only the ROS import; execute the real callback with a fake TF buffer.
    fn.body=[n for n in fn.body if not isinstance(n,ast.ImportFrom)]
    calls=[]
    def lookup(target,source,stamp,timeout):
        calls.append((target,source,stamp,timeout))
        return N(header=N(frame_id=target,stamp=N(sec=10,nanosec=0)),child_frame_id=source)
    scope=dict(self=N(buffer=N(lookup_transform=lookup)),Time=lambda **kw:kw['nanoseconds'],
        ClockType=N(ROS_TIME=1),Duration=lambda **kw:kw['seconds'],time=N(monotonic=lambda:1.02),tf_wait_deadline=1.05)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'actual-lookup','exec'),scope)
    scope['exact_lookup']('odom','base_scan',10_000_000_000)
    assert calls[0]==('odom','base_scan',10_000_000_000,0.)
    scope['exact_lookup']('odom','mapping_lidar_link',10_000_000_000)
    assert calls[1]==('odom','mapping_lidar_link',10_000_000_000,pytest.approx(.03))
    scope['time'].monotonic=lambda:1.1
    scope['exact_lookup']('odom','mapping_lidar_link',10_000_000_000)
    assert calls[-1][-1]==0


def test_detour_can_extend_beyond_goal_net_distance_without_early_stop():
    b=policy.historical_route_budget(2.,30.,14.)
    assert b['displacement_m']==16.
    assert b['odometry_m']==47.
    assert b['task_s']==180.
