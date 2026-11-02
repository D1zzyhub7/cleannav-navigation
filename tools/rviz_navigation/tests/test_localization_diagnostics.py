"""Diagnostic timing and telemetry tests; no ROS init, launch or simulation."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import importlib.util
import pytest


def test_tf_callbacks_work_with_the_actual_humble_single_argument_executor():
    import ast
    import asyncio
    from rclpy.executors import Executor
    from types import SimpleNamespace
    tree = ast.parse((Path(__file__).resolve().parents[1]/'localization_diagnostics.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute) and node.func.attr == 'create_subscription'
        and len(node.args) >= 3 and isinstance(node.args[1], ast.Constant)
        and node.args[1].value in ('/tf', '/tf_static')]
    recorded = []
    recorder = SimpleNamespace(convert=lambda msg: {'transforms': msg},
        write=lambda kind, payload: recorded.append((kind, payload)))
    for call in calls:
        callback = eval(compile(ast.Expression(call.args[2]), '<actual TF callback>', 'eval'), {'self': recorder})
        # Execute the installed Humble dispatch, without Node/ROS init or a spin.
        asyncio.run(Executor._execute_subscription(None, SimpleNamespace(callback=callback), ['transform']))
    assert {kind for kind, _ in recorded} == {'tf', 'tf_static'}
    assert all(item['publisher_gid'] is None for _, item in recorded)

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from localization_diagnostics import classify_info,safe_json,DiagnosticRecorder


def contract():
    paths=json.loads(Path(os.environ['CLEANNAV_STATIC_PREFIXES_JSON']).read_text())
    p=Path(paths['cleannav_simulation'])/'lib/cleannav_simulation/localization_health.py'
    spec=importlib.util.spec_from_file_location('installed_health_for_diagnostics_test',p)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.accepted_match


@pytest.mark.parametrize('loop,proximity,rejected,variance,expected',[
    (0,0,0,None,'INFO_WITHOUT_REPORTED_MATCH_CANDIDATE'),
    (0,0,1,None,'REJECTED_HYPOTHESIS_REPORTED'),
    (2,0,0,1.,'MATCH_ID_FAILS_PROJECT_CONTRACT'),
    (999,0,0,.01,'MATCH_ID_FAILS_PROJECT_CONTRACT'),
    (0,2,0,.01,'ACCEPTED_PROJECT_MATCH'),
])
def test_info_classification_uses_installed_health_contract(loop,proximity,rejected,variance,expected):
    stats={'Loop/RejectedHypothesis/':rejected,'Memory/Closest_node_distance/m':.001}
    if variance is not None:stats['Loop/Linear_variance/']=variance
    m=SimpleNamespace(stats_keys=list(stats),stats_values=list(stats.values()),ref_id=30,
        loop_closure_id=loop,proximity_detection_id=proximity)
    result=classify_info(m,[1,2,3],contract())
    assert result['classification']==expected
    assert result['accepted_by_installed_contract']==(expected=='ACCEPTED_PROJECT_MATCH')
    assert result['stats']==stats


def test_nonfinite_telemetry_remains_explicit_json_not_fake_numeric_evidence():
    value=safe_json({'range':float('inf'),'covariance':float('nan'),'missing':None})
    assert json.loads(json.dumps(value,allow_nan=False))=={'range':'inf','covariance':'nan','missing':None}


def test_diagnostic_profile_schedule_and_real_watchdog_code_with_fake_processes(tmp_path):
    program='''
import sys,json
from pathlib import Path
sys.path.insert(0,sys.argv[1]);sys.path.insert(0,sys.argv[2])
import supervisor_policy as p
assert p.timing_identity()['profile']=='localization-diagnostic'
assert p.SOFT_STOP_FROM_T0_S==245 and p.CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S==280
assert p.deadline_stop_reason(172,all_phases_pass=True,observation_elapsed_s=50,launch_elapsed_s=179) is None
assert p.deadline_stop_reason(223,all_phases_pass=True,observation_elapsed_s=50,launch_elapsed_s=180,startup_verification_complete=True)=='startup_soak_complete'
assert p.deadline_stop_reason(223,all_phases_pass=False,observation_elapsed_s=None,launch_elapsed_s=180)=='STARTUP_TIMEOUT'
s=p.DeadlineWatchdogState()
kw=dict(supervisor_alive=True,process_group_status='ALIVE',completion_recorded=False,readiness_complete=False)
assert s.actions(172,launch_elapsed_s=179,**kw)==[]
assert s.actions(223,launch_elapsed_s=180,**kw)[0]['kind']=='signal_supervisor'
assert s.actions(259,**kw)[0]['signal']=='SIGTERM'
assert s.actions(267,**kw)[0]['signal']=='SIGKILL'
assert s.actions(275,**kw)[0]['kind']=='hard_process_cutoff'
s=p.DeadlineWatchdogState()
assert s.actions(223,launch_elapsed_s=180,**{**kw,'readiness_complete':True})==[]
import test_navigation_supervisor as t
root=Path(sys.argv[3])
for name in ('test_watchdog_is_out_of_supervisor_loop_and_emergency_signals_when_loop_blocks',
             'test_watchdog_normal_completion_does_not_signal_any_process',
             'test_watchdog_unknown_group_scan_stays_fail_closed_at_hard_cutoff',
             'test_watchdog_cleanup_timeout_keeps_live_group_as_failure'):
    d=root/name;d.mkdir();getattr(t,name)(d)
print('diagnostic policy and four watchdog failure-injection paths passed')
'''
    result=subprocess.run([sys.executable,'-c',program,str(HERE),str(HERE/'tests'),str(tmp_path)],
        env={**os.environ,'CLEANNAV_RVIZ_TIMING_PROFILE':'localization-diagnostic'},
        text=True,capture_output=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr


def test_public_wrapper_rejects_diagnostic_profile_for_unrelated_map_before_setup(tmp_path):
    r=subprocess.run(['bash',str(HERE/'run_rviz_navigation.sh'),'--timing-profile','localization-diagnostic',
        '--map-profile','roof-lidar-360','--map-directory','/unused','--world','/unused'],
        text=True,capture_output=True,timeout=5)
    assert r.returncode==2
    assert 'requires the canonical localization07 profile' in r.stderr


def test_diagnostic_worker_drains_callbacks_without_supervisor_spin():
    import threading
    from localization_diagnostics import DiagnosticWorker
    class Executor:
        def __init__(self):self.done=threading.Event();self.stop=threading.Event();self.samples=0
        def spin(self):
            for i in range(2048):self.samples+=1
            self.done.set();self.stop.wait()
        def shutdown(self,timeout_sec):self.stop.set();return True
    class Recorder:
        def __init__(self):self.closed=False;self.destroyed=False;self.node=self
        def close(self):self.closed=True
        def destroy_node(self):self.destroyed=True
    executor=Executor();recorder=Recorder();worker=DiagnosticWorker(recorder,executor)
    worker.start()
    assert executor.done.wait(1.0) and executor.samples==2048
    worker.close()
    assert not worker.thread.is_alive() and recorder.closed and recorder.destroyed


def test_diagnostic_worker_callback_failure_is_not_reported_as_success():
    import threading
    from localization_diagnostics import DiagnosticWorker
    class Executor:
        def spin(self):raise ValueError('injected callback failure')
        def shutdown(self,timeout_sec):return True
    recorder=SimpleNamespace(node=SimpleNamespace(destroy_node=lambda:None),close=lambda:None)
    worker=DiagnosticWorker(recorder,Executor());worker.start();worker.thread.join(1)
    with pytest.raises(RuntimeError,match='injected callback failure'):worker.close()


def test_clock_recorder_logs_ros_sim_stamp_and_monotonic_receipt_separately():
    from rosgraph_msgs.msg import Clock
    obj=object.__new__(DiagnosticRecorder);obj.last_clock_ns=None
    rows=[]
    obj.write=lambda kind,payload,**kwargs:rows.append((kind,payload,kwargs))
    message=Clock();message.clock.sec=123;message.clock.nanosec=456
    obj.clock(message)
    kind,payload,kwargs=rows[0]
    assert kind=='clock'
    assert payload=={'topic':'/clock','source_stamp_ns':123_000_000_456,
        'source_stamp_domain':'ROS_SIM_TIME','receipt_clock_domain':'CLOCK_MONOTONIC'}
    assert obj.last_clock_ns==123_000_000_456
    assert isinstance(kwargs['callback_entry_monotonic_ns'],int)


def test_diagnostic_continue_flag_is_rejected_when_motion_is_enabled(capsys):
    from navigation_safety_guard import main
    with pytest.raises(SystemExit) as exit_info:
        main(['--diagnostic-continue-on-readiness-loss','--motion-enabled','true',
              '--motion-profile','short-goal',
              '--output','/tmp/no-write','--run-id','test-run'])
    assert exit_info.value.code==2
    assert 'requires --motion-enabled false' in capsys.readouterr().err


def test_profile_spawn_bindings_survive_delayed_include_scope_restoration(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path/"ros-log"))
    import importlib.util
    from launch import LaunchContext
    from launch.actions import TimerAction
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    import navigation_supervisor as supervisor
    argv=supervisor.bounded_launch_argv(world=Path('/world'),map_directory=Path('/map'),
        map_profile='mcity-localization07-canonical-debug',output_dir=Path('/run'),runtime_timeout_s=140)
    # Recreate the restored outer scope used by the real delayed spawn Node.
    context=LaunchContext();context.launch_configurations.update(spawn_x='0',spawn_y='-8.0',
        spawn_z='.02',spawn_yaw='0',spawn_delay_sec='10.0')
    for item in argv:
        if ':=' in item:
            key,value=item.split(':=',1);context.launch_configurations[key]=value
    path=HERE.parents[1]/'cleannav_simulation/launch/startup_smoke_simulation.launch.py'
    spec=importlib.util.spec_from_file_location('profile_spawn_child_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description()
    timer=next(x for x in description.entities if isinstance(x,TimerAction))
    node=timer.actions[0]
    command=[perform_substitutions(context,normalize_to_list_of_substitutions(item if isinstance(item,list) else [item])) for item in node.cmd]
    assert command[command.index('-y')+1]=='-5.0'
    assert command[command.index('-x')+1]=='0.0'
    assert float(perform_substitutions(context,timer.period))==60.0


def test_real_timer_records_start_and_fire_without_changing_actions(tmp_path, monkeypatch):
    import asyncio
    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_ID','timer-test-only')
    path=HERE.parents[1]/'cleannav_simulation/launch/startup_smoke_simulation.launch.py'
    spec=importlib.util.spec_from_file_location('audited_spawn_timer_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    async def exercise():
        context=LaunchContext();context._set_asyncio_loop(asyncio.get_running_loop())
        context.launch_configurations.update(spawn_x='0',spawn_y='-5',spawn_z='.02',
            spawn_yaw='0',spawn_delay_sec='.001')
        action=OpaqueFunction(function=lambda context: [])
        timer=module.DiagnosticSpawnTimer(period=module.LaunchConfiguration('spawn_delay_sec'),actions=[action])
        timer.execute(context)
        await asyncio.wait_for(timer.get_asyncio_future(),1)
        assert timer.handle(context)==[action]
    asyncio.run(exercise())
    rows=[json.loads(line) for line in (tmp_path/'spawn-timing.jsonl').read_text().splitlines()]
    assert [r['event'] for r in rows]==['wait_started','wait_fired']
    assert rows[1]['monotonic_ns']>rows[0]['monotonic_ns']
    assert all(r['spawn']['spawn_y']=='-5' and r['resolved_timer_period_s']==.001 for r in rows)


def test_scan_stamp_tf_failures_are_preserved_without_latest_fallback():
    from localization_diagnostics import DiagnosticRecorder
    from sensor_msgs.msg import LaserScan
    class Buffer:
        def __init__(self):self.calls=[]
        def lookup_transform(self,target,source,stamp):
            self.calls.append((target,source,stamp.nanoseconds))
            raise LookupError('injected scan-stamp transform failure')
    from collections import Counter
    obj=object.__new__(DiagnosticRecorder);obj.buffer=Buffer();obj.convert=lambda x:{}
    obj.counts=Counter();obj.run_id='test-run';obj.last_clock_ns=None
    rows=[];obj.write=lambda kind,payload,**kwargs:rows.append((kind,payload,kwargs))
    msg=LaserScan();msg.header.frame_id='base_scan';msg.header.stamp.sec=17
    msg.range_min=.12;msg.range_max=10.;msg.ranges=[.5,float('inf'),float('nan')]
    obj.scan('/scan',msg)
    recorded_ranges=rows[0][1]['ranges']
    assert recorded_ranges[:2]==[.5,float('inf')]
    assert math.isnan(recorded_ranges[2])
    assert obj.buffer.calls==[('odom','base_footprint',17*10**9),('base_footprint','base_scan',17*10**9)]
    queries=rows[-1][1]['queries']
    assert all(not q['available'] and q['error_type']=='LookupError' for q in queries)


def test_endpoint_snapshot_includes_odometry_and_tf_topics_and_qos_records():
    class Graph:
        def get_publishers_info_by_topic(self,topic):return [f'{topic}:publisher']
        def get_subscriptions_info_by_topic(self,topic):return [f'{topic}:subscriber']
    obj=object.__new__(DiagnosticRecorder);obj.node=Graph()
    obj.endpoint_record=lambda endpoint:{'identity':endpoint,'qos_profile':'recorded'}
    snapshot=obj.endpoint_snapshot()
    assert set(snapshot)=={'/scan','/mapping/scan_raw','/mapping/scan','/odom',
        '/wheel/odom','/wheel/tf','/tf','/tf_static'}
    for topic in ('/odom','/wheel/odom','/wheel/tf','/tf','/tf_static'):
        assert snapshot[topic]['publishers']==[
            {'identity':f'{topic}:publisher','qos_profile':'recorded'}]
        assert snapshot[topic]['subscribers']==[
            {'identity':f'{topic}:subscriber','qos_profile':'recorded'}]
