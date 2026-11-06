"""Static contracts for the repository-owned, no-motion startup entry."""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

import pytest


NAV = Path(__file__).resolve().parents[3]
ENTRY = NAV / "tools/startup_smoke"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source(relative: str) -> str:
    return (NAV / relative).read_text(encoding="utf-8")


def test_startup_manifest_has_safe_repo_relative_paths_and_matches_sources():
    manifest = json.loads((NAV / "STARTUP_MANIFEST.json").read_text())
    assert manifest["provenance"]["git_parent_commit"] == (
        "403639cafbc5b19d8a7ffc0699c0677831ba0fc6"
    )
    assert manifest["provenance"]["verified_reference_entry_sha256"] == (
        "bd07e9a48940e92660a9a349fca95c951d30418eb92dba1c5d35509356f73da8"
    )
    for relative, digest in manifest["files"].items():
        path = Path(relative)
        assert not path.is_absolute()
        assert ".." not in path.parts
        assert _sha256(NAV / path) == digest, relative


def test_shell_bootstrap_keeps_required_ros_initialization_order():
    shell = _source("tools/startup_smoke/startup_smoke.sh")
    assert "set -eo pipefail" in shell
    assert shell.index('export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"') < shell.index(
        "source /opt/ros/humble/setup.bash"
    )
    assert shell.index("source /opt/ros/humble/setup.bash") < shell.index("command -v ros2")
    assert shell.index("command -v xacro") < shell.index(
        'source "$CLEANNAV_STARTUP_OVERLAY_SETUP"'
    )
    assert "exec " not in shell
    assert "candidate-run/env.sh" not in shell
    assert "/home/d1zzy/" not in shell


def test_supervisor_mode_run_id_and_launch_arguments_match_repository_launch():
    supervisor = _source("tools/startup_smoke/startup_supervisor.py")
    module = ast.parse(supervisor)
    function = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                    and node.name == "launch_argv")
    return_node = next(node for node in ast.walk(function) if isinstance(node, ast.Return))
    def literal_prefix(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            return literal_prefix(node.left) + literal_prefix(node.right)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "str":
            return ""
        return ""

    actual = {prefix.split(":=", 1)[0] for item in return_node.value.elts
              if ":=" in (prefix := literal_prefix(item))}
    launch_source = _source("tools/startup_smoke/startup_smoke.launch.py")
    launch_module = ast.parse(launch_source)
    defaults_node = next(node.value for node in launch_module.body if isinstance(node, ast.Assign)
                         and any(isinstance(target, ast.Name) and target.id == "DEFAULTS"
                                 for target in node.targets))
    declared = {keyword.arg for keyword in defaults_node.keywords if keyword.arg}
    declared |= {"world", "model_path", "maps_root"}
    assert actual == {"mode", "maps_root", "world", "model_path", "sensor_only", "odometry_mode", "localization_mode", "map_yaml"}
    assert actual <= declared
    assert "choices=['startup_smoke']" in supervisor
    assert "p.add_argument('--run-id',required=True)" in supervisor
    assert "must be a unique startup-smoke-* identifier" in supervisor
    assert "/home/d1zzy/" not in supervisor
    assert "cleannav-deliverables/startup-and-git-recovery-20261021" not in supervisor


def test_startup_adapter_forwards_only_no_motion_profile():
    source = _source("tools/startup_smoke/startup_smoke.launch.py")
    module = ast.parse(source)
    defaults = next(node.value for node in module.body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "DEFAULTS"
                            for target in node.targets))
    values = {keyword.arg: keyword.value.value for keyword in defaults.keywords
              if keyword.arg and isinstance(keyword.value, ast.Constant)}
    assert values["mode"] == "startup_smoke"
    assert values["gui"] == values["start_rviz"] == "false"
    assert values["enable_rviz_goals"] == values["auto_start_task"] == "false"
    assert values["enable_motion"] == "false"
    assert "world" in source and "model_path" in source and "maps_root" in source


def test_mapping_composition_is_fail_closed_and_has_no_route_or_lease_node():
    source = _source("tools/startup_smoke/mapping_mode_retry.py")
    assert "if arg('mode') != 'startup_smoke':" in source
    assert "simulation_motion_authorization=False" in source
    assert "SetParameter(name='simulation_motion_authorization', value=False)" in source
    assert "use_sim_time='true', block_all='true'" in source
    assert "no broker/route/Lease" in source
    assert "lease_acquire" not in source
    assert "Mission" not in source
    assert "NavigateToPose" not in source
    assert "mapping_route_node" not in source
    assert "enable_motion" in source and "start_rviz" in source
    assert "RECOVERY_ROOF_XACRO = (\n    SOURCE_ROOT / 'cleannav_simulation' / 'urdf' / 'cleannav_ackermann_startup_smoke.urdf.xacro')" in source
    assert "RECOVERY_SIMULATION_LAUNCH = (\n    SOURCE_ROOT / 'cleannav_simulation' / 'launch' / 'startup_smoke_simulation.launch.py')" in source


def test_simulation_launch_requires_explicit_workspace_world_and_models():
    source = _source("cleannav_simulation/launch/startup_smoke_simulation.launch.py")
    module = ast.parse(source)
    calls = [node for node in ast.walk(module) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument"]
    explicit = {call.args[0].value: call for call in calls
                if call.args and isinstance(call.args[0], ast.Constant)
                and call.args[0].value in {"world", "model_path"}}
    assert set(explicit) == {"world", "model_path"}
    assert all(not any(keyword.arg == "default_value" for keyword in call.keywords)
               for call in explicit.values())
    assert "DEFAULT_WORLD" not in source and "DEFAULT_MODEL_PATH" not in source


def test_roof_and_low_lidar_source_contracts_are_both_preserved():
    xacro = _source("cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro")
    assert '<link name="base_scan">' in xacro
    assert '<link name="mapping_lidar_link">' in xacro
    assert "~/out:=scan_raw" in xacro
    assert "~/out:=/mapping/scan_raw" in xacro
    assert "~/out:=/mapping/scan" in xacro
    assert "<samples>360</samples>" in xacro
    assert "<update_rate>8</update_rate>" in xacro
    assert "frame_name>mapping_lidar_link" in xacro


def test_runtime_helpers_discover_workspace_and_keep_cgroup_exception_explicit():
    runtime = _source("tools/startup_smoke/roof_lidar_runtime.py")
    assert "def discover_workspace(anchor: Path)" in runtime
    assert "cleannav-build/locks/build.lock" in runtime
    assert "cleannav-build/locks/runtime.lock" in runtime
    assert '"status": "UNKNOWN"' in runtime
    assert "cgroup_memory_snapshot" in runtime


def test_package_prefix_gate_requires_operator_bound_exact_sim_and_rtab_prefixes(
    monkeypatch, tmp_path
):
    from ament_index_python import packages
    from roof_lidar_runtime import check_package_prefixes

    prefixes = {name: tmp_path / name for name in (
        "cleannav_simulation", "cleannav_rtabmap", "cleannav_safety_supervisor"
    )}
    for prefix in prefixes.values():
        (prefix / "share" / prefix.name).mkdir(parents=True)
    monkeypatch.setattr(packages, "get_package_prefix", lambda name: str(prefixes[name]))
    monkeypatch.setattr(packages, "get_package_share_directory",
                        lambda name: str(prefixes[name] / "share" / name))
    monkeypatch.setenv("CLEANNAV_EXPECTED_SIMULATION_PREFIX",
                       str(prefixes["cleannav_simulation"]))
    monkeypatch.setenv("CLEANNAV_EXPECTED_RTABMAP_PREFIX",
                       str(prefixes["cleannav_rtabmap"]))
    assert check_package_prefixes()["ok"] is True
    monkeypatch.setenv("CLEANNAV_EXPECTED_RTABMAP_PREFIX", str(tmp_path / "wrong"))
    result = check_package_prefixes()
    assert result["ok"] is False
    assert result["cleannav_rtabmap"]["prefix_matches_expected"] is False
    monkeypatch.delenv("CLEANNAV_EXPECTED_SIMULATION_PREFIX")
    assert check_package_prefixes()["ok"] is False


def test_documented_entry_and_required_static_checks_exist():
    guide = NAV / "docs/operations/STARTUP_SMOKE.md"
    assert guide.is_file()
    assert "tools/startup_smoke/startup_smoke.sh" in guide.read_text()
    assert (ENTRY / "run_static_checks.sh").is_file()


def test_supervisor_parser_refuses_other_modes_without_creating_output():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "startup_supervisor_static_test", ENTRY / "startup_supervisor.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    assert module.parse_args(["--run-id", "startup-smoke-static-contract"]).mode == "startup_smoke"
    with pytest.raises(SystemExit):
        module.parse_args(["--mode", "mapping", "--run-id", "startup-smoke-static-contract"])


def _startup_supervisor_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "startup_supervisor_ros_distro_test", ENTRY / "startup_supervisor.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_ros_distro_gate_accepts_humble():
    module = _startup_supervisor_module()
    assert module.validate_ros_distro("humble") == {
        "expected": "humble", "actual": "humble", "ok": True
    }


def test_ros_distro_gate_runs_before_ros_launch_creation():
    source = _source("tools/startup_smoke/startup_supervisor.py")
    main = source[source.index("def main():"):]
    assert main.index("validate_ros_distro(os.environ.get('ROS_DISTRO'))") < main.index(
        "subprocess.Popen(cmd"
    )


@pytest.mark.parametrize(("value", "expected_text"), [
    ("iron", "got 'iron'"),
    (None, "got <unset>"),
    ("", "got <empty>"),
])
def test_ros_distro_gate_fails_closed_for_non_humble_values(value, expected_text):
    module = _startup_supervisor_module()
    with pytest.raises(RuntimeError, match="ROS_DISTRO mismatch") as error:
        module.validate_ros_distro(value)
    assert expected_text in str(error.value)


def test_sensor_diagnostic_world_is_explicit_and_no_motion(tmp_path):
    import pytest
    module = _startup_supervisor_module()
    world = tmp_path / 'sensor.world'
    world.write_text('<sdf version="1.6"><world name="test"/></sdf>')
    with pytest.raises(SystemExit):
        module.parse_args(['--run-id', 'startup-smoke-test', '--world', str(world)])
    with pytest.raises(SystemExit):
        module.parse_args(['--run-id', 'startup-smoke-test', '--sensor-only', '--world', str(world)+'missing'])
    args = module.parse_args(['--run-id', 'startup-smoke-test', '--sensor-only', '--world', str(world)])
    argv = module.launch_argv(tmp_path, args.world, args.sensor_only)
    assert 'sensor_only:=true' in argv
    assert 'world:='+str(world) in argv
    assert 'sensor_only:=false' in module.launch_argv(tmp_path)
    source = _source('tools/startup_smoke/mapping_mode_retry.py')
    assert "return [collision_lifecycle] if arg('sensor_only') == 'true' else [collision_lifecycle, slam, input_guard]" in source
    assert "motion authorization disabled; no broker/route/Lease" in source
    assert "raise RuntimeError('nonzero control command '+t)" in _source('tools/startup_smoke/startup_supervisor.py')


def test_oracle_arguments_require_sensor_only_and_existing_map(tmp_path):
    module = _startup_supervisor_module()
    with pytest.raises(SystemExit):
        module.parse_args(['--run-id','startup-smoke-static','--odometry-mode','oracle'])
    map_file=tmp_path/'map.yaml';map_file.write_text('image: map.pgm\n')
    args=module.parse_args(['--run-id','startup-smoke-static','--sensor-only',
                           '--odometry-mode','oracle','--map-yaml',str(map_file)])
    argv=module.launch_argv(tmp_path,sensor_only=True,odometry_mode=args.odometry_mode,map_yaml=args.map_yaml)
    assert 'odometry_mode:=oracle' in argv
    assert 'map_yaml:='+str(map_file) in argv


def test_amcl_localization_requires_the_oracle_sensor_only_map_path(tmp_path):
    module = _startup_supervisor_module()
    map_file=tmp_path/'map.yaml';map_file.write_text('image: map.pgm\n')
    with pytest.raises(SystemExit):
        module.parse_args(['--run-id','startup-smoke-static','--localization-mode','amcl'])
    args=module.parse_args(['--run-id','startup-smoke-static','--sensor-only',
        '--odometry-mode','oracle','--localization-mode','amcl','--map-yaml',str(map_file)])
    argv=module.launch_argv(tmp_path,sensor_only=True,odometry_mode=args.odometry_mode,
        map_yaml=args.map_yaml,localization_mode=args.localization_mode)
    assert 'localization_mode:=amcl' in argv
    assert 'odometry_mode:=oracle' in argv
    launch=_source('tools/startup_smoke/startup_smoke.launch.py')
    defaults=ast.parse(launch)
    default_assignment=next(node.value for node in defaults.body if isinstance(node,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=='DEFAULTS' for target in node.targets))
    default_values={keyword.arg:keyword.value.value for keyword in default_assignment.keywords
        if keyword.arg and isinstance(keyword.value,ast.Constant)}
    assert default_values['odometry_mode']=='wheel'
    assert default_values['localization_mode']=='none'


def test_exact_scan_tf_window_anchors_to_first_buffered_transform():
    source=_source('tools/startup_smoke/startup_supervisor.py')
    assert 'self.scan_tf_anchor_ns=latest_ns' in source
    assert "if stamp_ns<self.scan_tf_anchor_ns:" in source
    assert "scan_time_tf_before_buffer_anchor" in source
    assert "lookup_transform(target,source,\n                        Time(nanoseconds=stamp_ns" in source


def test_oracle_runtime_metrics_reject_unrotated_pose_duplicate_authority_and_map_change(tmp_path):
    from types import SimpleNamespace as NS
    import math
    module=_startup_supervisor_module()
    logs=tmp_path/'ros-log';logs.mkdir()
    q=[0,0,math.sqrt(.5),math.sqrt(.5)]
    truth=[dict(stamp_ns=(i+1)*100000000,position=[3,4+i*.01,0],orientation=q,linear=[0,0,0],angular=[0,0,0]) for i in range(11)]
    (logs/'oracle-truth.jsonl').write_text(''.join(json.dumps(t)+'\n' for t in truth))
    (tmp_path/'map.yaml').write_text('resolution: 0.05\norigin: [0, 0, 0]\n')
    (tmp_path/'occupancy_grid.json').write_text(json.dumps(dict(width=1,height=1,data=[0])))
    pubs={'/tf':['robot_state_publisher','gazebo_oracle_odometry'],'/tf_static':['robot_state_publisher'],'/odom':['gazebo_oracle_odometry']}
    node=NS(odom_samples=[dict(stamp_ns=t['stamp_ns'],position=[i*.01,0,0],orientation=[0,0,0,1],frame='odom',child='base_footprint') for i,t in enumerate(truth)],
        clock_samples=[t['stamp_ns'] for t in truth],joint_samples=[dict(names=['wheel'],velocity=[.1])],
        tf_parents={'base_footprint':{'odom'},'base_link':{'base_footprint'}},
        map_message=NS(data=[0],header=NS(frame_id='map'),info=NS(width=1,height=1,resolution=.05,origin=NS(position=NS(x=0,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1)))),
        get_publishers_info_by_topic=lambda topic:[NS(node_name=n) for n in pubs[topic]])
    assert module.oracle_observation_stats(node,tmp_path,tmp_path/'map.yaml')['pass']
    node.odom_samples[-1]['position']=[0,.1,0]
    assert not module.oracle_observation_stats(node,tmp_path,tmp_path/'map.yaml')['pass']
    node.odom_samples[-1]['position']=[.1,0,0]
    pubs['/tf'].append('robot_state_publisher')
    assert not module.oracle_observation_stats(node,tmp_path,tmp_path/'map.yaml')['pass']
    pubs['/tf'].pop();node.map_message.data=[-1]
    assert not module.oracle_observation_stats(node,tmp_path,tmp_path/'map.yaml')['pass']


def test_amcl_runtime_metrics_align_stamped_truth_and_report_error_percentiles(tmp_path):
    from types import SimpleNamespace as NS
    module=_startup_supervisor_module()
    log_dir=tmp_path/'ros-log'/'run';log_dir.mkdir(parents=True)
    truth=[
        dict(stamp_ns=1_000_000_000,position=[0.0,0.0,0.0],orientation=[0,0,0,1]),
        dict(stamp_ns=2_000_000_000,position=[1.0,0.0,0.0],orientation=[0,0,0,1]),
    ]
    (log_dir/'oracle-truth.jsonl').write_text(''.join(json.dumps(item)+'\n' for item in truth))
    publishers={'/tf':['robot_state_publisher','gazebo_oracle_odometry','amcl'],
        '/tf_static':['robot_state_publisher'],'/amcl_pose':['amcl'],'/map':['map_server'],
        '/odom':['gazebo_oracle_odometry']}
    expected=module.expected_amcl_parameters(json.loads(
        _source('tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json')))
    readback={'status':'CAPTURED','values':expected,
        'contract':module.compare_amcl_parameters(expected,expected)}
    node=NS(amcl_samples=[
        dict(stamp_ns=1_000_000_000,frame='map',position=[0.0,0.0,0.0],orientation=[0,0,0,1],covariance_diagonal=[.1,.1,.05]),
        dict(stamp_ns=1_500_000_000,frame='map',position=[.6,0.0,0.0],orientation=[0,0,0,1],covariance_diagonal=[.1,.1,.05]),
        dict(stamp_ns=2_000_000_000,frame='map',position=[1.0,0.0,0.0],orientation=[0,0,0,1],covariance_diagonal=[.1,.1,.05]),
    ],map_to_odom_stamps=[1_000_000_000,1_500_000_000,2_000_000_000],
        amcl_parameter_readback=readback,
        tf_parents={'odom':{'map'},'base_footprint':{'odom'},'base_link':{'base_footprint'}},
        localization_lifecycle={'map_server':{'state_id':3},'amcl':{'state_id':3}},
        get_publishers_info_by_topic=lambda topic:[NS(node_name=name) for name in publishers[topic]])
    result=module.amcl_observation_stats(node,tmp_path)
    assert result['pass']
    assert result['truth_aligned_samples']==3
    assert result['position_error_m']['median']==pytest.approx(0.0)
    assert result['position_error_m']['max']==pytest.approx(0.1)
    assert result['map_to_odom_stamps_strictly_increasing']


def _scan_tf_request(stamp=7825000000):
    return dict(topic='/scan',frame='base_scan',stamp_ns=stamp,received_wall=0.0,deadline=1.0)


def _scan_tf_poll(module,pending,completed,lookup,**overrides):
    values=dict(now=0.1,deadline=145.0,clock_ns=7900000000,clock_age_s=0.0,
                latest_tf_ns=7801000000)
    values.update(overrides)
    return module.poll_scan_tf_requests(pending,completed,lookup,**values)


def test_scan_tf_real_buffer_waits_for_24ms_future_then_exact_interpolation(monkeypatch):
    from tf2_ros import Buffer
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from rclpy.clock import ClockType
    module=_startup_supervisor_module();buffer=Buffer()
    def transform(parent,child,stamp,x):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.header.stamp=Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME).to_msg()
        t.transform.translation.x=float(x);t.transform.rotation.w=1.0
        return t
    buffer.set_transform_static(transform('base_footprint','base_link',0,0),'test')
    buffer.set_transform_static(transform('base_link','base_scan',0,.5),'test')
    buffer.set_transform(transform('map','odom',7805000000,0),'test')
    buffer.set_transform(transform('map','odom',7930000000,0),'test')
    buffer.set_transform(transform('odom','base_footprint',6901000000,0),'test')
    buffer.set_transform(transform('odom','base_footprint',7001000000,.1),'test')
    def lookup(target,source,stamp):
        tr=buffer.lookup_transform(target,source,Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME))
        return tr.header.stamp.sec*1000000000+tr.header.stamp.nanosec
    pending=[_scan_tf_request(7055000000)];completed=[]
    monkeypatch.setattr(module.time,'sleep',lambda *args:pytest.fail('waiting must return to executor, not sleep'))
    calls=[]
    def tracked_lookup(*args):
        calls.append(args)
        return lookup(*args)
    _scan_tf_poll(module,pending,completed,tracked_lookup,clock_ns=7900000000,latest_tf_ns=7001000000)
    assert pending and not completed
    assert pending[0]['future_attempts']==1
    assert calls==[]
    # New Oracle TF is delivered between polls; AMCL's future parent remains unchanged.
    buffer.set_transform(transform('odom','base_footprint',7101000000,.2),'test')
    _scan_tf_poll(module,pending,completed,tracked_lookup,now=.2,clock_ns=7900000000,latest_tf_ns=7101000000)
    assert not pending and completed[0]['returned_stamps_ns']==[7055000000]*2
    assert [call[-1] for call in calls]==[7055000000,7055000000]
    tr=buffer.lookup_transform('odom','base_footprint',Time(nanoseconds=7055000000,clock_type=ClockType.ROS_TIME))
    assert tr.transform.translation.x==pytest.approx(.154)


def test_scan_tf_never_arrives_hits_fixed_deadline():
    module=_startup_supervisor_module()
    def absent(*args):pytest.fail('exact lookup must wait for Oracle time coverage')
    pending=[_scan_tf_request()];completed=[]
    _scan_tf_poll(module,pending,completed,absent)
    with pytest.raises(RuntimeError,match='scan TF deadline exceeded'):
        _scan_tf_poll(module,pending,completed,absent,now=1.0)
    assert not completed


def test_scan_tf_without_any_oracle_sample_waits_until_data_then_exactly_queries():
    module=_startup_supervisor_module();pending=[_scan_tf_request()];completed=[];calls=[]
    _scan_tf_poll(module,pending,completed,lambda *args:calls.append(args),latest_tf_ns=None)
    assert pending and not completed and not calls
    assert pending[0]['wait_reason']=='oracle_tf_time_does_not_cover_scan'
    def exact(target,source,stamp):
        calls.append((target,source,stamp));return stamp
    _scan_tf_poll(module,pending,completed,exact,now=.2,latest_tf_ns=7825000000)
    assert not pending and completed[0]['returned_stamps_ns']==[7825000000]*2
    assert len(calls)==2 and all(call[2]==7825000000 for call in calls)


@pytest.mark.parametrize('overrides,match',[
    ({'latest_tf_ns':5000000000},'stale TF'),
    ({'clock_age_s':1.01},'clock not advancing'),
    ({'clock_ns':0},'clock not advancing'),
    ({'clock_ns':1793295746000000000},'time source'),
    ({'now':.5,'deadline':.5},'supervisor deadline'),
])
def test_scan_tf_stale_clock_or_supervisor_deadline_cannot_pass(overrides,match):
    module=_startup_supervisor_module();completed=[]
    def should_not_lookup(*args):pytest.fail('invalid time/deadline must fail before lookup')
    with pytest.raises(RuntimeError,match=match):
        _scan_tf_poll(module,[_scan_tf_request()],completed,should_not_lookup,**overrides)
    assert not completed


def test_scan_tf_bad_scan_time_disconnection_and_past_error_are_not_transient():
    module=_startup_supervisor_module()
    with pytest.raises(RuntimeError,match='scan timestamp inconsistent'):
        _scan_tf_poll(module,[_scan_tf_request(1793295746000000000)],[],lambda *args:args[-1])
    def disconnected(*args):raise LookupError('TF tree disconnected')
    with pytest.raises(LookupError,match='disconnected'):
        _scan_tf_poll(module,[_scan_tf_request()],[],disconnected,latest_tf_ns=7825000000)
    class PastExtrapolation(Exception):pass
    def past(*args):raise PastExtrapolation('extrapolation into the past')
    with pytest.raises(PastExtrapolation):
        _scan_tf_poll(module,[_scan_tf_request()],[],past,latest_tf_ns=7825000000)


def test_scan_tf_latest_transform_cannot_substitute_for_requested_stamp():
    module=_startup_supervisor_module()
    with pytest.raises(RuntimeError,match='different timestamp'):
        _scan_tf_poll(module,[_scan_tf_request()],[],lambda *args:7801000000,latest_tf_ns=7825000000)


def test_scan_tf_odom_root_without_amcl_keeps_existing_exact_lookup_path():
    from tf2_ros import Buffer
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from rclpy.clock import ClockType
    module=_startup_supervisor_module();buffer=Buffer();requested=7825000000
    def sample(parent,child,stamp,x=0.0):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.header.stamp=Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME).to_msg()
        t.transform.translation.x=float(x);t.transform.rotation.w=1.0
        return t
    buffer.set_transform_static(sample('base_footprint','base_scan',0,.5),'test')
    buffer.set_transform(sample('odom','base_footprint',7701000000,0),'test')
    buffer.set_transform(sample('odom','base_footprint',7901000000,.2),'test')
    def lookup(target,source,stamp):
        result=buffer.lookup_transform(target,source,Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME))
        return result.header.stamp.sec*1000000000+result.header.stamp.nanosec
    pending=[_scan_tf_request(requested)];completed=[]
    _scan_tf_poll(module,pending,completed,lookup,clock_ns=7900000000,latest_tf_ns=7901000000)
    assert not pending and completed[0]['returned_stamps_ns']==[requested,requested]


def test_scan_tf_multiple_queued_scans_are_retained_and_consumed_once():
    module=_startup_supervisor_module()
    pending=[_scan_tf_request(8000000000),_scan_tf_request(7900000000)]
    completed=[];calls=[]
    def exact(target,source,stamp):
        calls.append((target,source,stamp));return stamp
    _scan_tf_poll(module,pending,completed,exact,now=.2,clock_ns=8100000000,latest_tf_ns=7950000000)
    assert [r['stamp_ns'] for r in pending]==[8000000000]
    assert [r['stamp_ns'] for r in completed]==[7900000000]
    assert len(calls)==2
    _scan_tf_poll(module,pending,completed,exact,now=.3,clock_ns=8100000000,latest_tf_ns=8050000000)
    assert not pending
    assert sorted(r['stamp_ns'] for r in completed)==[7900000000,8000000000]
    assert len(calls)==4 and all(call[2] in (7900000000,8000000000) for call in calls)


@pytest.mark.parametrize('requested,is_future',[(7221000000,True),(7001000000,False)])
def test_scan_tf_real_single_sample_future_waits_but_past_does_not(requested,is_future):
    from tf2_ros import Buffer,ExtrapolationException
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from rclpy.clock import ClockType
    module=_startup_supervisor_module();buffer=Buffer()
    def sample(parent,child,stamp):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.header.stamp=Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME).to_msg()
        t.transform.rotation.w=1.0
        return t
    buffer.set_transform_static(sample('base_footprint','base_scan',0),'test')
    buffer.set_transform(sample('odom','base_footprint',7101000000),'test')
    def lookup(target,source,stamp):
        t=buffer.lookup_transform(target,source,Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME))
        return t.header.stamp.sec*1000000000+t.header.stamp.nanosec
    pending=[_scan_tf_request(requested)];completed=[]
    if not is_future:
        with pytest.raises(ExtrapolationException):
            _scan_tf_poll(module,pending,completed,lookup,clock_ns=7300000000,latest_tf_ns=7101000000)
        return
    _scan_tf_poll(module,pending,completed,lookup,clock_ns=7300000000,latest_tf_ns=7101000000)
    assert pending[0]['future_attempts']==1
    assert pending[0]['wait_reason']=='oracle_tf_time_does_not_cover_scan'
    buffer.set_transform(sample('odom','base_footprint',7301000000),'test')
    _scan_tf_poll(module,pending,completed,lookup,now=.2,clock_ns=7400000000,latest_tf_ns=7301000000)
    assert not pending and completed[0]['returned_stamps_ns']==[requested]*2


def test_scan_tf_unrecognized_extrapolation_is_not_ignored():
    module=_startup_supervisor_module()
    class OtherExtrapolation(Exception):pass
    def error(*args):raise OtherExtrapolation('different TF failure')
    with pytest.raises(OtherExtrapolation):
        _scan_tf_poll(module,[_scan_tf_request()],[],error,latest_tf_ns=7825000000)


def test_amcl_parameter_contract_separates_yaml_request_and_effective_values(monkeypatch,tmp_path):
    import tempfile
    import yaml
    from launch import LaunchContext
    from launch_ros.actions import Node

    module=_startup_supervisor_module()
    contract=json.loads(_source('tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json'))
    expected=module.expected_amcl_parameters(contract)
    base=NAV/'cleannav_navigation/config/nav2_params_ackermann.yaml'
    yaml_values=yaml.safe_load(base.read_text())['amcl']['ros__parameters']
    assert yaml_values['use_sim_time'] is False
    assert yaml_values['transform_tolerance']==1.0
    assert yaml_values['scan_topic']=='scan'
    assert expected=={'use_sim_time':True,'transform_tolerance':0.0,
        'scan_topic':'/mapping/scan','global_frame_id':'map','odom_frame_id':'odom',
        'base_frame_id':'base_footprint','tf_broadcast':True}

    source=ast.parse(_source('tools/startup_smoke/mapping_mode_retry.py'))
    amcl_node=next(node for node in ast.walk(source) if isinstance(node,ast.Call)
        and isinstance(node.func,ast.Name) and node.func.id=='Node'
        and any(keyword.arg=='package' and isinstance(keyword.value,ast.Constant)
            and keyword.value.value=='nav2_amcl' for keyword in node.keywords))
    keywords={keyword.arg:keyword.value for keyword in amcl_node.keywords}
    assert isinstance(keywords['namespace'],ast.Constant) and keywords['namespace'].value=='/'
    assert isinstance(keywords['parameters'],ast.List) and len(keywords['parameters'].elts)==2

    monkeypatch.setattr(tempfile,'tempdir',str(tmp_path))
    overrides={**expected,'set_initial_pose':True,'initial_pose.x':0.0015082397731021,
        'initial_pose.y':-7.99616241455078,'initial_pose.z':0.0,'initial_pose.yaw':0.0131529364734888}
    action=Node(package='nav2_amcl',executable='amcl',name='amcl',namespace='/',
        parameters=[str(base),overrides])
    action._perform_substitutions(LaunchContext())
    generated=Path(action._Node__expanded_parameter_arguments[-1][0])
    materialized=yaml.safe_load(generated.read_text())
    assert list(materialized)==['/amcl']
    assert materialized['/amcl']['ros__parameters']==overrides


def test_amcl_get_parameters_readback_is_typed_and_contract_checked():
    from types import SimpleNamespace as NS
    from rcl_interfaces.msg import ParameterType
    module=_startup_supervisor_module()
    contract=json.loads(_source('tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json'))
    expected=module.expected_amcl_parameters(contract)
    values=[
        NS(type=ParameterType.PARAMETER_BOOL,bool_value=True),
        NS(type=ParameterType.PARAMETER_DOUBLE,double_value=0.0),
        NS(type=ParameterType.PARAMETER_STRING,string_value='/mapping/scan'),
        NS(type=ParameterType.PARAMETER_STRING,string_value='map'),
        NS(type=ParameterType.PARAMETER_STRING,string_value='odom'),
        NS(type=ParameterType.PARAMETER_STRING,string_value='base_footprint'),
        NS(type=ParameterType.PARAMETER_BOOL,bool_value=True),
    ]
    actual=module.decode_amcl_parameter_values(values,ParameterType)
    assert actual==expected
    assert module.compare_amcl_parameters(actual,expected)=={
        'status':'PASS','matches_contract':True,'mismatches':{}}
    actual['scan_topic']='scan'
    result=module.compare_amcl_parameters(actual,expected)
    assert result['status']=='MISMATCH' and result['mismatches']['scan_topic']['expected']=='/mapping/scan'
    assert module.compare_amcl_parameters(None,expected)['status']=='NOT_CAPTURED'
    values[0]=NS(type=ParameterType.PARAMETER_NOT_SET,bool_value=False)
    with pytest.raises(ValueError,match='unset or unexpected type'):
        module.decode_amcl_parameter_values(values,ParameterType)
