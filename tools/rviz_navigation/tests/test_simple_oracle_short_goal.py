"""Static action construction and actual policy callbacks; no ROS init or launch."""
import ast
import json
import math
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace as N
import sys
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import integration
from readiness_inputs import amcl_bootstrap_ready
from supervisor_policy import runtime_memory_decision
PROFILE='simple-four-wall-nohit-v1-oracle-short-goal'


def test_navigation_profile_keeps_static_contract_and_wheel_default():
    static=json.loads((HERE/'map_contract_simple_four_wall_nohit_v1_oracle_localization.json').read_text())
    nav=json.loads(integration.contract_path_for_profile(PROFILE).read_text())
    for name in ('files','world_sha256','map_geometry','spawn','localization_initial_pose'):
        assert nav[name]==static[name]
    for name in ('kind','package','package_version','base_frame_id','global_frame_id','odom_frame_id','scan_topic','tf_broadcast'):
        assert nav['localization_runtime'][name]==static['localization_runtime'][name]
    assert static['localization_runtime']['transform_tolerance_s']==0.0
    assert nav['localization_runtime']['transform_tolerance_s']==0.05
    profile=json.loads((integration.NAV/'tools/startup_smoke/amcl_private_buffer_diagnostic_profile.json').read_text())
    assert nav['localization_runtime']['executable_sha256']==profile['nodes']['amcl']['executable_sha256']
    assert nav['required_odometry_mode']=='oracle'
    assert nav['motion_profile']=='short-goal'
    assert json.loads((HERE/'map_contract_simple_four_wall_nohit_v1.json').read_text())['required_odometry_mode']=='wheel'
    integration.verify_map(integration.WORKSPACE/nav['map_directory_relative'],
        integration.WORKSPACE/nav['world_relative'],profile=PROFILE,odometry_mode='oracle')
    with pytest.raises(ValueError,match='odometry source'):
        integration.verify_map(integration.WORKSPACE/nav['map_directory_relative'],
            integration.WORKSPACE/nav['world_relative'],profile=PROFILE,odometry_mode='wheel')


@pytest.mark.parametrize('localizer,motion',[('wheel','none'),('amcl','none'),('amcl','short-goal')])
def test_actual_composition_has_one_localizer_and_serial_safety(monkeypatch,tmp_path,localizer,motion):
    monkeypatch.setenv('CLEANNAV_TF_PRIVATE_DIAGNOSTICS','1')
    module=integration.load_module(HERE/'rviz_navigation.launch.py','simple_goal_launch_test')
    monkeypatch.setattr(module.launch_config,'log_dir',str(tmp_path))
    selected=PROFILE if localizer=='amcl' else 'simple-four-wall-nohit-v1'
    profile=json.loads(integration.contract_path_for_profile(selected).read_text())
    context=LaunchContext()
    for d in module.generate_launch_description().entities:
        if isinstance(d,DeclareLaunchArgument) and d.default_value is not None:
            context.launch_configurations[d.name]=perform_substitutions(context,d.default_value)
    context.launch_configurations.update(world=str(integration.WORKSPACE/profile['world_relative']),
        map_directory=str(integration.WORKSPACE/profile['map_directory_relative']),map_profile=selected,
        odometry_mode='oracle' if localizer=='amcl' else 'wheel',motion_profile=motion,
        enable_motion='true' if motion=='short-goal' else 'false',start_rviz='false',runtime_timeout_sec='145')
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_ID','rviz-nav-static-fixture')
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_DIR',str(tmp_path))
    (tmp_path/'wrapper.json').write_text('{}') # no operational Run/Auth/Scope/Lease
    captured=[];includes=[];nodes=[]
    parent=module.base._compose
    def compose(ctx,integration=None):
        captured.append(integration)
        return parent(ctx,integration=integration)
    monkeypatch.setattr(module.base,'_compose',compose)
    original_include=module.base.IncludeLaunchDescription
    def include(*a,**kw):
        result=original_include(*a,**kw)
        result.launch_description_source.get_launch_description(context)
        includes.append(result.launch_description_source.location)
        return result
    monkeypatch.setattr(module.base,'IncludeLaunchDescription',include)
    original_node=module.Node
    def node(**kw):
        nodes.append(kw)
        return original_node(**kw)
    monkeypatch.setattr(module,'Node',node)
    actions=module._compose(context)
    assert actions
    cfg=captured[0]
    rtab_includes=[x for x in includes if 'cleannav_rtabmap_localization.launch.py' in x]
    amcl_nodes=[x for x in nodes if x.get('package')=='nav2_amcl']
    assert len(rtab_includes)==(1 if localizer=='wheel' else 0)
    assert len(amcl_nodes)==(0 if localizer=='wheel' else 1)
    assert cfg['forward_limit']=='0.15'
    assert cfg['reverse_limit']=='0.0'
    assert cfg['arguments']['cleannav_navigation/cleannav_ackermann_navigation.launch.py']['controller_cmd_vel_topic']=='/cleannav/navigation/cmd_vel_raw'
    import yaml
    cm=yaml.safe_load((tmp_path/'localization/collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic']=='/cleannav/navigation/cmd_vel_raw'
    assert cm['cmd_vel_out_topic']=='/cleannav/cmd_vel_candidate'
    params=yaml.safe_load((tmp_path/'localization/nav2_params.yaml').read_text())
    controller=params['controller_server']['ros__parameters']['FollowPath']
    assert controller['vx_max']==.15 and controller['vx_min']==0.
    assert params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is True
    if localizer=='amcl':
        amcl=amcl_nodes[0]
        assert amcl['namespace']=='/'
        assert amcl['remappings']==[('map','/cleannav/localization/map')]
        assert amcl['parameters'][1]['scan_tf_wait_timeout']==.3
        assert amcl['parameters'][1]['transform_tolerance']==0.05
        assert amcl['additional_env']['CLEANNAV_TF_PRIVATE_DIAGNOSTICS']=='1'
        assert cfg['localization_kind']=='amcl'
        from launch.actions import ExecuteProcess
        assert actions.index(cfg['environment_actions'][0]) < min(i for i,a in enumerate(actions) if isinstance(a,ExecuteProcess))
        assert not any('rtab_input_guard' in str(x) for x in cfg['localization_actions'])
        assert cfg['oracle_odometry']['identity']['authoritative_chain']['tf']=='odom -> base_footprint'
    if motion=='short-goal':
        cmd=cfg['mission_goal_actions'][0].cmd
        assert any('short_goal_rviz_adapter.py' in perform_substitutions(context,arg) for arg in cmd)
        assert cfg['defer_motion_authorization'] is True
        assert not list(tmp_path.rglob('motion-authorization.json'))
        assert not list(tmp_path.rglob('motion-scope.json'))


def _compose_amcl_profile(monkeypatch, tmp_path, *, motion, odometry):
    """Build the real AMCL launch composition. No ROS node, Run, or Lease."""
    monkeypatch.setenv('CLEANNAV_TF_PRIVATE_DIAGNOSTICS', '1')
    module = integration.load_module(HERE/'rviz_navigation.launch.py', 'amcl_historical_launch_test')
    monkeypatch.setattr(module.launch_config, 'log_dir', str(tmp_path))
    profile = json.loads(integration.contract_path_for_profile(PROFILE).read_text())
    context = LaunchContext()
    for declaration in module.generate_launch_description().entities:
        if isinstance(declaration, DeclareLaunchArgument) and declaration.default_value is not None:
            context.launch_configurations[declaration.name] = perform_substitutions(
                context, declaration.default_value)
    context.launch_configurations.update(
        world=str(integration.WORKSPACE/profile['world_relative']),
        map_directory=str(integration.WORKSPACE/profile['map_directory_relative']),
        map_profile=PROFILE, odometry_mode=odometry, motion_profile=motion,
        enable_motion='true' if motion in ('short-goal', 'historical', 'patrol') else 'false',
        start_rviz='false', runtime_timeout_sec='145')
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_ID', 'rviz-nav-static-fixture')
    monkeypatch.setenv('CLEANNAV_RVIZ_RUN_DIR', str(tmp_path))
    (tmp_path/'wrapper.json').write_text('{}')
    captured = []
    nodes = []
    parent = module.base._compose

    def compose(ctx, integration=None):
        captured.append(integration)
        return parent(ctx, integration=integration)

    monkeypatch.setattr(module.base, '_compose', compose)
    original_include = module.base.IncludeLaunchDescription

    def include(*args, **kwargs):
        result = original_include(*args, **kwargs)
        result.launch_description_source.get_launch_description(context)
        return result

    monkeypatch.setattr(module.base, 'IncludeLaunchDescription', include)
    original_node = module.Node

    def node(**kwargs):
        nodes.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(module, 'Node', node)
    actions = module._compose(context)
    return actions, captured[0], nodes, context


def _performed_cmd(context, action):
    rendered = []
    for part in action.cmd:
        rendered.append(part if isinstance(part, str) else perform_substitutions(context, part))
    return rendered


def test_amcl_oracle_historical_reuses_the_existing_business_path(monkeypatch, tmp_path):
    actions, cfg, nodes, context = _compose_amcl_profile(
        monkeypatch, tmp_path, motion='historical', odometry='oracle')
    assert actions
    assert cfg['localization_kind'] == 'amcl'
    assert cfg['odometry_mode'] == 'oracle'
    assert cfg['defer_motion_authorization'] is False
    assert cfg['mission_goal_actions'] is None
    assert cfg['forward_limit'] == '0.5'
    assert cfg['reverse_limit'] == '0.0'
    assert float(cfg['angular_limit']) == pytest.approx(0.5/1.12)
    amcl_nodes = [item for item in nodes if item.get('package') == 'nav2_amcl']
    assert len(amcl_nodes) == 1
    guard_cmd = _performed_cmd(context, cfg['initial_actions'][0])
    assert guard_cmd[guard_cmd.index('--motion-profile')+1] == 'historical'
    assert not any('short_goal_rviz_adapter.py' in str(action) for action in actions)
    session = json.loads((tmp_path/'localization'/'session.json').read_text())
    assert session['localization_kind'] == 'amcl'
    assert session['odometry_mode'] == 'oracle'
    assert session['map_profile'] == PROFILE
    import yaml
    controller = yaml.safe_load((tmp_path/'localization'/'nav2_params.yaml').read_text())[
        'controller_server']['ros__parameters']['FollowPath']
    assert controller['vx_max'] == 0.5 and controller['vx_min'] == 0.0
    assert controller['wz_max'] == pytest.approx(0.5/1.12)
    checker = yaml.safe_load((tmp_path/'localization'/'nav2_params.yaml').read_text())[
        'controller_server']['ros__parameters']['general_goal_checker']
    assert checker['xy_goal_tolerance'] == 0.25
    assert checker['yaw_goal_tolerance'] == 0.40


def test_amcl_launch_still_rejects_wheel_odometry(monkeypatch, tmp_path):
    with pytest.raises(ValueError, match='odometry source'):
        _compose_amcl_profile(monkeypatch, tmp_path, motion='historical', odometry='wheel')


def test_amcl_simple_map_shell_rejects_wheel_and_patrol(tmp_path):
    output = tmp_path/'output-must-not-exist'
    env = dict(os.environ, CLEANNAV_RVIZ_OUTPUT_ROOT=str(output))
    script = HERE/'run_rviz_navigation.sh'

    def invoke(*extra):
        return subprocess.run(
            ['bash', str(script), '--map-profile', PROFILE,
             '--map-directory', '/unused/map', '--world', '/unused/world.sdf', *extra],
            cwd=integration.NAV, env=env, capture_output=True, text=True, timeout=30)

    wheel = invoke('--odometry-mode', 'wheel', '--motion-profile', 'historical')
    assert wheel.returncode == 2
    assert 'only none, short-goal, or historical motion' in wheel.stderr
    patrol = invoke('--odometry-mode', 'oracle', '--motion-profile', 'patrol')
    assert patrol.returncode == 2
    assert 'patrol requires patrol-a-v4 with formal wheel odometry' in patrol.stderr
    assert not output.exists()


def _ready_health():
    return dict(localization_kind='amcl',pose_count=1,**dict.fromkeys([
        'localized','parameters_verified','assets_unchanged','sensors_fresh','frozen_map',
        'map_server_active','amcl_active','authority_ok','map_tf_valid','scan_tf_valid'],True))


@pytest.mark.parametrize('missing',list(_ready_health()))
def test_amcl_readiness_fails_for_missing_required_evidence(missing):
    state=_ready_health();state.pop(missing)
    assert not amcl_bootstrap_ready(state,.1)


@pytest.mark.parametrize('age',[1.51,-.1,math.nan,math.inf])
def test_amcl_health_age_is_bounded(age):
    assert not amcl_bootstrap_ready(_ready_health(),age)


def test_amcl_health_does_not_forge_rtabmap_match():
    from navigation_safety_guard import localization_bootstrap_ready
    assert amcl_bootstrap_ready(_ready_health(),.1)
    assert localization_bootstrap_ready(lambda *a:pytest.fail('native RTAB helper called'),_ready_health(),1.,1.1)
    assert 'info' not in _ready_health()


@pytest.mark.parametrize('reason',['memory_full_psi_high','SwapUsed_abnormal','cgroup_headroom_insufficient'])
def test_short_goal_soft_memory_keeps_pressure_gates(reason):
    good=dict(ok=False,reasons=['MemAvailable_below_5GiB'],MemAvailable_kib=4500000)
    assert runtime_memory_decision(good,timing_profile='short-goal-motion',no_motion_contract=False)['ok']
    bad=dict(good,reasons=good['reasons']+[reason])
    assert not runtime_memory_decision(bad,timing_profile='short-goal-motion',no_motion_contract=False)['ok']


def test_adapter_retains_mission_cancel_estop_and_transient_map():
    tree=ast.parse((HERE/'short_goal_rviz_adapter.py').read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='ShortGoalAdapter')
    methods={x.name:x for x in cls.body if isinstance(x,ast.FunctionDef)}
    goal=ast.unparse(methods['on_goal']);stop=ast.unparse(methods['stop_active_task'])
    assert 'SubmitNavigationGoal.Request()' in goal
    assert 'self.client.call_async(request)' in goal
    assert 'request.SOURCE_RVIZ' in goal and 'request.REJECT_IF_BUSY' in goal
    assert 'self.estop.publish(Bool(data=True))' in stop
    assert 'self.cancel.call_async(CancelGoal.Request())' in stop
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in ast.unparse(methods['__init__'])
    # Execute the existing stop callback against fake clients: no ROS runtime.
    stop_events=[]
    scope=dict(Bool=lambda **kw:kw,CancelGoal=N(Request=lambda:'cancel-request'))
    exec(compile(ast.Module(body=[methods['stop_active_task']],type_ignores=[]),'<actual-stop>','exec'),scope)
    node=N(execution_id='ros-execution-1',args=N(execution_id='ros-execution-1'),emit=lambda *a,**kw:None,
        estop=N(publish=lambda m:stop_events.append(('estop',m))),
        cancel=N(service_is_ready=lambda:True,call_async=lambda m:stop_events.append(('cancel',m))))
    scope['stop_active_task'](node,'test failure')
    assert stop_events==[('estop',{'data':True}),('cancel','cancel-request')]


def test_actual_amcl_monitor_holds_only_stationary_anchor_and_latches_motion_loss(monkeypatch,tmp_path):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_health_actual_test')
    now=[20.];monkeypatch.setattr(monitor.time,'monotonic',lambda:now[0])
    states=[];cancels=[]
    node=N(session={'amcl_expected_parameters':[]},_query=lambda *a:None,
        native_client=None,amcl_native=lambda response:None,last_asset_check=20.,amcl_pose_count=1,amcl_pose_valid=True,
        native_ok=True,assets_ok=True,amcl_parameters={},amcl_stamp_ns=1000000000,
        odom_pose=(0.,0.,0.),match_odom_pose=(0.,0.,0.),odom_history=[(19.,(0.,0.,0.))],
        match_time=5.,linear_update=.25,angular_update=.2,amcl_covariance=.01,
        had_localization=True,motion_evidence=monitor.MotionEvidence(),lifecycle={'amcl':True,'map_server':True},
        received={'frozen_map':True},_fresh=lambda k:True,
        get_node_names_and_namespaces=lambda:[('amcl','/')],
        get_publishers_info_by_topic=lambda t:[N(node_name='amcl')],
        _tf=lambda *a,**kw:True,_scan_tf=lambda **kw:True,
        cancel_sent=False,cancel_client=N(service_is_ready=lambda:True,call_async=lambda x:cancels.append(x)),
        health_pub=N(publish=lambda m:states.append(json.loads(m.data))),
        session_path=tmp_path/'session.json')
    tick=monitor.LocalizationMonitor._poll_amcl
    tick(node)
    assert states[-1]['localized'] and states[-1]['stationary_since_match']
    assert 'info' not in states[-1] # never invent ICP matches to satisfy native readiness
    node.odom_pose=(.5,0.,0.);node.odom_history=[(19.,node.odom_pose)]
    tick(node)
    now[0]=23.1
    tick(node)
    assert not states[-1]['localized'] and node.motion_evidence.lost and len(cancels)==1
    node.odom_pose=(0.,0.,0.)
    tick(node)
    assert not states[-1]['localized'] # unmatched return cannot reset loss


@pytest.mark.parametrize('bad',['_tf','_scan_tf','native_ok','assets_ok','amcl_pose_valid'])
def test_amcl_health_keeps_real_tf_and_parameter_failures(monkeypatch,tmp_path,bad):
    # Reuse the actual tick from a real monitor, with no Node creation.
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_health_negative_test')
    states=[];now=20.
    monkeypatch.setattr(monitor.time,'monotonic',lambda:now)
    node=N(session={'amcl_expected_parameters':[]},_query=lambda *a:None,native_client=None,amcl_native=lambda response:None,
        last_asset_check=20.,amcl_pose_count=1,amcl_pose_valid=True,native_ok=True,assets_ok=True,
        amcl_parameters={},amcl_stamp_ns=1,odom_pose=(0.,0.,0.),match_odom_pose=(0.,0.,0.),
        odom_history=[(19.,(0.,0.,0.))],match_time=19.,linear_update=.25,angular_update=.2,
        amcl_covariance=.01,had_localization=True,motion_evidence=monitor.MotionEvidence(),
        lifecycle={'amcl':True,'map_server':True},received={'frozen_map':True},_fresh=lambda k:True,
        get_node_names_and_namespaces=lambda:[('amcl','/')],
        get_publishers_info_by_topic=lambda t:[N(node_name='amcl')],
        _tf=lambda *a,**kw:True,_scan_tf=lambda **kw:True,cancel_sent=False,
        cancel_client=N(service_is_ready=lambda:False),health_pub=N(publish=lambda m:states.append(json.loads(m.data))),
        session_path=tmp_path/'session.json')
    setattr(node,bad,(lambda *a,**kw:False) if bad.startswith('_') else False)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False


def _amcl_health_fixture(monitor, tmp_path, mode, covariance, *, match_time=19.,
                         map_tf=True, scan_tf=True, evidence=None, pose_count=1,
                         pose_valid=True, stamp_ns=219410000000, had_localization=True):
    warnings=[];cancels=[];states=[]
    node=N(session={'amcl_expected_parameters':[],'amcl_overrides':{'max_particles':2000}},
        _query=lambda *a:None,
        native_client=None,amcl_native=lambda response:None,last_asset_check=20.,
        amcl_pose_count=pose_count,amcl_pose_valid=pose_valid,native_ok=True,assets_ok=True,
        amcl_parameters={},amcl_stamp_ns=stamp_ns,odom_pose=(0.,0.,0.),
        match_odom_pose=(0.,0.,0.),odom_history=[(19.,(0.,0.,0.))],
        match_time=match_time,linear_update=.25,angular_update=.2,
        amcl_covariance=covariance,had_localization=had_localization,
        motion_evidence=evidence or monitor.MotionEvidence(),
        lifecycle={'amcl':True,'map_server':True},received={'frozen_map':True},
        _fresh=lambda k:True,get_node_names_and_namespaces=lambda:[('amcl','/')],
        get_publishers_info_by_topic=lambda t:[N(node_name='amcl')],
        _tf=lambda *a,**kw:map_tf,_scan_tf=lambda **kw:scan_tf,cancel_sent=False,
        cancel_client=N(service_is_ready=lambda:True,call_async=lambda x:cancels.append(x)),
        health_pub=N(publish=lambda m:states.append(json.loads(m.data))),
        session_path=tmp_path/'session.json',odometry_mode=mode,
        get_logger=lambda:N(warning=lambda text:warnings.append(text)))
    return node,states,cancels,warnings


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_amcl_covariance_over_reference_warns_and_keeps_navigation(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_covariance_warn_'+mode)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    node,states,cancels,warnings=_amcl_health_fixture(monitor,tmp_path,mode,.262)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert node.odometry_mode==mode and states[-1]['localized'] is True
    assert node.motion_evidence.lost is False and cancels==[]
    assert states[-1]['covariance']==.262 and states[-1]['covariance_above_limit'] is True
    assert states[-1]['covariance_warning_stamp_ns']==219410000000
    assert states[-1]['map_tf_valid'] is True and states[-1]['scan_tf_valid'] is True
    assert len(warnings)==1 and '0.262000' in warnings[0] and '219410000000' in warnings[0]
    node.amcl_covariance=.30;node.amcl_stamp_ns=220000000000
    monitor.LocalizationMonitor._poll_amcl(node)
    node.amcl_covariance=.40;node.amcl_stamp_ns=221000000000
    monitor.LocalizationMonitor._poll_amcl(node)
    assert len(warnings)==1 and states[-1]['localized'] is True
    assert states[-1]['covariance']==.40 and node.motion_evidence.lost is False and cancels==[]
    node.amcl_covariance=.10;node.amcl_stamp_ns=222000000000
    monitor.LocalizationMonitor._poll_amcl(node)
    assert len(warnings)==2 and 'returned within 0.25' in warnings[1]
    assert states[-1]['localized'] is True and states[-1]['covariance_above_limit'] is False
    assert states[-1]['covariance_warning_stamp_ns']==222000000000
    assert node.motion_evidence.lost is False and cancels==[]


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_amcl_pose_timeout_still_cancels_navigation(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_pose_timeout_'+mode)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    evidence=monitor.MotionEvidence();evidence.match=10.;evidence.started=10.
    node,states,cancels,warnings=_amcl_health_fixture(
        monitor,tmp_path,mode,.01,match_time=10.,evidence=evidence)
    node.odom_pose=(1.,0.,0.);node.odom_history=[(19.,node.odom_pose)]
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1 and warnings==[]


def _advance(monkeypatch, monitor, clock, now):
    clock['t']=now
    monkeypatch.setattr(monitor.time,'monotonic',lambda:clock['t'])


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_motion_below_both_amcl_thresholds_does_not_go_lost(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_below_'+mode)
    clock={'t':20.}
    _advance(monkeypatch,monitor,clock,20.)
    node,states,cancels,warnings=_amcl_health_fixture(monitor,tmp_path,mode,.01,match_time=10.)
    node.odom_pose=(.20,0.,.05);node.odom_history=[(19.,node.odom_pose)]
    monitor.LocalizationMonitor._poll_amcl(node)
    _advance(monkeypatch,monitor,clock,24.)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is True and node.motion_evidence.lost is False
    assert node.motion_evidence.started is None and cancels==[] and warnings==[]


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_displacement_at_update_min_d_starts_the_existing_timeout(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_distance_'+mode)
    clock={'t':20.}
    _advance(monkeypatch,monitor,clock,20.)
    node,states,cancels,_=_amcl_health_fixture(monitor,tmp_path,mode,.01,match_time=10.)
    node.odom_pose=(.25,0.,0.);node.odom_history=[(19.,node.odom_pose)]
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is True and node.motion_evidence.lost is False
    assert node.motion_evidence.started==20. and cancels==[]
    _advance(monkeypatch,monitor,clock,23.1)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_yaw_at_update_min_a_starts_the_existing_timeout(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_yaw_'+mode)
    clock={'t':20.}
    _advance(monkeypatch,monitor,clock,20.)
    node,states,cancels,_=_amcl_health_fixture(monitor,tmp_path,mode,.01,match_time=10.)
    node.odom_pose=(0.,0.,.2);node.odom_history=[(19.,node.odom_pose)]
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is True and node.motion_evidence.started==20.
    _advance(monkeypatch,monitor,clock,23.1)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_new_amcl_pose_resets_the_update_wait(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_reset_'+mode)
    clock={'t':20.}
    _advance(monkeypatch,monitor,clock,20.)
    node,states,cancels,_=_amcl_health_fixture(monitor,tmp_path,mode,.01,match_time=10.)
    node.odom_pose=(.30,0.,0.);node.odom_history=[(19.,node.odom_pose)]
    monitor.LocalizationMonitor._poll_amcl(node)
    assert node.motion_evidence.started==20. and node.motion_evidence.lost is False
    _advance(monkeypatch,monitor,clock,22.)
    node.match_time=22.;node.match_odom_pose=node.odom_pose
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is True and node.motion_evidence.lost is False
    assert node.motion_evidence.started is None and node.motion_evidence.match==22.
    assert cancels==[]


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_stale_scan_still_cancels_below_the_amcl_threshold(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_scan_'+mode)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    node,states,cancels,warnings=_amcl_health_fixture(monitor,tmp_path,mode,.01,match_time=10.)
    node.odom_pose=(.10,0.,0.);node.odom_history=[(19.,node.odom_pose)]
    node._fresh=lambda key:key!='scan'
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and states[-1]['sensors_fresh'] is False
    assert node.motion_evidence.lost is True and len(cancels)==1 and warnings==[]


def test_short_goal_profile_yaw_tolerance_does_not_change_other_profiles():
    import yaml
    short=json.loads((HERE/'map_contract_simple_four_wall_nohit_v1_oracle_short_goal.json').read_text())
    other=json.loads((HERE/'map_contract_simple_four_wall_nohit_v1_oracle_localization.json').read_text())
    source=yaml.safe_load((HERE.parents[1]/'cleannav_navigation/config/nav2_params_ackermann.yaml').read_text())
    default=source['controller_server']['ros__parameters']['general_goal_checker']
    assert short['navigation_goal_tolerance']=={'xy_goal_tolerance_m':0.25,'yaw_goal_tolerance_rad':0.40}
    assert 'navigation_goal_tolerance' not in other
    assert default['xy_goal_tolerance']==0.25 and default['yaw_goal_tolerance']==0.25


@pytest.mark.parametrize('mode,failed',[['oracle','_tf'],['oracle','_scan_tf'],['wheel','_tf'],['wheel','_scan_tf']])
def test_amcl_invalid_tf_still_cancels_navigation(monkeypatch,tmp_path,mode,failed):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_tf_stop_'+mode+failed)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    node,states,cancels,warnings=_amcl_health_fixture(
        monitor,tmp_path,mode,.01,map_tf=failed!='_tf',scan_tf=failed!='_scan_tf')
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1 and warnings==[]
    assert states[-1]['map_tf_valid'] is (failed!='_tf')
    assert states[-1]['scan_tf_valid'] is (failed!='_scan_tf')


@pytest.mark.parametrize('mode',['oracle','wheel'])
@pytest.mark.parametrize('covariance',[math.nan,math.inf,-.01])
def test_amcl_nonfinite_or_negative_covariance_stays_invalid(monkeypatch,tmp_path,mode,covariance):
    label='nan' if math.isnan(covariance) else 'inf' if math.isinf(covariance) else 'negative'
    monitor=integration.load_module(HERE/'localization_status_monitor.py',
        'amcl_bad_covariance_'+mode+'_'+label)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    node,states,cancels,warnings=_amcl_health_fixture(monitor,tmp_path,mode,covariance)
    monitor.LocalizationMonitor._poll_amcl(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1 and warnings==[]
    assert states[-1]['covariance_above_limit'] is False


def _covariance_message(xx, yy, stamp_sec):
    covariance=[0.]*36
    covariance[0]=xx
    covariance[7]=yy
    covariance[35]=0.01
    return N(header=N(frame_id='map',stamp=N(sec=stamp_sec,nanosec=0)),
             pose=N(pose=N(position=N(x=0.,y=0.,z=0.),
                           orientation=N(x=0.,y=0.,z=0.,w=1.)),covariance=covariance))


def _fresh_amcl_node(monitor, tmp_path, mode):
    return _amcl_health_fixture(monitor, tmp_path, mode, None, pose_count=0, pose_valid=False,
                                stamp_ns=0, had_localization=False)


INVALID_AMCL_COVARIANCE=(
    ('xx_negative',-0.1,0.2),
    ('yy_negative',0.2,-0.1),
    ('both_negative',-0.1,-0.2),
    ('xx_nan',math.nan,0.2),
    ('yy_nan',0.2,math.nan),
    ('xx_inf',math.inf,0.2),
    ('yy_inf',0.2,math.inf),
)


@pytest.mark.parametrize('mode',['oracle','wheel'])
@pytest.mark.parametrize('label,xx,yy',INVALID_AMCL_COVARIANCE)
def test_amcl_pose_callback_rejects_invalid_covariance_and_cancels_after_fix(monkeypatch,tmp_path,mode,label,xx,yy):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_pose_callback_'+mode+'_'+label)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    callback=monitor.LocalizationMonitor.amcl_pose
    poll=monitor.LocalizationMonitor._poll_amcl
    node,states,cancels,warnings=_fresh_amcl_node(monitor,tmp_path,mode)
    callback(node,_covariance_message(xx,yy,10))
    assert node.amcl_pose_valid is False and node.amcl_covariance is None and node.amcl_pose_count==0
    poll(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is False
    assert cancels==[] and warnings==[]
    node,states,cancels,warnings=_fresh_amcl_node(monitor,tmp_path,mode)
    callback(node,_covariance_message(0.01,0.02,10))
    assert node.amcl_pose_valid is True and node.amcl_covariance==pytest.approx(0.02)
    poll(node)
    assert states[-1]['localized'] is True and node.motion_evidence.lost is False and cancels==[]
    callback(node,_covariance_message(xx,yy,11))
    assert node.amcl_pose_valid is False and node.amcl_covariance==pytest.approx(0.02)
    poll(node)
    assert states[-1]['localized'] is False and node.motion_evidence.lost is True
    assert len(cancels)==1 and warnings==[]


@pytest.mark.parametrize('mode',['oracle','wheel'])
def test_amcl_pose_callback_over_reference_warns_once(monkeypatch,tmp_path,mode):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_pose_callback_warn_'+mode)
    monkeypatch.setattr(monitor.time,'monotonic',lambda:20.)
    callback=monitor.LocalizationMonitor.amcl_pose
    poll=monitor.LocalizationMonitor._poll_amcl
    node,states,cancels,warnings=_fresh_amcl_node(monitor,tmp_path,mode)
    callback(node,_covariance_message(0.262,0.188,10))
    assert node.amcl_pose_valid is True and node.amcl_covariance==pytest.approx(0.262)
    poll(node)
    assert states[-1]['localized'] is True and node.motion_evidence.lost is False and cancels==[]
    assert len(warnings)==1 and '0.262000' in warnings[0] and '10000000000' in warnings[0]
    callback(node,_covariance_message(0.30,0.20,11))
    poll(node)
    callback(node,_covariance_message(0.40,0.21,12))
    poll(node)
    assert len(warnings)==1 and states[-1]['localized'] is True
    assert node.amcl_covariance==pytest.approx(0.40) and node.motion_evidence.lost is False
    assert cancels==[] and states[-1]['covariance_above_limit'] is True


def _adapter_method(name):
    tree=ast.parse((HERE/'short_goal_rviz_adapter.py').read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='ShortGoalAdapter')
    return next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name==name)


def test_actual_single_goal_callback_submits_once_through_mission_manager():
    class Request:
        SOURCE_RVIZ=1;REJECT_IF_BUSY=0
        def __init__(self):self.valid_for=N(sec=0)
    calls=[];events=[]
    q=N(x=0.,y=0.,z=0.,w=1.);p=N(x=1.,y=0.,z=0.)
    message=N(header=N(frame_id='map',stamp=N(sec=1,nanosec=0)),pose=N(position=p,orientation=q))
    node=N(goal_seen=False,service_ready_reported=True,args=N(run_id='static-fixture',authorization='unused',scope='unused',lifecycle='unused',execution_id='ros-execution-1'),
        admission_path=Path('/unused'),safety=dict(block_all=False,estop=False,enabled=False,owner='',monotonic_ns=1_000_000_000),
        get_publishers_info_by_topic=lambda t:[N(node_name='rviz',node_namespace='/',topic_type='geometry_msgs/msg/PoseStamped',endpoint_gid=[1])],
        emit=lambda name,**kw:events.append((name,kw)),stop_active_task=lambda r:events.append(('STOP',r)),
        buffer=N(lookup_transform=lambda *a:N(header=N(stamp=N(sec=1,nanosec=0)),transform=N(translation=N(x=0.,y=0.),rotation=q))),
        get_clock=lambda:N(now=lambda:N(nanoseconds=1_100_000_000)),map=None,
        client=N(service_is_ready=lambda:True,call_async=lambda r:calls.append(r)))
    scope=dict(Path=Path,time=N(monotonic_ns=lambda:1_100_000_000,monotonic=lambda:1.1),
        uuid=N(uuid4=lambda:N(hex='fixture')),INPUT='/cleannav/rviz/goal_pose',SERVICE='/cleannav/mission/submit_navigation_goal',EXPECTED_RVIZ_NODE='rviz',
        validate_motion_admission=lambda **kw:None,validate_authority_files=lambda **kw:({'status':'ACTIVE'},{'mission_execution_id':'ros-execution-1'}),
        validate_lifecycle=lambda **kw:{'accepted_goal_count':0,'status':'AUTHORIZED'},
        Time=lambda:None,_yaw=lambda q:0.,validate_forward_goal=lambda **kw:{'distance_m':1.},
        _known_free_neighborhood=lambda *a:True,SubmitNavigationGoal=N(Request=Request),MAX_GOAL_RUNTIME_S=45.,TaskStatus=N(STATE_FAILED=15))
    exec(compile(ast.Module(body=[_adapter_method('on_goal')],type_ignores=[]),'<actual-mission-submit>','exec'),scope)
    scope['on_goal'](node,message)
    assert len(calls)==1 and calls[0].goal is message and calls[0].source==Request.SOURCE_RVIZ
    assert calls[0].busy_policy==Request.REJECT_IF_BUSY and calls[0].command_id.startswith('rviz-short-')
    scope['on_goal'](node,message)
    assert len(calls)==1 and events[-1][0]=='STOP'


@pytest.mark.parametrize('terminal',[13,14,15,16])
def test_actual_task_status_preserves_mission_terminal_identity(terminal):
    seen=[];status=N(STATE_SUCCEEDED=13,STATE_CANCELED=14,STATE_FAILED=15,STATE_EMERGENCY_STOPPED=16)
    node=N(command_id='one',task_id=1,args=N(execution_id='ros-execution-1'),
        emit=lambda *a,**kw:None,maybe_finish=lambda:seen.append('finish'),stop_active_task=lambda r:pytest.fail(r))
    msg=N(command_id='one',task_id=1,execution_id='ros-execution-1',status_scope=1,state=terminal,
        reason_code=0,progress=1.,message='terminal')
    scope=dict(TaskStatus=status,time=N(monotonic=lambda:1.))
    exec(compile(ast.Module(body=[_adapter_method('on_status')],type_ignores=[]),'<actual-mission-status>','exec'),scope)
    scope['on_status'](node,msg)
    assert node.terminal_state==terminal and node.result_ok is (terminal==13) and seen==['finish']


def test_actual_amcl_parameter_response_is_required():
    monitor=integration.load_module(HERE/'localization_status_monitor.py','amcl_readback_test')
    expected=dict(use_sim_time=True,transform_tolerance=0.,scan_topic='/mapping/scan',global_frame_id='map',
        odom_frame_id='odom',base_frame_id='base_footprint',tf_broadcast=True,scan_tf_wait_timeout=.3,update_min_d=.25,update_min_a=.2)
    node=N(session={'amcl_overrides':expected,'amcl_expected_parameters':expected})
    values=[]
    for v in expected.values():
        values.append(N(type=1 if isinstance(v,bool) else 3 if isinstance(v,float) else 4,
            bool_value=v if isinstance(v,bool) else False,double_value=v if isinstance(v,float) else 0.,string_value=v if isinstance(v,str) else ''))
    read=monitor.LocalizationMonitor.amcl_native
    read(node,None);assert node.native_ok is False
    read(node,N(values=values));assert node.native_ok is True
    assert node.amcl_parameters==expected and node.linear_update==.25
    values[7].double_value=0.
    read(node,N(values=values));assert node.native_ok is False


@pytest.mark.parametrize('child,accepted',[('base_footprint',True),('ackermann_rear_axle',False),('wrong',False)])
def test_existing_monitor_oracle_odom_input_uses_footprint(child,accepted):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','oracle_probe_odom_test')
    marked=[]
    node=N(amcl_mode=True,odometry_mode='oracle',_mark=marked.append,_fresh=lambda key:True,odom_history=[])
    message=N(header=N(frame_id='odom'),child_frame_id=child,pose=N(pose=N(
        position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))))
    monitor.LocalizationMonitor._odom(node,message)
    assert bool(marked) is accepted and bool(node.odom_history) is accepted


@pytest.mark.parametrize('mode,child,expected',[('oracle','base_footprint',True),('oracle','ackermann_rear_axle',False),('wheel','ackermann_rear_axle',True),('wheel','base_footprint',False)])
def test_actual_guard_odom_contract_matches_selected_source(mode,child,expected):
    tree=ast.parse((HERE/'navigation_safety_guard.py').read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='on_odom')
    observed=[]
    node=N(inputs=N(observe=lambda *a,**kw:observed.append(kw['valid']) or kw['valid']),
        last_position=None,distance=0.,mark=lambda key:None)
    msg=N(header=N(frame_id='odom',stamp=N(sec=1,nanosec=0)),child_frame_id=child,
        pose=N(pose=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))))
    scope=dict(args=N(odometry_mode=mode,diagnostic_continue_on_readiness_loss=False),math=math,time=N(monotonic=lambda:1.))
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual-guard-odom>','exec'),scope)
    scope['on_odom'](node,msg)
    assert observed==[expected]


@pytest.mark.parametrize('variance,accepted',[(-5.4214410738495644e-12,True),(-1e-6,False),(.01,True),(math.nan,False)])
def test_pf_moment_roundoff_is_bounded_without_changing_raw_covariance(variance,accepted):
    monitor=integration.load_module(HERE/'localization_status_monitor.py','covariance_roundoff_test')
    c=[0.]*36;c[0]=-3.883646063160967e-19;c[7]=variance;c[35]=4.085620730620618e-14
    raw=c[:]
    msg=N(header=N(frame_id='map',stamp=N(sec=17,nanosec=426000000)),pose=N(pose=N(
        position=N(x=.0015082397731022486,y=-7.996162414551498,z=0.),
        orientation=N(x=0.,y=0.,z=.006576420831543448,w=.9999783751107053)),covariance=c))
    node=N(session={'amcl_overrides':{'max_particles':2000}},amcl_stamp_ns=0,amcl_pose_count=0,
        odom_pose=(0.,0.,0.))
    monitor.LocalizationMonitor.amcl_pose(node,msg)
    assert node.amcl_pose_valid is accepted
    assert all((a==b or (math.isnan(a) and math.isnan(b))) for a,b in zip(raw,c))
    if accepted:
        assert node.amcl_covariance>=0 and node.amcl_covariance_roundoff_bound<2e-10
        assert node.amcl_covariance_raw==[raw[0],raw[7],raw[35]]


@pytest.mark.parametrize('change,expected',[({},True),({'frame':'wrong'},False),({'data':[-1]*4},False),({'data':[0,0]},False),({'resolution':math.nan},False),({'data':[0,0,101,0]},False)])
def test_current_costmap_message_checks(change,expected):
    from guard_policy import grid_message_valid
    msg=N(header=N(frame_id=change.get('frame','map')),info=N(width=2,height=2,resolution=change.get('resolution',.05),
        origin=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))),data=change.get('data',[0,0,100,-1]))
    assert grid_message_valid(msg,'map') is expected


def test_truth_corridor_uses_actual_path_and_existing_inflation():
    from guard_policy import point_path_distance
    assert point_path_distance((.5,.1),[(0.,0.),(1.,0.)])==pytest.approx(.1)
    assert point_path_distance((1.5,0.),[(0.,0.),(1.,0.)])==pytest.approx(.5)
    assert point_path_distance((0.,0.),[])==math.inf


ENVELOPE_FIELDS=('event','run_id','monotonic_ns','wall_time_ns')
SAFETY_BUSINESS_FIELDS=('source','previous_owner','owner','block_all','estop','autonomous_enabled')


def _safety_status(owner,*,block_all=False,estop=False,enabled=False):
    return N(lease_owner_execution_id=owner,block_all=block_all,
             emergency_stop_active=estop,autonomous_enabled=enabled)


def _live_safety_adapter(tmp_path):
    """Bind the real emit and on_safety methods. No rclpy.init and no Run files."""
    import types
    ticks={'n':0}
    class Clock:
        def monotonic_ns(self):
            ticks['n']+=1
            return 1_000_000_000*ticks['n']
        def monotonic(self):
            return ticks['n']
        def time_ns(self):
            return 9_000_000_000_000_000_000
    events_path=tmp_path/'events.jsonl'
    node=N(args=N(run_id='static-safety-fixture',execution_id='mission-exec',
                  events_path=str(events_path),scope=str(tmp_path/'scope.json'),
                  lifecycle=str(tmp_path/'lifecycle.json'),
                  authorization=str(tmp_path/'authorization.json')),
           safety=None,safety_owner_history=[],lease_seen=False,lease_released=False,
           terminal_state=None,terminal=False,result_ok=False,command_id='',
           execution_id='',task_id=None,
           get_logger=lambda:N(info=lambda _text:None),
           events=N(publish=lambda _msg:None),
           estop=N(publish=lambda _msg:None),
           cancel=N(service_is_ready=lambda:False))
    methods=[_adapter_method(name) for name in ('emit','on_safety','maybe_finish','stop_active_task')]
    scope=dict(time=Clock(),json=json,os=os,Path=Path,
               String=lambda **kw:N(**kw),Bool=lambda **kw:kw,
               CancelGoal=N(Request=lambda:'cancel'))
    exec(compile(ast.Module(body=methods,type_ignores=[]),'<actual-safety-callback>','exec'),scope)
    for name in ('emit','on_safety','maybe_finish','stop_active_task'):
        setattr(node,name,types.MethodType(scope[name],node))
    return node,events_path


def _event_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def test_first_safety_status_callback_emits_one_envelope(tmp_path):
    node,path=_live_safety_adapter(tmp_path)
    node.on_safety(_safety_status(''))
    rows=_event_rows(path)
    assert [row['event'] for row in rows]==['SAFETY_LEASE_OWNER_TRANSITION']
    event=rows[0]
    assert event['run_id']==node.args.run_id
    assert event['source']=='/cleannav/safety_status'
    assert event['previous_owner'] is None and event['owner']==''
    assert event['block_all'] is False and event['estop'] is False
    assert event['autonomous_enabled'] is False
    history=node.safety_owner_history
    assert len(history)==1
    assert history[0]['monotonic_ns']==1_000_000_000
    assert event['monotonic_ns']==2_000_000_000
    assert event['monotonic_ns']!=history[0]['monotonic_ns']
    text=path.read_text().splitlines()[0]
    for key in ('run_id','monotonic_ns'):
        assert text.count(f'"{key}"')==1


def test_safety_owner_change_keeps_original_history_timestamp(tmp_path):
    node,path=_live_safety_adapter(tmp_path)
    node.on_safety(_safety_status(''))
    original=dict(node.safety_owner_history[0])
    node.on_safety(_safety_status('other-owner',block_all=True,enabled=True))
    assert node.safety_owner_history[0]['monotonic_ns']==original['monotonic_ns']
    assert node.safety_owner_history[0]['run_id']==original['run_id']
    assert node.safety_owner_history[0]['owner']==''
    change=node.safety_owner_history[1]
    assert change['previous_owner']=='' and change['owner']=='other-owner'
    assert change['monotonic_ns']==3_000_000_000
    assert change['block_all'] is True and change['autonomous_enabled'] is True
    transitions=[row for row in _event_rows(path) if row['event']=='SAFETY_LEASE_OWNER_TRANSITION']
    assert len(transitions)==2
    assert transitions[1]['previous_owner']=='' and transitions[1]['owner']=='other-owner'
    assert transitions[1]['monotonic_ns']!=change['monotonic_ns']
    assert transitions[1]['run_id']==node.args.run_id
    kinds=[row['event'] for row in _event_rows(path)]
    assert 'LEASE_OWNER_MISMATCH' in kinds and 'SUPERVISED_STOP_REQUESTED' in kinds


def test_repeated_safety_status_emits_only_on_owner_change(tmp_path):
    node,path=_live_safety_adapter(tmp_path)
    idle=_safety_status('')
    node.on_safety(idle)
    node.on_safety(idle)
    node.on_safety(idle)
    assert len(node.safety_owner_history)==1
    assert [row['event'] for row in _event_rows(path)]==['SAFETY_LEASE_OWNER_TRANSITION']
    preserved=node.safety_owner_history[0]['monotonic_ns']
    node.on_safety(_safety_status('owner-b'))
    node.on_safety(_safety_status('owner-b',estop=False))
    assert len(node.safety_owner_history)==2
    assert node.safety_owner_history[0]['monotonic_ns']==preserved
    transitions=[row for row in _event_rows(path) if row['event']=='SAFETY_LEASE_OWNER_TRANSITION']
    assert len(transitions)==2
    assert transitions[1]['owner']=='owner-b' and transitions[1]['previous_owner']==''
    for text in path.read_text().splitlines():
        row=json.loads(text)
        assert row['run_id']==node.args.run_id
        assert isinstance(row['monotonic_ns'],int)
        if row['event']=='SAFETY_LEASE_OWNER_TRANSITION':
            for key in ENVELOPE_FIELDS:
                assert text.count(f'"{key}"')==1
        elif 'safety' in row:
            assert row['safety']['monotonic_ns']!=row['monotonic_ns']


def test_safety_transition_event_schema_has_single_envelope(tmp_path):
    node,path=_live_safety_adapter(tmp_path)
    node.on_safety(_safety_status(''))
    text=path.read_text().splitlines()[0]
    event=json.loads(text)
    assert set(event)==set(ENVELOPE_FIELDS)|set(SAFETY_BUSINESS_FIELDS)
    assert event['run_id']=='static-safety-fixture'
    assert isinstance(event['monotonic_ns'],int) and event['monotonic_ns']>0
    assert isinstance(event['wall_time_ns'],int) and event['wall_time_ns']>0
    for key in ENVELOPE_FIELDS:
        assert text.count(f'"{key}"')==1
    assert node.safety_owner_history[0]['monotonic_ns'] not in (None,event['monotonic_ns'])


def test_adapter_emit_calls_do_not_repeat_envelope_keywords():
    tree=ast.parse((HERE/'short_goal_rviz_adapter.py').read_text())
    starred=[]
    for node in ast.walk(tree):
        if not isinstance(node,ast.Call) or not isinstance(node.func,ast.Attribute):
            continue
        if node.func.attr!='emit':
            continue
        names={kw.arg for kw in node.keywords if kw.arg is not None}
        assert names.isdisjoint(ENVELOPE_FIELDS)
        starred.extend(kw for kw in node.keywords if kw.arg is None)
    assert len(starred)==1
    bound=ast.get_source_segment((HERE/'short_goal_rviz_adapter.py').read_text(),starred[0].value)
    assert bound=='bound'
