"""Static composition/contract regressions: no launch execution or ROS init."""
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.utilities import perform_substitutions

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import integration
from guard_policy import command_error,missing_inputs


def adapter():
    return integration.load_module(HERE/'rviz_navigation.launch.py','rviz_static_composition_test')


def defaults(module, extras):
    context=LaunchContext()
    for declaration in module.generate_launch_description().entities:
        if isinstance(declaration,DeclareLaunchArgument) and declaration.default_value is not None:
            context.launch_configurations[declaration.name]=perform_substitutions(context,declaration.default_value)
    context.launch_configurations.update(extras)
    return context


def ini_values(path):
    values={}
    for line in Path(path).read_text().splitlines():
        if '=' not in line or line.lstrip().startswith(('#',';')):
            continue
        key,value=line.split('=',1)
        values[key.strip().replace('\\','/')]=value.strip()
    return values


def ini_key_lines(path):
    """Return every normalized key line in order so duplicates stay visible."""
    keys=[]
    for line in Path(path).read_text().splitlines():
        if '=' not in line or line.lstrip().startswith(('#',';')):
            continue
        keys.append(line.split('=',1)[0].strip().replace('\\','/'))
    return keys


def test_entry_defaults_have_no_motion_or_automatic_task():
    c=defaults(adapter(),{})
    for name in ['enable_motion','auto_start_task','gui']:
        assert c.launch_configurations[name]=='false'
    assert c.launch_configurations['ros_domain_id']=='204'
    assert c.launch_configurations['gazebo_port']=='11704'
    assert c.launch_configurations['map_profile']=='roof-lidar-360'
    assert c.launch_configurations['odometry_mode']=='wheel'
    declaration=next(x for x in adapter().generate_launch_description().entities
                     if isinstance(x,DeclareLaunchArgument) and x.name=='odometry_mode')
    assert declaration.choices==['wheel','oracle']
    assert 'world' not in c.launch_configurations
    assert 'map_directory' not in c.launch_configurations


def test_map_identity_and_explicit_scan_binding(tmp_path):
    d=tmp_path/'saved';d.mkdir();(d/'map.pgm').write_bytes(b'map')
    world=tmp_path/'world';world.write_bytes(b'world')
    c=dict(map_id='saved',files={'map.pgm':integration.digest(d/'map.pgm')},world_sha256=integration.digest(world),scan_topic='/mapping/scan',odom_frame_id='odom')
    p=tmp_path/'contract.json';p.write_text(json.dumps(c))
    assert integration.verify_map(d,world,p)['scan_topic']=='/mapping/scan'
    (d/'map.pgm').write_bytes(b'changed')
    with pytest.raises(ValueError,match='map asset identity'):integration.verify_map(d,world,p)


def test_historical_motion_speed_envelope_is_mode_specific():
    base=json.loads((HERE/'short_goal_profile.json').read_text())
    original=json.loads(json.dumps(base))
    historical=integration.motion_profile_for_mode(base,'historical')
    assert historical['max_forward_linear_x']==.5
    assert historical['max_angular_z']==pytest.approx(.5/1.12)
    assert historical['mppi']['vx_max']==.5
    assert historical['mppi']['wz_max']==pytest.approx(.5/1.12)
    assert historical['safety']['max_forward_linear_x']==.5
    assert historical['safety']['max_angular_z']==pytest.approx(.5/1.12)
    assert historical['vehicle']==original['vehicle']
    assert historical['vehicle']['min_turning_radius_m']==original['vehicle']['min_turning_radius_m']
    assert historical['max_reverse_linear_x']==0.
    assert integration.motion_profile_for_mode(base,'short-goal')==original
    assert integration.motion_profile_for_mode(base,'none')==original
    assert base==original


def test_wrong_world_fails_before_private_copy(tmp_path):
    d=tmp_path/'saved';d.mkdir();world=tmp_path/'world';world.write_text('wrong')
    c=tmp_path/'contract';c.write_text(json.dumps(dict(map_id='saved',files={},world_sha256='0'*64,scan_topic='/mapping/scan')))
    with pytest.raises(ValueError,match='world differs'):integration.verify_map(d,world,c)


@pytest.mark.parametrize('value',[None,'','iron'])
def test_wrong_ros_distro_fails_closed(value):
    with pytest.raises(ValueError,match='ROS_DISTRO'):integration.verify_packages(lambda p:'/not-used',{'ROS_DISTRO':value})


def test_missing_expected_prefix_fails_closed():
    with pytest.raises(ValueError,match='prefix mismatch'):integration.verify_packages(lambda p:'/unexpected',{'ROS_DISTRO':'humble'})


def test_ros_runtime_packages_must_resolve_from_humble_base():
    with pytest.raises(ValueError,match='prefix mismatch'):
        integration.verify_packages(lambda p:'/unapproved/overlay',{'ROS_DISTRO':'humble'})


def test_runtime_source_manifest_matches_integrated_entry():
    manifest=integration.verify_source_manifest()
    assert 'tools/rviz_navigation/rviz_navigation.launch.py' in manifest['files']
    assert 'tools/rviz_navigation/oracle_odometry.py' in manifest['files']
    assert 'tools/rviz_navigation/tests/test_oracle_odometry.py' in manifest['files']
    assert 'cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro' in manifest['files']


def test_source_manifest_rejects_changed_hash_and_escaping_path(tmp_path):
    changed=tmp_path/'changed.json'
    changed.write_text(json.dumps({'files':{'tools/rviz_navigation/rviz_navigation.launch.py':'0'*64}}))
    with pytest.raises(ValueError,match='identity mismatch'):
        integration.verify_source_manifest(changed)
    changed.write_text(json.dumps({'files':{'../outside':'0'*64}}))
    with pytest.raises(ValueError,match='escapes worktree'):
        integration.verify_source_manifest(changed)


def test_mcity_localization07_profile_binds_map_world_prior_and_runtime_sources():
    profile=json.loads(integration.contract_path_for_profile('mcity-localization07').read_text())
    map_dir=integration.WORKSPACE/profile['map_directory_relative']
    world=integration.WORKSPACE/profile['world_relative']
    verified=integration.verify_map(map_dir,world,profile='mcity-localization07')
    assert verified['scan_topic']=='/scan' and verified['scan_frame']=='base_scan'
    assert verified['odom_mode']=='topic' and verified['odom_frame_id']==''
    assert verified['odom_topic']=='/odom'
    assert verified['localization_initial_pose']=='0.3 0 0 0 0 0'
    assert verified['spawn']=={'x':'0.0','y':'-5.0','z':'0.02','yaw':'0.0'}
    assert set(verified['source_bindings']['expected_prefix_packages'])==set(json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())['cleannav_packages'])
    assert integration.verify_profile_source_bindings(verified)
    assert verified['startup_order']==[
        'safety_blocked','gazebo_vehicle','collision_monitor_active',
        'disk_map_and_rtabmap_localization','rviz_initialization_ui','nav2',
        'mission_manager','goal_adapter']
    bad_binding=dict(verified,source_bindings=dict(verified['source_bindings']))
    bad_binding['source_bindings']['simulation_model_xacro']=dict(
        bad_binding['source_bindings']['simulation_model_xacro'],sha256='0'*64)
    with pytest.raises(ValueError,match='source identity mismatch'):
        integration.verify_profile_source_bindings(bad_binding)


def test_canonical_mcity_debug_profile_binds_derivative_and_preserves_localization_inputs():
    profile_name='mcity-localization07-canonical-debug'
    profile=json.loads(integration.contract_path_for_profile(profile_name).read_text())
    map_dir=integration.WORKSPACE/profile['map_directory_relative']
    world=integration.WORKSPACE/profile['world_relative']
    verified=integration.verify_map(map_dir,world,profile=profile_name)
    assert verified['scan_topic']=='/scan' and verified['scan_frame']=='base_scan'
    assert verified['safety_scan_topic']=='/scan'
    assert verified['roof_mapping_topics']==['/mapping/scan_raw','/mapping/scan']
    assert verified['localization_initial_pose']=='0.3 0 0 0 0 0'
    assert verified['spawn']=={'x':'0.0','y':'-5.0','z':'0.02','yaw':'0.0'}
    assert verified['world_sha256']==integration.digest(world)
    assert verified['activation_status']=='READY_FOR_STATIC_ALLOWLISTED_VALIDATION'


def test_simple_four_wall_nohit_profile_binds_bundle_geometry_pose_and_wheel_mode():
    profile_name='simple-four-wall-nohit-v1'
    profile=json.loads(integration.contract_path_for_profile(profile_name).read_text())
    map_dir=integration.WORKSPACE/profile['map_directory_relative']
    world=integration.WORKSPACE/profile['world_relative']
    verified=integration.verify_map(map_dir,world,profile=profile_name,odometry_mode='wheel')
    assert verified['map_id']=='simple_four_wall_nohit_v1'
    assert verified['frame_id']=='base_footprint'
    assert verified['scan_topic']=='/mapping/scan'
    assert verified['scan_frame']=='mapping_lidar_link'
    assert verified['odom_mode']=='tf' and verified['odom_frame_id']=='odom'
    assert verified['odom_topic']=='/odom' and verified['required_odometry_mode']=='wheel'
    assert verified['localization_initial_pose']=='0.0015082397731021 -7.99616241455078 0 0 0 0.0131529364734888'
    assert verified['map_geometry']['width']==653 and verified['map_geometry']['height']==653
    assert verified['map_geometry']['resolution']==pytest.approx(0.05000000074505806)
    assert 'patrol' not in verified
    localization=verified['localization_contract']
    assert localization['database_sha256']==verified['files']['rtabmap.db']
    assert localization['params_sha256']==verified['files']['rtabmap_params.yaml']
    assert localization['map_to_odom']['publisher']=='rtabmap localization'
    resolved=integration.verify_profile_source_bindings(verified)
    assert resolved['optimized_pose_source'].endswith('/work/dump-b/optimized_poses.csv')
    with pytest.raises(ValueError,match='requested odometry source conflicts'):
        integration.verify_map(map_dir,world,profile=profile_name,odometry_mode='oracle')
    with pytest.raises(ValueError,match='requested odometry source conflicts'):
        integration.verify_map(map_dir,world,profile=profile_name)


def test_map_profile_selector_is_allow_listed():
    with pytest.raises(ValueError,match='unsupported RViz map profile'):
        integration.contract_path_for_profile('../map_contract')
    with pytest.raises(ValueError,match='unsupported RViz map profile'):
        integration.verify_map('/missing','/missing',profile='unreviewed')


def test_map_bound_native_pose_cannot_be_replaced():
    contract=json.loads(integration.contract_path_for_profile('mcity-localization07').read_text())
    assert integration.profile_initial_pose(contract,'')=='0.3 0 0 0 0 0'
    with pytest.raises(ValueError,match='conflicts with the selected map profile'):
        integration.profile_initial_pose(contract,'0.4 0 0 0 0 0')


@pytest.mark.parametrize("historical_motion", [False, True])
@pytest.mark.parametrize("odometry_mode", ["wheel", "oracle"])
def test_all_actual_include_arguments_are_declared_and_model_is_source_bound(
        monkeypatch,tmp_path,historical_motion,odometry_mode):
    data=os.environ.get('CLEANNAV_STATIC_PREFIXES_JSON')
    if not data:pytest.skip('explicit installed-prefix fixture required for local integration')
    prefixes=json.loads(Path(data).read_text())
    for pkg,b in json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())['cleannav_packages'].items():
        monkeypatch.setenv(b['prefix_env'],prefixes[pkg])
    monkeypatch.setenv('ROS_DISTRO','humble')
    module=adapter()
    # Explicit audited packages are pinned; other third-party packages resolve
    # through the selected, read-only AMENT_PREFIX_PATH fixture.
    import ament_index_python.packages as ament
    original_resolver=ament.get_package_prefix
    original_share_resolver=ament.get_package_share_directory
    def resolve_prefix(package):
        return prefixes[package] if package in prefixes else original_resolver(package)
    def resolve_share(package):
        return (str(Path(prefixes[package])/'share'/package) if package in prefixes
                else original_share_resolver(package))
    monkeypatch.setattr(module,'get_package_prefix',resolve_prefix)
    monkeypatch.setattr(ament,'get_package_prefix',resolve_prefix)
    monkeypatch.setattr(ament,'get_package_share_directory',resolve_share)
    monkeypatch.setattr(module.base,'get_package_share_directory',resolve_share)
    monkeypatch.setattr(module.launch_config,'log_dir',str(tmp_path))
    includes=[];nodes=[];processes=[]
    original=module.base.IncludeLaunchDescription
    def include(*a,**kw):
        child=original(*a,**kw);includes.append(child);return child
    monkeypatch.setattr(module.base,'IncludeLaunchDescription',include)
    def record_node(factory):
        def record(**kw):
            node=factory(**kw);nodes.append((node,kw));return node
        return record
    monkeypatch.setattr(module.base,'Node',record_node(module.base.Node))
    monkeypatch.setattr(module,'Node',record_node(module.Node))
    proc=module.ExecuteProcess
    def record_proc(**kw):
        item=proc(**kw);processes.append((item,kw));return item
    monkeypatch.setattr(module,'ExecuteProcess',record_proc)
    # The readiness probes and the monitor run the source monitor from the
    # full-simulation composition, so its ExecuteProcess is recorded too.
    monkeypatch.setattr(module.base,'ExecuteProcess',record_proc)
    parent_integrations=[]
    parent_compose=module.base._compose
    def capture_parent_compose(ctx,integration=None):
        parent_integrations.append(integration)
        return parent_compose(ctx,integration=integration)
    monkeypatch.setattr(module.base,'_compose',capture_parent_compose)
    world=Path(os.environ['CLEANNAV_STATIC_WORLD_FILE'])
    mapdir=Path(os.environ['CLEANNAV_STATIC_MAP_DIRECTORY'])
    profile=os.environ['CLEANNAV_STATIC_MAP_PROFILE']
    profile_contract=json.loads(integration.contract_path_for_profile(profile).read_text())
    monkeypatch.setenv('DISPLAY', ':static-only-test')
    context=defaults(module,dict(world=str(world),map_directory=str(mapdir),map_profile=profile,start_rviz='true',
        enable_motion='true' if historical_motion else 'false',
        motion_profile='historical' if historical_motion else 'none',
        odometry_mode=odometry_mode))
    actions=module._compose(context)
    assert actions
    assert len(parent_integrations)==1
    expected_forward=.5 if historical_motion else .15
    expected_angular=(.5/1.12) if historical_motion else .13392857142857142
    assert parent_integrations[0]['forward_limit']==str(expected_forward)
    assert parent_integrations[0]['angular_limit']==str(expected_angular)
    source_ini=mapdir/'rtabmap_config.ini'
    private_ini=tmp_path/'localization'/'rtabmap_config.ini'
    source_values=ini_values(source_ini)
    private_values=ini_values(private_ini)
    assert integration.digest(source_ini)==profile_contract['files']['rtabmap_config.ini']
    # The launch rewrites only this run's private INI, and only while motion is
    # enabled for a profile whose saved 0.1 m / 0.1 rad update thresholds would
    # suppress the final proximity scan:
    # ``motion_enabled and map_profile in ('mcity-localization07-canonical-debug',
    # 'patrol-a-v4')`` in rviz_navigation.launch.py. Historical motion
    # additionally lowers the ICP correspondence ratio. Both profiles must show
    # exactly the same three-key delta; any other difference stays a failure.
    motion_update_profiles=('mcity-localization07-canonical-debug','patrol-a-v4')
    if historical_motion and profile in motion_update_profiles:
        assert private_values['RGBD/LinearUpdate']=='0.005'
        assert private_values['RGBD/AngularUpdate']=='0.005'
        assert private_values['Icp/CorrespondenceRatio']=='0.075'
        # RTAB-Map 0.22.1 doubles the ratio for merged proximity scans.
        merged_ratio=2*float(private_values['Icp/CorrespondenceRatio'])
        assert merged_ratio==.15
        assert all(r>merged_ratio for r in (24/126,21/123,20/122))
        assert not .14>merged_ratio
        changed_keys={'RGBD/LinearUpdate','RGBD/AngularUpdate','Icp/CorrespondenceRatio'}
        # Exactly these keys differ from the immutable map asset, each appears
        # once, and nothing else changed: no extra rewrite and no duplicated INI
        # entry may hide behind the shared branch.
        assert {k for k,v in private_values.items() if v!=source_values.get(k)}==changed_keys
        private_key_lines=ini_key_lines(private_ini)
        assert {k:private_key_lines.count(k) for k in changed_keys}=={
            k:1 for k in changed_keys}
        assert {k:v for k,v in private_values.items() if k not in changed_keys}=={
            k:v for k,v in source_values.items() if k not in changed_keys}
    else:
        assert private_ini.read_bytes()==source_ini.read_bytes()
    selected={}
    for child in includes:
        description=child.launch_description_source.get_launch_description(context)
        path=child.launch_description_source.location
        names={x.name for x in description.entities if isinstance(x,DeclareLaunchArgument)}
        args=dict(child.launch_arguments)
        assert set(args)<=names, (path,set(args)-names)
        selected[Path(path).name]=(path,args)
    assert 'startup_smoke_simulation.launch.py' in selected, selected
    path,args=selected['startup_smoke_simulation.launch.py']
    assert Path(path)==integration.NAV/'cleannav_simulation/launch/startup_smoke_simulation.launch.py'
    if odometry_mode == 'oracle':
        runtime = tmp_path/'localization'
        assert Path(args['robot_xacro_file'])==runtime/'oracle_odometry_model.urdf.xacro'
        assert Path(args['controllers_file'])==runtime/'oracle_odometry_controllers.yaml'
        assert '/wheel/odom' in Path(args['robot_xacro_file']).read_text()
        assert '/wheel/tf' in Path(args['robot_xacro_file']).read_text()
        private_controller=yaml.safe_load(Path(args['controllers_file']).read_text())
        assert private_controller['ackermann_controller']['ros__parameters']['enable_odom_tf'] is False
        assert args['publish_rear_axle_tf']=='false'
        assert parent_integrations[0]['odometry_mode']=='oracle'
        assert parent_integrations[0]['oracle_odometry']['identity']['authoritative_chain']=={
            'odometry_topic':'/odom','tf':'odom -> base_footprint',
            'publisher':'gazebo_oracle_odometry'}
        assert 'cleannav_diagnostic_state' in (tmp_path/'diagnostic.world').read_text()
        assert any(p.get('name')=='gazebo_oracle_odometry' for _,p in processes)
    else:
        assert Path(args['robot_xacro_file'])==integration.NAV/'cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro'
        assert 'publish_rear_axle_tf' not in args
    assert args['spawn_y']==profile_contract.get('spawn',{}).get('y',context.launch_configurations['spawn_y'])
    assert args['spawn_delay_sec']==str(profile_contract.get('spawn_delay_sec',context.launch_configurations['spawn_delay_sec']))
    expected_controller=Path(prefixes['cleannav_simulation'])/'share/cleannav_simulation/config/ackermann_controllers.yaml'
    if odometry_mode == 'wheel':
        assert Path(args['controllers_file'])==expected_controller
    assert integration.digest(expected_controller)==json.loads(
        (HERE/'RUNTIME_DEPENDENCIES.json').read_text())['cleannav_packages']['cleannav_simulation']['files'][
            'share/cleannav_simulation/config/ackermann_controllers.yaml']
    assert args['sim_lidar_samples']=='181' and args['sim_lidar_rate']=='8'
    path,args=selected['cleannav_rtabmap_localization.launch.py']
    assert args['localization']=='true' and args['scan_topic']==profile_contract['scan_topic']
    assert args['odom_frame_id']==profile_contract['odom_frame_id']
    assert args['odom_topic']==profile_contract.get('odom_topic','/odom')
    assert args['initial_pose']==profile_contract.get('localization_initial_pose','')
    assert 'spawn_y' not in args and 'spawn_delay_sec' not in args
    path,args=selected['cleannav_ackermann_navigation.launch.py']
    assert args['controller_cmd_vel_topic']=='/cleannav/navigation/cmd_vel_raw'
    assert args['start_safety_supervisor']=='false'
    assert len([n for _,n in nodes if n.get('package')=='nav2_collision_monitor'])==1
    cm=yaml.safe_load((tmp_path/'localization/collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    assert cm['scan']['topic']=='/scan'
    assert cm['cmd_vel_in_topic']=='/cleannav/navigation/cmd_vel_raw'
    assert cm['cmd_vel_out_topic']=='/cleannav/cmd_vel_candidate'
    original_cm=yaml.safe_load((HERE/'collision_monitor.source.yaml').read_text())['collision_monitor']['ros__parameters']
    changed={k for k in cm if cm[k]!=original_cm[k]}
    assert changed=={'cmd_vel_in_topic'}
    # Keep the observed MappingForwardStop protective zone and LaserScan source
    # intact; only the raw command input is rebound for this RViz workflow.
    stop=cm['MappingForwardStop']
    assert stop['enabled'] is True
    assert stop['action_type']=='stop'
    assert stop['points']==[1.30,0.70,1.30,-0.70,0.30,-0.70,0.30,0.70]
    assert stop['max_points']==0
    assert cm['observation_sources']==['scan']
    assert cm['scan']['enabled'] is True and cm['scan']['topic']=='/scan'
    saved=json.loads((tmp_path/'localization/session.json').read_text())
    # The monitor selects its /odom contract from this recorded source.
    assert saved['odometry_mode']==odometry_mode
    assert integration.digest(mapdir/'rtabmap.db')==integration.digest(Path(saved['database_path']))
    params=yaml.safe_load((tmp_path/'localization/nav2_params.yaml').read_text())
    f=params['controller_server']['ros__parameters']['FollowPath']
    assert f['vx_min']==0 and f['vx_max']==expected_forward
    assert f['wz_max']==pytest.approx(expected_angular)
    assert not list(tmp_path.rglob('*authorization*')) and not list(tmp_path.rglob('*scope*'))
    # All one-shot dependencies use real process actions; no execution takes place.
    names={p['name'] for _,p in processes}
    assert {'rviz_collision_startup','rviz_guard_admission','navigation_safety_guard'}<=names
    guard,guard_args=next((p,k) for p,k in processes
                          if k.get('name')=='navigation_safety_guard')
    assert '--localization-scan-topic' in guard_args['cmd']
    topic_arg=guard_args['cmd'].index('--localization-scan-topic')+1
    assert guard_args['cmd'][topic_arg]==profile_contract['scan_topic']
    # Exercise phase callbacks using synthetic process-exit events. This checks
    # the actual closure transitions instead of searching source strings.
    handlers=[a.event_handler for a in actions
              if hasattr(a,'event_handler') and isinstance(a.event_handler,OnProcessExit)]
    from launch.events.process import ProcessExited
    def exits(action,rc,name='test'):
        event=ProcessExited(action=action,name=name,cmd=['never-executed'],cwd=None,env=None,pid=0,returncode=rc)
        return [x for h in handlers if h.matches(event)
                for x in (h.handle(event,context) or [])]
    sim=next(n for n,k in processes if k.get('name')=='cleannav_probe_simulation')
    cm_gate=next(n for n,k in processes if k['name']=='rviz_collision_startup')
    nav=next(n for n,k in processes if k.get('name')=='cleannav_probe_navigation')
    guard_gate=next(n for n,k in processes if k['name']=='rviz_guard_admission')
    manager=next(n for n,k in nodes if k.get('name')=='mission_manager_node')
    rviz=next(n for n,k in nodes if k.get('package')=='rviz2')
    assert next(k['name'] for n,k in nodes if n is rviz)=='rviz'
    cm_process=next(n for n,k in nodes if k.get('package')=='nav2_collision_monitor')
    # Actual critical-process exit closures still emit Shutdown in no-motion.
    assert any(isinstance(x,EmitEvent) for x in exits(cm_process,0,'collision_monitor'))
    assert any(isinstance(x,EmitEvent) for x in exits(cm_process,1,'safety_supervisor_node'))
    if odometry_mode == 'oracle':
        oracle_process=next(n for n,k in processes if k.get('name')=='gazebo_oracle_odometry')
        assert any(isinstance(x,EmitEvent) for x in exits(oracle_process,1,'gazebo_oracle_odometry'))
    assert cm_gate in exits(sim,0)
    assert any(isinstance(x,EmitEvent) for x in exits(cm_gate,1))
    assert not any(isinstance(x,EmitEvent) for x in exits(cm_gate,0))
    assert guard_gate in exits(nav,0)
    assert manager not in exits(nav,0)
    assert manager in exits(guard_gate,0)
    assert any(isinstance(x,EmitEvent) for x in exits(guard_gate,1))
    assert rviz not in actions, 'integrated RViz must not start at root launch admission'
    mission_probe=next(n for n,k in processes if k.get('name')=='cleannav_probe_mission')
    assert rviz in exits(cm_gate,0), 'RViz must be available with disk localization to set its initial pose'
    assert rviz not in exits(cm_gate,1), 'failed CM admission must block GUI initialization'
    assert rviz not in exits(mission_probe,0), 'do not launch a duplicate GUI after Mission readiness'
    assert manager not in exits(cm_gate,0), 'GUI availability does not grant Mission admission'
    # Every integrated readiness probe and the monitor itself run the source
    # monitor, whose odom contract follows the selected odometry source; the
    # installed copy only accepts the wheel contract.
    monitor_kw=next(k for n,k in processes if k.get('name')=='localization_status_monitor')
    assert monitor_kw['cmd'][1]==str(HERE/'localization_status_monitor.py')
    assert ('--enable-motion' in monitor_kw['cmd']) is historical_motion
    for stage in ('simulation','slam','navigation','mission'):
        probe_kw=next(k for n,k in processes if k.get('name')=='cleannav_probe_'+stage)
        assert probe_kw['cmd'][1]==monitor_kw['cmd'][1]
        assert probe_kw['cmd'][4:6]==['--probe',stage]
        assert '--session' in probe_kw['cmd']
    # The RTAB resume guard has the same split: the installed executable only
    # accepts the wheel rear-axle barrier, so Oracle runs the reviewed source.
    if odometry_mode == 'oracle':
        guard_kw=next(k for n,k in processes if k.get('name')=='rviz_rtab_input_guard')
        assert guard_kw['cmd'][1]==str(HERE/'mapping_status_monitor.py')
        assert '--resume-rtab' in guard_kw['cmd']
        assert not any(k.get('executable')=='mapping_status_monitor' for _,k in nodes)
    else:
        guard_node=next(n for n,k in nodes if k.get('executable')=='mapping_status_monitor')
        assert next(k['name'] for n,k in nodes if n is guard_node)=='rviz_rtab_input_guard'
        assert not any(k.get('name')=='rviz_rtab_input_guard' for _,k in processes)
    manager_kw=next(k for n,k in nodes if k.get('name')=='mission_manager_node')
    assert manager_kw['parameters'][0]['pose_motion_enabled'] is historical_motion
    old_adapter=next(n for n,k in nodes if k.get('executable')=='rviz_goal_adapter')
    assert old_adapter in exits(mission_probe,0)
    assert not any(k.get('name')=='cleannav_short_goal_rviz_adapter' for _,k in processes)

    # Replay the actual installed localization probe's output through the
    # real launch IO callback, so the outer supervisor can see its marker.
    from launch.event_handlers import OnProcessIO
    from launch.events.process import ProcessStdout
    from launch.actions import LogInfo
    probe=next(n for n,k in processes if k.get('name')=='cleannav_probe_slam')
    line='[LOCALIZATION] READY native accepted saved-node matches=1'
    event=ProcessStdout(action=probe,name='localization_status_monitor',cmd=['not-executed'],
                        cwd=None,env=None,pid=0,text=(line+'\n').encode())
    forwarded=[x for a in actions if hasattr(a,'event_handler')
               and isinstance(a.event_handler,OnProcessIO) and a.event_handler.matches(event)
               for x in (a.event_handler.handle(event,context) or [])]
    assert any(isinstance(x,LogInfo) and perform_substitutions(context,x.msg)==line
               for x in forwarded)


def test_rviz_initial_pose_and_goal_topics_are_preserved():
    vm=yaml.safe_load((HERE/'navigation.rviz').read_text())['Visualization Manager']
    assert vm['Global Options']['Fixed Frame']=='map'
    tools={x['Class']:x.get('Topic') for x in vm['Tools']}
    assert tools['rviz_default_plugins/SetGoal']=='/cleannav/rviz/goal_pose'
    assert tools['rviz_default_plugins/SetInitialPose']=='/rtabmap/initialpose'
    assert vm['Views']['Current']['Scale']==70


def _display(vm, name):
    matches=[item for item in vm['Displays'] if item.get('Name')==name]
    assert len(matches)==1, name
    return matches[0]


def _qos(topic):
    return {key: topic.get(key) for key in (
        'Value', 'Reliability Policy', 'Durability Policy', 'History Policy', 'Depth')}


def test_oracle_short_goal_rviz_shows_map_scans_and_costmaps_without_changing_wheel():
    wheel=json.loads((HERE/'map_contract_simple_four_wall_nohit_v1.json').read_text())
    short=json.loads((HERE/'map_contract_simple_four_wall_nohit_v1_oracle_short_goal.json').read_text())
    wheel_binding=wheel['source_bindings']['rviz_config']
    short_binding=short['source_bindings']['rviz_config']
    assert wheel_binding['path']=='tools/rviz_navigation/navigation.rviz'
    assert integration.digest(HERE/'navigation.rviz')==wheel_binding['sha256']
    assert short_binding['path']=='tools/rviz_navigation/navigation_oracle_short_goal.rviz'
    short_path=HERE/'navigation_oracle_short_goal.rviz'
    assert integration.digest(short_path)==short_binding['sha256']
    vm=yaml.safe_load(short_path.read_text())['Visualization Manager']
    assert vm['Global Options']['Fixed Frame']=='map'
    frozen=_display(vm, 'Frozen Map')
    assert frozen['Class']=='rviz_default_plugins/Map'
    assert frozen['Enabled'] is True
    assert frozen['Alpha']==pytest.approx(0.7)
    assert frozen['Color Scheme']=='map'
    assert _qos(frozen['Topic'])=={
        'Value': '/cleannav/localization/map', 'Reliability Policy': 'Reliable',
        'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last', 'Depth': 1}
    scan=_display(vm, 'LiDAR')
    mapping=_display(vm, 'Mapping LiDAR')
    for display, topic in ((scan, '/scan'), (mapping, '/mapping/scan')):
        assert display['Class']=='rviz_default_plugins/LaserScan' and display['Enabled'] is True
        assert display['Style']=='Points' and display['Size (Pixels)']==3
        assert _qos(display['Topic'])=={
            'Value': topic, 'Reliability Policy': 'Best Effort',
            'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Depth': 5}
    for name, topic in (('Global Costmap', '/global_costmap/costmap'),
                        ('Local Costmap', '/local_costmap/costmap')):
        display=_display(vm, name)
        assert display['Class']=='rviz_default_plugins/Map' and display['Enabled'] is True
        assert display['Alpha']==pytest.approx(0.6) and display['Color Scheme']=='costmap'
        assert _qos(display['Topic'])=={
            'Value': topic, 'Reliability Policy': 'Reliable',
            'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last', 'Depth': 1}
    view=vm['Views']['Current']
    assert view['Class']=='rviz_default_plugins/TopDownOrtho'
    assert view['Scale']==18
    assert view['X']==pytest.approx(-0.0243, abs=1e-3)
    assert view['Y']==pytest.approx(-0.0109, abs=1e-3)
    tools={item['Class']: item.get('Topic') for item in vm['Tools']}
    assert tools['rviz_default_plugins/SetGoal']=='/cleannav/rviz/goal_pose'
    assert tools['rviz_default_plugins/SetInitialPose']=='/rtabmap/initialpose'


@pytest.mark.parametrize('v,motion,safety,error',[
    ([0,0,0,0,0,0],False,None,False),
    ([.1,0,0,0,0,0],False,None,True),
    ([-.01,0,0,0,0,0],True,None,True),
    ([.2,0,0,0,0,0],True,None,True),
    ([.1,0,0,0,0,0],True,dict(block_all=False,estop=False,enabled=True,owner='existing-owner'),False),
    ([.1,0,0,0,0,0],True,dict(block_all=False,estop=False,enabled=True,owner=''),True),
    ([float('nan'),0,0,0,0,0],False,None,True),
])
def test_guard_uses_actual_safety_and_lease_state(v,motion,safety,error):
    assert bool(command_error(v,motion,safety))==error


def test_unknown_facts_cannot_admit():
    assert missing_inputs(dict(scan=True,cm=None,localization=False))==['cm','localization']


def test_facade_navigation_defers_legacy_package_resolution(monkeypatch, tmp_path):
    nav_launch=integration.load_module(
        HERE.parents[1]/'cleannav_navigation/launch/cleannav_ackermann_navigation.launch.py',
        'navigation_facade_lazy_legacy_test')
    resolved=[]
    def resolve(package):
        resolved.append(package)
        if package == 'cleannav_navigation':
            return str(tmp_path/'navigation')
        if package == 'cleannav_safety_supervisor':
            return str(tmp_path/'safety')
        if package in {'cleannav_global_planner','cleannav_path_executor'}:
            return str(tmp_path/package)
        raise AssertionError(package)
    monkeypatch.setattr(nav_launch,'get_package_share_directory',resolve)
    description=nav_launch.generate_launch_description()
    assert 'cleannav_global_planner' not in resolved
    assert 'cleannav_path_executor' not in resolved
    legacy=next(item for item in description.entities if item.__class__.__name__=='OpaqueFunction')
    context=LaunchContext();context.launch_configurations['execution_mode']='facade'
    assert legacy.execute(context)==[]

    context.launch_configurations['execution_mode']='legacy'
    resolved.clear()
    actions=legacy.execute(context)
    assert resolved==['cleannav_global_planner','cleannav_path_executor']
    assert len(actions)==2


def test_simple_four_wall_oracle_static_contract_preserves_wheel_default():
    name='simple-four-wall-nohit-v1-oracle-static'
    contract=json.loads(integration.contract_path_for_profile(name).read_text())
    directory=integration.WORKSPACE/contract['map_directory_relative']
    world=integration.WORKSPACE/contract['world_relative']
    result=integration.verify_map(directory,world,profile=name,odometry_mode='oracle')
    assert result['required_odometry_mode']=='oracle'
    assert 'localization_contract' not in contract
    with pytest.raises(ValueError,match='odometry source conflicts'):
        integration.verify_map(directory,world,profile='simple-four-wall-nohit-v1',odometry_mode='oracle')


def test_simple_four_wall_oracle_localization_contract_binds_initial_pose_and_amcl():
    name='simple-four-wall-nohit-v1-oracle-localization'
    contract=json.loads(integration.contract_path_for_profile(name).read_text())
    directory=integration.WORKSPACE/contract['map_directory_relative']
    world=integration.WORKSPACE/contract['world_relative']
    verified=integration.verify_map(directory,world,profile=name,odometry_mode='oracle')
    assert verified['localization_runtime']['kind']=='nav2_amcl'
    assert verified['localization_runtime']['scan_topic']=='/mapping/scan'
    assert verified['localization_runtime']['base_frame_id']=='base_footprint'
    assert verified['localization_runtime']['map_to_odom_publisher']=='amcl'
    assert verified['localization_runtime']['transform_tolerance_s']==0.0
    assert verified['localization_runtime']['uses_rtabmap_database'] is False
    assert verified['localization_initial_pose']=='0.0015082397731021 -7.99616241455078 0 0 0 0.0131529364734888'
    assert 'patrol' not in verified and 'route_registry' not in verified


def test_amcl_install_identity_checks_prefix_executable_and_params(tmp_path):
    import copy
    contract=json.loads(integration.contract_path_for_profile(
        'simple-four-wall-nohit-v1-oracle-localization').read_text())
    contract=copy.deepcopy(contract)
    prefix=tmp_path/'ros';(prefix/'share/nav2_amcl').mkdir(parents=True)
    (prefix/'lib/nav2_amcl').mkdir(parents=True)
    package_xml=prefix/'share/nav2_amcl/package.xml';package_xml.write_text('<package><version>1.1.20</version></package>')
    executable=prefix/'lib/nav2_amcl/amcl';executable.write_bytes(b'amcl-test')
    params=tmp_path/'nav2_params_ackermann.yaml';params.write_bytes(b'amcl: {}\n')
    import hashlib
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    contract['localization_runtime']['package_xml_sha256']=sha(package_xml)
    contract['localization_runtime']['executable_sha256']=sha(executable)
    contract['source_bindings']['amcl_params_source']['sha256']=sha(params)
    resolve=lambda package:str(prefix)
    result=integration.verify_localization_runtime(contract,resolve,params)
    assert result['version']=='1.1.20'
    assert result['package_prefix']==result['base_package_prefix']==str(prefix)
