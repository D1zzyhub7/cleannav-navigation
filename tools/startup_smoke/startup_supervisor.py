"""One bounded startup-only invocation, using the validated Gate2 resource/cleanup primitives."""
from __future__ import annotations
import argparse,hashlib,json,math,os,signal,subprocess,sys,time,traceback
from datetime import datetime,timezone
from pathlib import Path
NAV=Path(__file__).resolve().parents[2]
ROOT=None
sys.path.insert(0,str(Path(__file__).resolve().parent))
import roof_lidar_runtime as runtime
ROOT=runtime.ROOT
OUTPUT_ROOT=Path(os.environ.get('CLEANNAV_STARTUP_OUTPUT_ROOT',ROOT/'cleannav-deliverables/startup-smoke-runs')).expanduser().resolve()
MODEL_ASSETS=ROOT.parent/'cleannav_demo_assets/osrf_car_demo/car_demo'
WORLD=NAV/'cleannav_simulation/worlds/startup_smoke_mcity.world'
from publisher_graph_gate import PublisherGraphConvergence, endpoint_record
SCAN_TF_WAIT_S = 1.0
AMCL_PARAMETER_NAMES = (
    'use_sim_time', 'transform_tolerance', 'scan_topic', 'global_frame_id',
    'odom_frame_id', 'base_frame_id', 'tf_broadcast')


def expected_amcl_parameters(contract):
    runtime = contract['localization_runtime']
    return {
        'use_sim_time': True,
        'transform_tolerance': float(runtime['transform_tolerance_s']),
        'scan_topic': runtime['scan_topic'],
        'global_frame_id': runtime['global_frame_id'],
        'odom_frame_id': runtime['odom_frame_id'],
        'base_frame_id': runtime['base_frame_id'],
        'tf_broadcast': bool(runtime['tf_broadcast']),
    }


def decode_amcl_parameter_values(values, parameter_type):
    if len(values) != len(AMCL_PARAMETER_NAMES):
        raise ValueError('AMCL GetParameters returned an incomplete value list')
    expected_types = {
        'use_sim_time': parameter_type.PARAMETER_BOOL,
        'transform_tolerance': parameter_type.PARAMETER_DOUBLE,
        'scan_topic': parameter_type.PARAMETER_STRING,
        'global_frame_id': parameter_type.PARAMETER_STRING,
        'odom_frame_id': parameter_type.PARAMETER_STRING,
        'base_frame_id': parameter_type.PARAMETER_STRING,
        'tf_broadcast': parameter_type.PARAMETER_BOOL,
    }
    fields = {
        parameter_type.PARAMETER_BOOL: 'bool_value',
        parameter_type.PARAMETER_DOUBLE: 'double_value',
        parameter_type.PARAMETER_STRING: 'string_value',
    }
    result = {}
    for name, value in zip(AMCL_PARAMETER_NAMES, values):
        if value.type != expected_types[name]:
            raise ValueError(f'AMCL parameter {name} has unset or unexpected type {value.type}')
        result[name] = getattr(value, fields[value.type])
    return result


def compare_amcl_parameters(actual, expected):
    if actual is None:
        return {'status': 'NOT_CAPTURED', 'matches_contract': False, 'mismatches': {}}
    mismatches = {}
    for name in AMCL_PARAMETER_NAMES:
        if name not in actual or name not in expected:
            mismatches[name] = {'actual': actual.get(name), 'expected': expected.get(name)}
            continue
        wanted, observed = expected[name], actual[name]
        matches = (math.isclose(float(observed), float(wanted), rel_tol=0.0, abs_tol=1e-12)
                   if name == 'transform_tolerance' else observed == wanted)
        if not matches:
            mismatches[name] = {'actual': observed, 'expected': wanted}
    return {'status': 'PASS' if not mismatches else 'MISMATCH',
            'matches_contract': not mismatches, 'mismatches': mismatches}


def poll_scan_tf_requests(pending, completed, lookup, *, now, deadline,
                          clock_ns, clock_age_s, latest_tf_ns):
    """Defer until Oracle TF covers each scan, then query its exact stamp."""
    if now >= deadline:
        raise RuntimeError('scan TF wait reached supervisor deadline')
    if clock_ns <= 0 or clock_age_s >= 1.0:
        raise RuntimeError('scan TF wait: simulation clock not advancing')
    if latest_tf_ns is not None and latest_tf_ns > 0 and abs(clock_ns-latest_tf_ns) > 1000000000:
        raise RuntimeError('scan TF wait: stale TF or inconsistent time source')
    for request in list(pending):
        stamp = request['stamp_ns']
        if stamp <= 0 or abs(stamp-clock_ns) > 1000000000:
            raise RuntimeError('scan TF wait: scan timestamp inconsistent with simulation clock')
        if now >= request['deadline']:
            raise RuntimeError('scan TF deadline exceeded: '+str(request))
        if latest_tf_ns is None or latest_tf_ns <= 0 or stamp > latest_tf_ns:
            request['future_attempts'] = request.get('future_attempts', 0) + 1
            request['wait_reason'] = 'oracle_tf_time_does_not_cover_scan'
            request['latest_tf_ns'] = latest_tf_ns
            continue
        # Both queries retain the exact LaserScan stamp, including sensor extrinsics.
        # Once the named Oracle edge covers Q, every exact-lookup error is real.
        stamps = [lookup('odom', source, stamp)
                  for source in ('base_footprint', request['frame'])]
        if stamps != [stamp, stamp]:
            raise RuntimeError('scan TF query returned a different timestamp')
        completed.append({k:request[k] for k in ('topic','frame','stamp_ns','received_wall')})
        completed[-1].update(returned_stamps_ns=stamps, wait_wall_s=now-request['received_wall'],
            future_attempts=request.get('future_attempts',0), latest_tf_ns=latest_tf_ns,
            clock_ns=clock_ns)
        pending.remove(request)

def write(path,data):
    path.write_text(json.dumps(data,indent=2,default=str)+'\n')

def parse_args(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument('--mode',choices=['startup_smoke'],default='startup_smoke')
    p.add_argument('--run-id',required=True)
    p.add_argument('--sensor-only', action='store_true', help='no RTAB-Map; retain blocked Safety and CM')
    p.add_argument('--world', type=Path, help='existing diagnostic world, requires --sensor-only')
    p.add_argument('--odometry-mode', choices=['wheel','oracle'], default='wheel')
    p.add_argument('--localization-mode', choices=['none','amcl'], default='none')
    p.add_argument('--map-yaml', type=Path)
    a=p.parse_args(argv)
    if a.odometry_mode == 'oracle' and (not a.sensor_only or not a.map_yaml or not a.map_yaml.is_file()):
        p.error('oracle requires --sensor-only and an existing --map-yaml')
    if a.localization_mode == 'amcl' and (a.odometry_mode != 'oracle' or not a.sensor_only or not a.map_yaml or not a.map_yaml.is_file()):
        p.error('AMCL localization requires --sensor-only, --odometry-mode oracle, and an existing --map-yaml')
    if a.world is not None and (not a.sensor_only or not a.world.is_file()):
        p.error('--world requires --sensor-only and an existing file')
    if not a.run_id.startswith('startup-smoke-') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in a.run_id):
        p.error('run-id must be a unique startup-smoke-* identifier')
    return a

def validate_ros_distro(value):
    if value == 'humble':
        return {'expected':'humble','actual':value,'ok':True}
    if value is None:
        actual='<unset>'
    elif value == '':
        actual='<empty>'
    else:
        actual=repr(value)
    raise RuntimeError("ROS_DISTRO mismatch after ROS Humble and approved overlay setup: expected 'humble', got "+actual)

def verify_identity():
    manifest=NAV/'STARTUP_MANIFEST.json'
    data=json.loads(manifest.read_text())
    for rel,digest in data['files'].items():
        relpath=Path(rel)
        if relpath.is_absolute() or '..' in relpath.parts:raise RuntimeError('manifest path escapes repository: '+rel)
        path=NAV/relpath
        if not path.is_file() or runtime.sha256(path)!=digest:raise RuntimeError('identity mismatch: '+rel)
    return {'manifest':str(manifest),'sha256':runtime.sha256(manifest),
        'git_parent_commit':data['provenance']['git_parent_commit'],
        'verified_reference_entry_sha256':data['provenance']['verified_reference_entry_sha256'],
        'files_verified':len(data['files'])}

def verify_selected_amcl_diagnostic_profile(localization_mode, environment=None):
    from amcl_diagnostic_profile import verify_selected_profile
    return verify_selected_profile(localization_mode, environment)

def launch_argv(out, world=WORLD, sensor_only=False, odometry_mode="wheel", map_yaml=None, localization_mode="none"):
    return ['ros2','launch',str(NAV/'tools/startup_smoke/startup_smoke.launch.py'),
            'mode:=startup_smoke','maps_root:='+str(out/'sessions'),
            'world:='+str(world),'model_path:='+str(MODEL_ASSETS/'models'),
            'sensor_only:='+str(sensor_only).lower(), 'odometry_mode:='+odometry_mode,
            'localization_mode:='+localization_mode, 'map_yaml:='+str(map_yaml or '')]

def cleanup_result_ok(c):
    s=c.get('process_group_snapshot_after_cleanup',{})
    return c.get('cleanup_ok') is True and s.get('scan_complete') is True and not s.get('running_members') and not s.get('zombie_members') and s.get('status')=='EMPTY'

def perform_cleanup(proc,started,t0):
    try:return runtime.signal_and_reap(proc,started,t0)
    except BaseException as error:
        return {"started":started,"cleanup_ok":False,"status":"UNKNOWN","error":repr(error),"traceback":traceback.format_exc(),"process_group_snapshot_after_cleanup":runtime.process_group_snapshot(proc.pid) if proc else {}}

def oracle_observation_stats(node, out, map_yaml, localization_mode='none'):
    """Compare received odometry with the raw stamped Gazebo responses it used."""
    import numpy as np
    import yaml
    def rotation(q):
        x,y,z,w=q
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    def yaw(matrix):return math.atan2(matrix[1,0],matrix[0,0])
    def wrap(value):return math.atan2(math.sin(value),math.cos(value))
    paths=list((out/'ros-log').rglob('oracle-truth.jsonl'))
    if len(paths)!=1:raise ValueError('one raw Oracle truth ledger required')
    truth=[json.loads(line) for line in paths[0].read_text().splitlines()]
    if not truth:raise ValueError('no Gazebo truth samples')
    anchor=truth[0];r0=rotation(anchor['orientation']);p0=np.array(anchor['position'])
    indexed={sample['stamp_ns']:sample for sample in truth}
    errors=[]
    for odom in node.odom_samples:
        source=indexed.get(odom['stamp_ns'])
        if source is None:continue
        target=r0.T@(np.array(source['position'])-p0)
        expected_rotation=r0.T@rotation(source['orientation'])
        errors.append({'stamp_ns':odom['stamp_ns'],
            'position_error_m':float(np.linalg.norm(np.array(odom['position'])-target)),
            'yaw_error_rad':abs(wrap(yaw(rotation(odom['orientation']))-yaw(expected_rotation)))})
    write(out/'oracle-aligned-errors.json',errors)
    stamps=[sample['stamp_ns'] for sample in node.odom_samples]
    odom_dt=(stamps[-1]-stamps[0])/1e9 if len(stamps)>1 else 0
    msg=node.map_message;cfg=yaml.safe_load(Path(map_yaml).read_text())
    frozen=json.loads((Path(map_yaml).parent/'occupancy_grid.json').read_text())
    data=list(msg.data) if msg else []
    mismatches=sum(a!=b for a,b in zip(data,frozen['data'])) if len(data)==len(frozen['data']) else None
    map_ok=(msg is not None and msg.header.frame_id=='map' and
        msg.info.width==frozen['width'] and msg.info.height==frozen['height'] and
        abs(msg.info.resolution-cfg['resolution'])<1e-9 and
        abs(msg.info.origin.position.x-cfg['origin'][0])<1e-12 and
        abs(msg.info.origin.position.y-cfg['origin'][1])<1e-12 and
        abs(msg.info.origin.position.z)<1e-12 and abs(msg.info.origin.orientation.w-1)<1e-12 and
        all(abs(v)<1e-12 for v in (msg.info.origin.orientation.x,msg.info.origin.orientation.y,msg.info.origin.orientation.z)) and mismatches==0)
    tf_names=[e.node_name for e in node.get_publishers_info_by_topic('/tf')]
    static_names=[e.node_name for e in node.get_publishers_info_by_topic('/tf_static')]
    odom_names=[e.node_name for e in node.get_publishers_info_by_topic('/odom')]
    expected_tf={'robot_state_publisher','gazebo_oracle_odometry'}
    if localization_mode=='amcl':expected_tf.add('amcl')
    tf_ok=(len(tf_names)==len(set(tf_names)) and set(tf_names)==expected_tf
        and static_names==['robot_state_publisher'] and odom_names==['gazebo_oracle_odometry']
        and node.tf_parents.get('base_footprint')=={'odom'} and node.tf_parents.get('base_link')=={'base_footprint'}
        and (localization_mode!='amcl' or node.tf_parents.get('odom')=={'map'}))
    wheel_velocities=[abs(v) for sample in node.joint_samples for name,v in zip(sample['names'],sample['velocity']) if 'wheel' in name]
    max_position=max((e['position_error_m'] for e in errors),default=None)
    max_yaw=max((e['yaw_error_rad'] for e in errors),default=None)
    result={'truth_ledger':str(paths[0]),'truth_samples':len(truth),'odom_samples':len(stamps),
        'exact_stamp_pairs':len(errors),'odom_frequency_sim_hz':(len(stamps)-1)/odom_dt if odom_dt>0 else None,
        'odom_stamp_span_s':odom_dt,'odom_stamps_strictly_increasing':all(b>a for a,b in zip(stamps,stamps[1:])),
        'odom_frames':sorted({(o['frame'],o['child']) for o in node.odom_samples}),
        'anchor':anchor,'max_position_error_m':max_position,'max_yaw_error_rad':max_yaw,
        'comparison_note':'Exact Gazebo response stamps, first truth pose anchor, independent rotation-matrix calculation. Same service source as adapter, not an independent sensor.',
        'truth_max_planar_displacement_m':max(math.hypot(t['position'][0]-p0[0],t['position'][1]-p0[1]) for t in truth),
        'truth_final_planar_displacement_m':math.hypot(truth[-1]['position'][0]-p0[0],truth[-1]['position'][1]-p0[1]),
        'truth_yaw_change_rad':wrap(yaw(rotation(truth[-1]['orientation']))-yaw(r0)),
        'max_abs_wheel_velocity_rad_s':max(wheel_velocities,default=None),
        'clock_samples':len(node.clock_samples),'clock_span_s':(node.clock_samples[-1]-node.clock_samples[0])/1e9 if node.clock_samples else None,
        'clock_strictly_increasing':all(b>a for a,b in zip(node.clock_samples,node.clock_samples[1:])),
        'tf_publishers':tf_names,'tf_static_publishers':static_names,'odom_publishers':odom_names,
        'tf_child_parents':{k:sorted(v) for k,v in node.tf_parents.items()},'tf_authority_pass':tf_ok,
        'map_counts':{str(v):data.count(v) for v in (-1,0,100)},'map_cell_mismatches':mismatches,'map_matches':map_ok}
    result['pass']=(map_ok and tf_ok and len(errors)>=10 and max_position<1e-6 and max_yaw<1e-6
        and result['odom_stamps_strictly_increasing'] and result['clock_strictly_increasing'])
    return result


def amcl_observation_stats(node, out):
    """Align AMCL map-frame estimates with stamped Gazebo truth for reporting."""
    import bisect
    import numpy as np
    truth_paths=list((out/'ros-log').rglob('oracle-truth.jsonl'))
    if len(truth_paths)!=1:raise ValueError('one raw Oracle truth ledger required for AMCL alignment')
    truth=[json.loads(line) for line in truth_paths[0].read_text().splitlines()]
    if not truth:raise ValueError('no Gazebo truth samples for AMCL alignment')
    truth_stamps=[sample['stamp_ns'] for sample in truth]
    def yaw(q):
        x,y,z,w=q
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    def wrap(value):return math.atan2(math.sin(value),math.cos(value))
    errors=[];unmatched=[]
    for estimate in node.amcl_samples:
        stamp=estimate['stamp_ns'];i=bisect.bisect_left(truth_stamps,stamp)
        if i<len(truth) and truth_stamps[i]==stamp:
            left=right=truth[i];ratio=0.0
        elif i==0 or i==len(truth):
            unmatched.append(stamp);continue
        else:
            left,right=truth[i-1],truth[i]
            ratio=(stamp-left['stamp_ns'])/(right['stamp_ns']-left['stamp_ns'])
        xy=[left['position'][k]+ratio*(right['position'][k]-left['position'][k]) for k in (0,1)]
        left_yaw=yaw(left['orientation']);right_yaw=yaw(right['orientation'])
        truth_yaw=wrap(left_yaw+ratio*wrap(right_yaw-left_yaw))
        estimate_yaw=yaw(estimate['orientation'])
        errors.append({'stamp_ns':stamp,'truth_interpolation_ratio':ratio,
            'position_error_m':math.hypot(estimate['position'][0]-xy[0],estimate['position'][1]-xy[1]),
            'yaw_error_rad':abs(wrap(estimate_yaw-truth_yaw)),
            'amcl_xy':[estimate['position'][0],estimate['position'][1]],'truth_xy':xy,
            'amcl_yaw_rad':estimate_yaw,'truth_yaw_rad':truth_yaw,
            'covariance_diagonal':estimate['covariance_diagonal']})
    write(out/'amcl-aligned-errors.json',errors)
    pose_stamps=[sample['stamp_ns'] for sample in node.amcl_samples]
    tf_stamps=node.map_to_odom_stamps
    position=[item['position_error_m'] for item in errors]
    heading=[item['yaw_error_rad'] for item in errors]
    def summary(values):
        return {'median':float(np.median(values)),'p95':float(np.percentile(values,95)),
                'max':float(max(values))} if values else {'median':None,'p95':None,'max':None}
    covariances=list(zip(*(sample['covariance_diagonal'] for sample in node.amcl_samples))) if node.amcl_samples else [[],[],[]]
    map_contract=json.loads((NAV/'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json').read_text())
    publishers=lambda topic:sorted(e.node_name for e in node.get_publishers_info_by_topic(topic))
    result={'truth_ledger':str(truth_paths[0]),'truth_samples':len(truth),
        'amcl_pose_samples':len(pose_stamps),'amcl_pose_stamps_strictly_increasing':all(b>a for a,b in zip(pose_stamps,pose_stamps[1:])),
        'amcl_pose_span_s':(pose_stamps[-1]-pose_stamps[0])/1e9 if len(pose_stamps)>1 else None,
        'amcl_pose_frequency_sim_hz':(len(pose_stamps)-1)/((pose_stamps[-1]-pose_stamps[0])/1e9) if len(pose_stamps)>1 and pose_stamps[-1]>pose_stamps[0] else None,
        'truth_aligned_samples':len(errors),'unmatched_amcl_stamps_ns':unmatched,
        'position_error_m':summary(position),'yaw_error_rad':summary(heading),
        'covariance_diagonal_median':[float(np.median(axis)) if axis else None for axis in covariances],
        'map_to_odom_tf_samples':len(tf_stamps),'map_to_odom_stamps_strictly_increasing':all(b>a for a,b in zip(tf_stamps,tf_stamps[1:])),
        'map_to_odom_tf_span_s':(tf_stamps[-1]-tf_stamps[0])/1e9 if len(tf_stamps)>1 else None,
        'map_to_odom_tf_frequency_sim_hz':(len(tf_stamps)-1)/((tf_stamps[-1]-tf_stamps[0])/1e9) if len(tf_stamps)>1 and tf_stamps[-1]>tf_stamps[0] else None,
        'map_to_odom_max_gap_s':max(((b-a)/1e9 for a,b in zip(tf_stamps,tf_stamps[1:])),default=None),
        'map_tf_child_parents':{k:sorted(v) for k,v in node.tf_parents.items()},
        'publishers':{'/tf':publishers('/tf'),'/tf_static':publishers('/tf_static'),
            '/amcl_pose':publishers('/amcl_pose'),'/map':publishers('/map'),'/odom':publishers('/odom')},
        'lifecycle':node.localization_lifecycle,
        'amcl_parameter_readback':node.amcl_parameter_readback or {'status':'NOT_CAPTURED'},
        'amcl_parameter_contract':compare_amcl_parameters(
            (node.amcl_parameter_readback or {}).get('values'),
            expected_amcl_parameters(map_contract)),
        'map_world_alignment':map_contract['map_geometry']['coordinate_convention'],
        'initial_pose_map_base_footprint':map_contract['localization_initial_pose'],
        'amcl_transform_tolerance_s':map_contract['localization_runtime']['transform_tolerance_s'],
        'initial_pose_truth_evidence':map_contract['provenance']['initial_pose_truth'],
        'accuracy_threshold':'NOT SPECIFIED; errors are reported, not judged against an invented tolerance',
        'comparison_note':'AMCL map-frame poses are compared against the same stamped Gazebo entity-state truth source used by Oracle odometry; truth poses are linearly interpolated in xy and wrapped-yaw interpolated between bracketing samples.',
        'pass':(len(errors)>=2 and not unmatched and all(math.isfinite(v) for v in position+heading)
            and all(s['frame']=='map' for s in node.amcl_samples)
            and all(b>a for a,b in zip(pose_stamps,pose_stamps[1:]))
            and len(tf_stamps)>=2 and all(b>a for a,b in zip(tf_stamps,tf_stamps[1:]))
            and node.tf_parents.get('odom')=={'map'}
            and node.localization_lifecycle.get('map_server',{}).get('state_id')==3
            and node.localization_lifecycle.get('amcl',{}).get('state_id')==3
            and publishers('/amcl_pose')==['amcl'] and publishers('/map')==['map_server']
            and node.amcl_parameter_readback is not None
            and node.amcl_parameter_readback.get('status')=='CAPTURED'
            and node.amcl_parameter_readback.get('contract',{}).get('matches_contract'))}
    return result


def main():
    args=parse_args();world=(args.world or WORLD).resolve();t0=time.monotonic();out=OUTPUT_ROOT/args.run_id
    out.mkdir(parents=True,exist_ok=False)
    for name in ('ros-home','ros-log'): (out/name).mkdir()
    os.environ.update(ROS_HOME=str(out/'ros-home'),ROS_LOG_DIR=str(out/'ros-log'))
    result={'run_id':args.run_id,'mode':args.mode,'navigation_repository':str(NAV),'workspace_root':str(ROOT),'output_root':str(OUTPUT_ROOT),'utc':datetime.now(timezone.utc).isoformat(),
        'cwd':os.getcwd(),'pid':os.getpid(),'ppid':os.getppid(),'pid1':Path('/proc/1/comm').read_text().strip(),
        'dynamic_entry_calls':1,'total_limit_s':180,'cleanup_start_deadline_s':145,'observation_required_s':10 if args.odometry_mode == 'oracle' else 30,
        'observation_time_basis':'simulation' if args.odometry_mode == 'oracle' else 'wall', 'odometry_mode':args.odometry_mode,
        'localization_mode':args.localization_mode,
        'launch_started':False,'test_pass':False,'failures':[],'phases':{k:'NOT RUN' for k in ('preflight','startup','sensors','lifecycle','safety','localization','cleanup')}}
    proc=None;log=None;node=None;fds=[];ros=None;started=False;streams=[];waiver=None
    amcl_expected=None
    def elapsed():return time.monotonic()-t0
    def event(kind,**data):
        record={'event':kind,'t0_elapsed_s':elapsed(),'monotonic_ns':time.monotonic_ns(),**data}
        for s in streams:s.write(json.dumps(record,default=str)+'\n');s.flush()
    try:
        streams=[(out/'events.jsonl').open('w')]
        def interrupted(signum, _frame): raise RuntimeError('external signal '+str(signum))
        signal.signal(signal.SIGTERM,interrupted)
        result['identity']=verify_identity()
        diagnostic_identity=verify_selected_amcl_diagnostic_profile(args.localization_mode)
        if diagnostic_identity is not None:
            result['identity']['amcl_diagnostic_profile']=diagnostic_identity
        sys.path.insert(0,str(NAV/'tools/rviz_navigation'))
        from process_memory_telemetry import _read_system
        oom_baseline = _read_system(Path('/proc')).get('oom_kill_count')
        result['oom_kill_baseline']=oom_baseline
        fds,result['locks']=runtime.acquire_locks()
        pre=result['preflight']={}
        pre['source_paths']={'world':str(world),'world_exists':world.is_file(),'world_sha256':runtime.sha256(world),'sensor_only':args.sensor_only,
            'model_path':str(MODEL_ASSETS/'models'),'model_path_exists':(MODEL_ASSETS/'models').is_dir()}
        if not pre['source_paths']['world_exists'] or not pre['source_paths']['model_path_exists']:
            raise RuntimeError('repository world or workspace model assets are unavailable')
        pre['environment']={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_DISTRO','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','GAZEBO_MASTER_URI')}
        pre['ros_distro_gate']=validate_ros_distro(os.environ.get('ROS_DISTRO'))
        if os.environ.get('ROS_DOMAIN_ID')!='204' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('unexpected ROS domain/environment')
        for name,value in [('processes',runtime.process_visibility_and_conflicts()),('port',runtime.port_free(11704)),('prefixes',runtime.check_package_prefixes())]:
            pre[name]=value
            if not value.get('ok'):raise RuntimeError('preflight '+name+' rejected')
        pre['cgroup']=runtime.cgroup_memory_snapshot();waiver=runtime.wsl_cgroup_waiver_preflight();pre['waiver']=waiver
        pre['memory']=runtime.memory_snapshot(cgroup_waiver=waiver,min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES,memavailable_warning_only=args.odometry_mode=='oracle')
        if not pre['memory']['ok']:raise RuntimeError('resource preflight rejected')
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan,JointState
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry,OccupancyGrid
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from rtabmap_msgs.msg import Info
        from cleannav_interfaces.msg import SafetyStatus
        from lifecycle_msgs.srv import GetState
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        from tf2_ros import Buffer,TransformListener
        from rclpy.clock import ClockType
        from rclpy.time import Time
        if args.localization_mode=='amcl':
            localization_contract=json.loads((NAV/'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json').read_text())
            amcl_expected=expected_amcl_parameters(localization_contract)
            result['amcl_parameter_contract']=amcl_expected
        ros=rclpy;ros.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        class Observer(Node):
            def __init__(self):
                super().__init__('cleannav_startup_smoke_observer')
                self.scans={k:[] for k in ('/scan','/mapping/scan_raw','/mapping/scan')}
                self.latest={};self.odom=[];self.cmd=[];self.info=0;self.safety=None;self.controllers=[];self.cm=None
                self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
                self.clock_samples=[];self.odom_samples=[];self.joint_samples=[];self.tf_parents={};self.map_message=None
                self.amcl_samples=[];self.map_to_odom_stamps=[];self.localization_lifecycle={}
                self.amcl_parameter_future=None;self.amcl_parameter_readback=None
                self.scan_tf_pending=[];self.scan_tf_completed=[];self.scan_tf_enabled=False
                self.scan_tf_cutoff_ns=None;self.scan_tf_enqueued=0;self.scan_tf_anchor_ns=None
                self.scan_tf_pre_anchor=[]
                self.dynamic_tf_stamps=[];self.clock_advanced_wall=None
                if args.odometry_mode=='oracle':
                    self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map_message',m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    self.create_subscription(TFMessage,'/tf',self.on_tf,10)
                    self.create_subscription(TFMessage,'/tf_static',self.on_tf,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                if args.localization_mode=='amcl':
                    self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl_pose,10)
                for topic in self.scans:self.create_subscription(LaserScan,topic,lambda m,t=topic:self.scan(t,m),qos_profile_sensor_data)
                self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
                self.create_subscription(Odometry,'/odom',self.on_odom,10)
                self.create_subscription(Info,'/rtabmap/info',self.on_info,10)
                self.create_subscription(SafetyStatus,'/cleannav/safety_status',self.on_safety,10)
                self.create_subscription(JointState,'/joint_states',self.on_joints,10)
                for topic in ('/cmd_vel','/cleannav/cmd_vel_candidate','/cleannav/mapping/cmd_vel_raw','/cleannav/manual/cmd_vel_raw','/ackermann_controller/reference_unstamped'):
                    self.create_subscription(Twist,topic,lambda m,t=topic:self.command(t,m),10)
                self.lc=self.create_client(GetState,'/collision_monitor/get_state');self.lf=None
                self.map_lc=self.create_client(GetState,'/map_server/get_state');self.map_lf=None
                self.amcl_lc=self.create_client(GetState,'/amcl/get_state');self.amcl_lf=None
                self.amcl_pc=(self.create_client(GetParameters,'/amcl/get_parameters')
                              if args.localization_mode=='amcl' else None)
                self.expected_amcl_parameters=amcl_expected
                self.cc=self.create_client(ListControllers,'/controller_manager/list_controllers');self.cf=None
                self.pc=self.create_client(GetParameters,'/cleannav_safety_supervisor/get_parameters');self.pf=None;self.params=None
            def on_clock(self,m):
                stamp=m.clock.sec*1000000000+m.clock.nanosec
                if not self.clock_samples or stamp>self.clock_samples[-1]:
                    self.mark('clock',m);self.clock_advanced_wall=time.monotonic()
                elif args.odometry_mode=='oracle' and stamp<self.clock_samples[-1]:
                    raise RuntimeError('simulation clock moved backwards')
                self.clock_samples.append(stamp)
                event('clock',stamp_ns=stamp)
            def on_joints(self,m):
                self.mark('joint_states',m)
                self.joint_samples.append({'stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'names':list(m.name),'velocity':list(m.velocity)})
            def on_tf(self,m):
                for t in m.transforms:
                    self.tf_parents.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
                    if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint':
                        stamp=t.header.stamp.sec*1000000000+t.header.stamp.nanosec
                        if self.dynamic_tf_stamps and stamp<=self.dynamic_tf_stamps[-1]:
                            raise RuntimeError('non advancing Oracle TF timestamp')
                        self.dynamic_tf_stamps.append(stamp)
                        event('oracle_tf',stamp_ns=stamp)
                    if t.header.frame_id=='map' and t.child_frame_id=='odom':
                        stamp=t.header.stamp.sec*1000000000+t.header.stamp.nanosec
                        self.map_to_odom_stamps.append(stamp)
                        event('amcl_map_to_odom_tf',stamp_ns=stamp)
            def on_amcl_pose(self,m):
                stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
                pose=m.pose.pose
                sample={'stamp_ns':stamp,'frame':m.header.frame_id,
                    'position':[pose.position.x,pose.position.y,pose.position.z],
                    'orientation':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],
                    'covariance_diagonal':[m.pose.covariance[i] for i in (0,7,35)],
                    'received_wall':time.monotonic()}
                self.amcl_samples.append(sample)
                self.latest['amcl_pose']=sample['received_wall']
                event('amcl_pose',sample=sample)
            def mark(self,k,m):self.latest[k]=time.monotonic()
            def scan(self,t,m):
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                valid=sum(math.isfinite(v) and m.range_min<=v<=m.range_max for v in m.ranges)
                stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
                x={'wall_monotonic':time.monotonic(),'stamp':stamp,'stamp_ns':stamp_ns,'count':len(m.ranges),'frame':m.header.frame_id,'valid_ranges':valid,'angle_min':m.angle_min,'angle_max':m.angle_max,'angle_increment':m.angle_increment,'range_max':m.range_max}
                self.scans[t].append(x);self.latest[t]=time.monotonic()
                if self.scans[t][:-1] and stamp<=self.scans[t][-2]['stamp']:raise RuntimeError('non advancing scan stamp '+t)
                if args.odometry_mode=='oracle' and self.scan_tf_enabled:
                    if stamp_ns<self.scan_tf_anchor_ns:
                        self.scan_tf_pre_anchor.append({'topic':t,'stamp_ns':stamp_ns})
                        event('scan_time_tf_before_buffer_anchor',topic=t,stamp_ns=stamp_ns,
                              buffer_anchor_ns=self.scan_tf_anchor_ns)
                    elif self.scan_tf_cutoff_ns is None or stamp_ns<=self.scan_tf_cutoff_ns:
                        self.scan_tf_pending.append({'topic':t,'frame':m.header.frame_id,'stamp_ns':stamp_ns,
                            'received_wall':x['wall_monotonic'], 'deadline':min(x['wall_monotonic']+SCAN_TF_WAIT_S,t0+145)})
                        self.scan_tf_enqueued+=1
                event('scan',topic=t,ranges=[v if math.isfinite(v) else ('+Inf' if v>0 else ('-Inf' if v<0 else 'NaN')) for v in m.ranges],**x)
            def on_odom(self,m):
                self.mark('odom',m);p=m.pose.pose.position
                self.odom.append([time.monotonic(),p.x,p.y,m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.frame_id,m.child_frame_id])
                stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
                if args.odometry_mode=='oracle' and (m.header.frame_id!='odom' or m.child_frame_id!='base_footprint'
                        or (self.odom_samples and stamp_ns<=self.odom_samples[-1]['stamp_ns'])):
                    raise RuntimeError('Oracle odom frame or timestamp contract failed')
                self.odom_samples.append({'stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'position':[p.x,p.y,p.z],'orientation':[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w],'frame':m.header.frame_id,'child':m.child_frame_id})
                event('odom',sample=self.odom[-1],stamped_pose=self.odom_samples[-1])
            def on_info(self,m):self.mark('rtab_info',m);self.info+=1
            def on_safety(self,m):
                self.mark('safety',m);self.safety={'emergency_stop_active':m.emergency_stop_active,'autonomous_enabled':m.autonomous_enabled,'block_all':m.block_all,'lease_owner':m.lease_owner_execution_id,'message':m.message}
                event('safety',state=self.safety)
            def command(self,t,m):
                v=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
                self.cmd.append({'topic':t,'values':v,'nonzero':any(abs(x)>1e-8 for x in v),'t0_elapsed_s':elapsed()})
                if self.cmd[-1]['nonzero']:raise RuntimeError('nonzero control command '+t)
            def poll(self):
                if self.lf and self.lf.done():
                    self.cm=self.lf.result().current_state.id;self.latest['lifecycle']=time.monotonic();event('lifecycle',state_id=self.cm);self.lf=None
                for name,client_attr,future_attr in (('map_server','map_lc','map_lf'),('amcl','amcl_lc','amcl_lf')):
                    future=getattr(self,future_attr)
                    if future and future.done():
                        state=future.result().current_state.id
                        self.localization_lifecycle[name]={'state_id':state,'received_wall':time.monotonic()}
                        event('localization_lifecycle',node=name,state_id=state)
                        setattr(self,future_attr,None)
                if self.cf and self.cf.done():
                    self.controllers=[{'name':c.name,'state':c.state,'type':c.type} for c in self.cf.result().controller];self.cf=None;event('controllers',states=self.controllers)
                if self.pf and self.pf.done():
                    self.params=[v.bool_value for v in self.pf.result().values];self.pf=None;event('safety_parameters',names=['block_all','simulation_motion_authorization'],values=self.params)
                if self.amcl_parameter_future and self.amcl_parameter_future.done():
                    future=self.amcl_parameter_future;self.amcl_parameter_future=None
                    try:
                        values=decode_amcl_parameter_values(future.result().values,ParameterType)
                        comparison=compare_amcl_parameters(values,self.expected_amcl_parameters)
                        self.amcl_parameter_readback={'status':'CAPTURED','values':values,'contract':comparison}
                    except Exception as error:
                        self.amcl_parameter_readback={'status':'NOT_CAPTURED','error':repr(error)}
                    event('amcl_parameter_readback',**self.amcl_parameter_readback)
                if self.lf is None and self.lc.service_is_ready():self.lf=self.lc.call_async(GetState.Request())
                if self.map_lf is None and self.map_lc.service_is_ready():self.map_lf=self.map_lc.call_async(GetState.Request())
                if self.amcl_lf is None and self.amcl_lc.service_is_ready():self.amcl_lf=self.amcl_lc.call_async(GetState.Request())
                if self.cf is None and self.cc.service_is_ready():self.cf=self.cc.call_async(ListControllers.Request())
                if self.params is None and self.pf is None and self.pc.service_is_ready():
                    req=GetParameters.Request();req.names=['block_all','simulation_motion_authorization'];self.pf=self.pc.call_async(req)
                if (self.amcl_pc is not None and self.amcl_parameter_readback is None
                        and self.amcl_parameter_future is None and self.amcl_pc.service_is_ready()):
                    req=GetParameters.Request();req.names=list(AMCL_PARAMETER_NAMES)
                    self.amcl_parameter_future=self.amcl_pc.call_async(req)
            def poll_scan_transforms(self,now):
                if args.odometry_mode!='oracle':return
                try:
                    tr=self.buffer.lookup_transform('odom','base_footprint',Time())
                except Exception:
                    if self.scan_tf_enabled:raise
                    return
                if not self.clock_samples or not self.odom_samples:return
                latest_ns=tr.header.stamp.sec*1000000000+tr.header.stamp.nanosec
                if not self.scan_tf_enabled:
                    self.scan_tf_enabled=True;self.scan_tf_anchor_ns=latest_ns
                    for topic,records in self.scans.items():
                        for sample in records:
                            if sample['stamp_ns']<latest_ns:
                                self.scan_tf_pre_anchor.append({'topic':topic,'stamp_ns':sample['stamp_ns']})
                    event('scan_tf_enabled',first_odom_stamp_ns=self.odom_samples[0]['stamp_ns'],
                          first_buffer_tf_stamp_ns=latest_ns,exact_query_anchor_ns=self.scan_tf_anchor_ns,
                          pre_anchor_scan_count=len(self.scan_tf_pre_anchor))
                def lookup(target,source,stamp_ns):
                    transform=self.buffer.lookup_transform(target,source,
                        Time(nanoseconds=stamp_ns,clock_type=ClockType.ROS_TIME))
                    if transform.header.frame_id!=target or transform.child_frame_id!=source:
                        raise RuntimeError('unexpected scan TF frames')
                    return transform.header.stamp.sec*1000000000+transform.header.stamp.nanosec
                count=len(self.scan_tf_completed)
                poll_scan_tf_requests(self.scan_tf_pending,self.scan_tf_completed,lookup,
                    now=now,deadline=t0+145,clock_ns=self.clock_samples[-1],
                    clock_age_s=now-self.clock_advanced_wall,latest_tf_ns=latest_ns)
                for record in self.scan_tf_completed[count:]:event('scan_time_tf_complete',**record)
                for request in self.scan_tf_pending:
                    if request.get('future_attempts') and not request.get('notified'):
                        event('scan_time_tf_wait',**request);request['notified']=True
            def facts(self):
                now=time.monotonic();fresh=lambda k:k in self.latest and now-self.latest[k]<1.0
                active={c['name'] for c in self.controllers if c['state']=='active'}
                facts={'controllers':{'ackermann_controller','joint_state_broadcaster'}<=active,'clock':fresh('clock'),'odom':fresh('odom'),'joints':fresh('joint_states'),
                    'rtab_info':self.info>=3 and fresh('rtab_info'),'lifecycle':self.cm==3 and fresh('lifecycle'),
                    'safety':fresh('safety') and self.safety is not None and self.safety['block_all'] and not self.safety['lease_owner'] and not self.safety['autonomous_enabled'],
                    'motion_authorization_disabled':self.params==[True,False],
                    'zero_cmd_vel_received':any(x['topic']=='/cmd_vel' for x in self.cmd) and not any(x['nonzero'] for x in self.cmd)}
                for topic,count,frame in [('/scan',181,'base_scan'),('/mapping/scan_raw',360,'mapping_lidar_link'),('/mapping/scan',360,'mapping_lidar_link')]:
                    a=self.scans[topic];facts[topic]=len(a)>=10 and fresh(topic) and all(x['count']==count and x['frame']==frame and x['valid_ranges']>0 for x in a[-10:])
                if args.sensor_only:
                    facts.pop('rtab_info')
                tf_records={}
                for target,source in [('odom','base_footprint'),('base_footprint','mapping_lidar_link'),('base_footprint','base_scan'),('base_footprint','base_link'),('map','base_footprint')]:
                    if args.sensor_only and target == 'map' and args.localization_mode != 'amcl':continue
                    key=target+'<-'+source
                    try:
                        tr=self.buffer.lookup_transform(target,source,Time());stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                        age=self.get_clock().now().nanoseconds/1e9-stamp if self.get_parameter('use_sim_time').value else None
                        # Freshness for dynamic paths is checked against the incoming simulation /odom stamp by TF lookup at latest scan time below.
                        scan=self.scans['/mapping/scan']
                        if args.odometry_mode!='oracle' and target in ('odom','map') and scan:
                            self.buffer.lookup_transform(target,source,Time(seconds=max(0,scan[-1]['stamp']-.15)))
                        facts[key]=True;tf_records[key]={'stamp':stamp,'parent':tr.header.frame_id,'child':tr.child_frame_id,'translation':{'x':tr.transform.translation.x,'y':tr.transform.translation.y,'z':tr.transform.translation.z}}
                    except Exception as e:facts[key]=False;tf_records[key]={'error':str(e)}
                if args.odometry_mode=='oracle':
                    facts['scan_time_tf_verified']=set(x['topic'] for x in self.scan_tf_completed)==set(self.scans)
                    facts['frozen_map_received']=self.map_message is not None
                    facts['oracle_tf_authority']=self.tf_parents.get('base_footprint')=={'odom'} and self.tf_parents.get('base_link')=={'base_footprint'}
                    endpoints=self.get_publishers_info_by_topic('/odom')
                    facts['unique_oracle_odom']=len(endpoints)==1 and endpoints[0].node_name=='gazebo_oracle_odometry'
                if args.localization_mode=='amcl':
                    pose_window=self.amcl_samples[-10:]
                    amcl_pose_valid=(len(pose_window)>=2
                        and all(s['frame']=='map' and s['stamp_ns']>0
                            and all(math.isfinite(v) for v in s['position']+s['orientation']+s['covariance_diagonal'])
                            for s in pose_window)
                        and all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(pose_window,pose_window[1:])))
                    facts['amcl_pose']=amcl_pose_valid and fresh('amcl_pose')
                    facts['amcl_pose_time_valid']=bool(self.amcl_samples and self.clock_samples
                        and abs(self.clock_samples[-1]-self.amcl_samples[-1]['stamp_ns'])<1000000000)
                    facts['map_server_active']=self.localization_lifecycle.get('map_server',{}).get('state_id')==3
                    facts['amcl_active']=self.localization_lifecycle.get('amcl',{}).get('state_id')==3
                    facts['map_to_odom_authority']=self.tf_parents.get('odom')=={'map'}
                    facts['map_to_odom_fresh']=len(self.map_to_odom_stamps)>0 and self.clock_samples and abs(self.clock_samples[-1]-self.map_to_odom_stamps[-1])<1000000000
                    facts['map_base_tf']=facts.get('map<-base_footprint') is True
                    facts['localization_graph']= (
                        {e.node_name for e in self.get_publishers_info_by_topic('/map')}=={'map_server'}
                        and {e.node_name for e in self.get_publishers_info_by_topic('/amcl_pose')}=={'amcl'})
                    facts['amcl_parameters_match_contract']=bool(
                        self.amcl_parameter_readback
                        and self.amcl_parameter_readback.get('status')=='CAPTURED'
                        and self.amcl_parameter_readback.get('contract',{}).get('matches_contract'))
                self.tf_records=tf_records
                return facts
        node=Observer()
        pre['domain']=runtime.check_domain_isolated(ros,node)
        if not pre['domain']['ok']:raise RuntimeError('domain conflict')
        window=runtime.collect_resource_stability_window(lambda:runtime.memory_snapshot(cgroup_waiver=waiver,min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES,memavailable_warning_only=args.odometry_mode=='oracle'))
        pre['stability']=window
        pre['domain_after_stability']=runtime.check_domain_isolated(ros,node)
        pre['processes_after_stability']=runtime.process_visibility_and_conflicts();pre['port_after_stability']=runtime.port_free(11704)
        gate=runtime.cgroup_startup_gate(runtime.cgroup_memory_snapshot(),runtime.wsl_cgroup_waiver_preflight(),stable_window=window,auxiliary_gates_ok=all(pre[k]['ok'] for k in ('domain_after_stability','processes_after_stability','port_after_stability','prefixes')))
        pre['cgroup_gate']=gate
        if not gate['ok'] or window['status']!='VERIFIED':raise RuntimeError('cgroup/stability/auxiliary gate rejected')
        if elapsed()>60:raise RuntimeError('preflight used startup/observation budget')
        result['phases']['preflight']='PASS_WITH_EXCEPTION' if gate['cgroup_memory_disposition']=='WAIVED' else 'PASS'
        cmd=launch_argv(out,world,args.sensor_only,args.odometry_mode,args.map_yaml,args.localization_mode);result['launch_command']=cmd
        log=(out/'launch.log').open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True,close_fds=True)
        started=True;result['launch_started']=True;result['launch_pid']=proc.pid;event('launch_started',argv=cmd,pid=proc.pid)
        graph=None;graph_ok=False;observation=None;last_poll=last_resource=last_graph=0;lastfacts={}
        while elapsed()<145:
            ros.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None:raise RuntimeError('launch exited before observation complete: '+str(proc.returncode))
            now=time.monotonic()
            if now-last_poll>=.5:node.poll();last_poll=now
            if now-last_resource>=1:
                snap=runtime.memory_snapshot(cgroup_waiver=waiver,memavailable_warning_only=args.odometry_mode=='oracle');event('resource',snapshot=snap);event('process_group',snapshot=runtime.process_group_snapshot(proc.pid));last_resource=now
                if not snap['ok']:raise RuntimeError('runtime resource threshold rejected')
                oom_now=_read_system(Path('/proc')).get('oom_kill_count');event('oom_counter',count=oom_now,baseline=oom_baseline)
                if oom_baseline is not None and oom_now is not None and oom_now>oom_baseline:raise RuntimeError('kernel OOM kill counter increased')
                if node.safety and (node.safety['lease_owner'] or node.safety['autonomous_enabled'] or not node.safety['block_all']):raise RuntimeError('unexpected motion authorization or Lease')
                if node.odom:
                    x0,y0=node.odom[0][1:3]
                    if max(math.hypot(x-x0,y-y0) for _,x,y,*_ in node.odom)>.05:raise RuntimeError('unexpected odom displacement >0.05m')
            if node.safety is not None and node.cm==3 and now-last_graph>=.1:
                last_graph=now
                if graph is None:graph=PublisherGraphConvergence(time.monotonic_ns())
                decision=graph.sample_query(
                    lambda: node.get_publishers_info_by_topic('/cmd_vel'),
                    time.monotonic_ns,
                )
                event('publisher_graph',decision=decision)
                if graph.failure_reason:raise RuntimeError('publisher graph: '+graph.failure_reason)
                graph_ok=graph.converged
                for raw in ('/cleannav/mapping/cmd_vel_raw','/cleannav/manual/cmd_vel_raw'):
                    extra=[endpoint_record(x) for x in node.get_publishers_info_by_topic(raw)]
                    if extra:raise RuntimeError('unexpected raw command publisher '+raw+': '+str(extra))
            node.poll_scan_transforms(time.monotonic())
            facts=node.facts();lastfacts=facts
            if observation is None and all(facts.values()) and graph_ok:
                if elapsed()>115:raise RuntimeError('insufficient remaining observation budget')
                observation=now;observation_sim=node.clock_samples[-1];result['observation_start_sim_ns']=observation_sim;result['observation_start_elapsed_s']=elapsed();event('observation_started',facts=facts,simulation_stamp_ns=observation_sim)
            if observation is not None:
                if not all(facts.values()):raise RuntimeError('readiness lost during observation: '+','.join(k for k,v in facts.items() if not v))
                observed=(node.clock_samples[-1]-observation_sim)/1e9 if args.odometry_mode=='oracle' else now-observation
                if observed >= (10 if args.odometry_mode=='oracle' else 30):
                    if args.odometry_mode=='oracle':
                        if node.scan_tf_cutoff_ns is None:
                            node.scan_tf_cutoff_ns=node.clock_samples[-1]
                            result['observation_end_sim_ns']=node.scan_tf_cutoff_ns
                            event('scan_tf_window_end',stamp_ns=node.scan_tf_cutoff_ns,pending=len(node.scan_tf_pending))
                        if node.scan_tf_pending:continue
                        observed=(node.scan_tf_cutoff_ns-observation_sim)/1e9
                    result['observation_seconds']=observed
                    result['observation_wall_seconds']=now-observation
                    result['phases'].update(startup='PASS',sensors='PASS',lifecycle='PASS',safety='PASS');break
            elif elapsed()>=115:raise RuntimeError('startup not ready before observation admission cutoff')
        else:raise RuntimeError('cleanup deadline reached')
        result['facts']=lastfacts
    except BaseException as e:
        result['failures'].append(str(e));result['traceback']=traceback.format_exc()
        event('failure',error=str(e),traceback=result['traceback'])
    finally:
        if node is not None:
            result['last_readiness_facts']=locals().get('lastfacts',{})
            result['observations']={'scan_statistics':{},'rtab_info_messages':node.info,'controllers':node.controllers,'collision_monitor_state_id':node.cm,'safety':node.safety,'safety_parameters':node.params,'tf':getattr(node,'tf_records',{}),'command_messages':len(node.cmd),'nonzero_commands':sum(c['nonzero'] for c in node.cmd)}
            for topic,a in node.scans.items():
                dt=a[-1]['stamp']-a[0]['stamp'] if len(a)>1 else 0
                result['observations']['scan_statistics'][topic]={'messages':len(a),'frequency_hz':(len(a)-1)/dt if dt>0 else None,'frames':sorted(set(x['frame'] for x in a)),'beam_counts':sorted(set(x['count'] for x in a))}
            write(out/'commands.json',node.cmd)
            write(out/'odom.json',node.odom)
            if args.odometry_mode=='oracle':
                try:
                    result['oracle_runtime']=oracle_observation_stats(node,out,args.map_yaml,args.localization_mode)
                    write(out/'joint-samples.json',node.joint_samples)
                    write(out/'scan-time-tf.json',node.scan_tf_completed)
                    result['scan_time_tf']={'enqueued':node.scan_tf_enqueued,'completed':len(node.scan_tf_completed),
                        'pending':node.scan_tf_pending,'wait_bound_wall_s':SCAN_TF_WAIT_S,
                        'exact_query_anchor_ns':node.scan_tf_anchor_ns,
                        'pre_buffer_anchor_scans':node.scan_tf_pre_anchor,
                        'future_waited_queries':sum(x['future_attempts']>0 for x in node.scan_tf_completed),
                        'max_wait_wall_s':max((x['wait_wall_s'] for x in node.scan_tf_completed),default=None),
                        'exact_scan_timestamps':all(x['returned_stamps_ns']==[x['stamp_ns']]*2 for x in node.scan_tf_completed),
                        'dynamic_tf_stamps_ns':node.dynamic_tf_stamps}
                    if node.scan_tf_pending or node.scan_tf_enqueued!=len(node.scan_tf_completed):
                        result['failures'].append('scan-time TF requests unresolved')
                    if not result['oracle_runtime']['pass']:result['failures'].append('Oracle/map/TF runtime comparison failed')
                except Exception as error:result['failures'].append('Oracle evidence: '+repr(error))
            if args.localization_mode=='amcl':
                write(out/'amcl-pose-samples.json',node.amcl_samples)
                write(out/'map-to-odom-tf.json',node.map_to_odom_stamps)
                session_files=list((out/'sessions'/'.sessions').glob('*/session.json'))
                parameter_sources=None
                if len(session_files)==1:
                    try:
                        parameter_sources=json.loads(session_files[0].read_text()).get('amcl_parameter_sources')
                    except Exception:
                        parameter_sources=None
                effective=node.amcl_parameter_readback or {'status':'NOT_CAPTURED','reason':'GetParameters response was not captured'}
                result['amcl_parameters']={
                    'contract_expected':amcl_expected or {'status':'NOT_CAPTURED'},
                    'requested':(parameter_sources or {}).get('requested','NOT_CAPTURED'),
                    'yaml_defaults':(parameter_sources or {}).get('yaml_defaults','NOT_CAPTURED'),
                    'effective':effective,
                    'source_status':'CAPTURED' if parameter_sources else 'NOT_CAPTURED',
                }
                write(out/'amcl-parameters.json',result['amcl_parameters'])
                try:
                    result['localization_runtime']=amcl_observation_stats(node,out)
                    if result['localization_runtime']['pass']:
                        result['phases']['localization']='PASS'
                    else:
                        result['failures'].append('AMCL localization observations or map->odom authority failed')
                except Exception as error:result['failures'].append('AMCL evidence: '+repr(error))
            if 'graph' in locals() and graph is not None:write(out/'publisher-graph.json',graph.result())
        result['cleanup_started_elapsed_s']=elapsed()
        event('cleanup_started',snapshot=runtime.process_group_snapshot(proc.pid) if proc else None)
        cleanup=perform_cleanup(proc,started,t0)
        if started:
            cleanup['strict_no_zombies_ok']=cleanup_result_ok(cleanup)
            result['phases']['cleanup']='PASS' if cleanup['strict_no_zombies_ok'] else 'FAIL'
        else:result['phases']['cleanup']='NOT RUN'
        write(out/'cleanup.json',cleanup);result['cleanup']=cleanup
        if log:log.close()
        if node:node.destroy_node()
        if ros:ros.try_shutdown()
        for fd in fds:os.close(fd)
        result['elapsed_s']=elapsed()
        if result['elapsed_s']>180:result['failures'].append('total 180s budget exceeded')
        if started and not cleanup.get('strict_no_zombies_ok'):result['failures'].append('strict cleanup failed')
        if result['failures']:
            result['first_failure']=result['failures'][0]
            if not started:result['phases']['preflight']='BLOCKED'
        required_phases=('startup','sensors','lifecycle','safety','cleanup') + (('localization',) if args.localization_mode=='amcl' else ())
        result['test_pass']=not result['failures'] and all(result['phases'][k]=='PASS' for k in required_phases)
        result['exit_code']=0 if result['test_pass'] else 1
        write(out/'result.json',result)
        for stream in streams:stream.close()
        print(json.dumps({'run_dir':str(out),'exit_code':result['exit_code'],'test_pass':result['test_pass'],'first_failure':result.get('first_failure'),'elapsed_s':result['elapsed_s']}),flush=True)
    return result['exit_code']
if __name__=='__main__':sys.exit(main())
