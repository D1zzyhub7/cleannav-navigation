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
from publisher_graph_gate import PublisherGraphConvergence, endpoint_record, publisher_snapshot_error

def write(path,data):
    path.write_text(json.dumps(data,indent=2,default=str)+'\n')

def parse_args(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument('--mode',choices=['startup_smoke'],default='startup_smoke')
    p.add_argument('--run-id',required=True)
    a=p.parse_args(argv)
    if not a.run_id.startswith('startup-smoke-') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in a.run_id):
        p.error('run-id must be a unique startup-smoke-* identifier')
    return a

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

def launch_argv(out):
    return ['ros2','launch',str(NAV/'tools/startup_smoke/startup_smoke.launch.py'),
            'mode:=startup_smoke','maps_root:='+str(out/'sessions'),
            'world:='+str(WORLD),'model_path:='+str(MODEL_ASSETS/'models')]

def cleanup_result_ok(c):
    s=c.get('process_group_snapshot_after_cleanup',{})
    return c.get('cleanup_ok') is True and s.get('scan_complete') is True and not s.get('running_members') and not s.get('zombie_members') and s.get('status')=='EMPTY'

def perform_cleanup(proc,started,t0):
    try:return runtime.signal_and_reap(proc,started,t0)
    except BaseException as error:
        return {"started":started,"cleanup_ok":False,"status":"UNKNOWN","error":repr(error),"traceback":traceback.format_exc(),"process_group_snapshot_after_cleanup":runtime.process_group_snapshot(proc.pid) if proc else {}}

def main():
    args=parse_args();t0=time.monotonic();out=OUTPUT_ROOT/args.run_id
    out.mkdir(parents=True,exist_ok=False)
    for name in ('ros-home','ros-log'): (out/name).mkdir()
    os.environ.update(ROS_HOME=str(out/'ros-home'),ROS_LOG_DIR=str(out/'ros-log'))
    result={'run_id':args.run_id,'mode':args.mode,'navigation_repository':str(NAV),'workspace_root':str(ROOT),'output_root':str(OUTPUT_ROOT),'utc':datetime.now(timezone.utc).isoformat(),
        'cwd':os.getcwd(),'pid':os.getpid(),'ppid':os.getppid(),'pid1':Path('/proc/1/comm').read_text().strip(),
        'dynamic_entry_calls':1,'total_limit_s':180,'cleanup_start_deadline_s':145,'observation_required_s':30,
        'launch_started':False,'test_pass':False,'failures':[],'phases':{k:'NOT RUN' for k in ('preflight','startup','sensors','lifecycle','safety','cleanup')}}
    proc=None;log=None;node=None;fds=[];ros=None;started=False;streams=[];waiver=None
    def elapsed():return time.monotonic()-t0
    def event(kind,**data):
        record={'event':kind,'t0_elapsed_s':elapsed(),'monotonic_ns':time.monotonic_ns(),**data}
        for s in streams:s.write(json.dumps(record,default=str)+'\n');s.flush()
    try:
        streams=[(out/'events.jsonl').open('w')]
        def interrupted(signum, _frame): raise RuntimeError('external signal '+str(signum))
        signal.signal(signal.SIGTERM,interrupted)
        result['identity']=verify_identity()
        fds,result['locks']=runtime.acquire_locks()
        pre=result['preflight']={}
        pre['source_paths']={'world':str(WORLD),'world_exists':WORLD.is_file(),
            'model_path':str(MODEL_ASSETS/'models'),'model_path_exists':(MODEL_ASSETS/'models').is_dir()}
        if not pre['source_paths']['world_exists'] or not pre['source_paths']['model_path_exists']:
            raise RuntimeError('repository world or workspace model assets are unavailable')
        pre['environment']={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_DISTRO','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','GAZEBO_MASTER_URI')}
        if os.environ.get('ROS_DOMAIN_ID')!='204' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('unexpected ROS domain/environment')
        for name,value in [('processes',runtime.process_visibility_and_conflicts()),('port',runtime.port_free(11704)),('prefixes',runtime.check_package_prefixes())]:
            pre[name]=value
            if not value.get('ok'):raise RuntimeError('preflight '+name+' rejected')
        pre['cgroup']=runtime.cgroup_memory_snapshot();waiver=runtime.wsl_cgroup_waiver_preflight();pre['waiver']=waiver
        pre['memory']=runtime.memory_snapshot(cgroup_waiver=waiver,min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES)
        if not pre['memory']['ok']:raise RuntimeError('resource preflight rejected')
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan,JointState
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rtabmap_msgs.msg import Info
        from cleannav_interfaces.msg import SafetyStatus
        from lifecycle_msgs.srv import GetState
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        from tf2_ros import Buffer,TransformListener
        from rclpy.time import Time
        ros=rclpy;ros.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        class Observer(Node):
            def __init__(self):
                super().__init__('cleannav_startup_smoke_observer')
                self.scans={k:[] for k in ('/scan','/mapping/scan_raw','/mapping/scan')}
                self.latest={};self.odom=[];self.cmd=[];self.info=0;self.safety=None;self.controllers=[];self.cm=None
                self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
                for topic in self.scans:self.create_subscription(LaserScan,topic,lambda m,t=topic:self.scan(t,m),qos_profile_sensor_data)
                self.create_subscription(Clock,'/clock',lambda m:self.mark('clock',m),qos_profile_sensor_data)
                self.create_subscription(Odometry,'/odom',self.on_odom,10)
                self.create_subscription(Info,'/rtabmap/info',self.on_info,10)
                self.create_subscription(SafetyStatus,'/cleannav/safety_status',self.on_safety,10)
                self.create_subscription(JointState,'/joint_states',lambda m:self.mark('joint_states',m),10)
                for topic in ('/cmd_vel','/cleannav/cmd_vel_candidate','/cleannav/mapping/cmd_vel_raw','/cleannav/manual/cmd_vel_raw','/ackermann_controller/reference_unstamped'):
                    self.create_subscription(Twist,topic,lambda m,t=topic:self.command(t,m),10)
                self.lc=self.create_client(GetState,'/collision_monitor/get_state');self.lf=None
                self.cc=self.create_client(ListControllers,'/controller_manager/list_controllers');self.cf=None
                self.pc=self.create_client(GetParameters,'/cleannav_safety_supervisor/get_parameters');self.pf=None;self.params=None
            def mark(self,k,m):self.latest[k]=time.monotonic()
            def scan(self,t,m):
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                valid=sum(math.isfinite(v) and m.range_min<=v<=m.range_max for v in m.ranges)
                x={'wall_monotonic':time.monotonic(),'stamp':stamp,'count':len(m.ranges),'frame':m.header.frame_id,'valid_ranges':valid,'angle_min':m.angle_min,'angle_max':m.angle_max,'angle_increment':m.angle_increment,'range_max':m.range_max}
                self.scans[t].append(x);self.latest[t]=time.monotonic()
                if self.scans[t][:-1] and stamp<=self.scans[t][-2]['stamp']:raise RuntimeError('non advancing scan stamp '+t)
                event('scan',topic=t,**x)
            def on_odom(self,m):
                self.mark('odom',m);p=m.pose.pose.position
                self.odom.append([time.monotonic(),p.x,p.y,m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.frame_id,m.child_frame_id])
                event('odom',sample=self.odom[-1])
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
                if self.cf and self.cf.done():
                    self.controllers=[{'name':c.name,'state':c.state,'type':c.type} for c in self.cf.result().controller];self.cf=None;event('controllers',states=self.controllers)
                if self.pf and self.pf.done():
                    self.params=[v.bool_value for v in self.pf.result().values];self.pf=None;event('safety_parameters',names=['block_all','simulation_motion_authorization'],values=self.params)
                if self.lf is None and self.lc.service_is_ready():self.lf=self.lc.call_async(GetState.Request())
                if self.cf is None and self.cc.service_is_ready():self.cf=self.cc.call_async(ListControllers.Request())
                if self.params is None and self.pf is None and self.pc.service_is_ready():
                    req=GetParameters.Request();req.names=['block_all','simulation_motion_authorization'];self.pf=self.pc.call_async(req)
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
                tf_records={}
                for target,source in [('odom','base_footprint'),('base_footprint','mapping_lidar_link'),('base_footprint','base_scan'),('map','base_footprint')]:
                    key=target+'<-'+source
                    try:
                        tr=self.buffer.lookup_transform(target,source,Time());stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                        age=self.get_clock().now().nanoseconds/1e9-stamp if self.get_parameter('use_sim_time').value else None
                        # Freshness for dynamic paths is checked against the incoming simulation /odom stamp by TF lookup at latest scan time below.
                        scan=self.scans['/mapping/scan']
                        if target in ('odom','map') and scan:
                            self.buffer.lookup_transform(target,source,Time(seconds=max(0,scan[-1]['stamp']-.15)))
                        facts[key]=True;tf_records[key]={'stamp':stamp,'parent':tr.header.frame_id,'child':tr.child_frame_id,'translation':{'x':tr.transform.translation.x,'y':tr.transform.translation.y,'z':tr.transform.translation.z}}
                    except Exception as e:facts[key]=False;tf_records[key]={'error':str(e)}
                self.tf_records=tf_records
                return facts
        node=Observer()
        pre['domain']=runtime.check_domain_isolated(ros,node)
        if not pre['domain']['ok']:raise RuntimeError('domain conflict')
        window=runtime.collect_resource_stability_window(lambda:runtime.memory_snapshot(cgroup_waiver=waiver,min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES))
        pre['stability']=window
        pre['domain_after_stability']=runtime.check_domain_isolated(ros,node)
        pre['processes_after_stability']=runtime.process_visibility_and_conflicts();pre['port_after_stability']=runtime.port_free(11704)
        gate=runtime.cgroup_startup_gate(runtime.cgroup_memory_snapshot(),runtime.wsl_cgroup_waiver_preflight(),stable_window=window,auxiliary_gates_ok=all(pre[k]['ok'] for k in ('domain_after_stability','processes_after_stability','port_after_stability','prefixes')))
        pre['cgroup_gate']=gate
        if not gate['ok'] or window['status']!='VERIFIED':raise RuntimeError('cgroup/stability/auxiliary gate rejected')
        if elapsed()>60:raise RuntimeError('preflight used startup/observation budget')
        result['phases']['preflight']='PASS_WITH_EXCEPTION' if gate['cgroup_memory_disposition']=='WAIVED' else 'PASS'
        cmd=launch_argv(out);result['launch_command']=cmd
        log=(out/'launch.log').open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True,close_fds=True)
        started=True;result['launch_started']=True;result['launch_pid']=proc.pid;event('launch_started',argv=cmd,pid=proc.pid)
        graph=None;graph_ok=False;observation=None;last_poll=last_resource=last_graph=0;lastfacts={}
        while elapsed()<145:
            ros.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None:raise RuntimeError('launch exited before observation complete: '+str(proc.returncode))
            now=time.monotonic()
            if now-last_poll>=.5:node.poll();last_poll=now
            if now-last_resource>=1:
                snap=runtime.memory_snapshot(cgroup_waiver=waiver);event('resource',snapshot=snap);event('process_group',snapshot=runtime.process_group_snapshot(proc.pid));last_resource=now
                if not snap['ok']:raise RuntimeError('runtime resource threshold rejected')
                if node.safety and (node.safety['lease_owner'] or node.safety['autonomous_enabled'] or not node.safety['block_all']):raise RuntimeError('unexpected motion authorization or Lease')
                if node.odom:
                    x0,y0=node.odom[0][1:3]
                    if max(math.hypot(x-x0,y-y0) for _,x,y,*_ in node.odom)>.05:raise RuntimeError('unexpected odom displacement >0.05m')
            if node.safety is not None and node.cm==3 and now-last_graph>=.1:
                last_graph=now
                if graph is None:graph=PublisherGraphConvergence(time.monotonic_ns())
                if not graph_ok:
                    decision=graph.sample(node.get_publishers_info_by_topic('/cmd_vel'),time.monotonic_ns());event('publisher_graph',decision=decision)
                    if graph.failure_reason:raise RuntimeError('publisher graph: '+graph.failure_reason)
                    graph_ok=graph.converged
                else:
                    ends=[endpoint_record(x) for x in node.get_publishers_info_by_topic('/cmd_vel')]
                    why=publisher_snapshot_error(ends)
                    if why:raise RuntimeError('publisher graph after convergence: '+why)
                    for raw in ('/cleannav/mapping/cmd_vel_raw','/cleannav/manual/cmd_vel_raw'):
                        extra=[endpoint_record(x) for x in node.get_publishers_info_by_topic(raw)]
                        if extra:raise RuntimeError('unexpected raw command publisher '+raw+': '+str(extra))
            facts=node.facts();lastfacts=facts
            if observation is None and all(facts.values()) and graph_ok:
                if elapsed()>115:raise RuntimeError('insufficient remaining observation budget')
                observation=now;result['observation_start_elapsed_s']=elapsed();event('observation_started',facts=facts)
            if observation is not None:
                if not all(facts.values()):raise RuntimeError('readiness lost during observation: '+','.join(k for k,v in facts.items() if not v))
                if now-observation>=30:
                    result['observation_seconds']=now-observation
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
        result['test_pass']=not result['failures'] and all(result['phases'][k]=='PASS' for k in ('startup','sensors','lifecycle','safety','cleanup'))
        result['exit_code']=0 if result['test_pass'] else 1
        write(out/'result.json',result)
        for stream in streams:stream.close()
        print(json.dumps({'run_dir':str(out),'exit_code':result['exit_code'],'test_pass':result['test_pass'],'first_failure':result.get('first_failure'),'elapsed_s':result['elapsed_s']}),flush=True)
    return result['exit_code']
if __name__=='__main__':sys.exit(main())
