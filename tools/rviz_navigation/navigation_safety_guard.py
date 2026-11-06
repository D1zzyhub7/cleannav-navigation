#!/usr/bin/env python3
"""Observe the joined chain, publish its existing footprint, fail through ESTOP.

No velocity publisher, no Lease client, no motion-authorization service call.
The existing installed localization monitor remains the localization authority.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from guard_policy import (
    command_error, diagnostic_command_graph_errors,
    diagnostic_continuation_decision, missing_inputs,
)
from readiness_inputs import InputEvidence
from cm_state_poll import CMStatePoll
from supervisor_policy import MAX_NO_MOTION_DISPLACEMENT_M, MAX_HISTORICAL_ODOMETRY_M, historical_route_budget, observe_safety_sample


def localization_bootstrap_ready(bootstrap_ready, state, received_at, now):
    """Call the installed health contract with state first and a fresh age.

    Health arrives as JSON, so reject malformed shapes before delegating to
    the installed helper. This preserves its existing 1.5 s freshness rule
    and fails closed for negative/non-finite ages or malformed `info` data.
    """
    if isinstance(state,dict) and state.get('localization_kind')=='amcl':
        from readiness_inputs import amcl_bootstrap_ready
        return amcl_bootstrap_ready(state,now-received_at)
    if not isinstance(state, dict) or not isinstance(state.get('info', {}), dict):
        return False
    matches = state.get('info', {}).get('matches', 0)
    if (isinstance(matches, bool) or not isinstance(matches, (int, float))
            or not math.isfinite(matches)):
        return False
    if (isinstance(received_at, bool) or not isinstance(received_at, (int, float))
            or isinstance(now, bool) or not isinstance(now, (int, float))):
        return False
    age = now - received_at
    if not math.isfinite(age) or age < 0.0:
        return False
    return bool(bootstrap_ready(state, age))


def publish_shutdown_estop(context_ok, publish):
    """Publish shutdown ESTOP only while the ROS context can accept it.

    rclpy's SIGINT handler can invalidate the context before this process
    reaches its ``finally`` block. A publish attempted after that point raises
    RCLError and turns an otherwise completed shutdown into a child failure.
    If the context closes between the check and publish, report the delivery
    as unavailable; propagate unrelated errors while the context remains up.
    """
    if not context_ok():
        return False
    try:
        publish()
    except Exception:
        if context_ok():
            raise
        return False
    return True


# InputEvidence already accepts a sensor stamp up to 0.25 s ahead of the
# guard's latest /clock sample. A costmap publish can lead that sample by one
# clock step while the callback is still queued. That lead is not staleness.
# The caller-supplied limit remains the stale bound.
COSTMAP_STAMP_LEAD_S = 0.25


def costmap_stamp_current(clock_ns, stamp_ns, limit_s):
    if (isinstance(clock_ns, bool) or not isinstance(clock_ns, int)
            or isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int)
            or isinstance(limit_s, bool) or not isinstance(limit_s, (int, float))
            or not math.isfinite(limit_s) or limit_s < 0):
        return False
    age_ns = clock_ns - stamp_ns
    return -int(COSTMAP_STAMP_LEAD_S * 1_000_000_000) <= age_ns <= int(limit_s * 1_000_000_000)


def scan_range_summary(ranges, range_min, range_max):
    """Return read-only value categories for one LaserScan message.

    This is diagnostic only: the existing admission rule still requires at
    least one finite in-range return for the safety and mapping scans.
    """
    summary = {
        'beam_count': len(ranges), 'finite_in_range': 0, 'finite_below_min': 0,
        'finite_above_max': 0, 'nan': 0, 'positive_infinity': 0,
        'negative_infinity': 0,
    }
    for value in ranges:
        if math.isnan(value):
            summary['nan'] += 1
        elif math.isinf(value):
            summary['positive_infinity' if value > 0 else 'negative_infinity'] += 1
        elif value < range_min:
            summary['finite_below_min'] += 1
        elif value > range_max:
            summary['finite_above_max'] += 1
        else:
            summary['finite_in_range'] += 1
    return summary


def scan_admission(topic, frame, ranges, angle_min, angle_max, angle_increment,
                   range_min, range_max, *, localization_scan_topic):
    """Validate scan structure while distinguishing no-return rays from bad data.

    The localization scan requires at least one finite in-range return. Any
    other scan may contain only +Inf, including /scan when localization
    consumes /mapping/scan. That is a valid no-hit observation, not an
    obstacle or a localization certificate.
    """
    count, expected_frame = ((181, 'base_scan') if topic == '/scan'
                             else (360, 'mapping_lidar_link'))
    parameters_valid = (
        len(ranges) == count and frame == expected_frame
        and all(math.isfinite(x) for x in
                (angle_min, angle_max, angle_increment, range_min, range_max))
        and angle_increment > 0 and 0 <= range_min < range_max)
    finite_count = sum(math.isfinite(x) and range_min <= x <= range_max for x in ranges)
    range_values_valid = all(
        (math.isfinite(x) and range_min <= x <= range_max)
        or (math.isinf(x) and x > 0)
        for x in ranges)
    finite_return_required = topic == localization_scan_topic
    valid = parameters_valid and (
        finite_count > 0 if finite_return_required else range_values_valid)
    if not parameters_valid:
        reason = 'invalid_scan_parameters_or_identity'
    elif not range_values_valid:
        reason = 'invalid_range_value'
    elif finite_return_required and finite_count == 0:
        reason = 'no_finite_return_on_localization_or_safety_scan'
    else:
        reason = None
    return {
        'valid': valid,
        'reason': reason,
        'finite_ranges': finite_count,
        'finite_return_required': finite_return_required,
        'return_status': 'FINITE_RETURN' if finite_count else 'NO_RETURN_WITHIN_RANGE',
        'range_values_valid': range_values_valid,
    }


def readiness_transition_records(run_id, previous, current, observations, *,
                                 monotonic_ns, wall_time_ns):
    """Persist each boolean readiness edge with the exact age/reason evidence."""
    records=[]
    for key,value in sorted(current.items()):
        old=previous.get(key)
        if old is not None and old == value:
            continue
        observation=observations.get(key)
        if observation is None and key.endswith('_tf'):
            observation=observations.get(key[:-3])
        records.append({
            'run_id':run_id,'stage':'no_motion_guard','monotonic_ns':monotonic_ns,
            'wall_time_ns':wall_time_ns,'check':key,'previous':old,'value':value,
            'reason':(observation or {}).get('reason'),
            'receive_age_s':(observation or {}).get('receive_age_s'),
            'stamp_age_s':(observation or {}).get('stamp_age_s'),
            'freshness_limit_s':(observation or {}).get('freshness_limit_s'),
            'evidence':observation,
        })
    return records


def explicit_failure_state(failed):
    """Serialize a healthy guard state as explicit false, not JSON null.

    The supervisor's diagnostic-continuation contract treats a missing or
    non-boolean failure field as unknown and fails closed. ``self.failed`` is
    initialized to ``None`` for internal control flow, so receipts must map
    that healthy internal state to the explicit boolean ``False``.
    """
    return False if failed is None else failed


def wait_ready(path, timeout):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            state=json.loads(Path(path).read_text())
            if state.get('failed'):return 1
            if state.get('ready') and 0 <= time.monotonic()-state['monotonic'] <= .5:return 0
        except (OSError,ValueError,KeyError):pass
        time.sleep(.05)
    return 1


class ReceiptWriteError(RuntimeError):
    """A receipt did not reach disk. Callers must not treat the snapshot as saved."""


class ReceiptWriter:
    """Bounded current receipt; one background thread owns the files.

    ``wait_ready`` and the supervisor both reject a receipt older than 0.5 s.
    The ROS callback only queues a snapshot. Healthy snapshots still waiting
    to be popped are folded into the latest one. A failure, or a snapshot
    that records a readiness transition, stays in order ahead of that latest
    snapshot. ``close`` writes whatever is still queued.
    """
    def __init__(self, output, run_id):
        self.output = Path(output)
        self.run_id = run_id
        self.sequence = 0
        self._wake = threading.Condition()
        self._queue = []
        self._latest = None
        self._closed = False
        self._error = None
        self._inflight = False
        self._committed_sequence = 0
        self._thread = None

    def _ensure_thread(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='safety-guard-receipt')
            self._thread.start()

    @staticmethod
    def _must_keep(value):
        return value.get('failed') not in (None, False) or bool(value.get('readiness_transitions'))

    def timing(self, event, safety, **fields):
        checked = time.monotonic_ns()
        record = dict(run_id=self.run_id, event=event,
                      receiver_sequence=getattr(self, 'safety_sequence', 0),
                      check_monotonic_ns=checked, wall_time_ns=time.time_ns(),
                      safety=safety,
                      age_ns=None if not safety else checked-safety['monotonic_ns'],
                      **fields)
        with self.output.with_name('safety-timing.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')

    def write(self, state):
        with self._wake:
            if self._error is not None:
                raise ReceiptWriteError(self._error)
            if self._closed:
                raise ReceiptWriteError('safety guard receipt writer is closed')
            self._ensure_thread()
            self.sequence += 1
            value = dict(state, run_id=self.run_id, receipt_sequence=self.sequence,
                         writer_pid=os.getpid(), write_started_monotonic_ns=time.monotonic_ns())
            if 'graph_history' in value:
                value['graph_history'] = value['graph_history'][-5:]
            if self._must_keep(value):
                if self._latest is not None:
                    self._queue.append(self._latest)
                    self._latest = None
                self._queue.append(value)
            else:
                self._latest = value
            self._wake.notify()
        return value

    def _pop(self):
        if self._queue:
            return self._queue.pop(0)
        if self._latest is not None:
            value = self._latest
            self._latest = None
            return value
        return None

    def _run(self):
        while True:
            with self._wake:
                while not self._queue and self._latest is None and not self._closed:
                    self._wake.wait()
                value = self._pop()
                if value is None:
                    return
                self._inflight = True
            try:
                self._commit(value)
            except Exception as error:
                message = f'{type(error).__name__}: {error}'
                print(f'[SAFETY_GUARD] receipt write failed: {message}', file=sys.stderr)
                try:
                    self.output.with_name('safety-receipt-error.json').write_text(
                        json.dumps({'run_id': self.run_id, 'receipt_sequence': value.get('receipt_sequence'),
                                    'error': message}) + '\n')
                except OSError as secondary:
                    print(f'[SAFETY_GUARD] receipt error record failed: {secondary}', file=sys.stderr)
                with self._wake:
                    self._error = message
                    self._inflight = False
                    self._queue.clear()
                    self._latest = None
                    self._wake.notify_all()
                return
            with self._wake:
                self._inflight = False
                self._committed_sequence = value['receipt_sequence']
                self._wake.notify_all()

    def _commit(self, value):
        serialized = json.dumps(value, default=str, allow_nan=False) + '\n'
        serialized_at = time.monotonic_ns()
        pending = self.output.with_suffix('.tmp')
        pending.write_text(serialized)
        replace_started = time.monotonic_ns()
        pending.replace(self.output)
        replaced = time.monotonic_ns()
        with self.output.with_name('safety-input-observations.jsonl').open('a') as stream:
            stream.write(serialized)
        finished = time.monotonic_ns()
        audit = dict(run_id=self.run_id, receipt_sequence=value['receipt_sequence'],
                     snapshot_monotonic=value.get('monotonic'),
                     write_started_monotonic_ns=value['write_started_monotonic_ns'],
                     serialization_finished_monotonic_ns=serialized_at,
                     replace_started_monotonic_ns=replace_started,
                     replace_finished_monotonic_ns=replaced,
                     history_finished_monotonic_ns=finished,
                     file_mtime_ns=self.output.stat().st_mtime_ns,
                     receipt_bytes=len(serialized.encode()), writer_pid=os.getpid())
        with self.output.with_name('safety-receipt-writes.jsonl').open('a') as stream:
            stream.write(json.dumps(audit) + '\n')

    def flush(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        with self._wake:
            while True:
                if self._error is not None:
                    raise ReceiptWriteError(self._error)
                busy = bool(self._queue) or self._latest is not None or self._inflight
                if not busy and (self._thread is None or self._committed_sequence >= self.sequence):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ReceiptWriteError('safety guard receipt flush timed out')
                self._wake.wait(remaining)

    def close(self, timeout=5.0):
        with self._wake:
            self._closed = True
            self._wake.notify()
            thread = self._thread
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive():
                raise ReceiptWriteError('safety guard receipt writer did not stop')
        if self._error is not None:
            raise ReceiptWriteError(self._error)


def main(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument('--wait-ready');p.add_argument('--timeout',type=float,default=110)
    p.add_argument('--output');p.add_argument('--motion-enabled',choices=['true','false'],default='false')
    p.add_argument('--odometry-mode',choices=['wheel','oracle'],default='wheel')
    p.add_argument('--motion-profile',choices=['none','short-goal','historical','patrol'],default='none')
    p.add_argument('--run-id', default=os.environ.get('CLEANNAV_RVIZ_RUN_ID'))
    p.add_argument('--localization-scan-topic', choices=['/scan','/mapping/scan'],
                   default='/mapping/scan')
    p.add_argument('--diagnostic-continue-on-readiness-loss', action='store_true')
    args,ros_args=p.parse_known_args(argv)
    if (args.motion_enabled == 'true') != (args.motion_profile in ('short-goal','historical','patrol')):
        p.error('motion-enabled and motion-profile identities disagree')
    if args.diagnostic_continue_on_readiness_loss and args.motion_enabled != 'false':
        p.error('--diagnostic-continue-on-readiness-loss requires --motion-enabled false')
    if args.wait_ready:return wait_ready(args.wait_ready,args.timeout)
    if not args.output:p.error('--output is required for the observer')
    if not args.run_id:p.error('--run-id or CLEANNAV_RVIZ_RUN_ID is required for the observer')
    import math
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PolygonStamped,Point32,Twist
    from nav_msgs.msg import Odometry, Path as NavigationPath, OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMessage
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool,String
    from cleannav_interfaces.msg import SafetyStatus
    from lifecycle_msgs.srv import GetState
    from action_msgs.srv import CancelGoal
    from tf2_ros import Buffer,TransformListener
    sys.path.insert(0,str(Path(get_package_prefix('cleannav_simulation'))/'lib/cleannav_simulation'))
    from mapping_support import MAPPING_FOOTPRINT_POINTS
    from localization_health import bootstrap_ready
    sys.path.insert(0,str(HERE.parent/'startup_smoke'))
    from publisher_graph_gate import PublisherGraphConvergence,endpoint_record
    from short_goal_policy import (EXPECTED_EXECUTION_ID, command_policy,
                                   validate_lifecycle)
    output=Path(args.output);motion=args.motion_enabled=='true'
    short_goal=args.motion_profile=='short-goal'
    patrol=args.motion_profile=='patrol'
    historical=args.motion_profile in ('historical','patrol')
    class Guard(Node):
        def __init__(self):
            super().__init__('cleannav_navigation_safety_guard')
            self.start=time.monotonic();self.latest={};self.scans={};self.scan_diagnostics={};self.safety=None;self.safety_sync_deadline_ns=None;self.goal_received_ns=None
            self.inputs=InputEvidence();self.last_state={};self.last_facts=None
            self.amcl_goal=(short_goal and args.odometry_mode=='oracle')
            self.grid_status={};self.grid_messages={}
            if self.amcl_goal:
                session=json.loads(output.with_name('session.json').read_text())
                self.expected_grid=json.loads((Path(session['map_directory'])/'occupancy_grid.json').read_text())
                for name,topic in [('static','/cleannav/localization/map'),('global','/global_costmap/costmap'),('local','/local_costmap/costmap')]:
                    self.create_subscription(OccupancyGrid,topic,lambda m,n=name:self.on_grid(n,m),
                        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.health=None;self.cm=None;self.cm_poll=CMStatePoll();self.failed=None;self.armed=False
            self.graph=None;self.graph_ok=False;self.graph_history=[];self.distance=0.;self.last_position=None
            self.last_fact_values={};self.command_publishers={};self.last_publisher_snapshot=None
            self.rviz_goal_seen=False;self.mission_goal_accepted=False
            self.mission_runtime=None
            self.expected_owner=EXPECTED_EXECUTION_ID if not patrol else ''
            self.sequence_owners=set()
            self.route_budget=None;self.goal_position=None;self.goal_stamp_ns=None;self.goal_distance_base=0.
            if patrol:
                from cleannav_interfaces.msg import TaskSequenceStatus
                from integration import HERE as CONTRACT_DIR
                self.patrol_spec=json.loads((CONTRACT_DIR/'map_contract_patrol_a_v4.json').read_text())['patrol']
                self.route_budget=dict(odometry_m=self.patrol_spec['max_odometry_m'],task_s=self.patrol_spec['task_timeout_s'],odometry_origin_m=0.,command_id=args.run_id+'-patrol')
                self.create_subscription(TaskSequenceStatus,'/cleannav/task_sequence_status',self.on_sequence,10)
            if historical and not patrol:
                self.create_subscription(NavigationPath,'/cleannav/navigation/admitted_path',self.on_admitted_path,10)
            self.mission_command_id='';self.mission_terminal=False
            self.command_samples={topic:0 for topic in (
                '/cleannav/navigation/cmd_vel_raw','/cleannav/cmd_vel_candidate','/cmd_vel')}
            self.command_nonzero={topic:0 for topic in self.command_samples}
            self.command_last={}
            self.receipt_writer=ReceiptWriter(output,args.run_id)
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.footprint=self.create_publisher(PolygonStamped,'/cleannav/mapping/robot_footprint',1)
            self.estop=self.create_publisher(Bool,'/cleannav/safety/emergency_stop',10)
            self.cancel=self.create_client(CancelGoal,('/cleannav/navigate_within_polygon/_action/cancel_goal' if patrol else '/cleannav/navigate_to_pose/_action/cancel_goal'))
            self.lc=self.create_client(GetState,'/collision_monitor/get_state')
            for topic in ('/scan','/mapping/scan','/mapping/scan_raw'):
                self.create_subscription(LaserScan,topic,lambda m,t=topic:self.on_scan(t,m),qos_profile_sensor_data)
            self.create_subscription(ClockMessage,'/clock',self.on_clock,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/odom',self.on_odom,10)
            self.create_subscription(SafetyStatus,'/cleannav/safety_status',self.on_safety,10)
            self.create_subscription(String,'/cleannav/localization/health',self.on_health,10)
            if short_goal:
                self.create_subscription(String,'/cleannav/simulation/mission_runtime',self.on_mission_runtime,10)
            for topic in self.command_samples:
                self.create_subscription(Twist,topic,
                    lambda msg,t=topic:self.on_command(msg,t),10)
            if short_goal or (historical and not patrol):
                self.create_subscription(String,'/cleannav/simulation/rviz_goal_events',
                                         self.on_rviz_event,10)
            self.timer=self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def mark(self,key):self.latest[key]=time.monotonic()
        def fresh(self,key,age=1.):return time.monotonic()-self.latest.get(key,-math.inf)<=age
        def on_scan(self,topic,m):
            self.scan_diagnostics[topic]=dict(
                stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                frame_id=m.header.frame_id,
                range_min=m.range_min,range_max=m.range_max,
                **scan_range_summary(m.ranges,m.range_min,m.range_max))
            if topic=='/mapping/scan_raw':
                return
            count,frame=(181,'base_scan') if topic=='/scan' else (360,'mapping_lidar_link')
            admission=scan_admission(topic,m.header.frame_id,m.ranges,
                m.angle_min,m.angle_max,m.angle_increment,m.range_min,m.range_max,
                localization_scan_topic=args.localization_scan_topic)
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            valid_count=sum(math.isfinite(x) and m.range_min<=x<=m.range_max for x in m.ranges)
            if self.inputs.observe(topic,stamp,time.monotonic(),frame=m.header.frame_id,
                valid=admission['valid'],reason=admission['reason'],beams=len(m.ranges),
                expected_beams=count,expected_frame=frame,angle_min=m.angle_min,
                angle_max=m.angle_max,angle_increment=m.angle_increment,
                range_min=m.range_min,range_max=m.range_max,valid_ranges=valid_count,
                finite_return_required=admission['finite_return_required'],
                return_status=admission['return_status'],
                range_values_valid=admission['range_values_valid']):
                self.scans[topic]=m;self.mark(topic)
        def on_grid(self,name,m):
            from guard_policy import grid_message_valid
            valid=grid_message_valid(m,'odom' if name=='local' else 'map')
            if name=='static' and valid:
                g=self.expected_grid
                valid=(m.info.width==g['width'] and m.info.height==g['height']
                    and abs(m.info.resolution-g['resolution'])<1e-8
                    and abs(m.info.origin.position.x-g['origin'][0])<1e-8
                    and abs(m.info.origin.position.y-g['origin'][1])<1e-8
                    and abs(m.info.origin.orientation.z)<1e-8
                    and abs(m.info.origin.orientation.w-1)<1e-8 and list(m.data)==g['data'])
            self.grid_status[name]=bool(valid)
            if valid:self.grid_messages[name]=m;self.mark(name+'_grid')
            elif self.armed:self.fail(name+' map/costmap invalid or frozen-map mismatch')
        def on_clock(self,m):
            if self.inputs.observe('clock',m.clock.sec*10**9+m.clock.nanosec,time.monotonic()):
                self.mark('clock')
        def on_odom(self,m):
            p=m.pose.pose.position
            q=m.pose.pose.orientation
            valid=(m.header.frame_id=='odom' and m.child_frame_id==('base_footprint' if args.odometry_mode=='oracle' else 'ackermann_rear_axle')
                   and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)))
            if not self.inputs.observe('odom',m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                time.monotonic(),frame=m.header.frame_id,valid=valid,
                reason='invalid_odom_frame_or_pose',child_frame_id=m.child_frame_id):return
            if self.last_position:self.distance+=math.hypot(p.x-self.last_position[0],p.y-self.last_position[1])
            self.last_position=(p.x,p.y);self.mark('odom')
            if args.diagnostic_continue_on_readiness_loss and self.distance>MAX_NO_MOTION_DISPLACEMENT_M:
                self.fail('no-motion displacement limit exceeded')
        def on_mission_runtime(self,m):
            try:
                value=json.loads(m.data)
                if (value.get('run_id')!=args.run_id or value.get('real_adapters_ready') is not True
                        or value.get('automatic_task') is not False):
                    raise ValueError('invalid Mission runtime heartbeat')
                self.mission_runtime=value
                self.mark('mission_runtime')
            except (ValueError, AttributeError):
                self.mission_runtime=None
                if self.safety and self.safety.get('owner'):
                    self.fail('Mission runtime heartbeat identity/readiness lost')
        def on_safety(self,m):
            previous=self.safety
            self.safety=dict(block_all=m.block_all,estop=m.emergency_stop_active,
                             enabled=m.autonomous_enabled,owner=m.lease_owner_execution_id,
                             monotonic_ns=time.monotonic_ns(),
                             stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
            if short_goal:
                try:self.safety=observe_safety_sample(previous,self.safety)
                except ValueError as error:
                    self.fail(str(error));return
            self.receipt_writer.safety_sequence=getattr(self.receipt_writer,'safety_sequence',0)+1
            self.receipt_writer.timing('status_received',self.safety,
                ros_now_ns=self.get_clock().now().nanoseconds)
            self.mark('safety')
            if historical and self.safety.get('owner') == self.expected_owner:
                if (self.safety_sync_deadline_ns is not None
                        and time.monotonic_ns() >= self.safety_sync_deadline_ns):
                    self.fail('Safety Lease state synchronization timeout (0.5s)')
                    return
                error=command_error([.01,0.,0.,0.,0.,0.], motion, self.safety,
                    expected_owner=self.expected_owner,
                    minimum_state_ns=self.goal_received_ns, now_ns=time.monotonic_ns(), motion_profile=args.motion_profile)
                if error is None and self.mission_goal_accepted:
                    self.safety_sync_deadline_ns=None
                    self.get_logger().info('SAFETY_STATE_MATCH '+json.dumps(self.safety))
            if args.diagnostic_continue_on_readiness_loss and (m.emergency_stop_active or not m.block_all or m.autonomous_enabled or m.lease_owner_execution_id):
                self.fail('unexpected ESTOP or no-motion authority/Lease state')
            if short_goal and m.lease_owner_execution_id:
                if (m.lease_owner_execution_id != EXPECTED_EXECUTION_ID
                        or not self.mission_goal_accepted):
                    self.fail('unexpected or premature Safety Lease owner in short-goal mode')
            if short_goal and m.emergency_stop_active:
                self.fail('Safety ESTOP became active during short-goal mode')
            if short_goal and not m.block_all:
                admission_path=(Path(os.environ.get('CLEANNAV_RVIZ_RUN_DIR',''))
                                / 'motion-admission.json')
                try:
                    admission=json.loads(admission_path.read_text())
                    valid=(admission.get('run_id')==args.run_id
                           and admission.get('status') in ('OPENING','ADMITTED'))
                except (OSError,ValueError):
                    valid=False
                if not valid:
                    self.fail('Safety interlock opened without a run-bound Motion Admission')
        def on_sequence(self,m):
            if m.sequence_id != args.run_id or m.command_id != args.run_id+'-patrol':
                if m.sequence_id:self.fail('unexpected task sequence identity')
                return
            eps=self.get_publishers_info_by_topic('/cleannav/task_sequence_status')
            if len(eps)!=1 or eps[0].node_name!='mission_manager_node':
                self.fail('task sequence status publisher identity rejected');return
            if m.current_execution_id:
                self.expected_owner=m.current_execution_id
                self.sequence_owners.add(m.current_execution_id)
            self.mission_goal_accepted=True
            self.mission_terminal=m.state in (3,4,5,6)

        def on_rviz_event(self,m):
            try:item=json.loads(m.data)
            except (ValueError,TypeError):
                self.fail('malformed short-goal adapter event');return
            if short_goal and item.get('run_id') != args.run_id:
                self.fail('short-goal adapter event Run ID mismatch');return
            kind=item.get('event')
            if kind=='CLICK':
                if self.rviz_goal_seen or not item.get('command_id'):
                    self.fail('duplicate or unidentified RViz goal event');return
                self.rviz_goal_seen=True;self.mission_command_id=item['command_id']
                self.goal_received_ns=time.monotonic_ns()
                if historical:
                    self.goal_position=item.get('position')
                    stamp=item.get('stamp',[0,0])
                    self.goal_stamp_ns=stamp[0]*10**9+stamp[1]
                    self.goal_distance_base=self.distance
            elif kind=='ACCEPTED':
                if (not self.rviz_goal_seen or item.get('command_id') != self.mission_command_id
                        or item.get('execution_id') != EXPECTED_EXECUTION_ID):
                    self.fail('Mission acceptance is not bound to the one RViz goal');return
                self.mission_goal_accepted=True
            elif kind=='TASK_STATUS' and historical and item.get('state') in (13,14,15):
                self.mission_terminal=True
            elif kind=='MISSION_TERMINAL':
                if item.get('execution_id') != EXPECTED_EXECUTION_ID:
                    self.fail('terminal Mission event execution identity mismatch');return
                self.mission_terminal=True
        def on_admitted_path(self,msg):
            # Budget evidence only; command authority still requires the real Lease.
            if not self.rviz_goal_seen or self.goal_position is None:
                return
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if msg.header.frame_id!='map' or stamp < self.goal_stamp_ns or len(msg.poses)<2:
                return
            points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            if not all(math.isfinite(v) for p in points for v in p):
                self.fail('nonfinite admitted path budget evidence');return
            if math.dist(points[-1],self.goal_position[:2])>.5:
                return
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            origin=(self.route_budget or {}).get('origin_map',points[0])
            extent=max((self.route_budget or {}).get('path_extent_m',0.),
                       max(math.dist(origin,p) for p in points))
            try:
                budget=historical_route_budget(
                    math.dist(origin,self.goal_position[:2]),
                    self.distance-self.goal_distance_base+length,extent)
            except ValueError as error:
                self.fail(str(error));return
            budget.update(command_id=self.mission_command_id,plan_stamp_ns=stamp,origin_map=origin,
                          odometry_origin_m=self.goal_distance_base)
            self.route_budget=budget
        def on_health(self,m):
            try:self.health=json.loads(m.data);self.mark('health')
            except ValueError:self.health=None
        def on_command(self,m,topic='/cmd_vel'):
            self.mark('cmd' if topic=='/cmd_vel' else topic)
            values=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
            moving=any(abs(x)>1e-8 for x in values)
            self.command_samples[topic]+=1
            self.command_nonzero[topic]+=int(moving)
            self.command_last[topic]=dict(values=values,nonzero=moving,
                monotonic_ns=time.monotonic_ns())
            if short_goal:
                try:
                    run_dir=Path(os.environ['CLEANNAV_RVIZ_RUN_DIR'])
                    admission=json.loads((run_dir/'motion-admission.json').read_text())
                    validate_lifecycle(run_id=args.run_id,
                        lifecycle_path=run_dir/'motion-lifecycle.json',
                        authorization_path=run_dir/'motion-authorization.json',
                        scope_path=run_dir/'motion-scope.json',
                        now_monotonic_ns=time.monotonic_ns())
                    authority_active=(admission.get('run_id')==args.run_id
                                      and admission.get('status')=='ADMITTED')
                except (KeyError,OSError,ValueError):
                    authority_active=False
                safety=self.safety
                if (safety is None or time.monotonic_ns()-safety.get('monotonic_ns',0)
                        > 1_000_000_000):
                    safety=None
                error=command_policy(values,safety=safety,
                    admitted=authority_active and self.mission_goal_accepted,
                    goal_seen=self.rviz_goal_seen)
            else:
                error=command_error(values,motion,self.safety,
                    expected_owner=self.expected_owner if historical else None,
                    minimum_state_ns=self.goal_received_ns if historical and not patrol else None,
                    now_ns=time.monotonic_ns() if historical else None, motion_profile=args.motion_profile)
                # Raw/candidate are observed inputs, never forwarded here.  Only
                # the known idle pre-Lease snapshot may await its replacement.
                if (historical and moving and topic != '/cmd_vel' and error
                        and self.rviz_goal_seen and not self.mission_terminal
                        and self.safety and not self.safety['estop']
                        and not self.safety['block_all']
                        and not self.safety['enabled'] and not self.safety['owner']
                        and self.goal_received_ns is not None):
                    if self.safety_sync_deadline_ns is None:
                        self.safety_sync_deadline_ns=time.monotonic_ns()+500_000_000
                        self.get_logger().info('SAFETY_STATE_WAIT pre-Lease snapshot; final output must remain zero')
                    if time.monotonic_ns() < self.safety_sync_deadline_ns:
                        error=None
            self.last_state=dict(self.last_state,run_id=args.run_id,
                last_command={'topic':topic,'values':values,'nonzero':moving,
                              'monotonic_ns':time.monotonic_ns()},
                command_samples=self.command_samples,command_nonzero=self.command_nonzero)
            if moving or error:
                self.receipt_writer.timing('command_checked',self.safety,
                    topic=topic,values=values,error=error)
            if error:self.fail(error)
        def fail(self,why):
            if self.failed:return
            self.failed=why;self.estop.publish(Bool(data=True))
            if self.cancel.service_is_ready():self.cancel.call_async(CancelGoal.Request())
            # Persist callback failures even when the spin loop exits before
            # its next periodic observation. This is not a cleanup receipt.
            failure=dict(self.last_state,ready=False,failed=why,monotonic=time.monotonic(),
                         run_id=args.run_id,distance_m=self.distance)
            self.write_state(failure)
        def write_state(self,state):
            try:
                self.last_state=self.receipt_writer.write(state)
            except ReceiptWriteError as error:
                message='safety guard receipt write failed: '+str(error)
                self.get_logger().error(message)
                if not self.failed:
                    self.failed=message
        def tick(self):
            # Reuse the existing Guard timer and ESTOP/cancel path. No new watchdog.
            if (short_goal and self.safety and self.safety.get('owner')
                    and (self.mission_runtime is None or not self.fresh('mission_runtime',1.5))):
                self.fail('Mission runtime heartbeat expired while Lease is owned')
                return
            tick_started=time.monotonic_ns()
            if (self.safety_sync_deadline_ns is not None
                    and tick_started >= self.safety_sync_deadline_ns):
                self.fail('Safety Lease state synchronization timeout (0.5s)')
                return
            footprint=PolygonStamped();footprint.header.frame_id='base_link'
            footprint.header.stamp=self.get_clock().now().to_msg()
            footprint.polygon.points=[Point32(x=x,y=y,z=0.) for x,y in MAPPING_FOOTPRINT_POINTS]
            self.footprint.publish(footprint)
            cm_observation=self.cm_poll.step(self.lc,GetState.Request,time.monotonic())
            self.cm=cm_observation['state_id']
            for record in cm_observation['events']:
                with output.with_name('cm-state-requests.jsonl').open('a') as log:
                    log.write(json.dumps(dict(run_id=args.run_id,**record))+'\n')
            cm_finished=time.monotonic_ns()
            if self.safety:
                if self.graph is None:self.graph=PublisherGraphConvergence(time.monotonic_ns(), timeout_ns=int(args.timeout*1e9) if args.diagnostic_continue_on_readiness_loss else 5_000_000_000)
                state=self.graph.sample_query(
                    lambda:self.get_publishers_info_by_topic('/cmd_vel'),time.monotonic_ns)
                self.graph_history.append(state)
                self.graph_history=self.graph_history[-5:]
                with output.with_name('safety-publisher-graph.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(run_id=args.run_id,**state),default=str)+'\n')
                if state['failure_reason']:
                    self.last_state=dict(self.last_state,run_id=args.run_id,
                        command_publisher_graph_failure=state)
                if state['failure_reason']:
                    self.graph_ok=False
                    self.fail('cmd_vel publisher graph: '+str(state['failure_reason']))
                else:
                    self.graph_ok=state['converged']
            publisher_snapshot={topic:[endpoint_record(e) for e in self.get_publishers_info_by_topic(topic)]
                for topic in ('/cmd_vel','/cleannav/navigation/cmd_vel_raw',
                    '/cleannav/cmd_vel_candidate','/cleannav/manual/cmd_vel_raw',
                    '/cleannav/mapping/cmd_vel_raw')}
            self.command_publishers=publisher_snapshot
            if args.diagnostic_continue_on_readiness_loss:
                graph_errors=diagnostic_command_graph_errors(publisher_snapshot)
                if graph_errors:
                    self.last_state=dict(self.last_state,run_id=args.run_id,
                        command_publishers=publisher_snapshot,
                        diagnostic_command_graph_errors=graph_errors)
                    self.fail('diagnostic command publisher graph: '+','.join(graph_errors))
            graph_finished=time.monotonic_ns()
            tf_wait_deadline=time.monotonic()+.05
            def exact_lookup(target,source,stamp):
                from rclpy.duration import Duration
                # base_scan uses the non-blocking 0.05 s exact-stamp window in
                # InputEvidence. Sleeping here blocks the executor, so the TF
                # sample that is 15 ms behind never enters the buffer.
                remaining=0. if source=='base_scan' else max(0.,tf_wait_deadline-time.monotonic())
                tf=self.buffer.lookup_transform(target,source,Time(nanoseconds=stamp,clock_type=ClockType.ROS_TIME),
                    timeout=Duration(seconds=remaining))
                return dict(parent=tf.header.frame_id,child=tf.child_frame_id,
                            stamp_ns=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec)
            facts,observations=self.inputs.snapshot(time.monotonic(),exact_lookup,
                localization_scan_topic=args.localization_scan_topic)
            facts.update({key:self.fresh(key) for key in ['safety','cmd','cm']})
            facts['safety_no_estop']=(None if self.safety is None else not self.safety['estop'])
            facts['cm']=cm_observation['fresh'];facts['cm_active']=cm_observation['active'];facts['cmd_publisher']=self.graph_ok
            if self.amcl_goal:
                facts['frozen_map_matches']=self.grid_status.get('static') is True
                # Local publishes at 2 Hz. The 653-cell global map's observed
                # publish gap was about 2 s, so the previous 1.5 s window
                # marked a live global costmap stale every cycle.
                for label, limit_s in (('global', 3.0), ('local', 1.5)):
                    grid=self.grid_messages.get(label)
                    clock=self.inputs.valid.get('clock',{}).get('stamp_ns')
                    stamp=(grid.header.stamp.sec*10**9+grid.header.stamp.nanosec) if grid else None
                    current=costmap_stamp_current(clock, stamp, limit_s)
                    facts[label+'_costmap_valid']=(self.grid_status.get(label) is True
                        and self.fresh(label+'_grid', limit_s) and current)
                    received=self.latest.get(label+'_grid')
                    stamp_age_s=None
                    if (isinstance(clock, int) and not isinstance(clock, bool)
                            and isinstance(stamp, int) and not isinstance(stamp, bool)):
                        stamp_age_s=(clock-stamp)/1e9
                    observations[label+'_costmap_valid']=dict(
                        ready=facts[label+'_costmap_valid'], reason=None,
                        receive_age_s=(None if received is None else time.monotonic()-received),
                        stamp_age_s=stamp_age_s, freshness_limit_s=limit_s,
                        stamp_lead_allowance_s=COSTMAP_STAMP_LEAD_S,
                        clock_stamp_ns=(clock if isinstance(clock, int) and not isinstance(clock, bool) else None),
                        costmap_stamp_ns=stamp)
                tf_endpoints=self.get_publishers_info_by_topic('/tf')
                names=[e.node_name for e in tf_endpoints]
                facts['unique_localization_tf_sources']=(sorted(names)==['amcl','gazebo_oracle_odometry','robot_state_publisher'])
            facts['localization']=localization_bootstrap_ready(
                bootstrap_ready, self.health, self.latest.get('health', -math.inf), time.monotonic())
            for topic,expected in [('/cleannav/navigation/cmd_vel_raw','controller_server'),('/cleannav/cmd_vel_candidate','collision_monitor')]:
                eps=[endpoint_record(e) for e in self.get_publishers_info_by_topic(topic)]
                facts[topic+'_publisher']=(len(eps)==1 and eps[0].get('node_name')==expected and
                    eps[0].get('node_namespace')=='/' and eps[0].get('topic_type')=='geometry_msgs/msg/Twist')
            final_eps=publisher_snapshot['/cmd_vel']
            facts['/cmd_vel_authorized_publisher']=(len(final_eps)==1
                and final_eps[0].get('node_name')=='cleannav_safety_supervisor'
                and final_eps[0].get('node_namespace')=='/'
                and final_eps[0].get('topic_type')=='geometry_msgs/msg/Twist'
                and final_eps[0].get('endpoint_gid') not in (None,'',[],{}))
            if not motion:
                facts['no_motion_authority']=(None if self.safety is None else self.safety['block_all'] and
                    not self.safety['enabled'] and not self.safety['owner'])
            if patrol and self.safety and self.safety.get('owner') and self.safety['owner'] not in self.sequence_owners:
                facts['sequence_lease_identity']=False
            missing=missing_inputs(facts)
            continuation = None
            if args.diagnostic_continue_on_readiness_loss:
                continuation=diagnostic_continuation_decision(
                    facts,publisher_snapshot,nonzero_command_count=0,
                    distance_m=self.distance,
                    maximum_displacement_m=MAX_NO_MOTION_DISPLACEMENT_M)
                if self.armed and not continuation['allowed']:
                    self.fail('diagnostic hard safety gate: '+','.join(continuation['hard_failures']))
            transition_mono_ns=time.monotonic_ns()
            transition_wall_ns=time.time_ns()
            transitions=readiness_transition_records(
                args.run_id,self.last_fact_values,facts,observations,
                monotonic_ns=transition_mono_ns,wall_time_ns=transition_wall_ns)
            self.last_fact_values=dict(facts)
            # Save the decision's inputs before fail() can terminate the spin loop.
            self.last_state=dict(run_id=args.run_id,wall_time_ns=time.time_ns(),
                monotonic_ns=time.monotonic_ns(),monotonic=time.monotonic(),facts=facts,
                observations=observations,localization_health=self.health,
                missing=missing,distance_m=self.distance,route_budget=self.route_budget,cm=self.cm,safety=self.safety,
                scan_diagnostics=self.scan_diagnostics,cm_observation=cm_observation,
                command_publishers=publisher_snapshot,
                diagnostic_mode=args.diagnostic_continue_on_readiness_loss,
                diagnostic_continuation=continuation,
                readiness_transitions=transitions)
            motion_admission_status=''
            if short_goal:
                try:
                    motion_admission_status=json.loads((
                        Path(os.environ['CLEANNAV_RVIZ_RUN_DIR'])/'motion-admission.json'
                    ).read_text()).get('status','')
                except (KeyError,OSError,ValueError):
                    motion_admission_status=''
            if (self.armed and missing and not args.diagnostic_continue_on_readiness_loss
                    and (not short_goal or motion_admission_status=='ADMITTED')
                    and (not historical or self.mission_goal_accepted)):
                self.fail('lost safety admission: '+','.join(missing))
            if self.armed and not short_goal and (self.distance>=(self.goal_distance_base+self.route_budget['odometry_m'] if self.route_budget else MAX_HISTORICAL_ODOMETRY_M) if historical else self.distance>2.):
                self.fail('historical admitted-route accumulated odometry limit' if historical else '2m accumulated odometry limit')
            if self.armed and short_goal and self.distance>2.5:
                self.fail('2.5m short-goal accumulated odometry limit')
            if not missing:self.armed=True
            if time.monotonic()-self.start>args.timeout and not self.armed:self.fail('bounded readiness deadline')
            state=dict(self.last_state,ready=self.armed and not missing and not self.failed,
                       failed=explicit_failure_state(self.failed),
                       monotonic=time.monotonic(),facts=facts,missing=missing,distance_m=self.distance,
                       cm=self.cm,safety=self.safety,graph_history=self.graph_history,
                       callback_timings_ns=dict(start=tick_started,cm_finished=cm_finished,
                           graph_finished=graph_finished,observations_finished=time.monotonic_ns()),
                       diagnostic_mode=args.diagnostic_continue_on_readiness_loss,
                       diagnostic_continuation=continuation)
            self.last_state=state;self.write_state(state)
    rclpy.init(args=ros_args);node=Guard();rc=0
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.failed:executor.spin_once(timeout_sec=.05)
        if node.failed:rc=1
    except KeyboardInterrupt:pass
    finally:
        estop_delivered = publish_shutdown_estop(
            rclpy.ok, lambda: node.estop.publish(Bool(data=True)))
        if not estop_delivered:
            print('[SAFETY_GUARD] shutdown ESTOP delivery unavailable: ROS context is invalid')
        executor.shutdown(timeout_sec=2.)
        try:
            node.receipt_writer.close()
        except ReceiptWriteError as error:
            print('[SAFETY_GUARD] receipt close failed: '+str(error), file=sys.stderr)
            rc=1
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return rc


if __name__=='__main__':raise SystemExit(main())
