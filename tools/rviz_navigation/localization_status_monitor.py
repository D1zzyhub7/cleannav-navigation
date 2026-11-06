#!/usr/bin/env python3
"""Localization readiness and fail-closed health, using native matching evidence."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import sys
from ament_index_python.packages import get_package_prefix
sys.path.append(str(Path(get_package_prefix('cleannav_simulation'))/'lib/cleannav_simulation'))
from readiness_inputs import amcl_bootstrap_ready, motion_admission, readiness_payload
import rclpy
from rcl_interfaces.srv import GetParameters
from action_msgs.srv import CancelGoal
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rtabmap_msgs.msg import Info
from std_msgs.msg import String

from localization_health import accepted_match, bootstrap_ready as rtabmap_bootstrap_ready, stationary, stationary_hold, trusted, MotionEvidence


def bootstrap_ready(state, age):
    return (amcl_bootstrap_ready(state, age) if state.get('localization_kind') == 'amcl'
            else rtabmap_bootstrap_ready(state, age))
from mapping_assets import digest
from simulation_status_monitor import SimulationStatusMonitor, STAGES, missing_requirements, emit


class TFMetadataMixin:
    """Opt-in Humble MessageInfo capture, reusing the existing Layer-B approach."""
    def __init__(self, timing):
        self.timing = timing

    def _take_subscription(self, sub):
        # Keep every non-TF subscription on Humble's base message path. Only
        # time the TransformListener readers when the diagnostic is enabled.
        capture_tf = (sub.topic_name in ('/tf', '/tf_static')
                      and getattr(sub.callback, '__name__', '')
                      in ('callback', 'static_callback'))
        if capture_tf:
            started = time.monotonic_ns()
            started_realtime = time.time_ns()
        with sub.handle:
            data = sub.handle.take_message(sub.msg_type, sub.raw)
        if data is None:
            return None
        if not capture_tf:
            # Match Humble MultiThreadedExecutor._take_subscription exactly.
            return data[0]
        ended = time.monotonic_ns()
        ended_realtime = time.time_ns()
        if capture_tf:
            message, info = data
            samples = [dict(parent=t.header.frame_id, child=t.child_frame_id,
                            stamp_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec)
                       for t in message.transforms
                       if (t.header.frame_id,t.child_frame_id) in self.timing.EDGES]
            if samples:
                self.timing.record('rmw_take', started, ended_ns=ended, topic=sub.topic_name,
                    take_start_realtime_ns=started_realtime, take_end_realtime_ns=ended_realtime,
                    dds_source_timestamp=info.get('source_timestamp',0) if isinstance(info,dict) else 0,
                    dds_received_timestamp=info.get('received_timestamp',0) if isinstance(info,dict) else 0,
                    transforms=samples)
        return data

    def _take_client(self, client):
        # Preserve Humble's exact return object; record only metadata from a real take.
        started = time.monotonic_ns();realtime = time.time_ns()
        data = super()._take_client(client)
        ended = time.monotonic_ns();ended_realtime = time.time_ns()
        if self.timing is not None and data is not None and data[0] is not None:
            header = data[0]
            self.timing.record('rpc_response_take', started, ended_ns=ended,
                service=client.srv_name, client_entity_id=id(client), request_sequence_number=header.request_id.sequence_number,
                take_start_realtime_ns=realtime, take_end_realtime_ns=ended_realtime,
                dds_source_timestamp=header.source_timestamp,
                dds_received_timestamp=header.received_timestamp)
        return data

    async def _execute_subscription(self, sub, data):
        if data is None:
            return
        is_tf_listener = (sub.topic_name in ('/tf', '/tf_static')
                          and getattr(sub.callback, '__name__', '')
                          in ('callback', 'static_callback'))
        message = data[0] if is_tf_listener else data
        if message:
            await await_or_execute(sub.callback, message)



class TFMetadataExecutor(TFMetadataMixin, MultiThreadedExecutor):
    def __init__(self, timing):
        MultiThreadedExecutor.__init__(self, num_threads=3)
        TFMetadataMixin.__init__(self, timing)


class TFListenerExecutor(TFMetadataMixin, SingleThreadedExecutor):
    """Take and insert TF inline, without the monitoring worker queue."""
    def __init__(self, timing=None):
        SingleThreadedExecutor.__init__(self)
        TFMetadataMixin.__init__(self, timing)

    def _take_subscription(self, sub):
        if self.timing is None:
            return SingleThreadedExecutor._take_subscription(self, sub)
        return TFMetadataMixin._take_subscription(self, sub)

    async def _execute_subscription(self, sub, data):
        if self.timing is None:
            return await SingleThreadedExecutor._execute_subscription(self, sub, data)
        return await TFMetadataMixin._execute_subscription(self, sub, data)


def stop_tf_listener_executor(executor, dispatcher, node, timeout_sec=5.0):
    """Single-threaded executor has no worker pool; drain before destruction."""
    deadline = time.monotonic()+timeout_sec
    executor.remove_node(node)
    stopped = executor.shutdown(timeout_sec=max(0., deadline-time.monotonic()))
    dispatcher.join(timeout=max(0., deadline-time.monotonic()))
    if not stopped or dispatcher.is_alive():
        return False
    node.destroy_node()
    return True


def create_localization_executor(tf_timing=None):
    """Keep TF, serialized monitoring, and fresh scan ingress independently runnable."""
    return (TFMetadataExecutor(tf_timing) if tf_timing is not None
            else MultiThreadedExecutor(num_threads=3))


def configure_localization_panel(node):
    """Serialize panel queries with the health timer in the default group."""
    return node.create_timer(.5, node.panel, clock=node.steady)


def stop_localization_executor(executor, dispatcher, node, timeout_sec=5.0):
    """Never destroy a node while active or queued Humble workers can still use it."""
    deadline = time.monotonic()+timeout_sec
    executor.remove_node(node)
    stopped = executor.shutdown(timeout_sec=max(0., deadline-time.monotonic()))
    dispatcher.join(timeout=max(0., deadline-time.monotonic()))
    if not stopped or dispatcher.is_alive():
        return False
    # Installed Humble MultiThreadedExecutor does not shut down its worker pool.
    # The tracker can be idle while queued handlers still exist. Drain the owned
    # pool before node destruction, bounded by the same overall deadline.
    pool = getattr(executor, '_executor', None)
    if pool is None:
        return False
    drained = threading.Event()
    def drain_pool():
        pool.shutdown(wait=True, cancel_futures=True)
        drained.set()
    drainer = threading.Thread(target=drain_pool,
                               name='cleannav-localization-worker-drain', daemon=True)
    drainer.start()
    drainer.join(timeout=max(0., deadline-time.monotonic()))
    if drainer.is_alive() or not drained.is_set():
        return False
    node.destroy_node()
    return True


class LocalizationMonitor(SimulationStatusMonitor):
    def __init__(self, session, probe_stage=None, **kwargs):
        self.session_path = Path(session)
        self.session = json.loads(self.session_path.read_text())
        self.amcl_mode = self.session.get('localization_kind') == 'amcl'
        # The odometry source, not the localizer, owns the /odom child frame.
        # Wheel mode publishes at the rear axle; Oracle publishes at the
        # footprint. Absent key keeps the legacy wheel contract.
        self.odometry_mode = self.session.get('odometry_mode', 'wheel')
        self.probe_stage = probe_stage
        self.amcl_pose_count = 0
        self.amcl_stamp_ns = 0
        self.amcl_covariance = None
        self.amcl_covariance_above_limit = False
        self.amcl_covariance_warning_stamp_ns = None
        self.amcl_parameters = None
        self.amcl_pose_valid = False
        self.native_ok = False
        self.info_time = self.match_time = float('-inf')
        self.matches = 0
        self.linear_update = self.angular_update = 0.
        self.odom_history = deque(maxlen=300)
        self.motion_started = None
        self.motion_evidence = MotionEvidence()
        self.odom_pose = self.match_odom_pose = None
        self.last_info_stamp = None
        self.info_sample = {}
        self.assets_ok = True
        self.last_asset_check = 0.
        self.localized = False
        self.had_localization = False
        self.cancel_sent = False
        self.first_loss_recorded = False
        self.first_loss_event = None
        self.last_native_tf_context = None
        self.campus_security = None
        self.campus_ready = False
        # Move the existing listener readers to one exclusive TF node/executor.
        # No mirror reader, spin_thread, or node shared by two executors.
        self.tf_listener_node = Node('cleannav_native_tf_listener',
            use_global_arguments=False, enable_rosout=False,
            start_parameter_services=False)
        super().__init__(tf_listener_node=self.tf_listener_node, **kwargs)
        self.cancel_client = self.create_client(CancelGoal, '/cleannav/navigate_to_pose/_action/cancel_goal')
        self.health_pub = self.create_publisher(String, '/cleannav/localization/health', 10)
        if self.session.get('campus_security'):
            from campus_security_ros import CampusEvidence
            self.campus_security = CampusEvidence(self, self.session['campus_security'])
        from lifecycle_msgs.srv import GetState
        self.state_clients['map_server'] = self.create_client(GetState, '/map_server/get_state')
        if self.amcl_mode:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            self.native_client = self.create_client(GetParameters, '/amcl/get_parameters')
            self.state_clients['amcl'] = self.create_client(GetState, '/amcl/get_state')
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose, 10)
        else:
            self.native_client = self.create_client(GetParameters, '/rtabmap/rtabmap/get_parameters')
            self.create_subscription(Info, '/rtabmap/info', self.info, 10)
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.create_subscription(OccupancyGrid, '/cleannav/localization/map', self.frozen_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def frozen_map(self, msg):
        if (msg.header.frame_id == 'map' and msg.info.resolution > 0
                and msg.info.width * msg.info.height == len(msg.data)
                and len(msg.data) > 0 and any(v >= 0 for v in msg.data)):
            if self.amcl_mode:
                grid=json.loads((Path(self.session['map_directory'])/'occupancy_grid.json').read_text())
                if (msg.info.width!=grid['width'] or msg.info.height!=grid['height']
                        or abs(msg.info.resolution-grid['resolution'])>1e-8
                        or abs(msg.info.origin.position.x-grid['origin'][0])>1e-8
                        or abs(msg.info.origin.position.y-grid['origin'][1])>1e-8
                        or abs(msg.info.origin.orientation.z)>1e-8
                        or abs(msg.info.origin.orientation.w-1)>1e-8
                        or list(msg.data)!=grid['data']):
                    return
            self._mark('frozen_map')

    def _odom(self, msg):
        # The odometry source owns the /odom child-frame contract; AMCL and
        # RTAB-Map are independent of it. Same mapping as
        # navigation_safety_guard.on_odom, so one run has one odom contract.
        expected = ('base_footprint' if self.odometry_mode == 'oracle'
                    else 'ackermann_rear_axle')
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        if not (msg.header.frame_id == 'odom' and msg.child_frame_id == expected
                and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                and math.hypot(q.x,q.y,q.z,q.w)>1e-9):
            return
        self._mark('odom')
        if self._fresh('odom'):
            self.odom_pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y),
                                                  1-2*(q.y*q.y+q.z*q.z)))
            self.odom_history.append((time.monotonic(),self.odom_pose))

    def additional_missing(self, facts):
        missing = [k for k in ('localized','frozen_map','map_server_active') if not facts[k]]
        if getattr(self, 'campus_security', None) is not None and not self.campus_ready:
            missing.append('campus_keepout_ready')
        return missing

    def motion_admission(self):
        """Observe the external admission; this node never issues the interlock."""
        admission = motion_admission(self.safety, now=time.monotonic(),
            received_at=self.received.get('safety', -math.inf))
        if os.environ.get('CLEANNAV_RVIZ_MOTION_PROFILE') == 'short-goal' and admission['admitted']:
            try:
                from short_goal_policy import validate_runtime_admission
                validate_runtime_admission()
                admission['source']='supervisor_admitted_receipt'
            except (OSError, ValueError) as error:
                admission.update(admitted=False, reason=str(error))
        return admission

    def panel(self):
        """Publish the admission the outer supervisor already made.

        The integration launch deliberately withholds --enable-motion: a second
        authorization would compete with the supervisor's single Motion
        Admission. The inherited panel can only publish motion_enabled from that
        never-issued authorization, so Mission saw a permanent false. Report the
        observed interlock state instead and keep every other inherited
        behavior: no interlock call and no dependency ESTOP from this panel.
        """
        if self.enable_motion:
            # An explicit --enable-motion still owns its own authorization.
            return super().panel()
        facts = self.facts()
        missing = [item for stage in STAGES for item in missing_requirements(stage, facts)]
        if not facts['map']:
            missing.append('map (/rtabmap/map not yet usable)')
        missing.extend(self.additional_missing(facts))
        admission = self.motion_admission()
        payload = readiness_payload(
            ready=not missing, missing=missing, safety_present=self.safety is not None,
            emergency_stop_active=bool(self.safety is not None
                                       and self.safety.emergency_stop_active),
            authorization_failed=self.authorization_failed, admission=admission)
        self.readiness_pub.publish(String(data=json.dumps(payload)))
        if payload['ready']:
            self.ever_ready = True
            self._changed('CLEANNAV', 'READY',
                          'SYSTEM READY; Waiting for TaskCommand; '
                          + ('motion admitted by the navigation supervisor, Lease required'
                             if payload['motion_enabled'] else
                             'observation only, motion admission ' + str(admission['reason'])))
        elif self.safety is None:
            self._changed('CLEANNAV', 'BLOCKED', 'SafetyStatus missing; motion blocked')
        elif self.safety.emergency_stop_active:
            self._changed('CLEANNAV', 'BLOCKED', 'ESTOP active; reset through Mission Manager')
        elif self.authorization_failed:
            self._changed('CLEANNAV', 'BLOCKED',
                          'simulation authorization failed; restart required')
        else:
            self._changed('CLEANNAV', 'PARTIAL_READY', 'waiting: ' + ', '.join(missing))

    def info(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_info_stamp:
            return
        self.last_info_stamp = stamp
        self.info_time = time.monotonic()
        if accepted_match(msg, self.session['node_ids']):
            self.matches += 1
            self.match_time = self.info_time
            self.match_odom_pose = self.odom_pose
        self.info_sample = dict(stamp=list(stamp), ref_id=msg.ref_id,
            loop_closure_id=msg.loop_closure_id, proximity_detection_id=msg.proximity_detection_id,
            stats=dict(zip(msg.stats_keys, msg.stats_values)), matches=self.matches)

    def native(self, response):
        self.native_ok = False
        if response and len(response.values) == 4:
            try:
                linear, angular = [float(v.string_value) for v in response.values[2:]]
            except ValueError:
                return
            if (all(math.isfinite(v) and 0 <= v <= .2 for v in (linear,angular))
                    and response.values[0].string_value == 'false'
                    and response.values[1].string_value == 'true'):
                self.linear_update, self.angular_update = linear, angular
                self.native_ok = True

    def _poll(self):
        super()._poll()
        if self.probe_stage:
            return
        if not hasattr(self, 'native_client'):
            return
        if self.amcl_mode:
            self._poll_amcl()
            return
        self._query('native-localization', self.native_client,
            GetParameters.Request(names=['Mem/IncrementalMemory','Mem/InitWMWithAllNodes',
                                        'RGBD/LinearUpdate','RGBD/AngularUpdate']), self.native)
        now = time.monotonic()
        if now-self.last_asset_check > 5.:
            root = Path(self.session['map_directory'])
            self.assets_ok = all((root/n).is_file() and digest(root/n) == value
                                 for n, value in self.session['original_sha256'].items())
            self.last_asset_check = now
        ros_stamp = (self.last_info_stamp[0]+self.last_info_stamp[1]/1e9) if self.last_info_stamp else 0.
        sensors = (self._fresh('clock') and self._fresh('scan') and self._fresh('odom')
                   and -.25 <= self.get_clock().now().nanoseconds/1e9-ros_stamp <= 2.5)
        older = next((pose for stamp,pose in reversed(self.odom_history) if now-stamp >= .5),None)
        stopped = stationary(self.odom_pose, older, .005, .005)
        small_movement = self.info_sample.get('stats',{}).get('Memory/Small_movement/') == 1.
        near_anchor = stationary(self.odom_pose, self.match_odom_pose,
                                 self.linear_update+.02, self.angular_update+.03)
        holding = stationary_hold(stopped, near_anchor)
        motion_ok = self.motion_evidence.update(
            now, self.match_time, holding, self.had_localization, stopped=stopped)
        self.motion_started = self.motion_evidence.started
        covariance = self.info_sample.get('stats',{}).get('Loop/MapToBase_lin_var/m2')
        # Evaluate each TF predicate once and publish the same observations used
        # by this Native health decision. Campus Guard creates a separate context.
        tf_context_id = self._new_tf_context_id('native')
        native_map_tf = {}
        native_scan_tf = {}
        map_tf_valid = self._tf('map', 'base_link', observation=native_map_tf,
                                context_id=tf_context_id)
        scan_tf_valid = self._scan_tf(observation=native_scan_tf,
                                      context_id=tf_context_id)
        self.last_native_tf_context = dict(context_id=tf_context_id,
            context_kind='native_health', monotonic_ns=time.monotonic_ns(),
            map_to_base_link=native_map_tf, scan_exact_stamp=native_scan_tf,
            clock=self.clock_context())
        self.localized = motion_ok and self.assets_ok and trusted(now, self.info_time, self.match_time,
            self.matches, self.native_ok, sensors, map_tf_valid and scan_tf_valid,
            holding_stationary=holding, covariance=covariance, motion_started=self.motion_started)
        if self.had_localization and not self.localized:
            self.motion_evidence.lost = True
            self.motion_evidence.state = "LOST"
        state = dict(localized=self.localized, native_parameters_verified=self.native_ok,
            frozen_map='frozen_map' in self.received,
            map_server_active=self.lifecycle.get('map_server',False),
            assets_unchanged=self.assets_ok, sensors_fresh=sensors,
            map_tf_valid=map_tf_valid, scan_tf_valid=scan_tf_valid,
            scan_tf_diagnostics=self.scan_tf_diagnostics,
            tf_context=self.last_native_tf_context,
            info_age=now-self.info_time if self.info_time != float('-inf') else None,
            match_age=now-self.match_time if self.match_time != float('-inf') else None,
            stationary_since_match=holding, stopped=stopped, native_small_movement=small_movement,
            linear_update=self.linear_update, angular_update=self.angular_update,
            covariance=covariance, motion_started=self.motion_started, motion_state=self.motion_evidence.state, info=self.info_sample, reason='accepted saved-node ICP match, native covariance and fresh inputs' if self.localized
                else 'BLOCKED: need accepted saved-node match, bounded native covariance and fresh sensor/TF',
            ros_time=self.get_clock().now().nanoseconds/1e9)
        if self.had_localization and not self.localized and not self.first_loss_recorded:
            # Capture this decision before a later cancellation RPC or /proc
            # snapshot. The legacy loss file is a later cancel snapshot.
            loss_monotonic_ns = time.monotonic_ns()
            loss_realtime_ns = time.time_ns()
            ros_loss_ns = int(state['ros_time']*1_000_000_000)
            loss_sequence = None
            if self.tf_timing is not None:
                loss_sequence = self.tf_timing.record('first_native_lost',
                    loss_monotonic_ns, ended_ns=loss_monotonic_ns,
                    realtime_ns=loss_realtime_ns, ros_time_ns=ros_loss_ns,
                    scan_tf_valid=scan_tf_valid, map_tf_valid=map_tf_valid)
                self.first_loss_event = dict(sequence=loss_sequence,
                    monotonic_ns=loss_monotonic_ns, realtime_ns=loss_realtime_ns,
                    ros_time_ns=ros_loss_ns)
            self.session_path.with_name('first-native-loss.json').write_text(
                json.dumps(dict(wall_time=time.time(), state=state,
                    tf_timing_event_sequence=loss_sequence,
                    tf_timing_window_path=('first-native-tf-window.json'
                                           if self.first_loss_event else None)), indent=2))
            self.first_loss_recorded = True
        if self.localized:
            self.had_localization = True
        elif self.had_localization and not self.cancel_sent and self.cancel_client.service_is_ready():
            # Cancel the single execution owner; it cancels its FollowPath child.
            self.cancel_client.call_async(CancelGoal.Request())
            self.cancel_sent = True
            state['navigation_cancel_requested'] = True
            threads=[]
            for process in Path('/proc').iterdir():
                if not process.name.isdigit():continue
                try:
                    command=(process/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                    environment = dict(part.split(b'=',1) for part in (process/'environ').read_bytes().split(b'\0') if b'=' in part)
                    if ('/rtabmap_slam/rtabmap' in command
                            and environment.get(b'ROS_DOMAIN_ID') == os.environ.get('ROS_DOMAIN_ID','0').encode()):
                        threads.append(dict(pid=int(process.name),status=(process/'status').read_text(),
                            threads=[dict(tid=int(t.name),stat=(t/'stat').read_text(),wchan=(t/'wchan').read_text())
                                     for t in (process/'task').iterdir()]))
                except (OSError,ValueError):continue
            scan_header, _, _ = self.scan_snapshot()
            self.session_path.with_name('localization-loss.json').write_text(json.dumps(dict(state=state,
                scan_stamp=[scan_header.stamp.sec,scan_header.stamp.nanosec] if scan_header else None,
                tf_valid=map_tf_valid, tf_context=self.last_native_tf_context,
                threads=threads),indent=2))
        if self.campus_security is not None:
            self.campus_ready = self.campus_security.poll()
            state['campus_keepout_ready'] = self.campus_ready
        self.health_pub.publish(String(data=json.dumps(state)))
        self.session_path.with_name('health.json').write_text(json.dumps(state, indent=2))
        with self.session_path.with_name('health-timeline.jsonl').open('a') as timeline:
            timeline.write(json.dumps(dict(wall_time=time.time(), **state))+'\n')

    def amcl_pose(self, msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,*msg.pose.covariance]
        particles=self.session['amcl_overrides']['max_particles']
        roundoff=4*particles*sys.float_info.epsilon*max(1.,p.x*p.x,p.y*p.y)
        self.amcl_pose_valid=(msg.header.frame_id=='map' and stamp>self.amcl_stamp_ns
            and all(math.isfinite(v) for v in values)
            and all(msg.pose.covariance[i]>=-roundoff for i in (0,7,35))
            and abs(math.hypot(q.x,q.y,q.z,q.w)-1)<1e-3)
        if not self.amcl_pose_valid:
            return
        self.amcl_stamp_ns=stamp
        self.amcl_pose_count+=1
        self.match_time=time.monotonic()
        self.match_odom_pose=self.odom_pose
        self.amcl_covariance=max(0.,*(msg.pose.covariance[i] for i in (0,7)))
        self.amcl_covariance_raw=[msg.pose.covariance[i] for i in (0,7,35)]
        self.amcl_covariance_roundoff_bound=roundoff

    def amcl_native(self, response):
        expected=self.session['amcl_overrides']
        names=list(self.session['amcl_expected_parameters'])
        self.native_ok=False
        if response is None or len(response.values)!=len(names):return
        values={name:(v.bool_value if v.type==1 else v.double_value if v.type==3
                      else v.integer_value if v.type==2 else v.string_value if v.type==4 else None)
                for name,v in zip(names,response.values)}
        self.amcl_parameters=values
        self.native_ok=all(values[name]==expected[name] for name in names)
        if self.native_ok:
            self.linear_update=values['update_min_d']
            self.angular_update=values['update_min_a']

    def _note_amcl_covariance_limit(self):
        # 0.25 m^2 remains the reference warning. Oracle and wheel share this
        # path. Crossing it records one warning and leaves navigation running.
        value=self.amcl_covariance
        ok=value is not None and math.isfinite(value) and value>=0
        above=bool(ok and value>.25)
        previous=getattr(self,'amcl_covariance_above_limit',False)
        if above and not previous:
            self.get_logger().warning(
                'AMCL planar covariance %.6f m^2 exceeds 0.25 at pose stamp %d; '
                'recording only, navigation continues'%(value,self.amcl_stamp_ns))
            self.amcl_covariance_warning_stamp_ns=self.amcl_stamp_ns
        elif previous and ok and not above:
            self.get_logger().warning(
                'AMCL planar covariance %.6f m^2 returned within 0.25 at pose stamp %d'
                %(value,self.amcl_stamp_ns))
            self.amcl_covariance_warning_stamp_ns=self.amcl_stamp_ns
        if ok:
            self.amcl_covariance_above_limit=above
        return ok

    def _amcl_pose_update_due(self):
        """AMCL publishes only after update_min_d or update_min_a from the last pose."""
        current,anchor=self.odom_pose,self.match_odom_pose
        if current is None or anchor is None or self.linear_update<=0 or self.angular_update<=0:
            return True
        distance=math.hypot(current[0]-anchor[0],current[1]-anchor[1])
        angle=abs(math.atan2(math.sin(current[2]-anchor[2]),math.cos(current[2]-anchor[2])))
        return distance>=self.linear_update or angle>=self.angular_update

    def _amcl_motion_ok(self, now, holding, stopped, update_due):
        evidence=self.motion_evidence
        if evidence.lost:
            evidence.state='LOST'
            return False
        if not update_due:
            # Odometry has not reached the loaded AMCL update threshold, so a
            # frozen pose stamp is expected. Do not start or continue the
            # motion-loss clock. A later crossing uses the existing 3 s timeout.
            if self.match_time!=evidence.match and math.isfinite(self.match_time):
                evidence.match=self.match_time
                evidence.matched_during_motion=False
                evidence.stationary_match_pending=False
                evidence.initial_settle_deadline=None
            evidence.started=None
            evidence.state='MOVING' if 0<=now-self.match_time<=3. else 'RESUMING'
            return True
        return evidence.update(now,self.match_time,holding,self.had_localization,stopped=stopped)

    def _poll_amcl(self):
        self._query('native-amcl',self.native_client,
            GetParameters.Request(names=list(self.session['amcl_expected_parameters'])),self.amcl_native)
        now=time.monotonic()
        if now-self.last_asset_check>5.:
            root=Path(self.session['map_directory'])
            self.assets_ok=all((root/n).is_file() and digest(root/n)==value
                for n,value in self.session['original_sha256'].items())
            self.last_asset_check=now
        map_tf={}; scan_tf={}
        map_tf_valid=self._tf('map','base_link',observation=map_tf)
        scan_tf_valid=self._scan_tf(observation=scan_tf)
        sensors=self._fresh('clock') and self._fresh('scan') and self._fresh('odom')
        older=next((pose for stamp,pose in reversed(self.odom_history) if now-stamp>=.5),None)
        stopped=stationary(self.odom_pose,older,.005,.005)
        holding=stationary_hold(stopped,stationary(self.odom_pose,self.match_odom_pose,
            self.linear_update+.02,self.angular_update+.03))
        update_due=LocalizationMonitor._amcl_pose_update_due(self)
        # Once odometry has crossed an AMCL update threshold, the stationary
        # hold must not postpone the existing pose timeout.
        motion_ok=LocalizationMonitor._amcl_motion_ok(
            self,now,False if update_due else holding,stopped,update_due)
        nodes={name for name,namespace in self.get_node_names_and_namespaces()}
        pose_publishers=self.get_publishers_info_by_topic('/amcl_pose')
        authority_ok=(len(pose_publishers)==1 and pose_publishers[0].node_name=='amcl'
                      and 'rtabmap' not in nodes)
        # A finite, non-negative covariance is still required. The 0.25 m^2
        # reference warns once and does not cancel navigation in either
        # odometry mode. Pose age, TF, lifecycle and sensor checks stay.
        covariance_ok=LocalizationMonitor._note_amcl_covariance_limit(self)
        valid=(motion_ok and self.amcl_pose_valid and self.assets_ok and authority_ok
            and self.lifecycle.get('amcl',False)
            and self.native_ok and sensors and map_tf_valid and scan_tf_valid
            and self.amcl_pose_count>=1 and covariance_ok
            and (not update_due or 0<=now-self.match_time<=3.
                 or (self.motion_evidence.started is not None
                     and 0<=now-self.motion_evidence.started<=3.)))
        if self.had_localization and not valid:
            self.motion_evidence.lost=True
            if not self.cancel_sent and self.cancel_client.service_is_ready():
                self.cancel_client.call_async(CancelGoal.Request());self.cancel_sent=True
        self.localized=bool(valid)
        if self.localized:self.had_localization=True
        state=dict(localization_kind='amcl',localized=self.localized,
            parameters_verified=self.native_ok,effective_parameters=self.amcl_parameters,
            assets_unchanged=self.assets_ok,sensors_fresh=sensors,
            frozen_map='frozen_map' in self.received,map_server_active=self.lifecycle.get('map_server',False),
            amcl_active=self.lifecycle.get('amcl',False),authority_ok=authority_ok,
            pose_count=self.amcl_pose_count,pose_stamp_ns=self.amcl_stamp_ns,
            covariance=self.amcl_covariance,
            covariance_above_limit=getattr(self,'amcl_covariance_above_limit',False),
            covariance_warning_stamp_ns=getattr(self,'amcl_covariance_warning_stamp_ns',None),
            pose_age_s=now-self.match_time if self.amcl_pose_count else None,
            stationary_since_match=holding,motion_state=self.motion_evidence.state,
            map_tf_valid=map_tf_valid,scan_tf_valid=scan_tf_valid,
            tf_context=dict(map_to_base_link=map_tf,scan_exact_stamp=scan_tf),
            reason='AMCL pose/parameters, bounded odometry anchor, fresh exact TF')
        self.health_pub.publish(String(data=json.dumps(state)))
        self.session_path.with_name('health.json').write_text(json.dumps(state,indent=2))
        with self.session_path.with_name('health-timeline.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(wall_time=time.time(),**state))+'\n')

    def facts(self):
        facts = super().facts()
        if self.amcl_mode:
            facts['slam_node']=self.lifecycle.get('amcl',False)
        facts['localized'] = self.localized
        facts['map'] = facts['frozen_map'] = 'frozen_map' in self.received
        facts['map_server_active'] = self.lifecycle.get('map_server',False)
        return facts

    def finish_tf_timing_capture(self, reason):
        if self.tf_timing is None:
            return None
        # Guard ESTOP can precede Native health LOST. Save real shutdown
        # evidence without inventing a Native LOST transition in that case.
        filename = ('first-native-tf-window.json' if self.first_loss_event is not None
                    else 'native-tf-shutdown-window.json')
        return self.tf_timing.write_capture(
            self.session_path.with_name(filename),
            self.first_loss_event, reason=reason)


class LocalizationBootstrapProbe(Node):
    """Read the single monitor's history; never publish a competing health state."""
    def __init__(self):
        super().__init__('cleannav_localization_bootstrap_probe')
        self.state = {}
        self.received_at = float('-inf')
        self.state_lock = threading.Lock()
        self.create_subscription(String,'/cleannav/localization/health',self.health,10)

    def health(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state,dict):
                raise ValueError('localization health must be an object')
        except (ValueError,TypeError):
            with self.state_lock:
                self.state = {}
            return
        with self.state_lock:
            self.state = state
            self.received_at = time.monotonic()

    def ready_state(self):
        with self.state_lock:
            state, received_at = self.state, self.received_at
        return state if bootstrap_ready(state,time.monotonic()-received_at) else None

    def ready(self):
        return self.ready_state() is not None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--probe', choices=['simulation','slam','navigation','mission'])
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--tf-mirror-probe', action='store_true')
    parser.add_argument('--tf-timing', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    # The installed launch file only forwards fixed CLI arguments. This
    # explicit environment opt-in lets the reviewed runtime wrapper enable
    # timing on the Native monitor without adding a DDS reader or launch arg.
    args.tf_timing = (args.tf_timing
                      or os.environ.get('CLEANNAV_NATIVE_TF_TIMING') == '1')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = (LocalizationBootstrapProbe() if args.probe=='slam' else
            LocalizationMonitor(args.session,probe_stage=args.probe,enable_motion=args.enable_motion,
                                tf_mirror_probe=args.tf_mirror_probe, tf_timing=args.tf_timing))
    # TransformListener uses its own ReentrantCallbackGroup in Humble. Keep the
    # executor's spin loop continuously dispatching callbacks while the main
    # thread performs bounded bootstrap checks. Panel and health run in the
    # same mutually exclusive callback group; both query/write scan diagnostics.
    if not args.probe:
        configure_localization_panel(node)
    executor = create_localization_executor(getattr(node, 'tf_timing', None))
    executor.add_node(node)
    tf_executor = tf_executor_thread = None
    if hasattr(node, 'tf_listener_node'):
        tf_executor = TFListenerExecutor(node.tf_timing)
        tf_executor.add_node(node.tf_listener_node)
    stopping = False
    executor_errors = []
    def stop(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    def spin_executor():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as error:
            executor_errors.append(repr(error))
    executor_thread = threading.Thread(target=spin_executor,
                                       name='cleannav-localization-executor', daemon=True)
    def spin_tf_executor():
        try:
            tf_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as error:
            executor_errors.append('TF listener: '+repr(error))
    if tf_executor is not None:
        tf_executor_thread = threading.Thread(target=spin_tf_executor,
            name='cleannav-native-tf-executor', daemon=True)
    deadline = time.monotonic()+args.timeout
    result = 0
    try:
        if tf_executor_thread is not None:
            tf_executor_thread.start()
        executor_thread.start()
        while rclpy.ok() and not stopping:
            now = time.monotonic()
            if executor_errors:
                emit('LOCALIZATION','ERROR','executor failed: '+executor_errors[0])
                result=1;break
            if args.probe:
                ready_state = (node.ready_state() if args.probe=='slam' else
                    ({'probe':args.probe} if not missing_requirements(args.probe,node.facts()) else None))
                if ready_state is not None:
                    emit('LOCALIZATION' if args.probe=='slam' else args.probe.upper(),'READY',
                         'verified '+('AMCL pose/TF health' if ready_state.get('localization_kind')=='amcl' else args.probe))
                    break
                if now>deadline:
                    emit('LOCALIZATION','BLOCKED',
                         'bounded readiness timeout; set /rtabmap/initialpose in map and inspect health.json')
                    result=1;break
            time.sleep(.02)
    except ExternalShutdownException:
        result=1
    finally:
        shutdown_ok = stop_localization_executor(executor, executor_thread, node)
        if tf_executor is not None:
            tf_shutdown_ok = stop_tf_listener_executor(
                tf_executor, tf_executor_thread, node.tf_listener_node)
            shutdown_ok = shutdown_ok and tf_shutdown_ok
        if hasattr(node, 'finish_tf_timing_capture'):
            try:
                node.finish_tf_timing_capture(
                    'executor_shutdown' if shutdown_ok else 'executor_shutdown_timeout')
            except Exception as error:
                emit('LOCALIZATION','ERROR','TF timing capture failed: '+repr(error))
                result = 1
        if not shutdown_ok:
            emit('LOCALIZATION', 'ERROR', 'executor workers did not stop within 5 seconds')
            result = 1
        if shutdown_ok and rclpy.ok():rclpy.shutdown()
    return result


if __name__=='__main__':
    raise SystemExit(main())
