"""Read-only localization telemetry; no goal, command, authorization or Lease API."""
from collections import Counter
import json
import math
from pathlib import Path
import time
import threading


def classify_info(message, node_ids, accepted_match):
    stats = dict(zip(message.stats_keys, message.stats_values))
    accepted = bool(accepted_match(message, node_ids))
    rejected = stats.get('Loop/RejectedHypothesis/', 0.)
    ids = [message.loop_closure_id, message.proximity_detection_id]
    stage = ('ACCEPTED_PROJECT_MATCH' if accepted else
             'REJECTED_HYPOTHESIS_REPORTED' if rejected else
             'MATCH_ID_FAILS_PROJECT_CONTRACT' if any(i > 0 for i in ids) else
             'INFO_WITHOUT_REPORTED_MATCH_CANDIDATE')
    return dict(classification=stage, accepted_by_installed_contract=accepted,
                loop_closure_id=ids[0], proximity_detection_id=ids[1],
                rejected_hypothesis=rejected, ref_id=message.ref_id,
                stats=stats,
                limitations='Info exposes reported IDs/rejection, not every internal registration attempt')


def safe_json(value):
    if isinstance(value, dict): return {str(k):safe_json(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)): return [safe_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return str(value)
    return value


class DiagnosticRecorder:
    def __init__(self, node, run_dir, run_id, node_ids, accepted_match):
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rclpy.clock import Clock, ClockType
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rosgraph_msgs.msg import Clock as ClockMessage
        from tf2_msgs.msg import TFMessage
        from rtabmap_msgs.msg import Info
        from std_msgs.msg import String
        from gazebo_msgs.srv import GetEntityState
        from rcl_interfaces.srv import ListParameters, GetParameters
        from tf2_ros import Buffer, TransformListener
        from rosidl_runtime_py.convert import message_to_ordereddict
        from publisher_graph_gate import endpoint_record
        self.convert=message_to_ordereddict
        self.endpoint_record=endpoint_record
        self.node=node;self.run_id=run_id;self.path=Path(run_dir)/'diagnostics'
        self.path.mkdir();self.streams={};self.counts=Counter();self.classifications=Counter()
        self.last_odom_stamp=None;self.last_clock_ns=None;self.info_first=None;self.info_last=None
        self.last_endpoint_snapshot=None
        self.accepted_match=accepted_match;self.node_ids=node_ids
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,node)
        for topic in ('/scan','/mapping/scan_raw','/mapping/scan'):
            node.create_subscription(LaserScan,topic,lambda m,t=topic:self.scan(t,m),qos_profile_sensor_data)
        node.create_subscription(Odometry,'/odom',self.odom,10)
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',
            lambda m:self.write('amcl_pose',dict(message=self.convert(m))),
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Info,'/rtabmap/info',self.info,10)
        node.create_subscription(ClockMessage,'/clock',self.clock,qos_profile_sensor_data)
        node.create_subscription(String,'/cleannav/localization/health',self.health,10)
        # This installed Humble executor invokes callback(message), discarding
        # MessageInfo. Endpoint identities remain in endpoint_graph.jsonl;
        # individual TF-message publisher GIDs are explicitly unavailable.
        node.create_subscription(TFMessage,'/tf',lambda m:self.write('tf',dict(self.convert(m),publisher_gid=None)),100)
        node.create_subscription(TFMessage,'/tf_static',lambda m:self.write('tf_static',dict(self.convert(m),publisher_gid=None)),
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.state_client=node.create_client(GetEntityState,'/gazebo/get_entity_state')
        self.state_type=GetEntityState;self.state_pending=None;self.entity_index=0
        self.entities=('cleannav_ackermann','cleannav_ackermann::base_footprint')
        self.list_client=node.create_client(ListParameters,'/rtabmap/rtabmap/list_parameters')
        self.params_client=node.create_client(GetParameters,'/rtabmap/rtabmap/get_parameters')
        self.list_type=ListParameters;self.params_type=GetParameters
        self.params_future=None;self.params_names=None;self.params_done=False
        self.timer=node.create_timer(.5,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.write('status',dict(info_status='NO_INFO_RECEIVED',node_ids=node_ids))

    def write(self, kind, payload, *, callback_entry_monotonic_ns=None):
        self.counts[kind]+=1
        if kind not in self.streams:self.streams[kind]=(self.path/(kind+'.jsonl')).open('x')
        received_monotonic_ns=time.monotonic_ns()
        row=dict(run_id=self.run_id,received_monotonic_ns=received_monotonic_ns,
                 received_wall_ns=time.time_ns(),latest_clock_ns=self.last_clock_ns,**payload)
        if callback_entry_monotonic_ns is not None:
            row['callback_entry_monotonic_ns']=callback_entry_monotonic_ns
            row['callback_processing_elapsed_ns']=max(0,received_monotonic_ns-callback_entry_monotonic_ns)
        self.streams[kind].write(json.dumps(safe_json(row),allow_nan=False)+'\n')

    def scan(self,topic,msg):
        callback_entry=time.monotonic_ns()
        self.counts['scan:'+topic]+=1
        stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        record=dict(topic=topic,sequence=self.counts['scan:'+topic],
            source_stamp_ns=stamp_ns,source_stamp_domain='ROS_SIM_TIME',
            header=self.convert(msg.header),beams=len(msg.ranges),
            finite_returns=sum(math.isfinite(r) and msg.range_min<=r<=msg.range_max for r in msg.ranges),
            inf_returns=sum(r==math.inf for r in msg.ranges),nan_returns=sum(math.isnan(r) for r in msg.ranges),
            angle_min=msg.angle_min,angle_max=msg.angle_max,angle_increment=msg.angle_increment,
            range_min=msg.range_min,range_max=msg.range_max,
            receipt_clock_domain='CLOCK_MONOTONIC')
        if topic == '/scan':
            record['ranges']=list(msg.ranges)
        self.write('scan',record,callback_entry_monotonic_ns=callback_entry)

        if topic == '/scan':
            from rclpy.time import Time
            queries=[]
            for target,source in (('odom','base_footprint'),('base_footprint',msg.header.frame_id)):
                row=dict(target=target,source=source,query_stamp_ns=stamp_ns)
                try:
                    tf=self.buffer.lookup_transform(target,source,Time.from_msg(msg.header.stamp))
                    row.update(available=True,transform=self.convert(tf))
                except Exception as error:
                    row.update(available=False,error_type=type(error).__name__,error=str(error))
                queries.append(row)
            self.write('scan_tf_queries',dict(topic=topic,header=self.convert(msg.header),
                queries=queries,query_method='exact scan stamp at callback; no fallback or retry'))

    def odom(self,msg):
        self.last_odom_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.write('odom',dict(topic='/odom',message=self.convert(msg)))

    def clock(self,msg):
        callback_entry=time.monotonic_ns()
        self.last_clock_ns=msg.clock.sec*10**9+msg.clock.nanosec
        self.write('clock',dict(topic='/clock',source_stamp_ns=self.last_clock_ns,
            source_stamp_domain='ROS_SIM_TIME',receipt_clock_domain='CLOCK_MONOTONIC'),
            callback_entry_monotonic_ns=callback_entry)

    def endpoint_snapshot(self):
        topics=('/scan','/mapping/scan_raw','/mapping/scan','/odom','/wheel/odom',
                '/wheel/tf','/tf','/tf_static')
        return {
            topic:{
                'publishers':[self.endpoint_record(endpoint)
                    for endpoint in self.node.get_publishers_info_by_topic(topic)],
                'subscribers':[self.endpoint_record(endpoint)
                    for endpoint in self.node.get_subscriptions_info_by_topic(topic)],
            }
            for topic in topics
        }

    def health(self,msg):
        try: payload=json.loads(msg.data)
        except ValueError: payload={'invalid_json':msg.data}
        self.write('health',dict(message=payload))

    def info(self,msg):
        data=classify_info(msg,self.node_ids,self.accepted_match)
        self.classifications[data['classification']]+=1
        self.info_first=self.info_first or time.monotonic_ns();self.info_last=time.monotonic_ns()
        self.write('info',dict(topic='/rtabmap/info',header=self.convert(msg.header),
            message=self.convert(msg),**data))

    def tick(self):
        from rclpy.time import Time
        from rclpy.clock import ClockType
        states=[]
        for target,source in (('map','odom'),('odom','base_footprint'),('odom','ackermann_rear_axle'),('map','base_footprint')):
            row=dict(target=target,source=source,query_stamp_ns=self.last_odom_stamp,
                     query_method='exact last received odometry timestamp')
            try:
                if self.last_odom_stamp is None:raise ValueError('no odometry received')
                tf=self.buffer.lookup_transform(target,source,Time(nanoseconds=self.last_odom_stamp,clock_type=ClockType.ROS_TIME))
                row.update(available=True,transform=self.convert(tf))
            except Exception as e:row.update(available=False,error_type=type(e).__name__,error=str(e))
            states.append(row)
        self.write('tf_queries',dict(queries=states,state_service_ready=self.state_client.service_is_ready(),
            entity_request_pending_age_s=((time.monotonic_ns()-self.state_pending[2])/1e9 if self.state_pending else None)))
        endpoint_snapshot=self.endpoint_snapshot()
        if endpoint_snapshot != self.last_endpoint_snapshot:
            self.write('endpoint_graph',dict(source='rclpy graph endpoint inspection',
                topics=endpoint_snapshot,
                interpretation='publisher/subscriber endpoint QoS and identity; no source-side publication wall timestamp'))
            self.last_endpoint_snapshot=endpoint_snapshot
        if self.state_pending is not None:
            future,entity,sent_ns=self.state_pending
            if future.done():
                try:
                    response=future.result()
                    self.counts['successful_entity_responses']+=int(response.success)
                    self.write('entities',dict(entity=entity,reference_frame='world',source='/gazebo/get_entity_state',
                        request_monotonic_ns=sent_ns,response=self.convert(response)))
                except Exception as e:self.write('entities',dict(entity=entity,error=str(e)))
                self.state_pending=None
        elif self.state_client.service_is_ready():
            entity=self.entities[self.entity_index%len(self.entities)];self.entity_index+=1
            request=self.state_type.Request();request.name=entity;request.reference_frame='world'
            self.state_pending=(self.state_client.call_async(request),entity,time.monotonic_ns())
        if not self.params_done:
            if self.params_future is None and self.list_client.service_is_ready():
                self.params_future=self.list_client.call_async(self.list_type.Request())
            elif self.params_future is not None and self.params_future.done():
                response=self.params_future.result()
                if self.params_names is None:
                    self.params_names=list(response.result.names)
                    request=self.params_type.Request();request.names=self.params_names
                    self.params_future=self.params_client.call_async(request)
                else:
                    self.write('parameters',dict(source='/rtabmap/rtabmap/get_parameters',
                        names=self.params_names,response=self.convert(response)))
                    self.params_done=True
        for stream in self.streams.values():stream.flush()
        self.summary()

    def summary(self):
        data=dict(run_id=self.run_id,counts=dict(self.counts),info_classifications=dict(self.classifications),
                  info_status='INFO_RECEIVED' if self.info_first else 'NO_INFO_RECEIVED',
                  info_first_received_ns=self.info_first,info_last_received_ns=self.info_last,
                  parameter_snapshot_obtained=self.params_done,independent_truth='AVAILABLE' if self.counts['successful_entity_responses'] else 'UNKNOWN')
        (self.path/'summary.json').write_text(json.dumps(data,indent=2)+'\n')

    def close(self):
        self.summary()
        for stream in self.streams.values():stream.close()


class DiagnosticWorker:
    """A separate executor drains telemetry without the supervisor's poll cadence.

    A callback failure or an unjoined receiver thread is a failed diagnostic,
    never a localization or cleanup success. No ROS import is needed in tests.
    """
    def __init__(self, recorder, executor):
        self.recorder=recorder;self.executor=executor;self.error=None
        self.thread=threading.Thread(target=self._spin,name='localization-telemetry',daemon=True)

    def _spin(self):
        try:self.executor.spin()
        except BaseException as error:self.error=f'{type(error).__name__}: {error}'

    def start(self):self.thread.start()

    def close(self, timeout_s=1.0):
        settled=self.executor.shutdown(timeout_sec=timeout_s)
        self.thread.join(timeout_s)
        if settled is False or self.thread.is_alive():
            raise RuntimeError('diagnostic executor did not stop within its join budget')
        self.recorder.close()
        self.recorder.node.destroy_node()
        if self.error:raise RuntimeError('diagnostic callback failed: '+self.error)
