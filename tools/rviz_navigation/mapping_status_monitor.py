#!/usr/bin/env python3
"""Mapping readiness requires real sensor/TF/grid/controller/Safety inputs.

Reviewed source copy of the installed ``cleannav_simulation/mapping_status_monitor``
(SHA256 76b084077ed6d30e3fc502e356e42bc1d68d12654e210f4b861c7ef8ece7c4e2) with one
change: the RTAB resume barrier selects its ``/odom`` child frame and body-frame
TF from the recorded ``odometry_mode`` instead of always requiring the wheel
rear-axle chain. Every other readiness condition is unchanged.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from controller_manager_msgs.srv import ListControllers
from cleannav_interfaces.msg import SafetyStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, JointState
from rosgraph_msgs.msg import Clock as ClockMessage
from rtabmap_msgs.msg import Info
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener, TransformException
# This reviewed copy runs from the worktree while mapping_support stays in the
# selected installed package (same binding as localization_status_monitor.py).
sys.path.append(str(Path(get_package_prefix('cleannav_simulation'))/'lib/cleannav_simulation'))
from mapping_support import map_quality, usable_scan


class MappingMonitor(Node):
    def __init__(self, session, resume_rtab=False):
        super().__init__('mapping_status_monitor')
        self.session = session
        # The selected odometry source, not the localizer, defines the /odom
        # child frame and the body-frame TF this barrier may require. A session
        # without the recorded key keeps the historical wheel contract.
        self.odometry_mode = json.loads(Path(session).read_text()).get('odometry_mode', 'wheel')
        self.odom_child_frame = ('base_footprint' if self.odometry_mode == 'oracle'
                                 else 'ackermann_rear_axle')
        self.resume_rtab = resume_rtab
        self.boot_stage = 'check-paused'
        self.boot_future = None
        self.boot_deadline = 0.
        self.boot_rpc_attempts = 0
        self.boot_error = None
        self.boot_complete = False
        self.good_input_samples = 0
        self.last_input_stamp = None
        if resume_rtab:
            self.rtab_params = self.create_client(GetParameters, '/rtabmap/rtabmap/get_parameters')
            self.rtab_resume = self.create_client(Empty, '/rtabmap/rtabmap/resume')
        self.received = {}
        self.scan = self.grid = self.safety = None
        self.scan_history = deque(maxlen=8)
        self.controllers = False
        self.controller_future = None
        self.controller_deadline = 0.
        self.facts = {}
        self.last_panel = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.readiness_pub = self.create_publisher(String, '/cleannav/mapping/readiness', qos)
        self.session_pub = self.create_publisher(String, '/cleannav/mapping/session', qos)
        self.create_subscription(ClockMessage, '/clock', lambda m: self.mark('clock') if m.clock.sec or m.clock.nanosec else None, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', lambda m: self.mark('joints'), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/rtabmap/map', self.map_cb, qos)
        self.create_subscription(Info, '/rtabmap/info', lambda m: self.mark('rtab_info'), 10)
        self.create_subscription(SafetyStatus, '/cleannav/safety_status', self.safety_cb, 10)
        self.timer = self.create_timer(.5, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def mark(self, name):
        self.received[name] = time.monotonic()

    def fresh(self, name, age=3.):
        return time.monotonic()-self.received.get(name, -math.inf) < age

    def scan_cb(self, msg):
        if usable_scan(msg):
            self.scan = msg
            self.scan_history.append((time.monotonic(), msg.header.stamp))
            self.mark('scan')

    def odom_cb(self, msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == self.odom_child_frame:
            self.mark('odom')

    def map_cb(self, msg):
        self.grid = msg

    def safety_cb(self, msg):
        self.safety = msg
        self.mark('safety')

    def tf(self, target, source, stamp=None):
        transform = self.buffer.lookup_transform(target, source, stamp or Time())
        age = self.get_clock().now().nanoseconds/1e9 - transform.header.stamp.sec - transform.header.stamp.nanosec/1e9
        if stamp is None and transform.header.stamp.sec and not -.5 <= age <= 3.:
            raise ValueError('stale TF')
        return transform

    def poll(self):
        now = time.monotonic()
        if self.controller_future:
            if self.controller_future.done():
                response = self.controller_future.result()
                active = {c.name for c in response.controller if c.state == 'active'}
                self.controllers = {'ackermann_controller', 'joint_state_broadcaster'}.issubset(active)
                self.controller_future = None
            elif now > self.controller_deadline:
                self.controller_client.remove_pending_request(self.controller_future)
                self.controller_future = None
                self.controllers = False
        elif self.controller_client.service_is_ready():
            self.controller_future = self.controller_client.call_async(ListControllers.Request())
            self.controller_deadline = now+2.
        facts = dict(clock=self.fresh('clock'), vehicle=self.controllers and self.fresh('joints'),
                     scan=self.fresh('scan'), odom=self.fresh('odom'),
                     safety=self.fresh('safety'), rear_axle_tf=False, scan_tf=False,
                     map_odom_tf=False, map=False, rtabmap=self.fresh('rtab_info'))
        quality = None
        try:
            if self.odometry_mode == 'oracle':
                # Oracle publishes /odom at base_footprint and disables the
                # wheel rear-axle mount (publish_rear_axle_tf=false), so no
                # ackermann_rear_axle frame exists. The fresh odom->body chain
                # is the same barrier fact the wheel rear-axle link proves.
                self.tf('odom', 'base_footprint')
                facts['rear_axle_tf'] = True
            else:
                rear = self.tf('ackermann_rear_axle', 'base_footprint')
                facts['rear_axle_tf'] = abs(rear.transform.translation.x-.30) < 1e-6
                self.tf('odom', 'ackermann_rear_axle')
            if self.scan and self.fresh('scan'):
                # Scan and wheel TF arrive independently: use a transformable
                # recent scan, retaining the source timestamp rather than now.
                for received, stamp in reversed(self.scan_history):
                    if now-received > .5:
                        break
                    try:
                        self.tf('odom', 'base_scan', Time.from_msg(stamp))
                        facts['scan_tf'] = True
                        break
                    except TransformException:
                        continue
            self.tf('map', 'odom')
            facts['map_odom_tf'] = True
            pose = self.tf('map', 'base_link').transform.translation
            if self.grid:
                quality = map_quality(self.grid, (pose.x, pose.y))
                facts['map'] = quality['usable']
        except (TransformException, ValueError) as error:
            detail = str(error)
        else:
            detail = ''
        self.facts = facts
        ready = all(facts.values()) and self.safety is not None and not self.safety.emergency_stop_active
        value = dict(ready=ready, operation_mode='mapping', facts=facts, quality=quality,
                     session_path=self.session, error=detail)
        if self.resume_rtab:
            self.resume_when_ready(facts)
            return
        self.readiness_pub.publish(String(data=json.dumps(value)))
        self.session_pub.publish(String(data=self.session))
        # Keep panel readable; map quality data remains available on the topic.
        panel = (ready, tuple(k for k, v in facts.items() if not v))
        if panel != self.last_panel:
            print(f'[MAPPING] {"READY" if ready else "WAITING"} missing={panel[1]} quality={quality}', flush=True)
            self.last_panel = panel

    def resume_when_ready(self, facts):
        """Prime the new RTAB executor/TF listener before allowing scan callbacks.

        No transform validation is skipped. The native paused state is read back;
        fresh source-time TF and advancing scan stamps are required before resume.
        This checks our TF buffer, not RTAB's private buffer, so it is a startup
        barrier rather than a claim of direct access to RTAB's internal state.
        """
        now = time.monotonic()
        required = ('clock', 'vehicle', 'scan', 'odom', 'rear_axle_tf', 'scan_tf', 'safety')
        stamp = (self.scan.header.stamp.sec, self.scan.header.stamp.nanosec) if self.scan else None
        if all(facts[k] for k in required) and stamp != self.last_input_stamp:
            self.good_input_samples += 1
            self.last_input_stamp = stamp
        elif not all(facts[k] for k in required):
            self.good_input_samples = 0
        if self.boot_future:
            if not self.boot_future.done():
                if now > self.boot_deadline:
                    if self.boot_stage in ('check-paused', 'verify-resumed') and self.boot_rpc_attempts < 3:
                        # Read-only calls are safe to retry after DDS response
                        # discovery failures. Never retry an ambiguous resume.
                        self.rtab_params.remove_pending_request(self.boot_future)
                        self.boot_future = None
                        print('[MAPPING] RETRY bounded native parameter read after RPC timeout', flush=True)
                    else:
                        self.boot_error = 'native RTAB startup RPC timeout'
                return
            try:
                response = self.boot_future.result()
                if self.boot_stage in ('check-paused', 'verify-resumed'):
                    if len(response.values) != 1 or response.values[0].type != 1:
                        raise ValueError('installed RTAB paused parameter is not boolean')
                    paused = response.values[0].bool_value
                    if paused != (self.boot_stage == 'check-paused'):
                        raise ValueError('native RTAB startup paused-state mismatch')
                if self.boot_stage == 'check-paused':
                    self.boot_stage = 'wait-inputs'
                    self.boot_rpc_attempts = 0
                elif self.boot_stage == 'resume':
                    self.boot_stage = 'verify-resumed'
                    self.boot_rpc_attempts = 0
                else:
                    self.boot_complete = True
                    print('[MAPPING] RTAB_INPUT_READY native resume acknowledged; source TF checked', flush=True)
            except (RuntimeError, ValueError) as error:
                self.boot_error = str(error)
            self.boot_future = None
        if self.boot_error or self.boot_complete:
            return
        if self.boot_stage in ('check-paused', 'verify-resumed') and self.rtab_params.service_is_ready():
            self.boot_rpc_attempts += 1
            self.boot_future = self.rtab_params.call_async(GetParameters.Request(names=['is_rtabmap_paused']))
        elif (self.boot_stage == 'wait-inputs' and self.good_input_samples >= 3
              and self.rtab_resume.service_is_ready()):
            self.boot_stage = 'resume'
            self.boot_future = self.rtab_resume.call_async(Empty.Request())
        if self.boot_future:
            self.boot_deadline = now+2.


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--probe-simulation', action='store_true')
    parser.add_argument('--resume-rtab', action='store_true')
    parser.add_argument('--timeout', type=float, default=210.)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = MappingMonitor(args.session, resume_rtab=args.resume_rtab)
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    deadline = time.monotonic()+args.timeout
    result = 0
    ever_ready = False
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
            if args.resume_rtab:
                if node.boot_complete:
                    break
                if node.boot_error or time.monotonic() > deadline:
                    print(f'[MAPPING] FAILED RTAB startup: {node.boot_error or "input readiness timeout"}', flush=True)
                    result = 1
                    break
            elif args.probe_simulation:
                required = ('clock', 'vehicle', 'scan', 'odom', 'rear_axle_tf', 'scan_tf', 'safety')
                if node.facts and all(node.facts[k] for k in required):
                    print('[MAPPING] SIMULATION_READY sensor and rear-axle TF ready', flush=True)
                    break
                if time.monotonic() > deadline:
                    result = 1
                    break
            else:
                ever_ready = ever_ready or (bool(node.facts) and all(node.facts.values()))
                if not ever_ready and time.monotonic() > deadline:
                    print(f'[MAPPING] FAILED no initial SLAM readiness: {node.facts}', flush=True)
                    result = 1
                    break
    except ExternalShutdownException:
        result = 1
    finally:
        node.timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
