#!/usr/bin/env python3
"""Bounded wall-time readiness probes and a change-only simulation panel."""

import argparse
from collections import deque
import json
import math
import time

import rclpy
from cleannav_interfaces.msg import SafetyStatus, TaskStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatusArray


STAGES = ('simulation', 'slam', 'navigation', 'mission')
ERROR_LEVEL = int.from_bytes(Log.ERROR, 'little') if isinstance(Log.ERROR, bytes) else Log.ERROR


def missing_requirements(stage, facts):
    """Keep readiness policy pure; a running process alone is insufficient."""
    requirements = {
        'simulation': ('clock', 'spawn_service', 'vehicle', 'scan', 'odom', 'odom_tf', 'scan_tf'),
        'slam': ('slam_node', 'map_odom_tf', 'map_base_tf'),
        'navigation': ('planner_active', 'controller_active', 'planner_action',
                       'follow_action', 'facade_action', 'safety'),
        'mission': ('task_subscriber', 'task_status', 'real_adapters'),
    }
    return [key for key in requirements[stage] if not facts.get(key, False)]


def emit(component, state, detail):
    print(f'[{component}] {state} {detail}', flush=True)


class SimulationStatusMonitor(Node):
    """Observe real interfaces; do not reproduce task execution policy."""

    def __init__(self, *, enable_motion=False, quiet=False):
        super().__init__('cleannav_simulation_status_monitor')
        self._quiet_panel = quiet
        self.enable_motion = enable_motion
        self.received = {}
        self.last_clock = None
        self.scan_header = None
        self.scan_headers = deque(maxlen=8)
        self.clock_samples = 0
        self.lifecycle = {}
        self.controllers = False
        self.pending = {}
        self.safety = None
        self.runtime = False
        self.seen_status = False
        self.last_panel = {}
        self.authorized = False
        self.authorization_future = None
        self.authorization_deadline = 0.0
        self.authorization_failed = False
        self.ever_ready = False
        self.last_facts = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(ClockMessage, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._joints,
                                 qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.readiness_pub = self.create_publisher(
            String, '/cleannav/simulation/readiness', map_qos)
        self.create_subscription(OccupancyGrid, '/rtabmap/map', self._map, map_qos)
        self.create_subscription(SafetyStatus, '/cleannav/safety_status', self._safety, 10)
        self.create_subscription(TaskStatus, '/cleannav/task_status', self._status, map_qos)
        self.create_subscription(String, '/cleannav/simulation/mission_runtime',
                                 self._runtime, 1)
        self.create_subscription(Log, '/rosout', self._rosout, 100)
        for topic, stage in [('/compute_path_to_pose/_action/status', 'Planning'),
                             ('/follow_path/_action/status', 'Tracking')]:
            self.create_subscription(
                GoalStatusArray, topic,
                lambda msg, stage=stage: self._action_status(msg, stage), map_qos)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/cleannav/simulation/localization_pose', 10)
        self.estop_pub = self.create_publisher(Bool, '/cleannav/safety/emergency_stop', 10)
        self.motion_client = self.create_client(
            SetBool, '/cleannav/safety/set_simulation_motion_enabled')
        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('planner_server', 'controller_server')
        }
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.action_clients = {
            'planner_action': ActionClient(self, ComputePathToPose, '/compute_path_to_pose'),
            'follow_action': ActionClient(self, FollowPath, '/follow_path'),
            'facade_action': ActionClient(self, NavigateToPose, '/cleannav/navigate_to_pose'),
        }
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self._poll, clock=self.steady)

    def _mark(self, name):
        self.received[name] = time.monotonic()

    def _on_clock(self, msg):
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if stamp > 0 and stamp != self.last_clock:
            self.clock_samples += 1
            self.last_clock = stamp
            self._mark('clock')

    def _scan(self, msg):
        if (msg.ranges and math.isfinite(msg.angle_increment)
                and msg.angle_increment > 0 and msg.range_max > msg.range_min
                and any(not math.isnan(value) for value in msg.ranges)):
            self.scan_header = msg.header
            self.scan_headers.append((time.monotonic(), msg.header))
            self._mark('scan')

    def _odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (msg.header.frame_id == 'odom'
                and all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                and math.hypot(q.x, q.y, q.z, q.w) > 1e-9):
            self._mark('odom')

    def _joints(self, msg):
        if {'rear_left_wheel_joint', 'rear_right_wheel_joint'}.issubset(msg.name):
            self._mark('joints')

    def _map(self, msg):
        if (msg.header.frame_id == 'map' and msg.info.resolution > 0
                and msg.info.width * msg.info.height == len(msg.data)
                and len(msg.data) > 0 and any(value >= 0 for value in msg.data)):
            self._mark('map')

    def _safety(self, msg):
        self.safety = msg
        self._mark('safety')
        detail = ('ESTOP active' if msg.emergency_stop_active else
                  'Waiting for authorization' if msg.block_all else
                  f'Lease owner={msg.lease_owner_execution_id}' if msg.autonomous_enabled else
                  'Lease required / motion authorization released')
        state = 'ESTOP' if msg.emergency_stop_active else 'BLOCKED' if msg.block_all else 'READY'
        self._changed('SAFETY', state, detail)

    def _status(self, msg):
        self.seen_status = True
        if not msg.command_id:
            return
        # Generated message constants live on the metaclass in Humble.
        state = next((name.removeprefix('STATE_') for name in dir(TaskStatus)
                      if name.startswith('STATE_') and getattr(TaskStatus, name) == msg.state),
                     str(msg.state))
        self._changed(f'MISSION:{msg.command_id}:{msg.status_scope}', state,
                      f'task={msg.task_id} execution={msg.execution_id} '
                      f'reason={msg.reason_code} {msg.message}', label='MISSION')

    def _runtime(self, msg):
        try:
            self.runtime = json.loads(msg.data)['real_adapters_ready'] is True
            self._mark('runtime')
        except (ValueError, KeyError, TypeError):
            self.runtime = False

    def _action_status(self, msg, stage):
        if any(item.status == 2 for item in msg.status_list):
            self._changed('NAV', stage.upper(), stage)

    def _rosout(self, msg):
        if msg.level >= ERROR_LEVEL:
            self._changed(f'ERROR:{msg.name}', 'ERROR', f'{msg.name}: {msg.msg}', label='ERROR')

    def _changed(self, key, state, detail, label=None):
        value = (state, detail)
        if self.last_panel.get(key) != value:
            self.last_panel[key] = value
            if not self._quiet_panel:
                emit(label or key, state, detail)

    def _fresh(self, name, age=2.5):
        return time.monotonic() - self.received.get(name, -math.inf) <= age

    def _tf(self, target, source):
        try:
            transform = self.buffer.lookup_transform(target, source, Time())
            stamp = transform.header.stamp
            age = self.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
            return -0.5 <= age <= 3.0
        except Exception:
            return False

    def _query(self, key, client, request, consume):
        existing = self.pending.get(key)
        if existing is not None:
            future, deadline = existing
            if future.done():
                self.pending.pop(key)
                try:
                    consume(future.result())
                except Exception:
                    consume(None)
            elif time.monotonic() >= deadline:
                self.pending.pop(key)
                client.remove_pending_request(future)
                consume(None)
            return
        if client.service_is_ready():
            self.pending[key] = (client.call_async(request), time.monotonic() + 2.0)
        else:
            consume(None)

    def _poll(self):
        for name, client in self.state_clients.items():
            self._query(name, client, GetState.Request(),
                        lambda response, name=name: self.lifecycle.update({
                            name: response is not None and response.current_state.id == 3}))
        self._query('controllers', self.controller_client, ListControllers.Request(),
                    self._controllers)
        if self._tf('map', 'base_link'):
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
            pose = PoseWithCovarianceStamped()
            pose.header = transform.header
            pose.pose.pose.position.x = transform.transform.translation.x
            pose.pose.pose.position.y = transform.transform.translation.y
            pose.pose.pose.position.z = transform.transform.translation.z
            pose.pose.pose.orientation = transform.transform.rotation
            self.pose_pub.publish(pose)

    def _controllers(self, response):
        active = ({item.name for item in response.controller if item.state == 'active'}
                  if response else set())
        self.controllers = {'joint_state_broadcaster', 'ackermann_controller'}.issubset(active)

    def facts(self):
        nodes = {(namespace.rstrip('/') + '/' + name)
                 for name, namespace in self.get_node_names_and_namespaces()}
        services = {name for name, _ in self.get_service_names_and_types()}
        return {
            'clock': self.clock_samples >= 2 and self._fresh('clock'),
            'spawn_service': '/spawn_entity' in services,
            'vehicle': self.controllers and self._fresh('joints'),
            'scan': self._fresh('scan'), 'odom': self._fresh('odom'),
            'odom_tf': self._tf('odom', 'base_link'),
            'scan_tf': self._scan_tf(),
            'slam_node': '/rtabmap/rtabmap' in nodes,
            'map_odom_tf': self._tf('map', 'odom'),
            'map_base_tf': self._tf('map', 'base_link'), 'map': 'map' in self.received,
            'planner_active': self.lifecycle.get('planner_server', False),
            'controller_active': self.lifecycle.get('controller_server', False),
            **{key: client.server_is_ready() for key, client in self.action_clients.items()},
            'safety': self._fresh('safety'),
            'task_subscriber': any(
                item.node_name == 'mission_manager_node'
                for item in self.get_subscriptions_info_by_topic('/cleannav/hmi/task_command')),
            'task_status': self.seen_status,
            'real_adapters': self.runtime and self._fresh('runtime'),
        }

    def _scan_tf(self):
        """Check actual sensor frame at its source time before starting RTAB."""
        # A just-arrived scan may precede its odom TF in DDS. Accept an actual
        # transformable scan from the last 0.5 wall seconds, never stale history.
        for received, header in reversed(self.scan_headers):
            if time.monotonic()-received > 0.5 or header.frame_id != 'base_scan':
                continue
            try:
                stamp = Time.from_msg(header.stamp)
                if stamp.nanoseconds <= 0:
                    continue
                self.buffer.lookup_transform('base_footprint', 'base_scan', stamp)
                self.buffer.lookup_transform('odom', 'base_footprint', stamp)
                return True
            except Exception:
                continue
        return False

    def panel(self):
        facts = self.facts()
        missing = [item for stage in STAGES for item in missing_requirements(stage, facts)]
        if not facts['map']:
            missing.append('map (/rtabmap/map not yet usable)')
        ready = not missing
        self.readiness_pub.publish(String(data=json.dumps({
            'ready': (ready and self.safety is not None
                      and not self.safety.emergency_stop_active
                      and not self.authorization_failed),
            'motion_enabled': (self.enable_motion and self.authorized
                               and not self.authorization_failed),
            'missing': missing,
        })))
        if ready and self.safety and self.safety.emergency_stop_active:
            self._changed('CLEANNAV', 'BLOCKED', 'ESTOP active; reset through Mission Manager')
            return
        if ready and self.authorization_failed:
            self._changed('CLEANNAV', 'BLOCKED',
                          'simulation authorization failed; restart required')
            return
        if ready and self.enable_motion and not self.authorized and not self.authorization_failed:
            if self.authorization_future is None and self.motion_client.service_is_ready():
                self.authorization_future = self.motion_client.call_async(
                    SetBool.Request(data=True))
                self.authorization_deadline = time.monotonic() + 2.0
            elif self.authorization_future is not None:
                if self.authorization_future.done():
                    try:
                        self.authorized = self.authorization_future.result().success
                    except Exception:
                        self.authorized = False
                    self.authorization_failed = not self.authorized
                elif time.monotonic() >= self.authorization_deadline:
                    self.authorization_failed = True
                    self.estop_pub.publish(Bool(data=True))
            if not self.authorized:
                self._changed('CLEANNAV', 'BLOCKED', 'simulation authorization pending or failed')
                return
        if self.ever_ready and missing and self.enable_motion:
            # Fail closed after losing a dependency. Never automatically recover.
            self.estop_pub.publish(Bool(data=True))
            self.authorization_failed = True
        if ready:
            self.ever_ready = True
        self._changed('CLEANNAV', 'READY' if ready else 'PARTIAL_READY',
                      'SYSTEM READY; Waiting for TaskCommand; '
                      + ('motion explicitly enabled, Lease required' if self.enable_motion else
                         'observation only, Safety BLOCKED')
                      if ready else 'waiting: ' + ', '.join(missing))


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe', choices=STAGES)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--enable-motion', action='store_true')
    parsed, ros_args = parser.parse_known_args(args)
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        parser.error('timeout must be finite and positive')
    rclpy.init(args=ros_args)
    node = SimulationStatusMonitor(enable_motion=parsed.enable_motion, quiet=bool(parsed.probe))
    deadline = time.monotonic() + parsed.timeout
    next_panel = 0.0
    code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if parsed.probe:
                missing = missing_requirements(parsed.probe, node.facts())
                if not missing:
                    emit(parsed.probe.upper(), 'READY',
                         'actual messages / TF / interfaces verified')
                    if parsed.probe == 'slam' and 'map' not in node.received:
                        emit('SLAM', 'PARTIAL_READY',
                             '/rtabmap/map not yet generated; motion blocked')
                    break
                if now >= deadline:
                    emit('ERROR', 'FAILED', f'{parsed.probe} timeout: ' + ', '.join(missing))
                    code = 1
                    break
            elif now >= next_panel:
                node.panel()
                next_panel = now + 1.0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
