#!/usr/bin/env python3
"""Bounded read-only motion evidence, with separate receipt and source times."""

import argparse
from collections import defaultdict, deque
import json
import math
import re
import signal
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from cleannav_interfaces.msg import CleaningTargetArray, SafetyStatus, TaskCommand, TaskStatus
from control_msgs.msg import SteeringControllerStatus
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from nav2_msgs.msg import Costmap
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class VelocityStats:
    """Online statistics include every received command, without a sample buffer."""

    def __init__(self):
        self.count = self.nonzero = self.switches = 0
        self.total = self.maximum = 0.0
        self.direction = 0

    def add(self, value):
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.maximum = max(self.maximum, abs(value))
        self.nonzero += int(abs(value) > 0.001)
        direction = 1 if value > 0.001 else -1 if value < -0.001 else 0
        if direction:
            self.switches += int(self.direction != 0 and direction != self.direction)
            self.direction = direction

    def summary(self):
        return dict(count=self.count, mean=self.total / self.count if self.count else None,
                    max_abs=self.maximum, nonzero_ratio=self.nonzero/self.count
                    if self.count else None, direction_switches=self.switches)


class Diagnostics(Node):
    def __init__(self, directory):
        super().__init__('cleannav_simulation_motion_diagnostics')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.file = (self.directory / 'samples.jsonl').open('x')
        self.started = time.monotonic()
        self.last_write = {}
        self.stats = defaultdict(VelocityStats)
        self.poses = {}
        self.arc = defaultdict(float)
        self.windows = defaultdict(lambda: deque(maxlen=200))
        self.speed = VelocityStats()
        self.overruns = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = {}
        self.last_query = -math.inf
        self.latest_final = None
        self.goal = None
        self.errors = []
        self.counts = defaultdict(int)
        self.safety_samples = defaultdict(int)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topics = [
            ('/cleannav/cmd_vel_candidate', Twist, 10, 0.01),
            ('/cmd_vel', Twist, 10, 0.01),
            ('/joint_states', JointState, qos_profile_sensor_data, 0.2),
            ('/scan', LaserScan, qos_profile_sensor_data, 1.0),
            ('/scan_raw', LaserScan, qos_profile_sensor_data, 1.0),
            ('/cleannav/simulation/lidar_self_filter', String, 10, 0.0),
            ('/ackermann_controller/controller_state', SteeringControllerStatus, 10, 0.2),
            ('/odom', Odometry, qos_profile_sensor_data, 0.2),
            ('/plan', NavPath, 10, 0.0),
            ('/cleannav/simulation/navigation_goal', PoseStamped, 10, 0.0),
            ('/cleannav/rviz/goal_pose', PoseStamped, 10, 0.0),
            ('/cleannav/simulation/rviz_goal_events', String, 10, 0.0),
            ('/cleannav/simulation/cleaning_targets', CleaningTargetArray, 10, 0.0),
            ('/cleannav/hmi/task_command', TaskCommand, 10, 0.0),
            ('/cleannav/task_status', TaskStatus, latched, 0.0),
            ('/cleannav/safety_status', SafetyStatus, 10, 0.0),
            ('/cleannav/safety_supervisor_status', String, 10, 0.0),
            ('/cleannav/simulation/readiness', String, latched, 1.0),
            ('/follow_path/_action/feedback', FollowPath.Impl.FeedbackMessage, 10, 0.2),
            ('/follow_path/_action/status', GoalStatusArray, latched, 0.0),
            ('/compute_path_to_pose/_action/status', GoalStatusArray, latched, 0.0),
            ('/global_costmap/costmap', OccupancyGrid, latched, 5.0),
            ('/local_costmap/costmap', OccupancyGrid, latched, 5.0),
            ('/rtabmap/map', OccupancyGrid, latched, 1.0),
            ('/global_costmap/costmap_raw', Costmap, latched, 1.0),
            ('/local_costmap/costmap_raw', Costmap, latched, 1.0),
            ('/global_costmap/published_footprint', PolygonStamped, 10, 0.2),
            ('/local_costmap/published_footprint', PolygonStamped, 10, 0.2),
            ('/rosout', Log, 100, 0.0),
        ]
        for topic, msg_type, qos, interval in topics:
            self.create_subscription(msg_type, topic,
                                     lambda msg, t=topic, i=interval: self.sample(t, msg, i), qos)
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.params = self.create_client(GetParameters, '/controller_server/get_parameters')

    def record(self, kind, data, source_stamp=None, **extra):
        row = dict(kind=kind, wall_time=time.time(), monotonic=time.monotonic(),
                   ros_time_ns=self.get_clock().now().nanoseconds,
                   source_stamp=source_stamp, data=data, **extra)
        self.file.write(json.dumps(row, allow_nan=True) + '\n')

    def pose(self, name, pose, stamp=None):
        now = time.monotonic()
        point = [pose.position.x, pose.position.y, yaw(pose.orientation)]
        old = self.poses.get(name)
        if old:
            self.arc[name] += math.hypot(point[0]-old['last'][0], point[1]-old['last'][1])
            old['last'] = point
        else:
            self.poses[name] = {'first': point, 'last': point}
        window = self.windows[name]
        if not window or now-window[-1][0] >= 0.1:
            window.append((now, point))
        while window and now-window[0][0] > 15.0:
            window.popleft()

    def sample(self, topic, msg, interval):
        now = time.monotonic()
        self.counts[topic] += 1
        if isinstance(msg, Twist):
            self.stats[topic].add(msg.linear.x)
            self.stats[topic + '/wz'].add(msg.angular.z)
            if topic == '/cmd_vel':
                self.latest_final = (now, msg.linear.x, msg.angular.z)
        elif isinstance(msg, Odometry):
            self.pose('odom', msg.pose.pose)
        elif isinstance(msg, PoseStamped):
            self.goal = [msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)]
        elif topic == '/cleannav/safety_supervisor_status':
            self.safety_samples['count'] += 1
            self.safety_samples['limited'] += int('last_output_limited=True' in msg.data)
            self.safety_samples['fresh'] += int('candidate_fresh' in msg.data)
            match = re.search(r'last_age_sec=([-\d.]+)', msg.data)
            if match:
                self.stats['candidate_age_sec'].add(float(match.group(1)))
        elif isinstance(msg, Log):
            if msg.msg.startswith('NAVIGATION_GOAL '):
                goal = json.loads(msg.msg.removeprefix('NAVIGATION_GOAL '))
                self.goal = [goal['x'], goal['y'], goal['yaw']]
                self.record('navigation_goal', goal, goal['stamp'])
            text = msg.msg.lower()
            if 'control loop' in text and ('missed' in text or 'rate' in text):
                self.overruns += 1
            if msg.level >= 40 and len(self.errors) < 100:
                self.errors.append([msg.name, msg.msg])
            if msg.level < 30 and not msg.msg.startswith('NAVIGATION_GOAL '):
                return
        if now-self.last_write.get(topic, -math.inf) < interval:
            return
        self.last_write[topic] = now
        header = getattr(msg, 'header', None)
        stamp = message_to_ordereddict(header.stamp) if header else None
        self.record(topic, message_to_ordereddict(msg), stamp)

    def query(self, key, client, request, consume):
        if key in self.pending:
            future, deadline, requested = self.pending[key]
            if future.done():
                self.pending.pop(key)
                try:
                    consume(future.result(), requested)
                except Exception as error:
                    self.record('query_error', {'service': key, 'error': str(error)})
            elif time.monotonic() > deadline:
                self.pending.pop(key)
                client.remove_pending_request(future)
                self.record('query_timeout', {'service': key})
            return
        if client.service_is_ready():
            requested = time.monotonic()
            self.pending[key] = (client.call_async(request), requested+2.0, requested)

    def entity_response(self, response, requested):
        self.record('gazebo_state', message_to_ordereddict(response),
                    message_to_ordereddict(response.header.stamp), requested_monotonic=requested)
        if response.success:
            self.pose('gazebo', response.state.pose)
            self.speed.add(math.hypot(
                response.state.twist.linear.x, response.state.twist.linear.y))

    def poll(self):
        if 'odom' in self.poses:
            self.query('gazebo', self.entity, GetEntityState.Request(name='cleannav_ackermann'),
                       self.entity_response)
        for target, source in [('odom', 'base_footprint'), ('map', 'odom'),
                               ('map', 'base_link'), ('map', 'base_scan')]:
            try:
                tf = self.buffer.lookup_transform(target, source, Time())
                self.record('tf:' + target + ':' + source, message_to_ordereddict(tf),
                            message_to_ordereddict(tf.header.stamp))
                if target == 'map' and source == 'base_link':
                    pose = PoseStamped().pose
                    pose.position.x = tf.transform.translation.x
                    pose.position.y = tf.transform.translation.y
                    pose.orientation = tf.transform.rotation
                    self.pose('map', pose)
            except Exception as error:
                self.record('tf_unavailable', {'target': target, 'source': source,
                                               'error': str(error)})
        now = time.monotonic()
        if now-self.last_query > 5 or 'controllers' in self.pending:
            self.query('controllers', self.controllers, ListControllers.Request(),
                       lambda r, t: self.record('controllers', message_to_ordereddict(r)))
        if now-self.last_query > 5 or 'parameters' in self.pending:
            names = ['use_sim_time', 'controller_frequency',
                     'progress_checker.required_movement_radius',
                     'progress_checker.movement_time_allowance',
                     'general_goal_checker.xy_goal_tolerance',
                     'general_goal_checker.yaw_goal_tolerance', 'FollowPath.vx_std',
                     'FollowPath.motion_model', 'FollowPath.AckermannConstraints.min_turning_r']
            self.query('parameters', self.params, GetParameters.Request(names=names),
                       lambda r, t: self.record('parameters', dict(
                           names=names, response=message_to_ordereddict(r))))
        if now-self.last_query > 5:
            self.last_query = now
            self.record('graph', {'cmd_vel_publishers': [
                dict(name=x.node_name, namespace=x.node_namespace)
                for x in self.get_publishers_info_by_topic('/cmd_vel')],
                'cmd_vel_subscribers': [dict(name=x.node_name, namespace=x.node_namespace)
                                        for x in self.get_subscriptions_info_by_topic('/cmd_vel')],
                'topics': self.get_topic_names_and_types()})
        self.file.flush()

    def finish(self):
        summary = {'duration': time.monotonic()-self.started, 'counts': dict(self.counts),
                   'velocity': {key: value.summary() for key, value in self.stats.items()},
                   'poses': self.poses, 'arc_lengths': dict(self.arc),
                   'actual_speed': self.speed.summary(), 'goal': self.goal,
                   'safety_status_sample_counts': dict(self.safety_samples),
                   'controller_loop_overruns': self.overruns, 'errors': self.errors}
        for key, values in self.poses.items():
            values['displacement'] = math.dist(values['first'][:2], values['last'][:2])
            window = self.windows[key]
            values['progress_window_displacement'] = (
                math.dist(window[0][1][:2], window[-1][1][:2]) if window else None)
            if key == 'map' and self.goal:
                values['goal_distance_initial'] = math.dist(values['first'][:2], self.goal[:2])
                values['goal_distance_final'] = math.dist(values['last'][:2], self.goal[:2])
        self.file.close()
        (self.directory / 'summary.json').write_text(json.dumps(summary, indent=2))
        print('[DIAGNOSTICS] summary=' + str(self.directory / 'summary.json'), flush=True)

    def stop_inputs(self):
        """Stop collection while the context and all message types remain alive."""
        for timer in tuple(self.timers):
            timer.cancel()
            self.destroy_timer(timer)
        for future, _, _ in self.pending.values():
            future.cancel()
        self.pending.clear()
        self.listener.unregister()
        for subscription in tuple(self.subscriptions):
            self.destroy_subscription(subscription)


class DiagnosticsExecutor(SingleThreadedExecutor):
    """Preserve failures and identify a failed DDS take before any callback."""

    def _take_subscription(self, subscription):
        try:
            return super()._take_subscription(subscription)
        except RuntimeError:
            print('[DIAGNOSTICS] take_message failed: topic=' + subscription.topic_name
                  + ' type=' + subscription.msg_type.__module__ + '.'
                  + subscription.msg_type.__name__
                  + ' context_ok=' + str(self.context.ok()), flush=True)
            raise


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--duration', type=float, default=300)
    parsed, ros_args = parser.parse_known_args(args)
    if not math.isfinite(parsed.duration) or not 1 <= parsed.duration <= 1800:
        parser.error('duration must be in 1..1800 wall seconds')
    # Humble's default C signal handler shuts the context down asynchronously
    # with respect to an in-flight DDS take. Let the spin thread own teardown.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Diagnostics(parsed.output_dir)
    executor = DiagnosticsExecutor(context=node.context)
    executor.add_node(node)
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    deadline = time.monotonic()+parsed.duration
    next_poll = 0.0
    try:
        while not stopping and rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if not stopping and rclpy.ok() and time.monotonic() >= next_poll:
                node.poll()
                next_poll = time.monotonic()+0.2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)
        try:
            node.stop_inputs()
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
