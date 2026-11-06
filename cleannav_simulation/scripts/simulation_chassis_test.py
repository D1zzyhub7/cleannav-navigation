#!/usr/bin/env python3
"""Explicit bounded Gazebo-only lease holder; publishes candidates, never cmd_vel."""

import argparse
import math
import os
import time
from urllib.parse import urlparse

import rclpy
from cleannav_interfaces.msg import SafetyStatus, TaskStatus
from cleannav_interfaces.srv import SafetyLease
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def validate_environment():
    master = urlparse(os.environ.get('GAZEBO_MASTER_URI', ''))
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or not 1 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
            or master.hostname not in ('127.0.0.1', 'localhost')):
        raise ValueError('requires isolated nonzero ROS domain, localhost and local Gazebo master')


class ChassisTest(Node):
    def __init__(self):
        super().__init__('cleannav_simulation_chassis_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.owner = f'simulation-chassis-test-{os.getpid()}'
        self.ready = False
        self.status = None
        self.active_task = False
        self.last_status = -math.inf
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/cleannav/simulation/readiness', self.readiness, qos)
        self.create_subscription(SafetyStatus, '/cleannav/safety_status', self.safety, 10)
        self.create_subscription(TaskStatus, '/cleannav/task_status', self.task, qos)
        self.pub = self.create_publisher(Twist, '/cleannav/cmd_vel_candidate', 10)
        self.estop = self.create_publisher(Bool, '/cleannav/safety/emergency_stop', 10)
        self.acquire = self.create_client(SafetyLease, '/cleannav/safety/acquire_lease')
        self.release = self.create_client(SafetyLease, '/cleannav/safety/release_lease')
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')

    def readiness(self, msg):
        import json
        state = json.loads(msg.data)
        self.ready = state.get('ready') is True and state.get('motion_enabled') is True

    def safety(self, msg):
        self.status = msg
        self.last_status = time.monotonic()

    def task(self, msg):
        if msg.status_scope == TaskStatus.SCOPE_EXECUTION:
            self.active_task = msg.state not in (
                TaskStatus.STATE_IDLE, TaskStatus.STATE_SUCCEEDED, TaskStatus.STATE_FAILED,
                TaskStatus.STATE_CANCELED, TaskStatus.STATE_EMERGENCY_STOPPED,
            )

    def call(self, client, request, timeout=2):
        deadline = time.monotonic()+timeout
        while not client.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not client.service_is_ready():
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError(f'service timed out: {client.srv_name}')
        return future.result()

    def position(self):
        result = self.call(self.entity, GetEntityState.Request(name='cleannav_ackermann'))
        if not result.success:
            raise RuntimeError('Gazebo cleannav_ackermann ground truth missing')
        p = result.state.pose.position
        return p.x, p.y


def relinquish_motion(node, lease_possible, acquire_uncertain):
    """Latch ESTOP on an unknown acquire result, including a late server grant."""
    if not lease_possible:
        return True
    if acquire_uncertain:
        node.estop.publish(Bool(data=True))
    node.pub.publish(Twist())
    try:
        response = node.call(node.release, SafetyLease.Request(execution_id=node.owner))
        if not response.success:
            raise RuntimeError(response.message)
    except Exception:
        node.estop.publish(Bool(data=True))
        return False
    return not acquire_uncertain


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--enable-test', action='store_true', required=True)
    parser.add_argument('--vx', type=float, default=0.1)
    parser.add_argument('--wz', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=3)
    parser.add_argument('--max-distance', type=float, default=0.5)
    parsed, ros_args = parser.parse_known_args(args)
    validate_environment()
    if not (0 < parsed.vx <= 0.15 and abs(parsed.wz) <= parsed.vx / 1.12
            and 0 < parsed.duration <= 5 and 0 < parsed.max_distance <= 0.75):
        parser.error('requires vx in (0,.15], curvature <=1/1.12, duration<=5, distance<=.75')
    rclpy.init(args=ros_args)
    node = ChassisTest()
    lease_possible = False
    acquire_uncertain = False
    code = 1
    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and not (node.ready and node.status):
            rclpy.spin_once(node, timeout_sec=0.1)
        if (not node.ready or not node.status or node.status.block_all
                or node.status.emergency_stop_active or node.status.lease_owner_execution_id
                or node.active_task):
            raise RuntimeError('requires READY, authorized idle Safety and no mission')
        start = node.position()
        # A service timeout does not prove that the server did not grant a lease.
        lease_possible = acquire_uncertain = True
        result = node.call(node.acquire, SafetyLease.Request(execution_id=node.owner))
        acquire_uncertain = False
        if not result.success:
            lease_possible = False
            raise RuntimeError('lease refused: ' + result.message)
        deadline = time.monotonic()+parsed.duration
        next_pose = time.monotonic()
        distance = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.active_task or time.monotonic()-node.last_status > 1.5:
                raise RuntimeError('mission became active or SafetyStatus stale')
            if node.status.emergency_stop_active or node.status.block_all:
                raise RuntimeError('Safety gate closed')
            if time.monotonic() >= next_pose:
                # Zero before a potentially blocking service, then resume only on success.
                node.pub.publish(Twist())
                point = node.position()
                distance = math.dist(start, point)
                if distance >= parsed.max_distance:
                    break
                next_pose = time.monotonic()+0.25
            command = Twist()
            command.linear.x, command.angular.z = parsed.vx, parsed.wz
            node.pub.publish(command)
        print(f'[CHASSIS TEST] completed; last_checked_displacement={distance:.4f}', flush=True)
        code = 0
    except (Exception, KeyboardInterrupt) as error:
        print('[CHASSIS TEST] stopped: ' + str(error), flush=True)
    finally:
        if not relinquish_motion(node, lease_possible, acquire_uncertain):
            code = 1
        deadline = time.monotonic()+1
        while rclpy.ok() and time.monotonic() < deadline:
            # The release service invalidates its candidate. Do not overwrite
            # candidates from a new mission after relinquishing ownership.
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
