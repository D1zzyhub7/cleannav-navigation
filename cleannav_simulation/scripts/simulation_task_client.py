#!/usr/bin/env python3
"""Explicit simulation LEAF_PILE observation + existing Task 31 command."""

import json
import math
import time
import uuid

import rclpy
from cleannav_interfaces.msg import CleaningTarget, CleaningTargetArray, TaskCommand, TaskStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class SimulationTaskClient(Node):
    """Publish existing public contracts; only real Action results complete a task."""

    def __init__(self):
        super().__init__('cleannav_simulation_task_client')
        self.x = self.declare_parameter('goal_x', 0.8).value
        self.y = self.declare_parameter('goal_y', 0.0).value
        self.timeout = self.declare_parameter('execution_timeout', 120.0).value
        self.control_task_id = self.declare_parameter('control_task_id', 0).value
        if self.control_task_id not in (0, 4, 6):
            raise ValueError('control_task_id supports 0 (navigate), 4 (STOP), 6 (ESTOP)')
        if not all(math.isfinite(value) for value in (self.x, self.y, self.timeout)):
            raise ValueError('goal and timeout must be finite')
        if self.timeout <= 0 or not self.get_parameter('use_sim_time').value:
            raise ValueError('simulation client requires use_sim_time=true and positive timeout')
        self.command_id = 'simulation-' + uuid.uuid4().hex
        self.ready = False
        self.ready_at = -math.inf
        self.terminal = None
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/cleannav/simulation/readiness', self._ready, qos)
        self.create_subscription(TaskStatus, '/cleannav/task_status', self._status, qos)
        self.command_pub = self.create_publisher(TaskCommand, '/cleannav/hmi/task_command', 10)
        self.target_pub = self.create_publisher(
            CleaningTargetArray, '/cleannav/simulation/cleaning_targets', 10)

    def _ready(self, msg):
        data = json.loads(msg.data)
        self.ready = data.get('ready') is True and data.get('motion_enabled') is True
        self.ready_at = time.monotonic()

    def _status(self, msg):
        if msg.command_id == self.command_id and msg.state in (
            TaskStatus.STATE_SUCCEEDED, TaskStatus.STATE_FAILED,
            TaskStatus.STATE_CANCELED, TaskStatus.STATE_REJECTED,
            TaskStatus.STATE_EMERGENCY_STOPPED,
        ):
            self.terminal = msg
            print(f'[MISSION] terminal state={msg.state} reason={msg.reason_code} '
                  f'command={msg.command_id} {msg.message}', flush=True)

    def command(self, task_id, command_id):
        msg = TaskCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_version = '1.0'
        msg.command_id = command_id
        msg.source = TaskCommand.SOURCE_MOCK
        msg.task_id = task_id
        msg.confidence = 1.0
        msg.valid_for.sec = 10
        msg.raw_text = 'explicit Gazebo simulation command'
        return msg

    def target(self):
        batch = CleaningTargetArray()
        batch.header.frame_id = 'map'
        batch.header.stamp = self.get_clock().now().to_msg()
        batch.interface_version = '1.0'
        batch.source = 'simulation_manual_target'
        target = CleaningTarget()
        target.header = batch.header
        target.interface_version = '1.0'
        target.source = batch.source
        target.target_id = self.command_id + '-leaf-pile'
        target.target_type = CleaningTarget.TYPE_LEAF_PILE
        target.confidence = 1.0
        target.projection_valid = True
        target.centroid.x, target.centroid.y = self.x, self.y
        target.observation_state = CleaningTarget.OBSERVATION_CONFIRMED
        target.valid_for.sec = 30
        batch.targets = [target]
        return batch


def main(args=None):
    rclpy.init(args=args)
    node = SimulationTaskClient()
    deadline = time.monotonic() + 20.0
    sent = False
    samples = 0
    next_target = 0.0
    code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if not sent:
                if node.control_task_id:
                    if (node.command_pub.get_subscription_count() > 0
                            and node.get_clock().now().nanoseconds > 0):
                        node.command_pub.publish(node.command(
                            node.control_task_id, node.command_id))
                        sent = True
                        deadline = now + 10.0
                    continue
                if not (node.ready and now - node.ready_at < 2.5
                        and node.target_pub.get_subscription_count() > 0
                        and node.command_pub.get_subscription_count() > 0
                        and node.get_clock().now().nanoseconds > 0):
                    continue
                if now >= next_target:
                    node.target_pub.publish(node.target())
                    samples += 1
                    next_target = now + 0.2
                if samples >= 5:
                    node.command_pub.publish(node.command(31, node.command_id))
                    sent = True
                    deadline = now + node.timeout
                    print(f'[MISSION] submitted Task 31 LEAF_PILE at map=({node.x}, {node.y}); '
                          f'command={node.command_id}', flush=True)
            if node.terminal is not None:
                code = 0 if node.terminal.state == TaskStatus.STATE_SUCCEEDED else 1
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if sent and node.terminal is None and rclpy.ok():
            # A bounded client must never abandon an active task silently.
            node.command_pub.publish(node.command(6, node.command_id + '-timeout-estop'))
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
        if node.terminal is None:
            print('[ERROR] task client timed out or interrupted; no success inferred', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
