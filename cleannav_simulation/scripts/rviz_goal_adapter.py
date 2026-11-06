#!/usr/bin/env python3
"""RViz input only: submit a managed pose service, never navigation or velocity."""

from collections import deque
import json
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from cleannav_interfaces.msg import TaskStatus
from cleannav_interfaces.srv import SubmitNavigationGoal
from std_msgs.msg import String

TOPIC = '/cleannav/rviz/goal_pose'
SERVICE = '/cleannav/mission/submit_navigation_goal'


class RvizGoalAdapter(Node):
    """Bounded, asynchronous submission with no retries after a send."""

    def __init__(self):
        super().__init__('cleannav_rviz_goal_adapter')
        self.client = self.create_client(SubmitNavigationGoal, SERVICE)
        self.trace = self.create_publisher(String, '/cleannav/simulation/rviz_goal_events', 10)
        self.seen = deque(maxlen=128)
        self.known = deque(maxlen=256)
        self.pending = {}
        self.service_seen = False
        self.status_seen = {}
        self.create_subscription(PoseStamped, TOPIC, self.goal, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, '/cleannav/task_status', self.status, qos)
        self.create_timer(.1, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'[RVIZ_GOAL] input={TOPIC} service={SERVICE}; no automatic goal')

    def report(self, event, command_id, **data):
        """Expose click/response/status correlation to the main terminal and recorder."""
        payload = dict(event=event, command_id=command_id, wall_time=time.time(), **data)
        text = json.dumps(payload, allow_nan=True)
        self.get_logger().info('[RVIZ_GOAL] '+text)
        print('[RVIZ_GOAL] '+text, flush=True)
        self.trace.publish(String(data=text))

    def goal(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        key = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec,
               p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if key in self.seen:
            return
        self.seen.append(key)
        command_id = 'rviz-'+uuid.uuid4().hex
        self.known.append(command_id)
        self.status_seen = {k: v for k, v in self.status_seen.items() if k in self.known}
        self.report('CLICK', command_id, frame=msg.header.frame_id,
                    stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
                    position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])
        if len(self.pending) >= 32:
            self.report('REJECTED', command_id, reason='adapter pending limit')
            return
        request = SubmitNavigationGoal.Request()
        request.command_id, request.source = command_id, request.SOURCE_RVIZ
        request.goal, request.valid_for.sec = msg, 5
        request.busy_policy = request.REJECT_IF_BUSY
        self.pending[command_id] = dict(request=request, deadline=time.monotonic()+3,
                                        future=None, warned=False)

    def poll(self):
        if not self.service_seen and self.client.service_is_ready():
            self.service_seen = True
            self.report('SERVICE_READY', '', service=SERVICE)
        for command_id, entry in list(self.pending.items()):
            future = entry['future']
            if future is None and time.monotonic() < entry['deadline']:
                if self.client.service_is_ready():
                    entry['future'] = self.client.call_async(entry['request'])
                continue
            if future is not None and future.done():
                try:
                    result = future.result()
                    self.report('ACCEPTED' if result.accepted else 'REJECTED', command_id,
                                task_id=result.task_id, execution_id=result.execution_id,
                                reason=result.reason, reason_code=result.reason_code,
                                late=entry['warned'])
                except Exception as exc:
                    self.report('SERVICE_ERROR', command_id, reason=str(exc))
                del self.pending[command_id]
            elif time.monotonic() >= entry['deadline'] and not entry['warned']:
                entry['warned'] = True
                self.report('SERVICE_TIMEOUT' if future else 'SERVICE_UNAVAILABLE', command_id,
                            reason='3 wall seconds; sent requests are not retried',
                            acceptance_uncertain=future is not None)
                if future is None:
                    del self.pending[command_id]
            elif entry['warned'] and time.monotonic() > entry['deadline']+30:
                # The request has expired. TaskStatus still correlates any late execution.
                if future is not None:
                    self.client.remove_pending_request(future)
                del self.pending[command_id]

    def status(self, msg):
        if msg.command_id not in self.known:
            return
        key = (msg.state, msg.execution_id, msg.reason_code)
        if self.status_seen.get(msg.command_id) == key:
            return
        self.status_seen[msg.command_id] = key
        self.report('TASK_STATUS', msg.command_id, task_id=msg.task_id,
                    execution_id=msg.execution_id, state=msg.state,
                    reason_code=msg.reason_code, message=msg.message)


def main(args=None):
    """Run the input adapter with normal ROS shutdown."""
    rclpy.init(args=args)
    node = RvizGoalAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
