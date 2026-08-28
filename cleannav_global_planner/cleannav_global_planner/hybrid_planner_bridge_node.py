#!/usr/bin/env python3
"""Smac Hybrid-A* bridge：按目标请求路径并屏蔽过期 Action 结果。"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


_FAILURE_PREFIX = 'A* fail (Smac Hybrid):'


def _validate_goal(goal, map_frame):
    """返回兼容旧合同的校验错误；目标有效时返回 None。"""
    if goal.header.frame_id != map_frame:
        return f'goal frame [{goal.header.frame_id}] != [{map_frame}]'

    position = goal.pose.position
    if not (math.isfinite(position.x) and math.isfinite(position.y)):
        return 'goal position non-finite'

    orientation = goal.pose.orientation
    if not all(math.isfinite(value) for value in (
            orientation.x, orientation.y, orientation.z, orientation.w)):
        return 'goal quaternion non-finite'

    return None


class HybridPlannerBridgeNode(Node):
    """将 /goal_pose 转为 ComputePathToPose，并发布 Smac 原始路径。"""

    def __init__(self):
        super().__init__('cleannav_hybrid_planner_bridge')

        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('compute_path_action', '/compute_path_to_pose')

        self._planner_id = self.get_parameter('planner_id').value
        self._map_frame = self.get_parameter('map_frame').value
        self._action_name = self.get_parameter('compute_path_action').value

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, reliable_qos)
        self._path_pub = self.create_publisher(
            Path, '/cleannav/global_path', reliable_qos)
        self._status_pub = self.create_publisher(
            String, '/cleannav/global_planner_status', reliable_qos)
        self._action_client = ActionClient(
            self, ComputePathToPose, self._action_name)

        self._generation = 0
        self._pending_goal = None
        self._send_future = None
        self._active_goal_handle = None
        self._active_generation = None
        self._last_status = ''

        # 仅重试 action server availability；不会周期性重新规划。
        self._server_retry_timer = self.create_timer(
            0.25, self._try_send_pending_goal)

    def _publish_status(self, text):
        if text == self._last_status:
            return
        self._last_status = text
        self._status_pub.publish(String(data=text))

    def _publish_failure(self, detail):
        self._publish_status(f'{_FAILURE_PREFIX} {detail}')

    def _goal_callback(self, goal_msg):
        self._generation += 1
        generation = self._generation
        self._pending_goal = None
        self._cancel_active_goal()

        validation_error = _validate_goal(goal_msg, self._map_frame)
        if validation_error is not None:
            self._publish_status(validation_error)
            return

        # 新目标覆盖未发送的旧目标；旧 callback 受 generation gate 约束。
        self._pending_goal = (generation, goal_msg)
        self._last_status = ''
        self._try_send_pending_goal()

    def _cancel_active_goal(self):
        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        self._active_generation = None
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - 仅记录 best-effort cancel
            self.get_logger().warning(f'cancel stale ComputePath goal failed: {exc}')

    def _try_send_pending_goal(self):
        if self._pending_goal is None or self._send_future is not None:
            return

        generation, goal_msg = self._pending_goal
        if generation != self._generation:
            self._pending_goal = None
            return

        if not self._action_client.server_is_ready():
            self._publish_status(f'waiting for action server [{self._action_name}]')
            return

        request = ComputePathToPose.Goal()
        request.goal = goal_msg
        request.planner_id = self._planner_id
        request.use_start = False

        self._pending_goal = None
        try:
            future = self._action_client.send_goal_async(request)
        except Exception as exc:
            if generation == self._generation:
                self._publish_failure(f'exception {exc}')
            return

        self._send_future = future
        future.add_done_callback(
            lambda completed, request_generation=generation:
            self._goal_response_callback(completed, request_generation))

    def _goal_response_callback(self, future, generation):
        if future is self._send_future:
            self._send_future = None

        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation == self._generation:
                self._publish_failure(f'exception {exc}')
            self._try_send_pending_goal()
            return

        if generation != self._generation:
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:  # pragma: no cover - best-effort cancel
                    self.get_logger().warning(
                        f'cancel stale ComputePath goal failed: {exc}')
            self._try_send_pending_goal()
            return

        if not goal_handle.accepted:
            self._publish_failure('goal rejected')
            self._try_send_pending_goal()
            return

        self._active_goal_handle = goal_handle
        self._active_generation = generation
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, request_generation=generation:
                self._result_callback(completed, request_generation))
        except Exception as exc:
            self._active_goal_handle = None
            self._active_generation = None
            self._publish_failure(f'exception {exc}')

    def _result_callback(self, future, generation):
        if generation != self._generation:
            return

        if generation == self._active_generation:
            self._active_goal_handle = None
            self._active_generation = None

        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._publish_failure(f'exception {exc}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_failure(f'result status={wrapped_result.status}')
            return

        path = wrapped_result.result.path
        if not path.poses:
            self._publish_failure('empty path')
            return

        # Smac 已提供 SE2 orientation，保持 Path 内容原样。
        self._path_pub.publish(path)
        self._publish_status(f'path: {len(path.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = HybridPlannerBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
