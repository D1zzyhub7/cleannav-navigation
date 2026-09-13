#!/usr/bin/env python3
"""Smac Hybrid-A* bridge：周期重规划同一最终目标并屏蔽过期结果。"""

import copy
import hashlib
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
_REPLAN_FAILURE_PREFIX = 'replan fail (Smac Hybrid):'


def _path_signature(path_msg: Path) -> str:
    """与 Path Executor 一致的确定性路径身份（含最终朝向）。"""
    if not path_msg.poses:
        return ''
    first = path_msg.poses[0].pose.position
    last = path_msg.poses[-1].pose.position
    mid = path_msg.poses[len(path_msg.poses) // 2].pose.position
    last_orientation = path_msg.poses[-1].pose.orientation
    raw = (
        f'{path_msg.header.frame_id}|{len(path_msg.poses)}|'
        f'{first.x:.6f},{first.y:.6f}|'
        f'{last.x:.6f},{last.y:.6f}|'
        f'{mid.x:.6f},{mid.y:.6f}|'
        f'{last_orientation.x:.6f},{last_orientation.y:.6f},'
        f'{last_orientation.z:.6f},{last_orientation.w:.6f}'
    )
    return hashlib.sha256(raw.encode()).hexdigest()


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
    """将最终目标周期提交给 ComputePathToPose，并发布 Smac 原始路径。"""

    def __init__(self):
        super().__init__('cleannav_hybrid_planner_bridge')

        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('compute_path_action', '/compute_path_to_pose')
        self.declare_parameter('replan_period_sec', 1.0)

        self._planner_id = self.get_parameter('planner_id').value
        self._map_frame = self.get_parameter('map_frame').value
        self._action_name = self.get_parameter('compute_path_action').value
        self._replan_period_sec = float(
            self.get_parameter('replan_period_sec').value)

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, reliable_qos)
        self._executor_status_sub = self.create_subscription(
            String,
            '/cleannav/path_executor_status',
            self._executor_status_callback,
            reliable_qos,
        )
        self._path_pub = self.create_publisher(
            Path, '/cleannav/global_path', reliable_qos)
        self._status_pub = self.create_publisher(
            String, '/cleannav/global_planner_status', reliable_qos)
        self._action_client = ActionClient(
            self, ComputePathToPose, self._action_name)

        # generation 仅表示用户最终目标代际；周期 replan 不递增它。
        self._generation = 0
        self._final_goal = None
        self._has_successful_path = False
        self._latest_path_signature = ''
        self._completion_window_signature = ''

        # pending 最多保存一次最新请求；deferred 合并多个 timer tick。
        self._pending_goal = None
        self._replan_deferred = False

        self._request_sequence = 0
        self._send_future = None
        self._sending_request_id = None
        self._active_goal_handle = None
        self._active_generation = None
        self._active_request_id = None
        self._active_cancel_requested = False
        self._last_status = ''

        # server retry 只负责送出已经排队的请求。
        self._server_retry_timer = self.create_timer(
            0.25, self._try_send_pending_goal)
        self._replan_timer = None
        if self._replan_period_sec > 0.0:
            self._replan_timer = self.create_timer(
                self._replan_period_sec, self._replan_timer_callback)

    def _publish_status(self, text):
        if text == self._last_status:
            return
        self._last_status = text
        self._status_pub.publish(String(data=text))

    def _publish_failure(self, detail, generation):
        if generation != self._generation or self._final_goal is None:
            return
        prefix = (
            _REPLAN_FAILURE_PREFIX
            if self._has_successful_path else _FAILURE_PREFIX
        )
        self._publish_status(f'{prefix} {detail}')

    def _goal_callback(self, goal_msg):
        self._generation += 1
        generation = self._generation
        self._final_goal = None
        self._has_successful_path = False
        self._latest_path_signature = ''
        self._completion_window_signature = ''
        self._pending_goal = None
        self._replan_deferred = False

        validation_error = _validate_goal(goal_msg, self._map_frame)
        if validation_error is not None:
            self._publish_status(validation_error)
            self._request_cancel_active_goal()
            return

        # 保存最终目标；周期请求始终复用它，且不改变 generation。
        self._final_goal = copy.deepcopy(goal_msg)
        self._pending_goal = (generation, self._final_goal)
        self._last_status = ''
        self._request_cancel_active_goal()
        self._try_send_pending_goal()

    def _executor_status_callback(self, msg):
        text = msg.data
        if text.startswith('follow_path_near_goal:'):
            signature = text.partition(':')[2]
            if signature == self._latest_path_signature:
                self._completion_window_signature = signature
            return

        if text.startswith('follow_path_aborted:'):
            signature = text.partition(':')[2].split(' ', 1)[0]
            if signature != self._latest_path_signature:
                return
            self._completion_window_signature = ''
            self._schedule_replan()
            return

        if not text.startswith('follow_path_succeeded:'):
            return

        signature = text.partition(':')[2]
        # Executor terminal status must identify the latest published path.
        # This blocks an old final-goal success from clearing a newer goal.
        if signature != self._latest_path_signature:
            return

        # 只有正常执行成功才结束最终目标生命周期。
        self._final_goal = None
        self._has_successful_path = False
        self._latest_path_signature = ''
        self._completion_window_signature = ''
        self._pending_goal = None
        self._replan_deferred = False
        self._request_cancel_active_goal()

    def _replan_timer_callback(self):
        if self._replan_period_sec <= 0.0 or self._final_goal is None:
            return

        if (
                self._completion_window_signature
                and self._completion_window_signature == self._latest_path_signature):
            return

        self._schedule_replan()

    def _schedule_replan(self):
        if self._replan_period_sec <= 0.0 or self._final_goal is None:
            return

        if (
                self._send_future is not None
                or self._active_goal_handle is not None
                or self._pending_goal is not None):
            self._replan_deferred = True
            return

        self._pending_goal = (self._generation, self._final_goal)
        self._try_send_pending_goal()

    def _request_cancel_active_goal(self):
        goal_handle = self._active_goal_handle
        if goal_handle is None or self._active_cancel_requested:
            return
        self._active_cancel_requested = True
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - best-effort cancel
            self._active_cancel_requested = False
            self.get_logger().warning(
                f'cancel stale ComputePath goal failed: {exc}')

    def _try_send_pending_goal(self):
        if (
                self._pending_goal is None
                or self._send_future is not None
                or self._active_goal_handle is not None):
            return

        generation, goal_msg = self._pending_goal
        if generation != self._generation or self._final_goal is None:
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
        self._request_sequence += 1
        request_id = self._request_sequence
        try:
            future = self._action_client.send_goal_async(request)
        except Exception as exc:
            self._publish_failure(f'exception {exc}', generation)
            return

        self._send_future = future
        self._sending_request_id = request_id
        future.add_done_callback(
            lambda completed,
            request_generation=generation,
            sent_request_id=request_id:
            self._goal_response_callback(
                completed, request_generation, sent_request_id))

    def _goal_response_callback(self, future, generation, request_id):
        if (
                future is not self._send_future
                or request_id != self._sending_request_id):
            self._cancel_stale_response(future)
            return

        self._send_future = None
        self._sending_request_id = None

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_failure(f'exception {exc}', generation)
            self._advance_request_queue()
            return

        if goal_handle is None or not goal_handle.accepted:
            self._publish_failure('goal rejected', generation)
            self._advance_request_queue()
            return

        self._active_goal_handle = goal_handle
        self._active_generation = generation
        self._active_request_id = request_id
        self._active_cancel_requested = False
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed,
                request_generation=generation,
                active_request_id=request_id,
                active_handle=goal_handle:
                self._result_callback(
                    completed,
                    request_generation,
                    active_request_id,
                    active_handle,
                ))
        except Exception as exc:
            # 无法观察 terminal 时不能假装 action 已结束或发送并发请求。
            self.get_logger().error(f'get_result_async exception: {exc}')
            self._publish_failure(f'exception {exc}', generation)
            self._request_cancel_active_goal()
            return

        if generation != self._generation or self._final_goal is None:
            self._request_cancel_active_goal()

    def _cancel_stale_response(self, future):
        try:
            goal_handle = future.result()
        except Exception:
            return
        if goal_handle is None or not goal_handle.accepted:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - best-effort cancel
            self.get_logger().warning(
                f'cancel stale ComputePath goal failed: {exc}')

    def _result_callback(self, future, generation, request_id, goal_handle):
        if (
                request_id != self._active_request_id
                or goal_handle is not self._active_goal_handle):
            return

        self._active_goal_handle = None
        self._active_generation = None
        self._active_request_id = None
        self._active_cancel_requested = False

        if generation != self._generation or self._final_goal is None:
            self._advance_request_queue()
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._publish_failure(f'exception {exc}', generation)
            self._advance_request_queue()
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_failure(
                f'result status={wrapped_result.status}', generation)
            self._advance_request_queue()
            return

        path = wrapped_result.result.path
        if not path.poses:
            self._publish_failure('empty path', generation)
            self._advance_request_queue()
            return

        # Smac 已提供 SE2 orientation，保持 Path 内容原样。
        self._has_successful_path = True
        self._latest_path_signature = _path_signature(path)
        self._completion_window_signature = ''
        self._path_pub.publish(path)
        self._publish_status(f'path: {len(path.poses)} poses')
        self._advance_request_queue()

    def _advance_request_queue(self):
        if self._pending_goal is not None:
            self._try_send_pending_goal()
            return
        if self._replan_deferred and self._final_goal is not None:
            self._replan_deferred = False
            self._pending_goal = (self._generation, self._final_goal)
            self._try_send_pending_goal()


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
