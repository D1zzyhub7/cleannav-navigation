#!/usr/bin/env python3
"""CleanNav machine-to-machine navigation facade.

本模块只编排两个已有 Nav2 action：先规划，再执行路径。
外层使用 CleanNav 专属 action 名称，但复用 ROS Humble 的
``nav2_msgs/action/NavigateToPose`` 类型。
"""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future


FACADE_ACTION_NAME = '/cleannav/navigate_to_pose'
PLANNER_ACTION_NAME = '/compute_path_to_pose'
FOLLOW_ACTION_NAME = '/follow_path'

DEFAULT_PLANNER_ID = 'GridBased'
DEFAULT_USE_START = False
DEFAULT_CONTROLLER_ID = 'FollowPath'
DEFAULT_GOAL_CHECKER_ID = ''


class _Stage(Enum):
    PLANNING = 'PLANNING'
    FOLLOWING = 'FOLLOWING'


class _InnerOutcome(Enum):
    RESULT = 'RESULT'
    CANCELED = 'CANCELED'
    CANCEL_FAILED = 'CANCEL_FAILED'


class _WakeReason(Enum):
    RESULT = 'RESULT'
    CANCEL = 'CANCEL'


@dataclass
class _Session:
    """一条 outer goal 的异步生命周期状态。"""

    outer_goal_handle: Any
    stage: _Stage = _Stage.PLANNING
    planner_goal_handle: Any | None = None
    follow_goal_handle: Any | None = None
    cancel_requested: bool = False
    wake_future: Future | None = None
    terminal: bool = False


def _finite_pose(pose: PoseStamped) -> bool:
    """检查外层目标的基本 ROS 表示，不执行语义规划校验。"""
    values = (
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    if not pose.header.frame_id or not all(math.isfinite(value) for value in values):
        return False
    norm = math.hypot(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    return norm > 1e-9


def _path_is_valid(path: Path) -> bool:
    """按现有 Path Executor 的最小语义检查规划结果。"""
    if not path.header.frame_id or len(path.poses) < 2:
        return False

    for pose_stamped in path.poses:
        if pose_stamped.header.frame_id and (
            pose_stamped.header.frame_id != path.header.frame_id
        ):
            return False
        if not (
            math.isfinite(pose_stamped.pose.position.x)
            and math.isfinite(pose_stamped.pose.position.y)
        ):
            return False
    return True


def _prepare_controller_path(path: Path) -> Path:
    """复用现有 Path Executor 的 latest-TF 时间戳策略。"""
    controller_path = copy.deepcopy(path)
    controller_path.header.stamp.sec = 0
    controller_path.header.stamp.nanosec = 0
    for pose_stamped in controller_path.poses:
        pose_stamped.header.stamp.sec = 0
        pose_stamped.header.stamp.nanosec = 0
    return controller_path


def _goal_id_bytes(goal_handle: Any) -> bytes:
    """Return the ROS action goal UUID for cancel-response correlation."""
    goal_id = getattr(goal_handle, 'goal_id', None)
    uuid = getattr(goal_id, 'uuid', None)
    if uuid is None:
        return b''
    return bytes(uuid)


def _cancel_response_contains_goal(response: Any, goal_handle: Any) -> bool:
    """Require Humble CancelGoal success and the requested UUID in the reply."""
    if getattr(response, 'return_code', None) != CancelGoal.Response.ERROR_NONE:
        return False
    expected = _goal_id_bytes(goal_handle)
    if not expected:
        return False
    for goal_info in getattr(response, 'goals_canceling', ()):
        goal_id = getattr(goal_info, 'goal_id', None)
        if bytes(getattr(goal_id, 'uuid', ())) == expected:
            return True
    return False


class NavigationFacadeRuntime:
    """Testable asynchronous implementation behind the ROS ActionServer."""

    def __init__(
        self,
        planner_client: Any,
        follow_client: Any,
        *,
        planner_id: str = DEFAULT_PLANNER_ID,
        use_start: bool = DEFAULT_USE_START,
        controller_id: str = DEFAULT_CONTROLLER_ID,
        goal_checker_id: str = DEFAULT_GOAL_CHECKER_ID,
    ) -> None:
        self._planner_client = planner_client
        self._follow_client = follow_client
        self._planner_id = planner_id
        self._use_start = use_start
        self._controller_id = controller_id
        self._goal_checker_id = goal_checker_id

        self._lock = threading.Lock()
        self._busy = False
        self._reserved = False
        self._pending_cancel = False
        self._session: _Session | None = None

    @property
    def active_stage(self) -> _Stage | None:
        """Return the current inner stage for diagnostics and tests."""
        with self._lock:
            return self._session.stage if self._session is not None else None

    @property
    def busy(self) -> bool:
        """Return whether an outer goal slot is reserved or active."""
        with self._lock:
            return self._busy

    def accept_goal(self, goal_request: NavigateToPose.Goal) -> bool:
        """Reserve the only outer goal slot after basic representation checks."""
        if not _finite_pose(goal_request.pose):
            return False
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            self._reserved = True
            return True

    def request_cancel(self, outer_goal_handle: Any) -> bool:
        """Record cancellation without blocking the ROS executor."""
        with self._lock:
            session = self._session
            if session is None:
                if self._reserved:
                    # goal_callback 已接受但 execute_callback 尚未获得
                    # ServerGoalHandle；先记录取消，避免这个窄竞态丢失请求。
                    self._pending_cancel = True
                    return True
                return False
            if session.outer_goal_handle is not outer_goal_handle:
                return False
            if session.terminal:
                return False
            session.cancel_requested = True
            self._try_wake_locked(session, _WakeReason.CANCEL)
        return True

    async def execute(self, outer_goal_handle: Any) -> NavigateToPose.Result:
        """Run planner → path controller and finish the outer action once."""
        session = _Session(outer_goal_handle=outer_goal_handle)
        with self._lock:
            session.cancel_requested = self._pending_cancel
            self._pending_cancel = False
            self._session = session
            self._reserved = False

        try:
            planner_goal = ComputePathToPose.Goal()
            planner_goal.goal = outer_goal_handle.request.pose
            planner_goal.planner_id = self._planner_id
            planner_goal.use_start = self._use_start

            if not self._planner_client.server_is_ready():
                return self._finish(outer_goal_handle, 'abort')

            planner_handle = await self._send_goal(
                self._planner_client,
                planner_goal,
            )
            if planner_handle is None or not getattr(planner_handle, 'accepted', False):
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.planner_goal_handle = planner_handle

            planner_outcome, planner_wrapped = await self._wait_inner_result(
                session,
                planner_handle,
            )
            if planner_outcome is _InnerOutcome.CANCELED:
                return self._finish(outer_goal_handle, 'cancel')
            if planner_outcome is _InnerOutcome.CANCEL_FAILED:
                return self._finish(outer_goal_handle, 'abort')
            if planner_wrapped is None or (
                planner_wrapped.status != GoalStatus.STATUS_SUCCEEDED
            ):
                return self._finish(outer_goal_handle, 'abort')

            path = planner_wrapped.result.path
            if not _path_is_valid(path):
                return self._finish(outer_goal_handle, 'abort')

            # 取消发生在 planner 成功与 FollowPath submit 之间时，
            # planner 已经终止且不能再启动新的 inner goal。
            if self._is_cancel_requested(session):
                return self._finish(outer_goal_handle, 'cancel')

            follow_goal = FollowPath.Goal()
            follow_goal.path = _prepare_controller_path(path)
            follow_goal.controller_id = self._controller_id
            follow_goal.goal_checker_id = self._goal_checker_id

            if not self._follow_client.server_is_ready():
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.stage = _Stage.FOLLOWING
            follow_handle = await self._send_goal(
                self._follow_client,
                follow_goal,
            )
            if follow_handle is None or not getattr(follow_handle, 'accepted', False):
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.follow_goal_handle = follow_handle

            follow_outcome, follow_wrapped = await self._wait_inner_result(
                session,
                follow_handle,
            )
            if follow_outcome is _InnerOutcome.CANCELED:
                return self._finish(outer_goal_handle, 'cancel')
            if follow_outcome is _InnerOutcome.CANCEL_FAILED:
                return self._finish(outer_goal_handle, 'abort')
            if follow_wrapped is not None and (
                follow_wrapped.status == GoalStatus.STATUS_SUCCEEDED
            ):
                return self._finish(outer_goal_handle, 'succeed')
            return self._finish(outer_goal_handle, 'abort')
        except Exception:
            return self._finish(outer_goal_handle, 'abort')
        finally:
            with self._lock:
                session.terminal = True
                if self._session is session:
                    self._session = None
                    self._busy = False
                    self._reserved = False

    async def _send_goal(self, client: Any, goal: Any) -> Any:
        """Await one non-blocking ActionClient goal request."""
        try:
            return await client.send_goal_async(goal)
        except Exception:
            return None

    async def _wait_inner_result(
        self,
        session: _Session,
        inner_goal_handle: Any,
    ) -> tuple[_InnerOutcome, Any | None]:
        """Race inner result with outer cancellation without executor blocking."""
        try:
            inner_result_future = inner_goal_handle.get_result_async()
        except Exception:
            return _InnerOutcome.RESULT, None

        wake_future = Future()

        def _on_inner_result(_future: Any) -> None:
            self._try_wake(session, _WakeReason.RESULT)

        with self._lock:
            session.wake_future = wake_future
            cancel_requested = session.cancel_requested

        if cancel_requested:
            # 已经收到的 outer cancel 必须优先于尚未观察到的 inner result。
            self._try_wake(session, _WakeReason.CANCEL)
        else:
            inner_result_future.add_done_callback(_on_inner_result)

        try:
            wake_reason = await wake_future
        finally:
            with self._lock:
                if session.wake_future is wake_future:
                    session.wake_future = None

        if wake_reason is _WakeReason.CANCEL:
            # 若自然结果已经形成，不能因稍后的 cancel 请求伪造 CANCELED。
            if self._future_done(inner_result_future):
                try:
                    wrapped_result = inner_result_future.result()
                except Exception:
                    wrapped_result = None
                if (
                    wrapped_result is not None
                    and getattr(wrapped_result, 'status', None)
                    == GoalStatus.STATUS_SUCCEEDED
                ):
                    return _InnerOutcome.RESULT, wrapped_result
            if await self._cancel_and_confirm(inner_goal_handle):
                return _InnerOutcome.CANCELED, None
            return _InnerOutcome.CANCEL_FAILED, None

        try:
            wrapped_result = inner_result_future.result()
        except Exception:
            return _InnerOutcome.RESULT, None

        # 自然成功先于取消请求生效时，成功是唯一 terminal outcome。
        if (
            self._is_cancel_requested(session)
            and getattr(wrapped_result, 'status', None)
            == GoalStatus.STATUS_SUCCEEDED
        ):
            return _InnerOutcome.RESULT, wrapped_result
        if (
            self._is_cancel_requested(session)
            and getattr(wrapped_result, 'status', None)
            == GoalStatus.STATUS_CANCELED
        ):
            # result 与 cancel callback 同时完成时，仍须验证真实 cancel
            # response；不能仅凭 CANCELED 状态伪造确认。
            if await self._cancel_and_confirm(inner_goal_handle):
                return _InnerOutcome.CANCELED, None
            return _InnerOutcome.CANCEL_FAILED, None
        return _InnerOutcome.RESULT, wrapped_result

    @staticmethod
    def _future_done(future: Any) -> bool:
        """检查 Humble rclpy Future 的完成或取消状态。"""
        if future.done():
            return True
        cancelled = getattr(future, 'cancelled', None)
        return bool(cancelled is not None and cancelled())

    def _try_wake(self, session: _Session, reason: _WakeReason) -> bool:
        """在线程锁内以 first-writer-wins 唤醒当前 inner wait。"""
        with self._lock:
            return self._try_wake_locked(session, reason)

    def _try_wake_locked(self, session: _Session, reason: _WakeReason) -> bool:
        wake_future = session.wake_future
        if wake_future is None or self._future_done(wake_future):
            return False
        wake_future.set_result(reason)
        return True

    async def _cancel_and_confirm(self, inner_goal_handle: Any) -> bool:
        """Require accepted cancel response and a CANCELED terminal result."""
        try:
            response = await inner_goal_handle.cancel_goal_async()
        except Exception:
            return False
        if not _cancel_response_contains_goal(response, inner_goal_handle):
            return False

        try:
            wrapped_result = await inner_goal_handle.get_result_async()
        except Exception:
            return False
        return (
            getattr(wrapped_result, 'status', None)
            == GoalStatus.STATUS_CANCELED
        )

    def _is_cancel_requested(self, session: _Session) -> bool:
        with self._lock:
            return session.cancel_requested

    def _finish(
        self,
        outer_goal_handle: Any,
        terminal: str,
    ) -> NavigateToPose.Result:
        """Set exactly one outer terminal state and return an empty result."""
        with self._lock:
            session = self._session
            if session is None or session.terminal:
                return NavigateToPose.Result()
            session.terminal = True

        if terminal == 'succeed':
            outer_goal_handle.succeed()
        elif terminal == 'cancel':
            outer_goal_handle.canceled()
        else:
            outer_goal_handle.abort()
        return NavigateToPose.Result()


class NavigationFacadeNode(Node):
    """Expose the CleanNav outer action and own inner action clients."""

    def __init__(self) -> None:
        super().__init__('cleannav_navigation_facade')
        self.declare_parameter('planner_id', DEFAULT_PLANNER_ID)
        self.declare_parameter('use_start', DEFAULT_USE_START)
        self.declare_parameter('controller_id', DEFAULT_CONTROLLER_ID)
        self.declare_parameter('goal_checker_id', DEFAULT_GOAL_CHECKER_ID)

        self._planner_client = ActionClient(
            self,
            ComputePathToPose,
            PLANNER_ACTION_NAME,
        )
        self._follow_client = ActionClient(
            self,
            FollowPath,
            FOLLOW_ACTION_NAME,
        )
        self._runtime = NavigationFacadeRuntime(
            self._planner_client,
            self._follow_client,
            planner_id=self.get_parameter('planner_id').value,
            use_start=bool(self.get_parameter('use_start').value),
            controller_id=self.get_parameter('controller_id').value,
            goal_checker_id=self.get_parameter('goal_checker_id').value,
        )
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            FACADE_ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

    def _goal_callback(self, goal_request: NavigateToPose.Goal) -> GoalResponse:
        """Accept only a valid request when the single slot is free."""
        return (
            GoalResponse.ACCEPT
            if self._runtime.accept_goal(goal_request)
            else GoalResponse.REJECT
        )

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        """Accept cancellation only for the active outer goal."""
        return (
            CancelResponse.ACCEPT
            if self._runtime.request_cancel(goal_handle)
            else CancelResponse.REJECT
        )

    async def _execute_callback(
        self,
        goal_handle: Any,
    ) -> NavigateToPose.Result:
        """Delegate execution to the dependency-injected runtime."""
        return await self._runtime.execute(goal_handle)

    def destroy_node(self) -> None:
        """Destroy the action server before the ROS node."""
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """Run the facade node with the normal ROS executor."""
    rclpy.init(args=args)
    node = NavigationFacadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
