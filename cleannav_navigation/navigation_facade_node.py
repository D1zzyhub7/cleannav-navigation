#!/usr/bin/env python3
"""CleanNav machine-to-machine navigation facade.

本模块只编排两个已有 Nav2 action：先规划，再执行路径。
外层使用 CleanNav 专属 action 名称，但复用 ROS Humble 的
``nav2_msgs/action/NavigateToPose`` 类型。
"""

from __future__ import annotations

import copy
from collections import deque
import json
import math
import threading
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from typing import Any

from cleannav_global_planner.path_identity import path_signature
from cleannav_global_planner.path_safety import Grid, validate_path

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.srv import GetCostmap
from cleannav_interfaces.action import NavigateWithinPolygon
from boundary_runtime import PolygonNavigationMixin, CONSTRAINED_ACTION_NAME
from patrol_path_segments import closed_loop_splits
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Path, Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String


FACADE_ACTION_NAME = '/cleannav/navigate_to_pose'
PLANNER_ACTION_NAME = '/compute_path_to_pose'
FOLLOW_ACTION_NAME = '/follow_path'
PATROL_FOLLOW_ACTION_NAME = '/cleannav/follow_path'

def _outer_action(request):
    return NavigateWithinPolygon if isinstance(request,NavigateWithinPolygon.Goal) else NavigateToPose


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
    REPLAN = 'REPLAN'


@dataclass
class _Session:
    """一条 outer goal 的异步生命周期状态。"""

    outer_goal_handle: Any
    original_goal: PoseStamped | None = None
    stage: _Stage = _Stage.PLANNING
    planner_goal_handle: Any | None = None
    follow_goal_handle: Any | None = None
    cancel_requested: bool = False
    wake_future: Future | None = None
    terminal: bool = False
    inner_goal_handle: Any | None = None
    inner_result_future: Any | None = None
    inner_terminal: bool = True
    inner_actions: dict[bytes, bool] | None = None
    action_types: dict[bytes, str] | None = None
    handles: dict[bytes, Any] | None = None
    cancel_confirmed: set[bytes] | None = None
    result_futures: dict[bytes, Any] | None = None
    dispatches_pending: int = 0
    submission_unverified: bool = False
    replan_pending: bool = False
    replan_in_flight: bool = False
    distance_to_goal: float | None = None
    current_path_signature: str | None = None
    plan_revision: int = 0
    follow_revision: int = 0
    active_goal_checker_id: str = ''
    replan_failures: int = 0
    started_monotonic: float = 0.0


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
    """与Planner/Executor一致，拒绝所有非有限完整几何。"""
    try:
        return bool(path_signature(path))
    except ValueError:
        return False


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


def stationary_goal_proof(goal, transform, odometry, now_ns, *, odom_child_offset_m=None):
    """Conservative no-motion equality; not a replacement goal checker.

    Source stamps (not receipt times), full finite data and a stationary odom
    sample are required. A small TF lead tolerates asynchronous sim delivery.
    Invalid evidence returns None and preserves normal navigation behavior.
    """
    if odometry is None or now_ns <= 0 or not _finite_pose(goal):
        return None
    if transform.header.frame_id != goal.header.frame_id or transform.child_frame_id != 'base_link':
        return None
    stamps = [m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in (transform, odometry)]
    if min(stamps) <= 0 or not odometry.child_frame_id:
        return None
    offset = 0. if odometry.child_frame_id == 'base_link' else odom_child_offset_m
    if offset is None or not math.isfinite(offset) or offset < 0:
        return None
    ages = [(now_ns-stamp)/1e9 for stamp in stamps]
    if not all(-.1 <= age <= .3 for age in ages):
        return None
    t = transform.transform.translation
    q = transform.transform.rotation
    v = odometry.twist.twist
    values = [t.x,t.y,t.z,q.x,q.y,q.z,q.w,
              v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
    if not all(math.isfinite(x) for x in values):
        return None
    angular_speed = math.hypot(v.angular.x,v.angular.y,v.angular.z)
    # ROS odometry is at the rear axle in this vehicle. Rotation preserves
    # speed norms; the rigid-body offset contributes at most |w| * |r|.
    if math.hypot(v.linear.x,v.linear.y,v.linear.z) + angular_speed*offset > .01 or angular_speed > .01:
        return None
    def yaw(rotation):
        norm = math.hypot(*(getattr(rotation, k) for k in ('x','y','z','w')))
        if not math.isfinite(norm) or not .95 <= norm <= 1.05:
            raise ValueError('invalid quaternion')
        x,y,z,w = [getattr(rotation,k)/norm for k in ('x','y','z','w')]
        if abs(x)+abs(y) > .02:
            raise ValueError('nonplanar orientation')
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    try:
        error_yaw = abs(math.remainder(yaw(goal.pose.orientation)-yaw(q), 2*math.pi))
    except ValueError:
        return None
    error_xy = math.hypot(goal.pose.position.x-t.x,goal.pose.position.y-t.y)
    if error_xy > .02 or abs(goal.pose.position.z-t.z) > .1 or error_yaw > .02:
        return None
    return dict(position_error_m=error_xy,yaw_error_rad=error_yaw,
                tf_age_seconds=ages[0],odom_age_seconds=ages[1],inner_goals_sent=0)


def goal_distance_from_transform(goal, transform, now_ns, *, max_age_s=0.5,
                                 max_future_s=0.3):
    """Return planar goal distance only from a recent transform in its frame.

    FollowPath feedback is useful telemetry, but the installed controller has
    been observed reporting zero while the robot is more than a metre from the
    goal. Replan suppression therefore needs an independent, timestamp-checked
    pose source. Invalid or stale TF returns None, which permits a replan and
    never grants navigation success or motion authorization.
    """
    if not _finite_pose(goal) or now_ns <= 0:
        return None
    if (transform.header.frame_id != goal.header.frame_id or
            transform.child_frame_id != 'base_link'):
        return None
    stamp_ns = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
    if stamp_ns <= 0:
        return None
    age_s = (now_ns - stamp_ns) / 1e9
    if age_s > max_age_s or age_s < -max_future_s:
        return None
    x = transform.transform.translation.x
    y = transform.transform.translation.y
    gx = goal.pose.position.x
    gy = goal.pose.position.y
    if not all(math.isfinite(v) for v in (x, y, gx, gy)):
        return None
    return math.hypot(gx - x, gy - y)


def waypoint_stop_proof(samples, *, now_ros_ns, now_monotonic,
                        terminal_monotonic, terminal_ros_ns,
                        zero_since, last_command_received,
                        localization, localization_received,
                        command_zero_s=2.0, motion_span_s=1.5,
                        max_sample_gap_s=0.5):
    """Apply the Supervisor's zero-command and settled-motion bounds at a waypoint.

    Samples are (ROS source stamp ns, monotonic receipt, linear speed m/s,
    angular speed rad/s). This observes the final command and measured odometry;
    it never publishes or changes motion commands.
    """
    now_ros_ns = int(now_ros_ns)
    now_monotonic = float(now_monotonic)
    terminal_monotonic = float(terminal_monotonic)
    selected = []
    previous_stamp = None
    for sample in samples:
        try:
            stamp_ns, received, linear, angular = sample
            stamp_ns = int(stamp_ns)
            received = float(received)
            linear = float(linear)
            angular = float(angular)
        except (TypeError, ValueError):
            selected = []
            previous_stamp = None
            continue
        if (stamp_ns <= 0 or not all(math.isfinite(value) for value in
                                    (received, linear, angular)) or
                received < terminal_monotonic or stamp_ns < terminal_ros_ns):
            continue
        if previous_stamp is not None:
            gap = (stamp_ns - previous_stamp) / 1e9
            if gap <= 0 or gap > max_sample_gap_s:
                selected = []
        if linear > .02 or angular > .02:
            selected = []
            previous_stamp = stamp_ns
            continue
        selected.append((stamp_ns, received, linear, angular))
        previous_stamp = stamp_ns
    motion_span = ((selected[-1][0] - selected[0][0]) / 1e9
                   if len(selected) >= 2 else 0.0)
    last = selected[-1] if selected else None
    odom_fresh = bool(last and 0 <= (now_ros_ns-last[0])/1e9 <= .5
                       and 0 <= now_monotonic-last[1] <= .5)
    odom_continuous = bool(len(selected) >= 4 and motion_span >= motion_span_s)
    command_fresh = bool(last_command_received is not None and
                         0 <= now_monotonic-float(last_command_received) <= .3)
    zero_duration = (now_monotonic - max(float(zero_since), terminal_monotonic)
                     if zero_since is not None else 0.0)
    command_zero_stable = bool(command_fresh and zero_duration >= command_zero_s)
    localization_age = (now_monotonic-float(localization_received)
                        if localization_received is not None else None)
    localization_fresh = bool(
        isinstance(localization, dict) and localization.get('localized') is True
        and localization_age is not None and 0 <= localization_age <= 1.5)
    checks = dict(command_fresh=command_fresh,
                  final_command_zero_s=zero_duration,
                  final_command_zero_stable=command_zero_stable,
                  odometry_fresh=odom_fresh,
                  odometry_continuous=odom_continuous,
                  odometry_sample_count=len(selected),
                  odometry_span_s=motion_span,
                  odometry_max_linear_mps=max((x[2] for x in selected), default=None),
                  odometry_max_angular_rps=max((x[3] for x in selected), default=None),
                  localization_fresh=localization_fresh,
                  localization_age_s=localization_age)
    checks['ready'] = bool(command_zero_stable and odom_fresh and
                           odom_continuous and localization_fresh)
    return checks


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
        polygon_goal_checker_id: str = '',
        diagnostic=None,
        path_validator=None,
        stationary_goal_checker=None,
        session_initializer=None,
        goal_distance_provider=None,
        start_snapshot_provider=None,
        post_success_guard=None,
        replan_near_goal_tolerance: float = 0.25,
        max_replan_failures: int = 3,
    ) -> None:
        self._diagnostic = diagnostic
        self._path_validator = path_validator
        self._stationary_goal_checker = stationary_goal_checker
        self._session_initializer = session_initializer
        self._goal_distance_provider = goal_distance_provider
        self._start_snapshot_provider = start_snapshot_provider
        self._post_success_guard = post_success_guard
        self._planner_client = planner_client
        self._follow_client = follow_client
        self._planner_id = planner_id
        self._use_start = use_start
        self._controller_id = controller_id
        self._goal_checker_id = goal_checker_id
        self._polygon_goal_checker_id = polygon_goal_checker_id
        self._replan_near_goal_tolerance = max(0.0, float(replan_near_goal_tolerance))
        self._max_replan_failures = max(1, int(max_replan_failures))

        self._lock = threading.Lock()
        self._busy = False
        self._reserved = False
        self._pending_cancel = False
        self._session: _Session | None = None

    def _report(self, data):
        """Observability failures must never affect action ownership or motion."""
        try:
            self._diagnostic(data)
        except Exception:
            pass

    def _goal_checker_for(self, session: _Session) -> str:
        """Choose tight completion only for the region-constrained patrol action.

        Ordinary NavigateToPose keeps the configured Nav2 default checker. A
        polygon action may opt into a dedicated checker; an empty setting
        preserves the historical shared-checker behavior.
        """
        request = getattr(session.outer_goal_handle, 'request', None)
        if (self._polygon_goal_checker_id and
                isinstance(request, NavigateWithinPolygon.Goal)):
            return self._polygon_goal_checker_id
        return self._goal_checker_id

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
            self._report(dict(event='OUTER_CANCEL_REQUEST',outer_uuid=_goal_id_bytes(outer_goal_handle).hex()))
            self._try_wake_locked(session, _WakeReason.CANCEL)
        return True

    def request_replan(self) -> bool:
        """Coalesce a Legacy-style timer tick into the active Facade session."""
        with self._lock:
            session = self._session
        if session is not None and self._goal_distance_provider is not None:
            self._refresh_goal_distance(session)
        with self._lock:
            session = self._session
            if (
                session is None
                or session.terminal
                or session.cancel_requested
                or (session.stage is not _Stage.FOLLOWING and not session.replan_in_flight)
            ):
                return False
            if (
                session.distance_to_goal is not None
                and session.distance_to_goal <= self._replan_near_goal_tolerance
            ):
                self._report(dict(event='REPLAN_SUPPRESSED_NEAR_GOAL',
                                  distance_to_goal=session.distance_to_goal,
                                  distance_source='tf' if self._goal_distance_provider else 'follow_path_feedback',
                                  tolerance=self._replan_near_goal_tolerance))
                return False
            session.replan_pending = True
            if session.stage is _Stage.FOLLOWING and not session.replan_in_flight:
                self._try_wake_locked(session, _WakeReason.REPLAN)
        return True

    async def execute(self, outer_goal_handle: Any) -> NavigateToPose.Result:
        """Run planner → path controller and finish the outer action once."""
        session = _Session(
            outer_goal_handle=outer_goal_handle,
            original_goal=copy.deepcopy(outer_goal_handle.request.pose),
            inner_actions={},
            action_types={},
            handles={},
            cancel_confirmed=set(),
            result_futures={},
            started_monotonic=time.monotonic(),
        )
        with self._lock:
            session.cancel_requested = self._pending_cancel
            self._pending_cancel = False
            self._session = session
            self._reserved = False

        try:
            if self._session_initializer is not None:
                await self._session_initializer(outer_goal_handle)
            # A degenerate start==goal is not a drivable path. Only a fresh,
            # stationary localization proof may complete without inner actions.
            if self._stationary_goal_checker is not None:
                proof = self._stationary_goal_checker(session.original_goal)
                if proof is not None:
                    self._report(dict(event='GOAL_ALREADY_SATISFIED', **proof))
                    if self._is_cancel_requested(session):
                        return self._finish(outer_goal_handle, 'cancel')
                    return await self._complete_success(session, source='STATIONARY_GOAL_PROOF')
            if self._is_cancel_requested(session):
                return self._finish(outer_goal_handle, 'cancel')
            planner_goal = ComputePathToPose.Goal()
            # Retain the immutable destination for future STOP/WAIT/REPLAN
            # stages. Only this runtime submits the managed inner actions.
            planner_goal.goal = copy.deepcopy(session.original_goal)
            planner_goal.planner_id = self._planner_id
            planner_goal.use_start = self._use_start
            session.active_goal_checker_id = self._goal_checker_for(session)
            if (self._start_snapshot_provider is not None and
                    isinstance(outer_goal_handle.request, NavigateWithinPolygon.Goal)):
                actual_start, start_evidence = await self._start_snapshot_provider(
                    session.original_goal)
                if actual_start.header.frame_id != session.original_goal.header.frame_id:
                    raise ValueError('actual planning start frame differs from patrol goal frame')
                if not _finite_pose(actual_start):
                    raise ValueError('actual planning start pose is invalid')
                planner_goal.start = copy.deepcopy(actual_start)
                planner_goal.use_start = True
                self._report(dict(event='PLAN_START_CAPTURED',
                                  goal_checker_id=session.active_goal_checker_id or None,
                                  planner_id=planner_goal.planner_id,
                                  target_xy_m=[session.original_goal.pose.position.x,
                                               session.original_goal.pose.position.y],
                                  target_yaw_rad=math.atan2(
                                      2*(session.original_goal.pose.orientation.w*
                                         session.original_goal.pose.orientation.z+
                                         session.original_goal.pose.orientation.x*
                                         session.original_goal.pose.orientation.y),
                                      1-2*(session.original_goal.pose.orientation.y**2+
                                           session.original_goal.pose.orientation.z**2)),
                                  use_start=planner_goal.use_start, **start_evidence))
            self._report(dict(event='PLANNER_GOAL_DISPATCHED',
                              planner_id=planner_goal.planner_id,
                              use_start=planner_goal.use_start,
                              goal_checker_id=session.active_goal_checker_id or None,
                              target_frame=planner_goal.goal.header.frame_id,
                              target_xy_m=[planner_goal.goal.pose.position.x,
                                           planner_goal.goal.pose.position.y]))

            if not self._planner_client.server_is_ready():
                return self._finish(outer_goal_handle, 'abort')

            # Until a reply proves rejection, a dispatched request may have
            # created a remote goal even if the transport reply is lost.
            planner_handle = await self._send_goal(
                self._planner_client,
                planner_goal,
                session=session,
            )
            if planner_handle is None or not getattr(planner_handle, 'accepted', False):
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.planner_goal_handle = planner_handle
                session.inner_goal_handle = planner_handle
                session.inner_result_future = None

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

            if self._path_validator is not None:
                try:
                    proof = await self._path_validator(path, session.original_goal)
                    if proof is None:
                        raise ValueError("path validation supplied no proof")
                    self._report(dict(event="PATH_VALIDATED", **proof))
                except Exception as error:
                    self._report(dict(event="PATH_REJECTED", reason=str(error)))
                    return self._finish(outer_goal_handle, "cancel" if self._is_cancel_requested(session) else "abort")

            # 取消发生在 planner 成功与 FollowPath submit 之间时，
            # planner 已经终止且不能再启动新的 inner goal。
            if self._is_cancel_requested(session):
                return self._finish(outer_goal_handle, 'cancel')

            follow_goal = FollowPath.Goal()
            follow_goal.path = _prepare_controller_path(path)
            follow_goal.controller_id = self._controller_id
            follow_goal.goal_checker_id = self._goal_checker_for(session)
            session.active_goal_checker_id = follow_goal.goal_checker_id
            self._report(dict(event='FOLLOW_GOAL_DISPATCHED',
                              goal_checker_id=follow_goal.goal_checker_id,
                              path_sha256=path_signature(path)))

            if not self._follow_client.server_is_ready():
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.stage = _Stage.FOLLOWING
            follow_handle = await self._send_goal(
                self._follow_client,
                follow_goal,
                session=session,
            )
            if follow_handle is None or not getattr(follow_handle, 'accepted', False):
                if self._is_cancel_requested(session) and session.inner_terminal:
                    return self._finish(outer_goal_handle, 'cancel')
                if not session.inner_terminal and session.submission_unverified:
                    await self._await_inner_terminal(session)
                return self._finish(outer_goal_handle, 'abort')

            with self._lock:
                session.follow_goal_handle = follow_handle
                session.inner_goal_handle = follow_handle
                session.inner_result_future = None
            return await self._follow_with_replanning(session, follow_handle, path)
        except Exception as error:
            self._report(dict(event='FACADE_INTERNAL_ERROR', reason=str(error),
                              stage=session.stage.name))
            if session.stage is _Stage.FOLLOWING and not session.inner_terminal:
                # An Action transport exception is not proof that FollowPath
                # stopped. Keep Mission's navigation execution (and Lease)
                # open until an actual inner terminal is observed.
                for goal_id, handle in (session.handles or {}).items():
                    if (session.action_types or {}).get(goal_id) != 'FollowPath' or session.inner_actions.get(goal_id, True):
                        continue
                    try:
                        await self._cancel_and_confirm(handle, session=session)
                    except Exception:
                        pass
                await self._await_inner_terminal(session)
            return self._finish(outer_goal_handle, 'abort')
        finally:
            for goal_id, future in (session.result_futures or {}).items():
                action = session.inner_actions.get(goal_id) if session.inner_actions else None
                if action is not None:
                    handle = (session.handles or {}).get(goal_id)
                    if handle is not None:
                        self._observe_inner_terminal(session, handle, future)
            if session.inner_result_future is not None and session.inner_goal_handle is not None:
                self._observe_inner_terminal(
                    session, session.inner_goal_handle, session.inner_result_future,
                )
            with self._lock:
                session.terminal = True
                if self._session is session and session.inner_terminal:
                    self._session = None
                    self._busy = False
                    self._reserved = False
            if not session.inner_terminal and not session.cancel_requested:
                # An outer ABORT is not proof that its inner action stopped.
                # Keep the slot closed until a real terminal result is seen.
                for goal_id, handle in (session.handles or {}).items():
                    if session.inner_actions.get(goal_id, True):
                        continue
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass

    def _refresh_inner_terminal_locked(self, session) -> None:
        actions = getattr(session, 'inner_actions', None) or {}
        pending = getattr(session, 'dispatches_pending', 0)
        unverified = getattr(session, 'submission_unverified', False)
        session.inner_terminal = (
            pending == 0 and not unverified and all(actions.values())
        )
        if session.inner_terminal:
            self._try_wake_locked(session, _WakeReason.RESULT)

    async def _await_inner_terminal(self, session) -> None:
        """Keep the outer Action/mission lease open until every inner owner is terminal."""
        cancel_attempted = set()
        while True:
            with self._lock:
                self._refresh_inner_terminal_locked(session)
                if session.inner_terminal:
                    return
                wake = Future()
                session.wake_future = wake
                active_ids = {
                    goal_id for goal_id, terminal in (session.inner_actions or {}).items()
                    if not terminal
                }
                cancel_requested = session.cancel_requested and bool(active_ids - cancel_attempted)
            if cancel_requested:
                self._try_wake(session, _WakeReason.CANCEL)
            try:
                reason = await wake
            finally:
                with self._lock:
                    if session.wake_future is wake:
                        session.wake_future = None
            if reason is _WakeReason.CANCEL:
                handles = list((session.handles or {}).items())
                # Stop the controller before waiting for planner cleanup.
                handles.sort(key=lambda item: 0 if (session.action_types or {}).get(item[0]) == 'FollowPath' else 1)
                for goal_id, handle in handles:
                    if session.inner_actions.get(goal_id, True):
                        continue
                    if goal_id in cancel_attempted:
                        continue
                    cancel_attempted.add(goal_id)
                    try:
                        await self._cancel_and_confirm(handle, session=session)
                    except Exception:
                        pass

    def _track_dispatch_response(self, session, handle, goal) -> None:
        with self._lock:
            session.dispatches_pending = max(0, session.dispatches_pending - 1)
            if handle is None:
                session.submission_unverified = True
            elif getattr(handle, 'accepted', False):
                goal_id = _goal_id_bytes(handle)
                if not goal_id:
                    session.submission_unverified = True
                else:
                    session.inner_actions[goal_id] = False
                    session.handles[goal_id] = handle
                    session.action_types[goal_id] = (
                        'FollowPath' if isinstance(goal, FollowPath.Goal)
                        else 'ComputePathToPose'
                    )
                    if session.cancel_requested:
                        self._try_wake_locked(session, _WakeReason.CANCEL)
            self._refresh_inner_terminal_locked(session)

    def _track_dispatch_exception(self, session) -> None:
        with self._lock:
            session.dispatches_pending = max(0, session.dispatches_pending - 1)
            session.submission_unverified = True
            self._refresh_inner_terminal_locked(session)

    def _result_future(self, session, goal_handle):
        goal_id = _goal_id_bytes(goal_handle)
        if goal_id and session.result_futures is not None:
            existing = session.result_futures.get(goal_id)
            if existing is not None:
                return existing
        future = goal_handle.get_result_async()
        if goal_id and session.result_futures is not None:
            session.result_futures[goal_id] = future
        session.inner_goal_handle = goal_handle
        session.inner_result_future = future
        future.add_done_callback(
            lambda completed: self._observe_inner_terminal(
                session, goal_handle, completed,
            )
        )
        return future

    async def _complete_success(self, session, *, source='FOLLOW_ACTION'):
        """Delay outer success until a constrained stop/localization proof is complete."""
        self._report(dict(event='FOLLOW_ACTION_TERMINAL' if source == 'FOLLOW_ACTION'
                          else 'NAVIGATION_TERMINAL',
                          outcome='SUCCEEDED', source=source,
                          goal_checker_id=session.active_goal_checker_id or None,
                          outer_uuid=_goal_id_bytes(session.outer_goal_handle).hex()))
        if self._post_success_guard is not None:
            try:
                proof = await self._post_success_guard(
                    session.original_goal,
                    session.active_goal_checker_id,
                    time.monotonic(),
                )
            except Exception as error:
                self._report(dict(event='WAYPOINT_STOP_STABILITY_FAILED',
                                  reason=str(error), source=source,
                                  goal_checker_id=session.active_goal_checker_id or None))
                return self._finish(session.outer_goal_handle, 'abort')
            stable_event = dict(event='WAYPOINT_STOP_STABLE', source=source)
            stable_event.update(proof or {})
            stable_event.setdefault('goal_checker_id',
                                    session.active_goal_checker_id or None)
            self._report(stable_event)
        return self._finish(session.outer_goal_handle, 'succeed')

    async def _follow_with_replanning(self, session, initial_handle, initial_path):
        """Keep one outer owner while serializing plan revisions and path replacement."""
        follow_handle = initial_handle
        current_path = initial_path
        session.current_path_signature = path_signature(current_path)
        session.follow_revision += 1
        session.replan_failures = 0
        while True:
            try:
                follow_future = self._result_future(session, follow_handle)
            except Exception as error:
                self._report(dict(event='FOLLOW_RESULT_UNVERIFIED', reason=str(error)))
                return self._finish(session.outer_goal_handle, 'abort')

            wake_future = Future()
            with self._lock:
                session.stage = _Stage.FOLLOWING
                session.wake_future = wake_future
                cancel_requested = session.cancel_requested
                replan_pending = session.replan_pending
            if cancel_requested:
                self._try_wake(session, _WakeReason.CANCEL)
            elif replan_pending:
                self._try_wake(session, _WakeReason.REPLAN)
            else:
                expected_revision = session.follow_revision

                def _wake_current_follow(_future, expected_revision=expected_revision):
                    # A replaced FollowPath stays done. Its callback must not
                    # be read as the result of the follow that replaced it.
                    if session.follow_revision != expected_revision:
                        return
                    self._try_wake(session, _WakeReason.RESULT)

                follow_future.add_done_callback(_wake_current_follow)
            try:
                reason = await wake_future
            finally:
                with self._lock:
                    if session.wake_future is wake_future:
                        session.wake_future = None

            if reason is _WakeReason.CANCEL:
                if self._future_done(follow_future):
                    try:
                        wrapped = follow_future.result()
                    except Exception:
                        wrapped = None
                    if wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                        return await self._complete_success(session)
                if await self._cancel_and_confirm(follow_handle, session=session):
                    return self._finish(session.outer_goal_handle, 'cancel')
                self._report(dict(event='FOLLOW_CANCEL_UNCONFIRMED',
                                  goal_id=_goal_id_bytes(follow_handle).hex()))
                await self._await_inner_terminal(session)
                if self._future_done(follow_future):
                    try:
                        wrapped = follow_future.result()
                    except Exception:
                        wrapped = None
                    if wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                        return await self._complete_success(session)
                return self._finish(session.outer_goal_handle, 'abort')

            if reason is _WakeReason.RESULT:
                try:
                    wrapped = follow_future.result() if self._future_done(follow_future) else None
                except Exception as error:
                    self._report(dict(event='FOLLOW_RESULT_UNVERIFIED', reason=str(error)))
                    try:
                        await self._cancel_and_confirm(follow_handle, session=session)
                    except Exception:
                        pass
                    await self._await_inner_terminal(session)
                    return self._finish(session.outer_goal_handle, 'abort')
                if wrapped is None:
                    # rclpy Future.result() is None while the future is unfinished.
                    # A stale wake must not cancel the follow that is still running.
                    self._report(dict(event='FOLLOW_RESULT_UNVERIFIED', reason='result unavailable'))
                    if not self._future_done(follow_future):
                        continue
                    return self._finish(session.outer_goal_handle, 'abort')
                if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                    return await self._complete_success(session)
                if wrapped.status != GoalStatus.STATUS_ABORTED:
                    self._report(dict(event='FOLLOW_UNEXPECTED_TERMINAL', status=wrapped.status))
                    return self._finish(session.outer_goal_handle, 'abort')
                if session.replan_failures >= self._max_replan_failures:
                    return self._finish(session.outer_goal_handle, 'abort')
                # Legacy treats an aborted FollowPath as a reason to plan the
                # same immutable outer target again. The old action is already
                # terminal, so this retry cannot overlap a controller owner.
                candidate = await self._plan_retry(session, reason='FOLLOW_ABORTED')
                if candidate is None:
                    return self._finish(
                        session.outer_goal_handle,
                        'cancel' if self._is_cancel_requested(session) else 'abort',
                    )
                next_handle = await self._submit_follow(session, candidate)
                if next_handle is None or not getattr(next_handle, 'accepted', False):
                    return self._finish(
                        session.outer_goal_handle,
                        'cancel' if self._is_cancel_requested(session) and session.inner_terminal else 'abort',
                    )
                follow_handle = next_handle
                current_path = candidate
                session.current_path_signature = path_signature(candidate)
                session.follow_revision += 1
                continue

            if reason is not _WakeReason.REPLAN:
                return self._finish(session.outer_goal_handle, 'abort')

            with self._lock:
                session.replan_pending = False
                session.replan_in_flight = True
                session.stage = _Stage.PLANNING
                session.plan_revision += 1
                revision = session.plan_revision
            self._report(dict(event='REPLAN_STARTED', revision=revision,
                              outer_uuid=_goal_id_bytes(session.outer_goal_handle).hex(),
                              active_path_sha256=session.current_path_signature))
            replacement = await self._plan_while_following(
                session, follow_handle, follow_future, revision,
            )
            with self._lock:
                session.replan_in_flight = False
                cancelled = session.cancel_requested
                pending = session.replan_pending
                session.stage = _Stage.FOLLOWING
            if replacement == 'OUTER_SUCCEEDED':
                return await self._complete_success(session)
            if replacement == 'OUTER_CANCELED':
                return self._finish(session.outer_goal_handle, 'cancel')
            if replacement == 'OUTER_ABORTED':
                return self._finish(session.outer_goal_handle, 'abort')
            if replacement == 'NO_CHANGE':
                session.replan_failures = 0
                continue
            if replacement is None:
                session.replan_failures += 1
                self._report(dict(event='REPLAN_FAILED', revision=revision,
                                  failures=session.replan_failures))
                if cancelled:
                    if await self._cancel_and_confirm(follow_handle, session=session):
                        return self._finish(session.outer_goal_handle, 'cancel')
                    await self._await_inner_terminal(session)
                    return self._finish(session.outer_goal_handle, 'abort')
                if (isinstance(session.outer_goal_handle.request, NavigateWithinPolygon.Goal)
                        and self._path_validator is not None and not session.submission_unverified
                        and not session.dispatches_pending
                        and all(terminal for ident,terminal in (session.inner_actions or {}).items()
                                if (session.action_types or {}).get(ident) != 'FollowPath')):
                    # A rejected replacement is not evidence that the active
                    # path is unsafe. Revalidate its complete geometry against
                    # a fresh costmap + the unchanged task polygon before
                    # retaining the current controller owner.
                    try:
                        proof = await self._path_validator(current_path,session.original_goal)
                        if proof is None:
                            raise ValueError('active path supplied no fresh proof')
                        self._report(dict(event='REPLAN_REJECTED_ACTIVE_PATH_REVALIDATED',
                            plan_revision=revision,**proof))
                        session.replan_failures = 0
                        continue
                    except Exception as error:
                        self._report(dict(event='ACTIVE_PATH_REJECTED',reason=str(error)))
                        if await self._cancel_and_confirm(follow_handle,session=session):
                            return self._finish(session.outer_goal_handle,'abort')
                        await self._await_inner_terminal(session)
                        return self._finish(session.outer_goal_handle,'abort')
                if session.replan_failures >= self._max_replan_failures:
                    if self._future_done(follow_future):
                        try:
                            ended = follow_future.result()
                        except Exception:
                            ended = None
                        if ended is not None and ended.status == GoalStatus.STATUS_SUCCEEDED:
                            return await self._complete_success(session)
                    if await self._cancel_and_confirm(follow_handle, session=session):
                        return self._finish(session.outer_goal_handle, 'abort')
                    await self._await_inner_terminal(session)
                    return self._finish(session.outer_goal_handle, 'abort')
                continue

            candidate = replacement
            # One planner request is allowed in flight. Ticks received during
            # that request are coalesced; the next loop consumes at most one.
            if cancelled:
                if await self._cancel_and_confirm(follow_handle, session=session):
                    return self._finish(session.outer_goal_handle, 'cancel')
                return self._finish(session.outer_goal_handle, 'abort')
            next_handle = await self._submit_follow(session, candidate)
            if next_handle is None or not getattr(next_handle, 'accepted', False):
                return self._finish(
                    session.outer_goal_handle,
                    'cancel' if self._is_cancel_requested(session) and session.inner_terminal else 'abort',
                )
            follow_handle = next_handle
            current_path = candidate
            session.current_path_signature = path_signature(candidate)
            session.follow_revision += 1
            session.replan_failures = 0
            self._report(dict(event='PATH_REPLACED', plan_revision=revision,
                              follow_revision=session.follow_revision,
                              path_sha256=session.current_path_signature,
                              deferred_replan=pending))

    async def _plan_retry(self, session, *, reason):
        """Submit a bounded same-goal retry after the previous controller ended."""
        with self._lock:
            session.stage = _Stage.PLANNING
            session.plan_revision += 1
            session.replan_failures += 1
            revision = session.plan_revision
        if self._is_cancel_requested(session) or not self._planner_client.server_is_ready():
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = copy.deepcopy(session.original_goal)
        goal.planner_id = self._planner_id
        goal.use_start = self._use_start
        handle = await self._send_goal(self._planner_client, goal, session=session)
        if handle is None or not getattr(handle, 'accepted', False):
            return None
        session.planner_goal_handle = handle
        outcome, wrapped = await self._wait_inner_result(session, handle)
        if outcome is not _InnerOutcome.RESULT or wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        path = wrapped.result.path
        if not _path_is_valid(path):
            return None
        if self._path_validator is not None:
            try:
                proof = await self._path_validator(path, session.original_goal)
                if proof is None:
                    raise ValueError('path validation supplied no proof')
                self._report(dict(event='PATH_VALIDATED', plan_revision=revision, **proof))
            except Exception as error:
                self._report(dict(event='PATH_REJECTED', plan_revision=revision, reason=str(error)))
                return None
        self._report(dict(event='REPLAN_RESULT', plan_revision=revision, reason=reason,
                          path_sha256=path_signature(path)))
        return path

    async def _submit_follow(self, session, path):
        if self._is_cancel_requested(session) or not self._follow_client.server_is_ready():
            return None
        goal = FollowPath.Goal()
        goal.path = _prepare_controller_path(path)
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_for(session)
        session.active_goal_checker_id = goal.goal_checker_id
        self._report(dict(event='FOLLOW_GOAL_DISPATCHED',
                          goal_checker_id=goal.goal_checker_id,
                          path_sha256=path_signature(path),
                          follow_revision=session.follow_revision + 1))
        return await self._send_goal(self._follow_client, goal, session=session)

    async def _plan_while_following(self, session, follow_handle, follow_future, revision):
        """Plan against the fixed outer target while its current path is active."""
        if not self._planner_client.server_is_ready():
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = copy.deepcopy(session.original_goal)
        goal.planner_id = self._planner_id
        goal.use_start = self._use_start
        planner_handle = await self._send_goal(
            self._planner_client, goal, session=session,
            cancel_active_handle=follow_handle,
        )
        if planner_handle is None or not getattr(planner_handle, 'accepted', False):
            return 'OUTER_CANCELED' if self._is_cancel_requested(session) else None
        session.planner_goal_handle = planner_handle
        try:
            planner_future = self._result_future(session, planner_handle)
        except Exception as error:
            self._report(dict(event='REPLAN_RESULT_UNVERIFIED', revision=revision, reason=str(error)))
            return None

        while True:
            wake = Future()
            with self._lock:
                session.wake_future = wake
                cancel_requested = session.cancel_requested
            if not self._future_done(planner_future):
                planner_future.add_done_callback(
                    lambda _future: self._try_wake(session, _WakeReason.RESULT)
                )
            if not self._future_done(follow_future):
                follow_future.add_done_callback(
                    lambda _future: self._try_wake(session, _WakeReason.RESULT)
                )
            if cancel_requested:
                self._try_wake(session, _WakeReason.CANCEL)
            elif self._future_done(planner_future) or self._future_done(follow_future):
                self._try_wake(session, _WakeReason.RESULT)
            try:
                await wake
            finally:
                with self._lock:
                    if session.wake_future is wake:
                        session.wake_future = None

            # A completed controller result takes precedence over a late plan.
            if self._future_done(follow_future):
                try:
                    ended = follow_future.result()
                except Exception:
                    ended = None
                if ended is not None and ended.status == GoalStatus.STATUS_SUCCEEDED:
                    if not self._future_done(planner_future):
                        try:
                            planner_handle.cancel_goal_async()
                        except Exception:
                            pass
                    return 'OUTER_SUCCEEDED'
                if ended is not None and ended.status == GoalStatus.STATUS_CANCELED:
                    if not self._future_done(planner_future):
                        try:
                            planner_handle.cancel_goal_async()
                        except Exception:
                            pass
                    return 'OUTER_CANCELED' if self._is_cancel_requested(session) else 'OUTER_ABORTED'

            if self._is_cancel_requested(session):
                stopped = await self._cancel_and_confirm(follow_handle, session=session)
                if not self._future_done(planner_future):
                    try:
                        planner_handle.cancel_goal_async()
                    except Exception:
                        pass
                await self._await_inner_terminal(session)
                if self._future_done(follow_future):
                    try:
                        ended = follow_future.result()
                    except Exception:
                        ended = None
                    if ended is not None and ended.status == GoalStatus.STATUS_SUCCEEDED:
                        return 'OUTER_SUCCEEDED'
                return 'OUTER_CANCELED' if stopped else 'OUTER_ABORTED'

            if self._future_done(planner_future):
                try:
                    wrapped = planner_future.result()
                except Exception as error:
                    self._report(dict(event='REPLAN_RESULT_UNVERIFIED', revision=revision,
                                      reason=str(error)))
                    return None
                if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    return None
                candidate = wrapped.result.path
                if not _path_is_valid(candidate):
                    return None
                if self._path_validator is not None:
                    try:
                        proof = await self._path_validator(candidate, session.original_goal)
                        if proof is None:
                            raise ValueError('path validation supplied no proof')
                        self._report(dict(event='PATH_VALIDATED', plan_revision=revision, **proof))
                    except Exception as error:
                        self._report(dict(event='PATH_REJECTED', plan_revision=revision,
                                          reason=str(error)))
                        return None
                if self._is_cancel_requested(session):
                    if await self._cancel_and_confirm(follow_handle, session=session):
                        return 'OUTER_CANCELED'
                    return 'OUTER_ABORTED'
                distance = self._refresh_goal_distance(session)
                if distance is not None and distance <= self._replan_near_goal_tolerance:
                    self._report(dict(event='REPLAN_RESULT_SUPPRESSED_NEAR_GOAL',
                                      distance_to_goal=distance,
                                      distance_source='tf' if self._goal_distance_provider else 'follow_path_feedback',
                                      path_sha256=path_signature(candidate)))
                    return 'NO_CHANGE'
                candidate_signature = path_signature(candidate)
                active_done = self._future_done(follow_future)
                try:
                    active_result = follow_future.result() if active_done else None
                except Exception:
                    active_result = None
                if candidate_signature == session.current_path_signature and not active_done:
                    self._report(dict(event='REPLAN_UNCHANGED', revision=revision,
                                      path_sha256=candidate_signature))
                    return 'NO_CHANGE'
                if active_done:
                    if active_result is not None and active_result.status == GoalStatus.STATUS_SUCCEEDED:
                        return 'OUTER_SUCCEEDED'
                    if active_result is None or active_result.status != GoalStatus.STATUS_ABORTED:
                        try:
                            await self._cancel_and_confirm(follow_handle, session=session)
                        except Exception:
                            pass
                        await self._await_inner_terminal(session)
                        return 'OUTER_ABORTED'
                    return candidate
                if await self._cancel_and_confirm(follow_handle, session=session):
                    self._report(dict(event='FOLLOW_REPLACEMENT_TERMINAL',
                                      follow_revision=session.follow_revision,
                                      goal_id=_goal_id_bytes(follow_handle).hex()))
                    return candidate
                # A natural ABORTED terminal is also safe to replace. A
                # canceled/unverified result is not treated as a replacement.
                if self._future_done(follow_future):
                    try:
                        ended = follow_future.result()
                    except Exception:
                        ended = None
                    if ended is not None and ended.status == GoalStatus.STATUS_ABORTED:
                        return candidate
                    if ended is not None and ended.status == GoalStatus.STATUS_SUCCEEDED:
                        return 'OUTER_SUCCEEDED'
                await self._await_inner_terminal(session)
                return 'OUTER_ABORTED'

    def _observe_inner_terminal(self, session, goal_handle, future) -> None:
        """Release ownership only after an identified inner terminal result."""
        if not self._future_done(future):
            return
        try:
            wrapped = future.result()
            status = wrapped.status
            if self._diagnostic:
                result = getattr(wrapped, 'result', None)
                goal_id = _goal_id_bytes(goal_handle)
                action_type = (getattr(session, 'action_types', None) or {}).get(goal_id)
                self._report(dict(event='INNER_RESULT', stage=session.stage.name,
                    goal_id=goal_id.hex(), status=status,
                    action_type=action_type,
                    goal_checker_id=(session.active_goal_checker_id or None
                                     if action_type == 'FollowPath' else None),
                    result_observed_monotonic=time.monotonic(),
                    error_code=getattr(result, 'error_code', None),
                    error_msg=getattr(result, 'error_msg', None)))
        except Exception:
            return
        if status not in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ):
            return
        with self._lock:
            goal_id = _goal_id_bytes(goal_handle)
            actions = getattr(session, 'inner_actions', None)
            if actions is None:
                # Keep the small private helper useful for legacy unit probes.
                session.inner_terminal = True
            elif goal_id in actions:
                actions[goal_id] = True
                self._refresh_inner_terminal_locked(session)
            else:
                return
            if session.terminal and self._session is session and session.inner_terminal:
                self._session = None
                self._busy = False
                self._reserved = False

    async def _send_goal(
        self, client: Any, goal: Any, *, session=None, cancel_active_handle=None,
    ) -> Any:
        """Await one non-blocking ActionClient goal request."""
        def feedback(message):
            value = message.feedback
            action = type(goal).__qualname__
            reported_distance = getattr(value, 'distance_to_goal', None)
            distance = reported_distance
            if session is not None and isinstance(goal, FollowPath.Goal):
                distance = self._on_follow_feedback(session, value)
            self._report(dict(event='INNER_FEEDBACK', action=action,
                goal_id=bytes(getattr(message.goal_id, 'uuid', ())).hex(),
                distance_to_goal=distance, reported_distance_to_goal=reported_distance,
                distance_source='tf' if self._goal_distance_provider and session is not None
                    and isinstance(goal, FollowPath.Goal) else 'follow_path_feedback',
                speed=getattr(value, 'speed', None)))

        kwargs = {'feedback_callback': feedback} if session is not None or self._diagnostic else {}
        if session is None:
            future = client.send_goal_async(goal, **kwargs)
        else:
            # cancel记录与实际Action提交在同一锁中排序；不持锁await。
            with self._lock:
                if session.cancel_requested:
                    return None
                session.dispatches_pending += 1
                session.inner_terminal = False
                try:
                    future = client.send_goal_async(goal, **kwargs)
                except Exception:
                    session.dispatches_pending = max(0, session.dispatches_pending - 1)
                    session.submission_unverified = True
                    self._refresh_inner_terminal_locked(session)
                    raise
        try:
            if session is not None:
                wake = Future()
                with self._lock:
                    session.wake_future = wake
                    cancel_requested = session.cancel_requested
                future.add_done_callback(
                    lambda _future: self._try_wake(session, _WakeReason.RESULT)
                )
                if cancel_requested:
                    self._try_wake(session, _WakeReason.CANCEL)
                try:
                    reason = await wake
                finally:
                    with self._lock:
                        if session.wake_future is wake:
                            session.wake_future = None
                if reason is _WakeReason.CANCEL and cancel_active_handle is not None:
                    await self._cancel_and_confirm(
                        cancel_active_handle, session=session,
                    )
                handle = await future
            else:
                handle = await future
        except Exception:
            if session is not None:
                self._track_dispatch_exception(session)
            raise
        if session is not None:
            self._track_dispatch_response(session, handle, goal)
        self._report(dict(event='INNER_GOAL_RESPONSE', action=type(goal).__qualname__,
                          goal_id=_goal_id_bytes(handle).hex(),
                          accepted=getattr(handle, 'accepted', None),
                          path_sha256=path_signature(goal.path) if isinstance(goal, FollowPath.Goal) else None))
        return handle

    def _on_follow_feedback(self, session, value) -> None:
        if self._goal_distance_provider is not None:
            distance = self._refresh_goal_distance(session)
        else:
            distance = getattr(value, 'distance_to_goal', None)
            if distance is not None and not math.isfinite(float(distance)):
                distance = None
        with self._lock:
            if self._goal_distance_provider is None:
                session.distance_to_goal = max(0.0, float(distance)) if distance is not None else None
            distance = session.distance_to_goal
        try:
            feedback = _outer_action(session.outer_goal_handle.request).Feedback()
            pose = getattr(value, 'current_pose', None)
            if pose is not None:
                feedback.current_pose = copy.deepcopy(pose)
            elapsed = max(0, int(time.monotonic() - session.started_monotonic))
            feedback.navigation_time.sec = elapsed
            feedback.distance_remaining = float(distance) if distance is not None else 0.0
            feedback.number_of_recoveries = 0
            session.outer_goal_handle.publish_feedback(feedback)
        except Exception:
            pass
        return distance

    def _refresh_goal_distance(self, session):
        provider = self._goal_distance_provider
        if provider is None:
            return session.distance_to_goal
        distance = None
        try:
            value = provider(copy.deepcopy(session.original_goal))
            if value is not None and math.isfinite(float(value)) and float(value) >= 0.0:
                distance = float(value)
        except Exception:
            pass
        with self._lock:
            if self._session is session and not session.terminal:
                session.distance_to_goal = distance
                return distance
        return None

    async def _wait_inner_result(
        self,
        session: _Session,
        inner_goal_handle: Any,
    ) -> tuple[_InnerOutcome, Any | None]:
        """Race inner result with outer cancellation without executor blocking."""
        try:
            inner_result_future = self._result_future(session, inner_goal_handle)
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
                    in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
                ):
                    return _InnerOutcome.RESULT, wrapped_result
            if await self._cancel_and_confirm(inner_goal_handle, session=session):
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
            if await self._cancel_and_confirm(inner_goal_handle, session=session):
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

    async def _cancel_and_confirm(self, inner_goal_handle: Any, *, session=None) -> bool:
        """Require accepted cancel response and a CANCELED terminal result."""
        goal_id = _goal_id_bytes(inner_goal_handle)
        if session is not None and goal_id and goal_id in session.cancel_confirmed:
            try:
                future = self._result_future(session, inner_goal_handle)
                return self._future_done(future) and future.result().status == GoalStatus.STATUS_CANCELED
            except Exception:
                return False
        try:
            self._report(dict(event='INNER_CANCEL_REQUEST',goal_id=goal_id.hex(),
                caller=[f'{v.name}:{v.lineno}' for v in traceback.extract_stack(limit=4)[:-1]]))
            response = await inner_goal_handle.cancel_goal_async()
            self._report(dict(event='INNER_CANCEL_RESPONSE',goal_id=goal_id.hex(),
                return_code=getattr(response,'return_code',None),
                canceling=[bytes(v.goal_id.uuid).hex() for v in getattr(response,'goals_canceling',())]))
        except Exception:
            return False
        if not _cancel_response_contains_goal(response, inner_goal_handle):
            return False

        try:
            future = (
                self._result_future(session, inner_goal_handle)
                if session is not None
                else inner_goal_handle.get_result_async()
            )
            wrapped_result = await future
        except Exception:
            return False
        confirmed = (
            getattr(wrapped_result, 'status', None)
            == GoalStatus.STATUS_CANCELED
        )
        if confirmed and session is not None and goal_id:
            session.cancel_confirmed.add(goal_id)
        return confirmed

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
        if session is not None and session.inner_result_future is not None:
            self._observe_inner_terminal(session, session.inner_goal_handle, session.inner_result_future)
        with self._lock:
            if session is None or session.terminal:
                return _outer_action(outer_goal_handle.request).Result()
            session.terminal = True

        self._report(dict(event='OUTER_TERMINAL', outcome=terminal,
                          inner_terminal=session.inner_terminal,
                          outer_uuid=_goal_id_bytes(outer_goal_handle).hex()))
        if terminal == 'succeed':
            outer_goal_handle.succeed()
        elif terminal == 'cancel':
            outer_goal_handle.canceled()
        else:
            outer_goal_handle.abort()
        return _outer_action(outer_goal_handle.request).Result()


class NavigationFacadeNode(PolygonNavigationMixin, Node):
    """Expose the CleanNav outer action and own inner action clients."""

    def __init__(self) -> None:
        super().__init__('cleannav_navigation_facade')
        self._setup_polygon_navigation()
        self.declare_parameter('m0_forward_only', False)
        # Opt in only in a bound candidate whose controller tolerances exceed
        # this conservative 2 cm / 0.02 rad no-motion equality test.
        self.declare_parameter('stationary_goal_completion', False)
        self._completion_odom = None
        self._completion_odom_received = 0.
        self._completion_odom_samples = deque(maxlen=512)
        self._final_command_received = None
        self._final_command_received_previous = None
        self._final_command_zero_since = None
        self._localization_health = None
        self._localization_health_received = None
        self._completion_tf = Buffer()
        self._completion_listener = TransformListener(self._completion_tf, self)
        self._completion_subscription = self.create_subscription(
            Odometry, '/odom', self._on_completion_odom, qos_profile_sensor_data)
        # Read-only observations of the final velocity output and existing
        # localization health contract; this node never publishes velocity.
        self._final_command_subscription = self.create_subscription(
            Twist, '/cmd_vel', self._on_final_command, 10)
        self._localization_health_subscription = self.create_subscription(
            String, '/cleannav/localization/health', self._on_localization_health, 10)
        self._costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self._path_evidence = self.create_publisher(Path, '/cleannav/navigation/planner_path', 10)
        self._admitted_evidence = self.create_publisher(Path, '/cleannav/navigation/admitted_path', 10)
        self.declare_parameter('planner_id', DEFAULT_PLANNER_ID)
        self.declare_parameter('use_start', DEFAULT_USE_START)
        self.declare_parameter('controller_id', DEFAULT_CONTROLLER_ID)
        self.declare_parameter('goal_checker_id', DEFAULT_GOAL_CHECKER_ID)
        self.declare_parameter('polygon_goal_checker_id', '')
        self.declare_parameter('replan_period_sec', 1.0)
        self.declare_parameter('replan_near_goal_tolerance', 0.25)

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
            polygon_goal_checker_id=self.get_parameter('polygon_goal_checker_id').value,
            replan_near_goal_tolerance=float(
                self.get_parameter('replan_near_goal_tolerance').value
            ),
            path_validator=self._validate_controller_path,
            stationary_goal_checker=self._stationary_goal_proof,
            goal_distance_provider=self._goal_distance,
            start_snapshot_provider=self._capture_patrol_start_snapshot,
            post_success_guard=self._wait_for_waypoint_stop_and_snapshot,
            session_initializer=self._initialize_boundary,
            diagnostic=lambda data: self.get_logger().info('NAV2_DIAGNOSTIC '+json.dumps(data)),
        )
        self._replan_timer = None
        period = float(self.get_parameter('replan_period_sec').value)
        if math.isfinite(period) and period > 0.0:
            self._replan_timer = self.create_timer(
                period,
                self._runtime.request_replan,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
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
        self._constrained_action_server = ActionServer(
            self, NavigateWithinPolygon, CONSTRAINED_ACTION_NAME,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
            goal_callback=self._constrained_goal_callback, cancel_callback=self._cancel_callback)
        self._patrol_path_active = False
        self._patrol_follow_server = ActionServer(
            self, FollowPath, PATROL_FOLLOW_ACTION_NAME,
            execute_callback=self._execute_patrol_path,
            callback_group=self._callback_group,
            goal_callback=self._accept_patrol_path,
            cancel_callback=self._cancel_patrol_path)

    def _on_completion_odom(self, message):
        self._completion_odom = message
        received = time.monotonic()
        self._completion_odom_received = received
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        twist = message.twist.twist
        try:
            if message.child_frame_id == 'base_link':
                offset = 0.0
            else:
                offset_tf = self._completion_tf.lookup_transform(
                    'base_link', message.child_frame_id, Time())
                translation = offset_tf.transform.translation
                offset = math.hypot(translation.x, translation.y, translation.z)
            linear = math.hypot(twist.linear.x, twist.linear.y, twist.linear.z)
            angular = math.hypot(twist.angular.x, twist.angular.y, twist.angular.z)
            linear += angular * offset
            planar_angular = abs(twist.angular.z)
            values = (linear, planar_angular)
            if (stamp_ns > 0 and all(math.isfinite(value) for value in values)
                    and (not self._completion_odom_samples or
                         stamp_ns > self._completion_odom_samples[-1][0])):
                self._completion_odom_samples.append(
                    (stamp_ns, received, linear, planar_angular))
        except Exception:
            # Missing child-frame TF is missing stop evidence, never zero speed.
            return

    def _on_final_command(self, message):
        now = time.monotonic()
        previous = self._final_command_received_previous
        values = (message.linear.x, message.angular.z)
        self._final_command_received = now
        if not all(math.isfinite(value) for value in values):
            self._final_command_zero_since = None
            self._final_command_received_previous = now
            return
        is_zero = abs(values[0]) <= .001 and abs(values[1]) <= .001
        if not is_zero:
            self._final_command_zero_since = None
        elif (self._final_command_zero_since is None or
              (previous is not None and now-previous > .3)):
            self._final_command_zero_since = now
        self._final_command_received_previous = now

    def _on_localization_health(self, message):
        try:
            value = json.loads(message.data)
            self._localization_health = value if isinstance(value, dict) else None
        except (TypeError, ValueError):
            self._localization_health = None
        self._localization_health_received = time.monotonic()

    def _localization_health_proof(self):
        age = (time.monotonic()-self._localization_health_received
               if self._localization_health_received is not None else None)
        value = self._localization_health
        if (not isinstance(value, dict) or value.get('localized') is not True or
                age is None or not 0 <= age <= 1.5):
            raise ValueError('localization health is absent, unhealthy, or stale')
        return dict(localized=True, localization_health_age_s=age,
                    localization_reason=value.get('reason'),
                    native_parameters_verified=value.get('native_parameters_verified'),
                    sensors_fresh=value.get('sensors_fresh'),
                    map_tf_valid=value.get('map_tf_valid'),
                    scan_tf_valid=value.get('scan_tf_valid'))

    def _pose_from_current_tf(self, frame_id):
        transform = self._completion_tf.lookup_transform(frame_id, 'base_link', Time())
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
        age = (now_ns-stamp_ns)/1e9 if stamp_ns > 0 else float('inf')
        if transform.header.frame_id != frame_id or transform.child_frame_id != 'base_link':
            raise ValueError('current localization TF frame identity mismatch')
        if not -0.3 <= age <= 0.5:
            raise ValueError('current localization TF is stale or future dated')
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = copy.deepcopy(transform.header.stamp)
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
        if not _finite_pose(pose):
            raise ValueError('current localization TF pose is nonfinite')
        q = pose.pose.orientation
        qnorm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not .95 <= qnorm <= 1.05 or abs(q.x)+abs(q.y) > .02:
            raise ValueError('current localization TF orientation is invalid or nonplanar')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return pose, dict(actual_start_frame=frame_id,
                          actual_start_xy_m=[pose.pose.position.x, pose.pose.position.y],
                          actual_start_yaw_rad=yaw,
                          actual_start_tf_stamp_ns=stamp_ns,
                          actual_start_tf_age_s=age)

    async def _steady_tick(self, period_s=.05):
        wake = Future()
        timer = self.create_timer(
            period_s, lambda: None if wake.done() else wake.set_result(True),
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            await wake
        finally:
            self.destroy_timer(timer)

    async def _request_costmap_snapshot(self, frame_id):
        """Fetch and validate one complete, recent global master costmap."""
        if not self._costmap_client.service_is_ready():
            raise ValueError('global costmap service unavailable')
        response_future = self._costmap_client.call_async(GetCostmap.Request())
        wake = Future()
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        def complete(future):
            if not wake.done():
                wake.set_result(future)
        def timeout():
            if not wake.done():
                wake.set_result(None)
        timer = self.create_timer(2., timeout, clock=steady)
        response_future.add_done_callback(complete)
        try:
            received = await wake
        finally:
            self.destroy_timer(timer)
        if received is None:
            self._costmap_client.remove_pending_request(response_future)
            raise ValueError('global costmap request timeout')
        try:
            costmap = received.result().map
        except Exception as error:
            raise ValueError('global costmap request failed: '+str(error)) from error
        meta = costmap.metadata
        stamp_ns = meta.update_time.sec*10**9+meta.update_time.nanosec
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9 if stamp_ns > 0 else float('inf')
        if costmap.header.frame_id != frame_id:
            raise ValueError('global costmap frame differs from navigation frame')
        if not 0 <= age <= 1.5:
            raise ValueError('global costmap stale or future dated')
        if (meta.size_x <= 0 or meta.size_y <= 0 or
                not math.isfinite(meta.resolution) or meta.resolution <= 0):
            raise ValueError('global costmap metadata is invalid')
        origin = meta.origin
        origin_values = (origin.position.x, origin.position.y, origin.position.z,
                         origin.orientation.x, origin.orientation.y,
                         origin.orientation.z, origin.orientation.w)
        if not all(math.isfinite(value) for value in origin_values):
            raise ValueError('global costmap origin is nonfinite')
        q = origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(abs(q.w)-1.) > 1e-6:
            raise ValueError('costmap origin rotation unsupported')
        try:
            costs = bytes(costmap.data)
        except (TypeError, ValueError) as error:
            raise ValueError('global costmap data encoding is invalid') from error
        if len(costs) != meta.size_x*meta.size_y:
            raise ValueError('global costmap data length is incomplete')
        grid = Grid(costmap.header.frame_id, meta.size_x, meta.size_y,
                    meta.resolution, origin.position.x, origin.position.y, costs)
        import hashlib
        evidence = dict(costmap_frame=grid.frame,
                        costmap_update_ns=stamp_ns,
                        costmap_age_s=age,
                        costmap_width=grid.width,
                        costmap_height=grid.height,
                        costmap_resolution_m=grid.resolution,
                        costmap_origin_xy_m=[grid.origin_x, grid.origin_y],
                        costmap_sha256=hashlib.sha256(costs).hexdigest())
        return grid, evidence

    async def _capture_patrol_start_snapshot(self, goal):
        """Capture the stopped, localized actual start before a patrol plan."""
        health = self._localization_health_proof()
        now = time.monotonic()
        if (self._final_command_received is None or now-self._final_command_received > .3
                or self._final_command_zero_since is None):
            raise ValueError('final velocity command is not freshly confirmed zero')
        if not self._completion_odom_samples:
            raise ValueError('no measured odometry sample for patrol planning start')
        sample = self._completion_odom_samples[-1]
        ros_now = self.get_clock().now().nanoseconds
        if (now-sample[1] > .5 or not 0 <= (ros_now-sample[0])/1e9 <= .5
                or sample[2] > .02 or sample[3] > .02):
            raise ValueError('measured odometry is stale or vehicle is not settled')
        pose, tf_evidence = self._pose_from_current_tf(goal.header.frame_id)
        _grid, costmap_evidence = await self._request_costmap_snapshot(goal.header.frame_id)
        record = dict(source='fresh_map_tf_after_stop',
                      odometry_stamp_ns=sample[0],
                      odometry_linear_speed_mps=sample[2],
                      odometry_angular_speed_rps=sample[3],
                      final_command_zero_age_s=now-self._final_command_zero_since,
                      **health, **tf_evidence, **costmap_evidence)
        return pose, record

    async def _wait_for_waypoint_stop_and_snapshot(self, goal, goal_checker_id,
                                                   terminal_monotonic):
        """Require the Supervisor stop bounds before exposing waypoint success."""
        terminal_ros_ns = self.get_clock().now().nanoseconds
        deadline = terminal_monotonic + 12.0
        last_error = 'stop evidence incomplete'
        while time.monotonic() < deadline:
            health = self._localization_health
            proof = waypoint_stop_proof(
                list(self._completion_odom_samples),
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_monotonic=time.monotonic(),
                terminal_monotonic=terminal_monotonic,
                terminal_ros_ns=terminal_ros_ns,
                zero_since=self._final_command_zero_since,
                last_command_received=self._final_command_received,
                localization=health,
                localization_received=self._localization_health_received,
            )
            if proof['ready']:
                try:
                    health_proof = self._localization_health_proof()
                    pose, tf_evidence = self._pose_from_current_tf(goal.header.frame_id)
                    _grid, costmap_evidence = await self._request_costmap_snapshot(goal.header.frame_id)
                    q = pose.pose.orientation
                    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    return dict(
                        goal_checker_id=goal_checker_id or None,
                        terminal_monotonic=terminal_monotonic,
                        terminal_ros_ns=terminal_ros_ns,
                        terminal_pose_xy_m=[pose.pose.position.x, pose.pose.position.y],
                        terminal_pose_yaw_rad=yaw,
                        stop_evidence=proof,
                        **health_proof, **tf_evidence, **costmap_evidence)
                except Exception as error:
                    last_error = str(error)
            await self._steady_tick()
        raise TimeoutError('waypoint stop/localization/costmap proof timed out: '+last_error)

    def _stationary_goal_proof(self, goal):
        if (not self.get_parameter('stationary_goal_completion').value
                or self._completion_odom is None
                or time.monotonic()-self._completion_odom_received > .3):
            return None
        try:
            transform = self._completion_tf.lookup_transform(
                goal.header.frame_id, 'base_link', Time())
            odom_child = self._completion_odom.child_frame_id
            offset_tf = self._completion_tf.lookup_transform('base_link', odom_child, Time())
            t = offset_tf.transform.translation
            offset = math.hypot(t.x,t.y,t.z)
            return stationary_goal_proof(
                goal, transform, self._completion_odom,
                self.get_clock().now().nanoseconds, odom_child_offset_m=offset)
        except Exception:
            # Missing localization never becomes success. Use the normal
            # planner / path-admission / controller chain instead.
            return None

    def _goal_distance(self, goal):
        """Use recent localization TF for replan suppression and feedback."""
        try:
            transform = self._completion_tf.lookup_transform(
                goal.header.frame_id, 'base_link', Time())
            return goal_distance_from_transform(
                goal, transform, self.get_clock().now().nanoseconds)
        except Exception:
            return None

    async def _validate_controller_path(self, path, goal):
        """只使用规划完成之后取到的完整 costmap；请求超时或快照陈旧拒绝交付。"""
        self._path_evidence.publish(path)
        grid, snapshot = await self._request_costmap_snapshot(goal.header.frame_id)
        started = time.monotonic()
        proof = validate_path(path, grid, goal=goal,
                              m0=bool(self.get_parameter('m0_forward_only').value))
        # In-process check must not hand off a costmap that aged during validation.
        elapsed = time.monotonic()-started
        if elapsed > .5 or snapshot['costmap_age_s'] + elapsed > 1.5:
            raise ValueError('path validation exceeded snapshot budget')
        proof.update(self._validate_boundary_path(path))
        proof.update(snapshot)
        self._admitted_evidence.publish(path)
        return proof

    def _accept_patrol_path(self, goal_request: FollowPath.Goal) -> GoalResponse:
        """Accept one offline path while this slot is free. No planning."""
        path = goal_request.path
        if self._patrol_path_active or not _path_is_valid(path):
            return GoalResponse.REJECT
        if path.header.frame_id != 'map' or len(path.poses) < 2:
            return GoalResponse.REJECT
        try:
            closed_loop_splits([
                (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
            ])
        except ValueError:
            return GoalResponse.REJECT
        self._patrol_path_active = True
        return GoalResponse.ACCEPT

    def _cancel_patrol_path(self, goal_handle: Any) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_patrol_path(self, goal_handle: Any) -> FollowPath.Result:
        """Forward the offline path to the existing controller FollowPath action.

        The controller still publishes /cleannav/cmd_vel_candidate. This method
        does not publish velocity and does not call the planner.
        """
        result = FollowPath.Result()
        inner_handle = None
        try:
            path = goal_handle.request.path
            points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
            pieces = []
            for start, stop in closed_loop_splits(points):
                piece = Path()
                piece.header.frame_id = 'map'
                piece.poses = list(path.poses[start:stop])
                pieces.append(piece)
            self._admitted_evidence.publish(path)
            for piece in pieces:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                if not self._follow_client.server_is_ready():
                    goal_handle.abort()
                    return result
                follow_goal = FollowPath.Goal()
                follow_goal.path = _prepare_controller_path(piece)
                follow_goal.controller_id = 'FollowPath'
                follow_goal.goal_checker_id = 'patrol_goal_checker'
                send_future = self._follow_client.send_goal_async(follow_goal)
                inner_handle = await self._wait_action_future(send_future)
                if inner_handle is None or not getattr(inner_handle, 'accepted', False):
                    goal_handle.abort()
                    return result
                result_future = inner_handle.get_result_async()
                while not result_future.done():
                    if goal_handle.is_cancel_requested:
                        cancel_future = inner_handle.cancel_goal_async()
                        await self._wait_action_future(cancel_future)
                        goal_handle.canceled()
                        return result
                    await self._steady_tick()
                wrapped = result_future.result()
                inner_handle = None
                if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.abort()
                    return result
            goal_handle.succeed()
            return result
        except Exception:
            self.get_logger().error('patrol path follow failed\n' + traceback.format_exc())
            if inner_handle is not None:
                try:
                    inner_handle.cancel_goal_async()
                except Exception:
                    pass
            goal_handle.abort()
            return result
        finally:
            self._patrol_path_active = False

    async def _wait_action_future(self, future):
        while not future.done():
            await self._steady_tick()
        return future.result()

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
        pose = goal_handle.request.pose
        q = pose.pose.orientation
        self.get_logger().info('NAVIGATION_GOAL ' + json.dumps({
            'frame_id': pose.header.frame_id,
            'stamp': [pose.header.stamp.sec, pose.header.stamp.nanosec],
            'x': pose.pose.position.x, 'y': pose.pose.position.y,
            'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
        }))
        return await self._runtime.execute(goal_handle)

    def destroy_node(self) -> None:
        """Destroy the action server before the ROS node."""
        self._constrained_action_server.destroy()
        self._action_server.destroy()
        if self._replan_timer is not None:
            self.destroy_timer(self._replan_timer)
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
