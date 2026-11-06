"""Navigation facade FollowPath 异步生命周期离线回归（无 Gazebo、无 DDS）。

目标：固定 2026-11-04 单圈运行暴露的 `'NoneType' object has no attribute 'status'`
缺陷，以及 GOTO 到点容差只作用于 NavigateToPose 的契约。

复用既有 Mock Action 夹具（FakeActionClient / FakeGoalHandle /
HeldCallbackFuture），不搭建新的仿真系统。
"""

from __future__ import annotations

import itertools
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as PathMessage
from cleannav_interfaces.action import NavigateWithinPolygon
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR
CANDIDATE_DIR = PACKAGE_DIR
sys.path.insert(0, str(SOURCE_DIR))

from navigation_facade_node import (  # noqa: E402
    NavigationFacadeRuntime,
    _Stage,
    _WakeReason,
)

NAV2_PARAMS = CANDIDATE_DIR / 'config' / 'nav2_local_recovery.yaml'


@pytest.fixture
def executor():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    value = SingleThreadedExecutor(context=context)
    try:
        yield value
    finally:
        value.shutdown()
        rclpy.shutdown(context=context)


def _goal() -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.orientation.w = 1.0
    goal.pose.pose.position.x = 1.0
    goal.pose.pose.position.y = 2.0
    return goal


def _polygon_goal() -> NavigateWithinPolygon.Goal:
    goal = NavigateWithinPolygon.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.orientation.w = 1.0
    goal.pose.pose.position.x = 1.0
    goal.pose.pose.position.y = 2.0
    return goal


def _path(end_x=1.0) -> PathMessage:
    path = PathMessage()
    path.header.frame_id = 'map'
    for x in (0.0, end_x):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = x
        path.poses.append(pose)
    return path


def _wrapped(status, path=None):
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(path=_path() if path is None else path),
    )


class FakeGoalHandle:
    _ids = itertools.count(1)

    def __init__(self, *, accepted=True, result=None, cancel_response=None):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=next(self._ids).to_bytes(16, 'big'))
        self.result_future = Future()
        if result is not None:
            self.result_future.set_result(result)
        self.cancel_response = cancel_response
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        future = Future()
        response = self.cancel_response
        if response is None:
            response = SimpleNamespace(
                return_code=CancelGoal.Response.ERROR_NONE,
                goals_canceling=[SimpleNamespace(goal_id=self.goal_id)],
            )
        future.set_result(response)
        return future


class HeldCallbackFuture(Future):
    """Hold the replaced follow's result wake until the test releases it."""

    def __init__(self):
        super().__init__()
        self.held = []
        self._hold = True

    def add_done_callback(self, callback):
        if self._hold and getattr(callback, '__name__', '') == '_wake_current_follow':
            self.held.append(callback)
            return None
        return super().add_done_callback(callback)

    def release(self):
        self._hold = False
        pending = self.held
        self.held = []
        for callback in pending:
            super().add_done_callback(callback)


class FakeActionClient:
    def __init__(self, goal_handle=None, *, ready=True, goal_handles=None):
        self.goal_handle = goal_handle
        self.goal_handles = list(goal_handles) if goal_handles is not None else None
        self.ready = ready
        self.send_calls = []
        self.feedback_callbacks = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, **kwargs):
        self.send_calls.append(goal)
        self.feedback_callbacks.append(kwargs.get('feedback_callback'))
        future = Future()
        if self.goal_handles is not None:
            handle = self.goal_handles.pop(0) if self.goal_handles else None
        else:
            handle = self.goal_handle
        future.set_result(handle)
        return future


class FakeOuterGoal:
    def __init__(self, request=None):
        self.request = _goal() if request is None else request
        self.terminals = []
        self.feedback = []

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.terminals.append('SUCCEEDED')

    def abort(self):
        self.terminals.append('ABORTED')

    def canceled(self):
        self.terminals.append('CANCELED')


def _runtime(planner_handle, follow_handle, **kwargs):
    planner = FakeActionClient(planner_handle)
    follow = FakeActionClient(follow_handle)
    runtime = NavigationFacadeRuntime(planner, follow, **kwargs)
    return runtime, planner, follow


def _start(runtime, outer, executor):
    assert runtime.accept_goal(outer.request)
    return executor.create_task(runtime.execute(outer))


def _spin_until_done(executor, task, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while not task.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert task.done(), 'executor task did not finish before timeout'
    return task.result()


def _spin_until(executor, predicate, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate(), 'condition did not become true before timeout'


def _spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def _execute(runtime, outer, executor):
    return _spin_until_done(executor, _start(runtime, outer, executor))


def _no_internal_error(events):
    failures = [e for e in events if e.get('event') == 'FACADE_INTERNAL_ERROR']
    assert failures == [], failures


# --- 1. 正常 FollowPath 完成 -------------------------------------------------

def test_follow_success_finishes_outer_once(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, planner, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    result = _execute(runtime, outer, executor)
    assert outer.terminals == ['SUCCEEDED']
    assert isinstance(result, NavigateToPose.Result)
    assert len(planner.send_calls) == 1
    assert len(follow.send_calls) == 1
    assert follow.send_calls[0].controller_id == 'FollowPath'


# --- 2. 正常取消 FollowPath -------------------------------------------------

def test_cancel_during_follow_finishes_outer_canceled(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert runtime.request_cancel(outer)
    follow_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until_done(executor, task)
    assert follow_handle.cancel_calls == 1
    assert len(follow.send_calls) == 1
    assert outer.terminals == ['CANCELED']


# --- 3. 结果 Future 完成为 None ---------------------------------------------

def test_completed_follow_result_none_is_not_success(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    events = []
    runtime, _, follow = _runtime(
        planner_handle, follow_handle, diagnostic=events.append,
    )
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    follow_handle.result_future.set_result(None)
    _spin_until_done(executor, task)
    assert outer.terminals == ['ABORTED']
    assert len(follow.send_calls) == 1
    _no_internal_error(events)
    assert any(e.get('event') == 'FOLLOW_RESULT_UNVERIFIED' for e in events)


# --- 4. 取消与结果回调同时到达 ----------------------------------------------

def test_natural_success_wins_when_cancel_arrives_after_terminal(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, _, _ = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert runtime.request_cancel(outer) is False
    assert outer.terminals == ['SUCCEEDED']


# --- 5. 旧 Goal 结果晚于新 Goal 返回（本次缺陷的精确回归） -------------------

def test_stale_replaced_follow_result_does_not_abort_new_follow(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_replan = FakeGoalHandle()
    follow_first = FakeGoalHandle()
    follow_first.result_future = HeldCallbackFuture()
    follow_second = FakeGoalHandle()
    planner = FakeActionClient(goal_handles=[planner_first, planner_replan])
    follow = FakeActionClient(goal_handles=[follow_first, follow_second])
    events = []

    async def validate(path, goal):
        return {'accepted': True}

    runtime = NavigationFacadeRuntime(
        planner, follow, path_validator=validate, diagnostic=events.append,
    )
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: len(follow.send_calls) == 1)
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(planner.send_calls) == 2)
    planner_replan.result_future.set_result(
        _wrapped(GoalStatus.STATUS_SUCCEEDED, _path(1.4))
    )
    _spin_until(executor, lambda: follow_first.cancel_calls == 1)
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until(
        executor,
        lambda: (
            len(follow.send_calls) == 2
            and runtime._session is not None
            and runtime._session.follow_revision >= 2
            and runtime._session.wake_future is not None
            and len(follow_first.result_future.held) == 1
        ),
    )
    follow_first.result_future.release()
    executor.spin_once(timeout_sec=0.01)
    assert not task.done()
    assert outer.terminals == []
    _no_internal_error(events)
    follow_second.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']
    _no_internal_error(events)


# --- 6. 连续多次重规划 ------------------------------------------------------

def test_multiple_consecutive_replans_replace_each_follow_once(executor):
    planner_handles = [FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))]
    follow_handles = [FakeGoalHandle()]
    for _index in range(3):
        planner_handles.append(FakeGoalHandle())
        follow_handles.append(FakeGoalHandle())
    planner = FakeActionClient(goal_handles=planner_handles)
    follow = FakeActionClient(goal_handles=follow_handles)

    async def validate(path, goal):
        return {'accepted': True}

    runtime = NavigationFacadeRuntime(planner, follow, path_validator=validate)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    for index in range(3):
        _spin_until(executor, lambda index=index: len(follow.send_calls) == index + 1)
        assert runtime.request_replan()
        _spin_until(executor, lambda index=index: len(planner.send_calls) == index + 2)
        planner_handles[index + 1].result_future.set_result(
            _wrapped(GoalStatus.STATUS_SUCCEEDED, _path(1.0 + 0.4 * (index + 1)))
        )
        _spin_until(executor, lambda index=index: follow_handles[index].cancel_calls == 1)
        follow_handles[index].result_future.set_result(
            _wrapped(GoalStatus.STATUS_CANCELED)
        )
        _spin_until(executor, lambda index=index: len(follow.send_calls) == index + 2)
    assert follow_handles[3].cancel_calls == 0
    follow_handles[3].result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']
    for handle in follow_handles[:3]:
        assert handle.cancel_calls == 1
    assert len(planner.send_calls) == 4
    assert len(follow.send_calls) == 4


# --- 7. 重复取消（幂等） ----------------------------------------------------

def test_repeated_cancel_requests_are_idempotent(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert runtime.request_cancel(outer)
    assert runtime.request_cancel(outer)
    _spin_until(executor, lambda: follow_handle.cancel_calls == 1)
    follow_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until_done(executor, task)
    assert follow_handle.cancel_calls == 1
    assert len(follow.send_calls) == 1
    assert outer.terminals == ['CANCELED']


# --- 8. Action Server 不可用 ------------------------------------------------

def test_follow_server_unavailable_aborts_without_second_owner(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    planner = FakeActionClient(planner_handle)
    follow = FakeActionClient(follow_handle, ready=False)
    runtime = NavigationFacadeRuntime(planner, follow)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert follow.send_calls == []
    assert not runtime.busy


# --- 9. Planner Future 被取消后返回 None（第二条 NoneType 路径） -------------

def test_cancelled_planner_future_is_not_read_as_a_result(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_replan = FakeGoalHandle()
    follow_first = FakeGoalHandle()
    planner = FakeActionClient(goal_handles=[planner_first, planner_replan])
    follow = FakeActionClient(goal_handles=[follow_first])
    events = []

    async def validate(path, goal):
        return {'accepted': True}

    runtime = NavigationFacadeRuntime(
        planner, follow, path_validator=validate, diagnostic=events.append,
    )
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: len(follow.send_calls) == 1)
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(planner.send_calls) == 2)
    # rclpy Future.cancel() 使 done() 为 False 但 cancelled() 为 True，
    # result() 返回 None；旧代码在此直接访问 wrapped.status 崩溃。
    planner_replan.result_future.cancel()
    assert planner_replan.result_future.cancelled()
    _spin_for(executor, 0.1)
    assert not task.done()
    assert outer.terminals == []
    assert follow_first.cancel_calls == 0
    assert len(follow.send_calls) == 1
    _no_internal_error(events)
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']
    _no_internal_error(events)


# --- 10. Future 抛异常：fail-closed，绝不当成功 -----------------------------

def test_follow_result_exception_never_reports_success(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    follow_handle.result_future.set_exception(RuntimeError('transport'))
    _spin_for(executor, 0.2)
    assert outer.terminals == []
    assert runtime.busy
    assert not task.done()
    assert len(follow.send_calls) == 1
    task.cancel()


# --- 11. Mission Manager 只收到一个终止状态 ---------------------------------

def test_outer_terminal_is_emitted_exactly_once(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, _, _ = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    runtime._finish(outer, 'abort')
    runtime._finish(outer, 'cancel')
    assert outer.terminals == ['SUCCEEDED']


# --- GOTO：Goal Checker 选择契约（GOTO 用 general，PATROL 用 patrol） --------

def test_goto_and_patrol_select_their_own_goal_checkers(executor):
    cases = (
        (_goal(), 'general-checker'),
        (_polygon_goal(), 'patrol-checker'),
    )
    for request, expected in cases:
        planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
        follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
        runtime, _planner, follow = _runtime(
            planner_handle,
            follow_handle,
            goal_checker_id='general-checker',
            polygon_goal_checker_id='patrol-checker',
        )
        _execute(runtime, FakeOuterGoal(request), executor)
        assert follow.send_calls[0].goal_checker_id == expected


def test_goto_keeps_default_checker_when_polygon_checker_is_unset(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, _planner, follow = _runtime(
        planner_handle, follow_handle, goal_checker_id='general-checker',
    )
    _execute(runtime, FakeOuterGoal(_goal()), executor)
    assert follow.send_calls[0].goal_checker_id == 'general-checker'


# --- GOTO：到点判据与容差配置 ----------------------------------------------

def _goal_checker_reached(dx, dy, dyaw, xy_tolerance, yaw_tolerance):
    """nav2_controller::SimpleGoalChecker 的到点判据（XY 与 Yaw 同时满足）。"""
    return (math.hypot(dx, dy) <= xy_tolerance
            and abs(dyaw) <= yaw_tolerance)


def _controller_parameters():
    document = yaml.safe_load(NAV2_PARAMS.read_text(encoding='utf-8'))
    return document['controller_server']['ros__parameters']


def test_goto_tolerances_are_confined_to_the_navigate_to_pose_checker():
    controller = _controller_parameters()
    assert controller['current_goal_checker'] == 'general_goal_checker'
    assert controller['goal_checker_plugins'] == [
        'general_goal_checker', 'patrol_goal_checker',
    ]
    general = controller['general_goal_checker']
    assert general['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert general['stateful'] is True
    assert general['xy_goal_tolerance'] == pytest.approx(0.30)
    assert general['yaw_goal_tolerance'] == pytest.approx(0.35)
    patrol = controller['patrol_goal_checker']
    assert patrol['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert patrol['stateful'] is True
    assert patrol['xy_goal_tolerance'] == pytest.approx(0.10)
    assert patrol['yaw_goal_tolerance'] == pytest.approx(0.35)


@pytest.mark.parametrize(
    'dx,dy,dyaw,expected',
    [
        (0.299, 0.0, 0.0, True),      # XY 内边界
        (0.301, 0.0, 0.0, False),     # XY 外边界
        (0.0, 0.0, 0.349, True),      # Yaw 内边界
        (0.0, 0.0, 0.351, False),     # Yaw 外边界
        (0.20, 0.20, 0.10, True),     # 同时满足 XY/Yaw
        (0.20, 0.20, 0.40, False),    # 仅满足 XY，不满足 Yaw
        (0.40, 0.0, 0.10, False),     # 仅满足 Yaw，不满足 XY
    ],
)
def test_goto_goal_checker_boundaries(dx, dy, dyaw, expected):
    controller = _controller_parameters()
    general = controller['general_goal_checker']
    reached = _goal_checker_reached(
        dx, dy, dyaw,
        general['xy_goal_tolerance'], general['yaw_goal_tolerance'],
    )
    assert reached is expected


@pytest.mark.parametrize(
    'distance,yaw_error',
    [
        (0.2627, -0.1441),   # r3 实测最近接近点（map 帧 TF）
        (0.2792, -0.2107),   # r3 实测前一个满足点
        (0.2704, -0.1441),   # Facade 自身 TF 距离最小值
    ],
)
def test_measured_closest_approach_now_completes_goto(distance, yaw_error):
    """2026-11-04 r3 实测点：旧 0.25/0.25 不通过，新 0.30/0.35 通过。"""
    assert not _goal_checker_reached(0.0, distance, yaw_error, 0.25, 0.25)
    assert _goal_checker_reached(0.0, distance, yaw_error, 0.30, 0.35)


def test_patrol_goal_checker_is_stricter_than_goto_in_xy():
    controller = _controller_parameters()
    assert (controller['patrol_goal_checker']['xy_goal_tolerance']
            < controller['general_goal_checker']['xy_goal_tolerance'])


FROZEN_SIMULATION_LIB = Path(
    '/home/d1zzy/code/cleannav_modularization/cleannav-experiments/'
    'cleannav-patrol-pillar-v1-e2e-mask-lifecycle-freshness-fix-20261104T031832Z/'
    'install/cleannav_simulation/lib/cleannav_simulation')


def test_runtime_profile_generation_preserves_goto_and_patrol_checkers(tmp_path):
    """launch 的 profile 生成器只改写 FollowPath，Goal Checker 必须原样传递。"""
    if not FROZEN_SIMULATION_LIB.is_dir():
        pytest.skip('frozen cleannav_simulation prefix is unavailable')
    import json

    sys.path.insert(0, str(FROZEN_SIMULATION_LIB))
    try:
        from mapping_support import apply_mppi_motion_profile
    finally:
        sys.path.pop(0)
    profile = json.loads((
        FROZEN_SIMULATION_LIB.parents[1]
        / 'share/cleannav_simulation/config/motion_profiles/patrol_030_candidate.json'
    ).read_text(encoding='utf-8'))
    generated = apply_mppi_motion_profile(
        NAV2_PARAMS, tmp_path / 'nav2_params_patrol_030_candidate.yaml', profile)
    controller = yaml.safe_load(
        Path(generated).read_text(encoding='utf-8')
    )['controller_server']['ros__parameters']
    assert controller['general_goal_checker']['xy_goal_tolerance'] == pytest.approx(0.30)
    assert controller['general_goal_checker']['yaw_goal_tolerance'] == pytest.approx(0.35)
    assert controller['patrol_goal_checker']['xy_goal_tolerance'] == pytest.approx(0.10)
    assert controller['patrol_goal_checker']['yaw_goal_tolerance'] == pytest.approx(0.35)
    follow = controller['FollowPath']
    assert follow['vx_max'] == pytest.approx(0.30)
    assert follow['wz_max'] == pytest.approx(0.26785714285714285)
    assert follow['AckermannConstraints']['min_turning_r'] == pytest.approx(1.12)







