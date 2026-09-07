"""Navigation facade V1 的无 Gazebo、无 DDS 单元测试。"""

from __future__ import annotations

import inspect
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Path as PathMessage
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

SOURCE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE_DIR))

from navigation_facade_node import (  # noqa: E402
    DEFAULT_CONTROLLER_ID,
    DEFAULT_GOAL_CHECKER_ID,
    DEFAULT_PLANNER_ID,
    DEFAULT_USE_START,
    FACADE_ACTION_NAME,
    FOLLOW_ACTION_NAME,
    PLANNER_ACTION_NAME,
    NavigationFacadeRuntime,
    _Stage,
)


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


def _path() -> PathMessage:
    path = PathMessage()
    path.header.frame_id = 'map'
    for x in (0.0, 1.0):
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
    def __init__(self, *, accepted=True, result=None, cancel_response=None):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=bytes(range(16)))
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
                goals_canceling=[
                    SimpleNamespace(goal_id=self.goal_id),
                ],
            )
        future.set_result(response)
        return future


class FakeActionClient:
    def __init__(self, goal_handle=None, *, ready=True):
        self.goal_handle = goal_handle
        self.ready = ready
        self.send_calls = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.send_calls.append(goal)
        future = Future()
        future.set_result(self.goal_handle)
        return future


class FakeOuterGoal:
    def __init__(self):
        self.request = _goal()
        self.terminals = []

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


def _spin_until_done(executor, task, timeout_sec=1.0):
    deadline = time.monotonic() + timeout_sec
    while not task.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert task.done(), 'executor task did not finish before timeout'
    return task.result()


def _spin_until(executor, predicate, timeout_sec=1.0):
    deadline = time.monotonic() + timeout_sec
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate(), 'condition did not become true before timeout'


def _execute(runtime, outer, executor):
    return _spin_until_done(executor, _start(runtime, outer, executor))


def test_outer_endpoint_and_action_type_are_frozen(executor):
    assert FACADE_ACTION_NAME == '/cleannav/navigate_to_pose'
    assert FACADE_ACTION_NAME != '/navigate_to_pose'
    assert hasattr(NavigateToPose.Goal(), 'pose')
    assert hasattr(NavigateToPose.Result(), 'result')


def test_inner_endpoints_and_verified_defaults_are_frozen(executor):
    assert PLANNER_ACTION_NAME == '/compute_path_to_pose'
    assert FOLLOW_ACTION_NAME == '/follow_path'
    assert DEFAULT_PLANNER_ID == 'GridBased'
    assert DEFAULT_USE_START is False
    assert DEFAULT_CONTROLLER_ID == 'FollowPath'
    assert DEFAULT_GOAL_CHECKER_ID == ''
    assert hasattr(ComputePathToPose.Goal(), 'use_start')
    assert hasattr(FollowPath.Goal(), 'goal_checker_id')


def test_invalid_outer_pose_is_rejected(executor):
    runtime = NavigationFacadeRuntime(FakeActionClient(), FakeActionClient())
    request = _goal()
    request.pose.header.frame_id = ''
    assert runtime.accept_goal(request) is False
    assert runtime.busy is False


def test_second_outer_goal_is_rejected_without_preemption(executor):
    planner_handle = FakeGoalHandle()
    runtime, planner, follow = _runtime(planner_handle, None)
    first = FakeOuterGoal()
    second = FakeOuterGoal()
    task = _start(runtime, first, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.PLANNING)
    assert runtime.accept_goal(second.request) is False
    assert runtime.request_cancel(second) is False
    planner_handle.result_future.set_result(
        _wrapped(GoalStatus.STATUS_CANCELED)
    )
    _spin_until_done(executor, task)
    assert planner.send_calls
    assert follow.send_calls == []


def test_planner_success_is_forwarded_to_follow_and_follow_success_finishes(
    executor,
):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED, _path())
    )
    planner_handle.result_future.result().result.path.header.stamp.sec = 7
    planner_handle.result_future.result().result.path.poses[0].header.stamp.sec = 8
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, planner, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['SUCCEEDED']
    assert len(planner.send_calls) == 1
    assert planner.send_calls[0].goal is outer.request.pose
    assert planner.send_calls[0].planner_id == 'GridBased'
    assert planner.send_calls[0].use_start is False
    assert len(follow.send_calls) == 1
    assert len(follow.send_calls[0].path.poses) == 2
    assert follow.send_calls[0].path.header.stamp.sec == 0
    assert follow.send_calls[0].path.poses[0].header.stamp.sec == 0
    assert follow.send_calls[0].controller_id == 'FollowPath'
    assert follow.send_calls[0].goal_checker_id == ''


@pytest.mark.parametrize(
    'planner_status',
    [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED],
)
def test_planner_non_success_finishes_outer_aborted(planner_status, executor):
    planner_handle = FakeGoalHandle(result=_wrapped(planner_status))
    runtime, planner, follow = _runtime(planner_handle, None)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert len(planner.send_calls) == 1
    assert follow.send_calls == []


def test_planner_rejected_finishes_outer_aborted(executor):
    planner_handle = FakeGoalHandle(accepted=False)
    runtime, planner, follow = _runtime(planner_handle, None)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert len(planner.send_calls) == 1
    assert follow.send_calls == []


def test_planner_empty_path_finishes_outer_aborted(executor):
    empty = PathMessage()
    empty.header.frame_id = 'map'
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED, empty)
    )
    runtime, _, follow = _runtime(planner_handle, None)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert follow.send_calls == []


@pytest.mark.parametrize(
    'follow_status',
    [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED],
)
def test_follow_non_success_finishes_outer_aborted(follow_status, executor):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(result=_wrapped(follow_status))
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert len(follow.send_calls) == 1


def test_follow_rejected_finishes_outer_aborted(executor):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(accepted=False)
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert len(follow.send_calls) == 1


def test_planner_unavailable_aborts_without_goal_submit(executor):
    runtime, planner, follow = _runtime(None, None)
    planner.ready = False
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert planner.send_calls == []
    assert follow.send_calls == []


def test_rclpy_executor_regression_has_no_running_event_loop_dependency(executor):
    runtime, planner, follow = _runtime(None, None)
    planner.ready = False
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until_done(executor, task)
    assert outer.terminals == ['ABORTED']
    assert planner.send_calls == []
    assert follow.send_calls == []


def test_follow_unavailable_aborts_after_planning(executor):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, _, follow = _runtime(planner_handle, None)
    follow.ready = False
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert follow.send_calls == []


def test_cancel_during_planning_requires_inner_cancel_and_terminal_canceled(
    executor,
):
    planner_handle = FakeGoalHandle()
    runtime, _, follow = _runtime(planner_handle, None)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.PLANNING)
    assert runtime.request_cancel(outer)
    planner_handle.result_future.set_result(
        _wrapped(GoalStatus.STATUS_CANCELED)
    )
    _spin_until_done(executor, task)
    assert planner_handle.cancel_calls == 1
    assert outer.terminals == ['CANCELED']
    assert follow.send_calls == []


def test_cancel_during_follow_requires_follow_cancel_and_terminal_canceled(
    executor,
):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle()
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert runtime.request_cancel(outer)
    follow_handle.result_future.set_result(
        _wrapped(GoalStatus.STATUS_CANCELED)
    )
    _spin_until_done(executor, task)
    assert follow_handle.cancel_calls == 1
    assert len(follow.send_calls) == 1
    assert outer.terminals == ['CANCELED']


@pytest.mark.parametrize(
    'cancel_response',
    [
        SimpleNamespace(
            return_code=CancelGoal.Response.ERROR_REJECTED,
            goals_canceling=[],
        ),
        SimpleNamespace(
            return_code=CancelGoal.Response.ERROR_NONE,
            goals_canceling=[],
        ),
    ],
)
def test_inner_cancel_failure_finishes_outer_aborted(cancel_response, executor):
    planner_handle = FakeGoalHandle(cancel_response=cancel_response)
    runtime, _, follow = _runtime(planner_handle, None)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.PLANNING)
    assert runtime.request_cancel(outer)
    _spin_until_done(executor, task)
    assert planner_handle.cancel_calls == 1
    assert outer.terminals == ['ABORTED']
    assert follow.send_calls == []


class _CancelBeforeFollowRuntime(NavigationFacadeRuntime):
    async def _wait_inner_result(self, session, inner_goal_handle):
        outcome, result = await super()._wait_inner_result(
            session,
            inner_goal_handle,
        )
        if session.stage is _Stage.PLANNING:
            with self._lock:
                session.cancel_requested = True
        return outcome, result


def test_cancel_between_planner_result_and_follow_submit_does_not_submit_follow(
    executor,
):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    planner = FakeActionClient(planner_handle)
    follow = FakeActionClient(follow_handle)
    runtime = _CancelBeforeFollowRuntime(planner, follow)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['CANCELED']
    assert follow.send_calls == []


def test_natural_follow_success_wins_when_cancel_arrives_after_terminal_result(
    executor,
):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, _, _ = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert runtime.request_cancel(outer) is False
    assert outer.terminals == ['SUCCEEDED']


def test_terminal_state_is_emitted_once(executor):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, _, _ = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    runtime._finish(outer, 'abort')
    runtime._finish(outer, 'cancel')
    assert outer.terminals == ['SUCCEEDED']


def test_custom_ids_are_forwarded_without_task_specific_logic(executor):
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, planner, follow = _runtime(
        planner_handle,
        follow_handle,
        planner_id='CustomPlanner',
        use_start=True,
        controller_id='CustomController',
        goal_checker_id='CustomChecker',
    )
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert planner.send_calls[0].planner_id == 'CustomPlanner'
    assert planner.send_calls[0].use_start is True
    assert follow.send_calls[0].controller_id == 'CustomController'
    assert follow.send_calls[0].goal_checker_id == 'CustomChecker'


def test_source_boundary_has_no_control_or_business_side_effects(executor):
    source = Path(__file__).resolve().parents[1] / 'navigation_facade_node.py'
    text = source.read_text(encoding='utf-8')
    for forbidden in (
        'Ackermann',
        'Hybrid-A*',
        'MPPI',
        'cmd_vel',
        'Safety',
        'HTTP',
        'APP',
        'task_id',
        'create_publisher',
    ):
        assert forbidden not in text
    assert 'FollowPath' in text
    assert 'ComputePathToPose' in text


def test_source_has_no_asyncio_runtime_dependencies(executor):
    source = Path(__file__).resolve().parents[1] / 'navigation_facade_node.py'
    text = source.read_text(encoding='utf-8')
    for forbidden in (
        'asyncio',
        'get_running_loop',
        'asyncio.Event',
        'asyncio.create_task',
        'asyncio.ensure_future',
        'asyncio.wait',
        'call_soon_threadsafe',
    ):
        assert forbidden not in text
    assert 'from rclpy.task import Future' in text


def test_runtime_is_async_and_does_not_use_blocking_spin_or_status_strings(
    executor,
):
    from navigation_facade_node import NavigationFacadeRuntime

    assert inspect.iscoroutinefunction(NavigationFacadeRuntime.execute)
    source = inspect.getsource(NavigationFacadeRuntime)
    assert 'spin_once' not in source
    assert 'path_executor_status' not in source
    assert 'follow_path_succeeded' not in source
