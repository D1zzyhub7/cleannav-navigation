"""Navigation facade V1 的无 Gazebo、无 DDS 单元测试。"""

from __future__ import annotations

import inspect
import itertools
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Path as PathMessage
from cleannav_interfaces.action import NavigateWithinPolygon
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
    _Session,
    goal_distance_from_transform,
    waypoint_stop_proof,
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
                goals_canceling=[
                    SimpleNamespace(goal_id=self.goal_id),
                ],
            )
        future.set_result(response)
        return future


class DeferredFirstCallbackFuture(Future):
    """Expose a completed result before its first observer callback runs."""

    def __init__(self):
        super().__init__()
        self.deferred_callbacks = []

    def add_done_callback(self, callback, **kwargs):
        if not self.deferred_callbacks:
            self.deferred_callbacks.append(callback)
            return
        return super().add_done_callback(callback, **kwargs)


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


def test_cancel_in_follow_readiness_check_prevents_follow_dispatch(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, _planner, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    def ready_with_cancel():
        assert runtime.request_cancel(outer)
        return True
    follow.server_is_ready = ready_with_cancel
    _execute(runtime, outer, executor)
    assert follow.send_calls == []
    assert outer.terminals == ['CANCELED']
    assert not runtime.busy


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
    assert planner.send_calls[0].goal == outer.request.pose
    assert planner.send_calls[0].goal is not outer.request.pose
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


@pytest.mark.parametrize('stage', ['planner', 'follow'])
@pytest.mark.parametrize('raises', [False, True])
def test_ambiguous_goal_submit_keeps_ownership_closed(executor, stage, raises):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, planner, follow = _runtime(planner_handle, None)
    client = planner if stage == 'planner' else follow

    response_futures = []

    def ambiguous_send(_goal, **_kwargs):
        future = Future()
        response_futures.append(future)
        return future

    client.send_goal_async = ambiguous_send
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: bool(response_futures))
    if stage == 'follow':
        if raises:
            response_futures[0].set_exception(RuntimeError('goal reply lost'))
        else:
            response_futures[0].set_result(None)
        _spin_until(
            executor,
            lambda: task.done() or (
                runtime._session is not None
                and runtime._session.submission_unverified
            ),
        )
        assert not task.done()
        assert outer.terminals == []
        assert runtime.busy
        assert not runtime.accept_goal(FakeOuterGoal().request)
    else:
        if raises:
            response_futures[0].set_exception(RuntimeError('goal reply lost'))
        else:
            response_futures[0].set_result(None)
        _spin_until_done(executor, task)
        assert outer.terminals == ['ABORTED']
        assert runtime.busy
        assert not runtime.accept_goal(FakeOuterGoal().request)


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
    assert len(follow.send_calls) == (4 if follow_status == GoalStatus.STATUS_ABORTED else 1)


def test_replan_replaces_follow_only_after_old_terminal_and_revalidates_path(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_replan = FakeGoalHandle()
    follow_first = FakeGoalHandle()
    follow_second = FakeGoalHandle()
    planner = FakeActionClient(goal_handles=[planner_first, planner_replan])
    follow = FakeActionClient(goal_handles=[follow_first, follow_second])
    validations = []

    async def validate(path, goal):
        validations.append((path, goal))
        return {'path_sha256': 'offline-proof', 'accepted': True}

    runtime = NavigationFacadeRuntime(planner, follow, path_validator=validate)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: len(follow.send_calls) == 1)
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(planner.send_calls) == 2)
    planner_replan.result_future.set_result(
        _wrapped(GoalStatus.STATUS_SUCCEEDED, _path(1.4))
    )
    _spin_until(executor, lambda: follow_first.cancel_calls == 1)
    assert len(follow.send_calls) == 1
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until(executor, lambda: len(follow.send_calls) == 2)
    assert follow.send_calls[1].path.poses[-1].pose.position.x == pytest.approx(1.4)
    assert len(validations) == 2
    follow_second.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']
    assert len(planner.send_calls) == 2


def test_unchanged_replan_keeps_current_follow_goal(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_replan = FakeGoalHandle()
    follow_first = FakeGoalHandle()
    runtime, _, _ = _runtime(planner_first, follow_first)
    runtime._planner_client = FakeActionClient(goal_handles=[planner_first, planner_replan])
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    runtime._planner_client.goal_handles = [planner_replan]
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(runtime._planner_client.send_calls) == 2)
    planner_replan.result_future.set_result(
        _wrapped(GoalStatus.STATUS_SUCCEEDED, _path())
    )
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert follow_first.cancel_calls == 0
    assert len(runtime._follow_client.send_calls) == 1
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']


def test_near_goal_feedback_suppresses_periodic_replan_and_reaches_outer(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    runtime, planner, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    callback = follow.feedback_callbacks[0]
    callback(SimpleNamespace(
        goal_id=follow_handle.goal_id,
        feedback=SimpleNamespace(
            distance_to_goal=0.12,
            speed=0.05,
            current_pose=PoseStamped(),
        ),
    ))
    assert len(outer.feedback) == 1
    assert outer.feedback[0].distance_remaining == pytest.approx(0.12)
    assert runtime.request_replan() is False
    assert len(planner.send_calls) == 1
    follow_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']


def test_replan_gate_uses_fresh_tf_instead_of_zero_follow_feedback():
    runtime, _, _ = _runtime(
        None, None, goal_distance_provider=lambda _goal: 1.25,
    )
    session = _Session(
        outer_goal_handle=object(), original_goal=_goal().pose,
        stage=_Stage.FOLLOWING, distance_to_goal=0.0,
    )
    runtime._session = session

    assert runtime.request_replan() is True
    assert session.distance_to_goal == pytest.approx(1.25)


def test_unavailable_tf_does_not_reuse_zero_feedback_to_suppress_replan():
    runtime, _, _ = _runtime(
        None, None, goal_distance_provider=lambda _goal: None,
    )
    session = _Session(
        outer_goal_handle=object(), original_goal=_goal().pose,
        stage=_Stage.FOLLOWING, distance_to_goal=0.0,
    )
    runtime._session = session

    assert runtime.request_replan() is True
    assert session.distance_to_goal is None


@pytest.mark.parametrize(
    'stamp_ns, expected',
    [
        (10_000_000_000, pytest.approx(0.7810249676)),
        (9_400_000_000, None),
        (10_400_000_001, None),
    ],
)
def test_goal_distance_requires_recent_transform(stamp_ns, expected):
    goal = _goal().pose
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 0.4
    transform.transform.translation.y = 1.5

    result = goal_distance_from_transform(goal, transform, 10_000_000_000)
    if expected is None:
        assert result is None
    else:
        assert result == expected


def test_outer_cancel_during_replan_stops_follow_and_ignores_late_plan(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_late = FakeGoalHandle()
    follow_first = FakeGoalHandle()
    planner = FakeActionClient(goal_handles=[planner_first, planner_late])
    follow = FakeActionClient(goal_handles=[follow_first, FakeGoalHandle()])
    runtime = NavigationFacadeRuntime(planner, follow)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: len(follow.send_calls) == 1)
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(planner.send_calls) == 2)
    assert runtime.request_cancel(outer)
    _spin_until(executor, lambda: follow_first.cancel_calls == 1)
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until(executor, lambda: follow_first.result_future.done())
    assert not task.done()
    assert outer.terminals == []
    assert len(follow.send_calls) == 1
    assert runtime.busy  # late planner result still owns the unresolved inner action
    planner_late.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['CANCELED']
    _spin_until(executor, lambda: not runtime.busy)
    assert len(follow.send_calls) == 1


def test_failed_periodic_plan_does_not_cancel_the_good_active_path(executor):
    planner_first = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_failed = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_ABORTED))
    follow_first = FakeGoalHandle()
    planner = FakeActionClient(goal_handles=[planner_first, planner_failed])
    follow = FakeActionClient(follow_first)
    runtime = NavigationFacadeRuntime(planner, follow)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: len(follow.send_calls) == 1)
    assert runtime.request_replan()
    _spin_until(executor, lambda: len(planner.send_calls) == 2)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert follow_first.cancel_calls == 0
    assert len(follow.send_calls) == 1
    follow_first.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['SUCCEEDED']


def test_follow_result_transport_unknown_holds_outer_owner_and_lease(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle()
    follow_handle.result_future.set_exception(RuntimeError('result transport lost'))
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: follow_handle.cancel_calls >= 1)
    assert not task.done()
    assert outer.terminals == []
    assert runtime.busy
    assert len(follow.send_calls) == 1
    assert not runtime.accept_goal(_goal())
    assert runtime.request_cancel(outer)
    _spin_until(executor, lambda: follow_handle.cancel_calls >= 2)
    assert not task.done()
    assert outer.terminals == []


def test_finalizer_observes_all_completed_inner_actions(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    planner_handle.result_future = DeferredFirstCallbackFuture()
    planner_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle.result_future = DeferredFirstCallbackFuture()
    follow_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_SUCCEEDED))
    runtime, _, _ = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['SUCCEEDED']
    assert not runtime.busy
    assert runtime._session is None


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
    assert runtime.busy
    assert not runtime.accept_goal(_goal())
    planner_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_ABORTED))
    assert not runtime.busy


def test_cancel_failure_cannot_overlap_a_second_follow_path(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(cancel_response=SimpleNamespace(
        return_code=CancelGoal.Response.ERROR_REJECTED, goals_canceling=[],
    ))
    runtime, _, follow = _runtime(planner_handle, follow_handle)
    outer = FakeOuterGoal()
    task = _start(runtime, outer, executor)
    _spin_until(executor, lambda: runtime.active_stage is _Stage.FOLLOWING)
    assert runtime.request_cancel(outer)
    _spin_until(executor, lambda: follow_handle.cancel_calls >= 1)
    assert not task.done()
    follow_handle.result_future.set_result(_wrapped(GoalStatus.STATUS_CANCELED))
    _spin_until_done(executor, task)
    assert outer.terminals == ['ABORTED']
    assert not runtime.busy
    assert len(follow.send_calls) == 1


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


@pytest.mark.parametrize(
    ('goal_request', 'expected_checker'),
    [
        (_goal(), 'general-checker'),
        (NavigateWithinPolygon.Goal(), 'patrol-checker'),
    ],
)
def test_polygon_action_uses_separate_goal_checker_without_changing_goto(
    executor, goal_request, expected_checker,
):
    if isinstance(goal_request, NavigateWithinPolygon.Goal):
        goal_request.pose.header.frame_id = 'map'
        goal_request.pose.pose.orientation.w = 1.0
        goal_request.pose.pose.position.x = 1.0
    planner_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    follow_handle = FakeGoalHandle(
        result=_wrapped(GoalStatus.STATUS_SUCCEEDED)
    )
    runtime, _planner, follow = _runtime(
        planner_handle,
        follow_handle,
        goal_checker_id='general-checker',
        polygon_goal_checker_id='patrol-checker',
    )
    _execute(runtime, FakeOuterGoal(goal_request), executor)
    assert follow.send_calls[0].goal_checker_id == expected_checker


def test_patrol_plan_uses_fresh_actual_start_snapshot(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    events = []
    actual = PoseStamped()
    actual.header.frame_id = 'map'
    actual.header.stamp.sec = 17
    actual.pose.position.x = .42
    actual.pose.position.y = .73
    actual.pose.orientation.w = 1.

    async def start_snapshot(goal):
        assert goal.header.frame_id == 'map'
        return actual, dict(actual_start_xy_m=[.42, .73], costmap_sha256='fresh')

    runtime, planner, _follow = _runtime(
        planner_handle, follow_handle,
        start_snapshot_provider=start_snapshot,
        diagnostic=events.append,
    )
    request = NavigateWithinPolygon.Goal()
    request.pose = _goal().pose
    outer = FakeOuterGoal(request)
    _execute(runtime, outer, executor)

    assert outer.terminals == ['SUCCEEDED']
    assert len(planner.send_calls) == 1
    assert planner.send_calls[0].use_start is True
    assert planner.send_calls[0].start.header.frame_id == 'map'
    assert planner.send_calls[0].start.pose.position.x == pytest.approx(.42)
    assert planner.send_calls[0].start.pose.position.y == pytest.approx(.73)
    assert any(event['event'] == 'PLAN_START_CAPTURED' for event in events)


def test_patrol_outer_success_waits_for_stop_guard_after_follow_terminal(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    events = []
    outer = FakeOuterGoal()

    async def guard(goal, checker, terminal):
        assert goal == outer.request.pose
        assert checker == 'patrol-checker'
        assert terminal > 0
        assert outer.terminals == []
        events.append('guard')
        return dict(stop_evidence=dict(ready=True))

    runtime, _planner, _follow = _runtime(
        planner_handle, follow_handle,
        polygon_goal_checker_id='patrol-checker',
        post_success_guard=guard,
        diagnostic=lambda item: events.append(item.get('event')),
    )
    request = NavigateWithinPolygon.Goal()
    request.pose = _goal().pose
    outer = FakeOuterGoal(request)
    _execute(runtime, outer, executor)
    assert outer.terminals == ['SUCCEEDED']
    assert events.index('FOLLOW_ACTION_TERMINAL') < events.index('guard')
    assert events.index('guard') < events.index('OUTER_TERMINAL')


def test_failed_stop_guard_fails_closed_without_reporting_navigation_success(executor):
    planner_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    follow_handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    events = []

    async def guard(_goal, _checker, _terminal):
        raise TimeoutError('no settled odometry')

    runtime, _planner, _follow = _runtime(
        planner_handle, follow_handle,
        post_success_guard=guard,
        diagnostic=events.append,
    )
    outer = FakeOuterGoal()
    _execute(runtime, outer, executor)
    assert outer.terminals == ['ABORTED']
    assert any(event['event'] == 'WAYPOINT_STOP_STABILITY_FAILED' for event in events)


def test_waypoint_stop_proof_matches_existing_zero_and_settling_bounds():
    terminal = 10.0
    terminal_ros = 10_000_000_000
    samples = [(terminal_ros+i*500_000_000, terminal+i*.5, .01, .01)
               for i in range(4)]
    proof = waypoint_stop_proof(
        samples,
        now_ros_ns=11_500_000_000,
        now_monotonic=12.0,
        terminal_monotonic=terminal,
        terminal_ros_ns=terminal_ros,
        zero_since=9.0,
        last_command_received=11.9,
        localization={'localized': True, 'map_tf_valid': True},
        localization_received=11.9,
    )
    assert proof['ready'] is True
    assert proof['odometry_sample_count'] == 4
    assert proof['odometry_span_s'] == pytest.approx(1.5)
    assert proof['final_command_zero_s'] == pytest.approx(2.0)


@pytest.mark.parametrize('failure', ['moving', 'gap', 'stale_localization', 'nonzero_command'])
def test_waypoint_stop_proof_rejects_incomplete_or_unsafe_evidence(failure):
    terminal = 10.0
    terminal_ros = 10_000_000_000
    samples = [(terminal_ros+i*500_000_000, terminal+i*.5, .01, .01)
               for i in range(4)]
    localization = {'localized': True}
    zero_since = 9.0
    localization_received = 11.9
    if failure == 'moving':
        samples[1] = (samples[1][0], samples[1][1], .03, .0)
    elif failure == 'gap':
        samples[2] = (terminal_ros+1_100_000_000, terminal+1.0, .01, .01)
    elif failure == 'stale_localization':
        localization_received = 10.0
    elif failure == 'nonzero_command':
        zero_since = 10.1
    proof = waypoint_stop_proof(
        samples,
        now_ros_ns=11_500_000_000,
        now_monotonic=12.0,
        terminal_monotonic=terminal,
        terminal_ros_ns=terminal_ros,
        zero_since=zero_since,
        last_command_received=11.9,
        localization=localization,
        localization_received=localization_received,
    )
    assert proof['ready'] is False


def test_source_boundary_has_no_control_or_business_side_effects(executor):
    source = Path(__file__).resolve().parents[1] / 'navigation_facade_node.py'
    text = source.read_text(encoding='utf-8')
    for forbidden in (
        'Ackermann',
        'Hybrid-A*',
        'MPPI',
        'Safety',
        'HTTP',
        'APP',
        'task_id',
    ):
        assert forbidden not in text
    assert "create_publisher(Path, '/cleannav/navigation/planner_path'" in text
    assert "Twist, '/cmd_vel'" in text
    assert 'create_publisher(Twist' not in text
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


def test_diagnostic_exception_cannot_prevent_inner_terminal_observation():
    runtime = NavigationFacadeRuntime(None, None, diagnostic=lambda _: (_ for _ in ()).throw(RuntimeError('log sink failed')))
    handle = FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_CANCELED))
    session = SimpleNamespace(inner_goal_handle=handle, inner_terminal=False, terminal=False, stage=_Stage.FOLLOWING)
    runtime._observe_inner_terminal(session, handle, handle.result_future)
    assert session.inner_terminal


@pytest.mark.parametrize('outcome', ['proof', 'missing', 'cancel'])
def test_stationary_equal_goal_has_no_inner_owner_or_late_cancel(executor, outcome):
    runtime, planner, follow = _runtime(
        FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED)),
        FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED)))
    outer = FakeOuterGoal()
    def checker(goal):
        if outcome == 'cancel':
            assert runtime.request_cancel(outer)
        return None if outcome == 'missing' else dict(position_error_m=0.)
    runtime._stationary_goal_checker = checker
    task = _start(runtime, outer, executor)
    _spin_until_done(executor, task)
    assert outer.terminals == ['CANCELED' if outcome == 'cancel' else 'SUCCEEDED']
    assert len(planner.send_calls) == (1 if outcome == 'missing' else 0)
    assert len(follow.send_calls) == (1 if outcome == 'missing' else 0)
    assert not runtime.busy
    assert runtime.accept_goal(_goal())


@pytest.mark.parametrize('invalid', ['valid','stale_tf','future_tf','stale_odom',
    'missing','wrong_frame','wrong_child','zero_stamp','moving','turning','nan',
    'zero_quaternion','far','heading','future_small','nonplanar'])
def test_stationary_goal_requires_fresh_finite_stationary_pose(invalid):
    import copy, math
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from navigation_facade_node import stationary_goal_proof
    goal = _goal().pose
    tf = TransformStamped();tf.header.frame_id='map';tf.child_frame_id='base_link'
    tf.header.stamp.sec=10;tf.transform.translation.x=1.;tf.transform.translation.y=2.
    tf.transform.rotation.w=1.
    odom=Odometry();odom.header.stamp.sec=10;odom.child_frame_id='base_link'
    if invalid=='stale_tf':tf.header.stamp.sec=9
    if invalid=='future_tf':tf.header.stamp.sec=11
    if invalid=='stale_odom':odom.header.stamp.sec=9
    if invalid=='missing':odom=None
    if invalid=='wrong_frame':tf.header.frame_id='odom'
    if invalid=='wrong_child':odom.child_frame_id='odom'
    if invalid=='zero_stamp':tf.header.stamp.sec=0
    if invalid=='moving':odom.twist.twist.linear.x=.02
    if invalid=='turning':odom.twist.twist.angular.z=.02
    if invalid=='nan':odom.twist.twist.linear.x=float('nan')
    if invalid=='zero_quaternion':tf.transform.rotation.w=0.
    if invalid=='far':tf.transform.translation.x+=.1
    if invalid=='heading':tf.transform.rotation.z=math.sin(.1);tf.transform.rotation.w=math.cos(.1)
    if invalid=='future_small':tf.header.stamp.nanosec=150000000
    if invalid=='nonplanar':tf.transform.rotation.x=.2
    proof=stationary_goal_proof(goal,tf,odom,10100000000)
    assert (proof is not None) == (invalid in ('valid','future_small'))


@pytest.mark.parametrize('offset,linear,wz,accepted', [
    (None,0.,0.,False),(.3,0.,0.,True),(.3,.009,.01,False),
    (.3,.005,.005,True),(float('nan'),0.,0.,False)])
def test_stationary_rear_axle_twist_requires_known_transform(offset,linear,wz,accepted):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from navigation_facade_node import stationary_goal_proof
    goal=_goal().pose
    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='base_link';tf.header.stamp.sec=10
    tf.transform.translation.x=1.;tf.transform.translation.y=2.;tf.transform.rotation.w=1.
    odom=Odometry();odom.header.stamp.sec=10;odom.child_frame_id='ackermann_rear_axle'
    odom.twist.twist.linear.x=linear;odom.twist.twist.angular.z=wz
    assert (stationary_goal_proof(goal,tf,odom,10100000000,odom_child_offset_m=offset) is not None)==accepted
