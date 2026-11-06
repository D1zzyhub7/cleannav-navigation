"""Path Executor latest-wins FollowPath hot-swap 状态机单元测试。"""

from concurrent.futures import Future
from pathlib import Path as FilePath
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from cleannav_path_executor import path_executor_node as executor_module


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeGoalHandle:
    _next_id = 0

    def __init__(self, accepted=True, cancel_exception=None):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_calls = 0
        self.cancel_exception = cancel_exception
        self.goal_id = UUID()
        self.goal_id.uuid[0] = FakeGoalHandle._next_id
        FakeGoalHandle._next_id += 1

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_exception is not None:
            raise self.cancel_exception
        future = Future()
        future.set_result(SimpleNamespace(goals_canceling=[object()]))
        return future

    def finish(self, status):
        self.result_future.set_result(SimpleNamespace(status=status))


class FakeActionClient:
    def __init__(self, *_args, **_kwargs):
        self.ready = True
        self.send_calls = []
        self.response_futures = []
        self.feedback_callbacks = []

    def wait_for_server(self, timeout_sec=0.0):
        del timeout_sec
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.send_calls.append(goal)
        future = Future()
        self.response_futures.append(future)
        self.feedback_callbacks.append(feedback_callback)
        return future

    def respond(self, index, goal_handle):
        self.response_futures[index].set_result(goal_handle)

    def feedback(self, index, distance):
        callback = self.feedback_callbacks[index]
        feedback = FollowPath.Impl.FeedbackMessage()
        feedback.feedback.distance_to_goal = distance
        feedback.goal_id = self.response_futures[index].result().goal_id
        callback(feedback)


class FakeTransformBuffer:
    def __init__(self, translation=None, error=None):
        self.translation = (0.0, 0.0) if translation is None else translation
        self.error = error
        self.lookup_calls = []

    def lookup_transform(self, target_frame, source_frame, time):
        self.lookup_calls.append((target_frame, source_frame, time))
        if self.error is not None:
            raise self.error
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(
                    x=self.translation[0], y=self.translation[1])))


def _path(offset=0.0):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp.sec = 8
    for x in (offset, offset + 1.0, offset + 2.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 9
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    monkeypatch.setattr(executor_module, 'ActionClient', FakeActionClient)
    value = executor_module.PathExecutorNode()
    value._status_pub = CapturePublisher()
    value._tf_buffer = FakeTransformBuffer()
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def _activate(node, path=None, handle=None):
    path = _path() if path is None else path
    handle = FakeGoalHandle() if handle is None else handle
    node._path_cb(path)
    node._action_client.respond(len(node._action_client.send_calls) - 1, handle)
    return handle


def test_first_path_is_sent_normally_and_zero_stamped(node):
    node._path_cb(_path())

    assert len(node._action_client.send_calls) == 1
    goal = node._action_client.send_calls[0]
    assert goal.path.header.stamp.sec == 0
    assert all(pose.header.stamp.sec == 0 for pose in goal.path.poses)


def test_near_goal_feedback_publishes_identity_tagged_status(node):
    _activate(node, _path(0.0))
    node._tf_buffer.translation = (1.75, 0.0)

    node._action_client.feedback(0, 0.25)

    assert node._status_pub.messages[-1].data.startswith(
        'follow_path_near_goal:')


def test_send_goal_registers_feedback_callback_and_real_message_shape(node):
    path = _path()
    node._path_cb(path)
    handle = FakeGoalHandle()
    node._action_client.respond(0, handle)

    assert node._action_client.feedback_callbacks[0] == node._feedback_callback

    far_feedback = FollowPath.Impl.FeedbackMessage()
    far_feedback.goal_id = handle.goal_id
    far_feedback.feedback.distance_to_goal = 0.2501
    node._action_client.feedback_callbacks[0](far_feedback)
    assert all(
        not message.data.startswith('follow_path_near_goal:')
        for message in node._status_pub.messages)

    near_feedback = FollowPath.Impl.FeedbackMessage()
    near_feedback.goal_id = handle.goal_id
    node._tf_buffer.translation = (1.75, 0.0)
    near_feedback.feedback.distance_to_goal = 0.25
    node._action_client.feedback_callbacks[0](near_feedback)

    assert node._status_pub.messages[-1].data == (
        'follow_path_near_goal:'
        + executor_module._path_signature(path))


def test_far_feedback_does_not_enter_completion_window(node):
    _activate(node)
    node._tf_buffer.translation = (0.0, 0.0)

    node._action_client.feedback(0, 0.26)

    assert all(
        not message.data.startswith('follow_path_near_goal:')
        for message in node._status_pub.messages)


def test_replacement_refreshes_feedback_identity(node):
    active = _activate(node, _path())
    first_callback = node._action_client.feedback_callbacks[0]
    node._tf_buffer.translation = (1.75, 0.0)
    node._action_client.feedback(0, 0.20)

    node._path_cb(_path(10.0))
    active.finish(GoalStatus.STATUS_CANCELED)
    replacement = FakeGoalHandle()
    node._action_client.respond(1, replacement)
    node._tf_buffer.translation = (11.75, 0.0)

    assert node._action_client.feedback_callbacks[1] == node._feedback_callback
    node._action_client.feedback(1, 0.20)
    assert node._status_pub.messages[-1].data == (
        'follow_path_near_goal:'
        + executor_module._path_signature(_path(10.0)))
    assert first_callback == node._feedback_callback


def test_xy_distance_triggers_even_when_path_arc_length_is_long(node):
    path = _path()
    _activate(node, path)
    node._tf_buffer.translation = (1.9, 0.0)

    node._action_client.feedback(0, 0.9)

    assert node._status_pub.messages[-1].data == (
        'follow_path_near_goal:' + executor_module._path_signature(path))


def test_short_path_arc_does_not_trigger_when_xy_is_far(node):
    path = _path()
    _activate(node, path)
    node._tf_buffer.translation = (0.0, 0.0)

    node._action_client.feedback(0, 0.05)

    assert all(
        not message.data.startswith('follow_path_near_goal:')
        for message in node._status_pub.messages)


def test_near_goal_is_latched_once_for_active_path(node):
    _activate(node, _path())
    node._tf_buffer.translation = (1.8, 0.0)

    node._action_client.feedback(0, 0.9)
    node._action_client.feedback(0, 0.1)

    near = [
        message for message in node._status_pub.messages
        if message.data.startswith('follow_path_near_goal:')
    ]
    assert len(near) == 1


def test_tf_unavailable_does_not_publish_or_crash(node):
    _activate(node, _path())
    node._tf_buffer = FakeTransformBuffer(error=executor_module.tf2_ros.LookupException())

    node._action_client.feedback(0, 0.1)

    assert all(
        not message.data.startswith('follow_path_near_goal:')
        for message in node._status_pub.messages)


def test_late_old_feedback_cannot_mark_replacement_path(node):
    active = _activate(node, _path())
    old_feedback = FollowPath.Impl.FeedbackMessage()
    old_feedback.goal_id = active.goal_id
    old_feedback.feedback.distance_to_goal = 0.1

    node._path_cb(_path(10.0))
    active.finish(GoalStatus.STATUS_CANCELED)
    replacement = FakeGoalHandle()
    node._action_client.respond(1, replacement)
    node._tf_buffer.translation = (0.0, 0.0)
    node._feedback_callback(old_feedback)

    assert all(
        not message.data.startswith('follow_path_near_goal:')
        for message in node._status_pub.messages)


def test_new_path_while_active_becomes_pending_and_requests_cancel(node):
    active = _activate(node)

    node._path_cb(_path(10.0))

    assert active.cancel_calls == 1
    assert node._active_goal_handle is active
    assert node._pending_path[1] == executor_module._path_signature(_path(10.0))
    assert len(node._action_client.send_calls) == 1


def test_paths_during_cancel_are_latest_wins_without_repeated_cancel(node):
    active = _activate(node)
    node._path_cb(_path(10.0))
    node._path_cb(_path(20.0))
    node._path_cb(_path(30.0))

    assert active.cancel_calls == 1
    assert node._pending_path[1] == executor_module._path_signature(_path(30.0))


def test_replacement_is_not_sent_before_old_terminal(node):
    active = _activate(node)
    node._path_cb(_path(10.0))

    assert node._active_goal_handle is active
    assert len(node._action_client.send_calls) == 1


def test_canceled_old_goal_advances_latest_pending_path(node):
    active = _activate(node)
    node._path_cb(_path(10.0))

    active.finish(GoalStatus.STATUS_CANCELED)

    assert len(node._action_client.send_calls) == 2
    assert node._active_goal_handle is None
    assert node._sending_signature == executor_module._path_signature(
        _path(10.0))


@pytest.mark.parametrize(
    'terminal_status',
    [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED],
)
def test_cancel_race_terminal_also_advances_pending(node, terminal_status):
    active = _activate(node)
    node._path_cb(_path(10.0))

    active.finish(terminal_status)

    assert len(node._action_client.send_calls) == 2
    if terminal_status == GoalStatus.STATUS_SUCCEEDED:
        assert node._status_pub.messages[-2].data.startswith(
            'follow_path_replaced')


def test_replan_failure_does_not_cancel_active_follow_path(node):
    active = _activate(node)

    node._status_cb(String(data='replan fail (Smac Hybrid): no path'))

    assert active.cancel_calls == 0
    assert node._active_goal_handle is active


def test_hard_astar_failure_still_cancels_active_follow_path(node):
    active = _activate(node)

    node._status_cb(String(data='A* fail (Smac Hybrid): no path'))

    assert active.cancel_calls == 1
    assert node._active_goal_handle is active
    assert node._status_pub.messages[-1].data == (
        'cancel_requested_due_to_planner_status')


def test_active_and_pending_signatures_are_deduplicated(node):
    active_path = _path()
    pending_path = _path(10.0)
    active = _activate(node, active_path)

    node._path_cb(active_path)
    assert active.cancel_calls == 0
    node._path_cb(pending_path)
    node._path_cb(pending_path)

    assert active.cancel_calls == 1
    assert node._pending_path[1] == executor_module._path_signature(pending_path)


def test_same_signature_can_run_again_after_terminal(node):
    path = _path()
    active = _activate(node, path)
    active.finish(GoalStatus.STATUS_ABORTED)

    node._path_cb(path)

    assert len(node._action_client.send_calls) == 2


def test_success_status_contains_active_path_identity(node):
    active = _activate(node)
    active.finish(GoalStatus.STATUS_SUCCEEDED)

    status = node._status_pub.messages[-1].data
    assert status.startswith('follow_path_succeeded:')
    assert status.endswith(executor_module._path_signature(_path()))


def test_abort_status_contains_active_path_identity(node):
    active = _activate(node)
    active.finish(GoalStatus.STATUS_ABORTED)

    status = node._status_pub.messages[-1].data
    assert status.startswith('follow_path_aborted:')
    assert executor_module._path_signature(_path()) in status


def test_cancel_exception_keeps_active_goal_and_blocks_replacement(node):
    active = FakeGoalHandle(cancel_exception=RuntimeError('cancel failed'))
    _activate(node, handle=active)

    node._path_cb(_path(10.0))

    assert node._active_goal_handle is active
    assert node._pending_path is not None
    assert len(node._action_client.send_calls) == 1
    assert node._status_pub.messages[-1].data == 'follow_path_cancel_failed'


def test_update_while_send_response_pending_has_no_dual_active_goal(node):
    node._path_cb(_path())
    node._path_cb(_path(10.0))

    assert len(node._action_client.send_calls) == 1
    first = FakeGoalHandle()
    node._action_client.respond(0, first)
    assert first.cancel_calls == 1
    assert len(node._action_client.send_calls) == 1

    first.finish(GoalStatus.STATUS_CANCELED)

    assert len(node._action_client.send_calls) == 2


def test_source_has_no_blocking_spin_wait_loop(node):
    source = (
        FilePath(__file__).resolve().parents[1]
        / 'cleannav_path_executor'
        / 'path_executor_node.py'
    ).read_text(encoding='utf-8')

    assert 'spin_once' not in source
    assert '_last_sent_signature' not in source


def test_launch_exposes_goal_checker_aligned_tf_parameters(node):
    launch_source = (
        FilePath(__file__).resolve().parents[1]
        / 'launch'
        / 'cleannav_path_executor.launch.py'
    ).read_text(encoding='utf-8')

    assert "default_value='0.25'" in launch_source
    assert "default_value='base_link'" in launch_source
    assert "'robot_base_frame': robot_base_frame" in launch_source
