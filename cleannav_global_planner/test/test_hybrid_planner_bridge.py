"""Hybrid Planner Bridge 周期重规划状态机单元测试。"""

from concurrent.futures import Future
from pathlib import Path as FilePath
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from cleannav_global_planner import hybrid_planner_bridge_node as bridge_module


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        future = Future()
        future.set_result(SimpleNamespace(goals_canceling=[object()]))
        return future

    def finish(self, status, path=None):
        result_path = _path() if path is None else path
        self.result_future.set_result(SimpleNamespace(
            status=status,
            result=SimpleNamespace(path=result_path),
        ))


class FakeActionClient:
    def __init__(self, *_args, **_kwargs):
        self.ready = True
        self.send_calls = []
        self.response_futures = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.send_calls.append(goal)
        future = Future()
        self.response_futures.append(future)
        return future

    def respond(self, index, goal_handle):
        self.response_futures[index].set_result(goal_handle)


def _goal(x=1.0):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.orientation.w = 1.0
    return goal


def _path(offset=0.0):
    path = Path()
    path.header.frame_id = 'map'
    for x in (offset, offset + 1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    monkeypatch.setattr(bridge_module, 'ActionClient', FakeActionClient)
    value = bridge_module.HybridPlannerBridgeNode()
    value._path_pub = CapturePublisher()
    value._status_pub = CapturePublisher()
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def _complete_request(node, index, status=GoalStatus.STATUS_SUCCEEDED):
    handle = FakeGoalHandle()
    node._action_client.respond(index, handle)
    goal_x = node._action_client.send_calls[index].goal.pose.position.x
    path = _path(goal_x)
    handle.finish(status, path=path)
    return handle


def _published_path_signature(node):
    return bridge_module._path_signature(node._path_pub.messages[-1])


def _executor_status(node, status, signature=None):
    if signature is None:
        signature = _published_path_signature(node)
    node._executor_status_callback(
        String(data=f'{status}:{signature}'))


def test_new_goal_plans_immediately(node):
    node._goal_callback(_goal(2.0))

    assert len(node._action_client.send_calls) == 1
    request = node._action_client.send_calls[0]
    assert request.goal.pose.position.x == 2.0
    assert request.planner_id == 'GridBased'
    assert request.use_start is False
    assert node._generation == 1


def test_success_then_timer_replans_same_final_goal(node):
    node._goal_callback(_goal(3.0))
    _complete_request(node, 0)

    node._replan_timer_callback()

    assert len(node._path_pub.messages) == 1
    assert len(node._action_client.send_calls) == 2
    assert node._action_client.send_calls[1].goal.pose.position.x == 3.0
    assert node._generation == 1


def test_timer_does_not_start_concurrent_compute_path(node):
    node._goal_callback(_goal())

    node._replan_timer_callback()

    assert len(node._action_client.send_calls) == 1
    assert node._replan_deferred is True


def test_multiple_timer_ticks_collapse_to_one_deferred_replan(node):
    node._goal_callback(_goal())
    node._replan_timer_callback()
    node._replan_timer_callback()
    node._replan_timer_callback()

    _complete_request(node, 0)

    assert len(node._action_client.send_calls) == 2
    assert node._replan_deferred is False


def test_new_goal_gates_old_result_and_waits_for_old_terminal(node):
    node._goal_callback(_goal(1.0))
    node._goal_callback(_goal(9.0))
    old_handle = FakeGoalHandle()

    assert len(node._action_client.send_calls) == 1
    node._action_client.respond(0, old_handle)
    assert old_handle.cancel_calls == 1
    assert len(node._action_client.send_calls) == 1

    old_handle.finish(GoalStatus.STATUS_CANCELED)

    assert node._path_pub.messages == []
    assert len(node._action_client.send_calls) == 2
    assert node._action_client.send_calls[1].goal.pose.position.x == 9.0


def test_first_planning_failure_remains_hard_failure(node):
    node._goal_callback(_goal())
    _complete_request(node, 0, GoalStatus.STATUS_ABORTED)

    assert node._status_pub.messages[-1].data.startswith(
        'A* fail (Smac Hybrid):')


def test_periodic_failure_after_success_is_non_danger_and_keeps_goal(node):
    node._goal_callback(_goal(4.0))
    _complete_request(node, 0)
    node._replan_timer_callback()
    _complete_request(node, 1, GoalStatus.STATUS_ABORTED)

    assert node._status_pub.messages[-1].data.startswith(
        'replan fail (Smac Hybrid):')
    assert node._final_goal.pose.position.x == 4.0


def test_follow_path_success_clears_final_goal_and_stops_replan(node):
    node._goal_callback(_goal())
    _complete_request(node, 0)
    sends_before_success = len(node._action_client.send_calls)

    _executor_status(node, 'follow_path_succeeded')
    node._replan_timer_callback()

    assert node._final_goal is None
    assert len(node._action_client.send_calls) == sends_before_success


def test_follow_path_abort_keeps_final_goal(node):
    node._goal_callback(_goal(5.0))
    _complete_request(node, 0)

    _executor_status(node, 'follow_path_aborted')

    assert node._final_goal.pose.position.x == 5.0
    assert len(node._action_client.send_calls) == 2


def test_far_from_goal_keeps_periodic_replanning(node):
    node._goal_callback(_goal(5.0))
    _complete_request(node, 0)

    node._replan_timer_callback()

    assert len(node._action_client.send_calls) == 2


def test_completion_window_suppresses_periodic_replacement(node):
    node._goal_callback(_goal(5.0))
    _complete_request(node, 0)
    _executor_status(node, 'follow_path_near_goal')

    node._replan_timer_callback()

    assert node._completion_window_signature == _published_path_signature(node)
    assert len(node._action_client.send_calls) == 1


def test_completion_window_success_clears_final_goal(node):
    node._goal_callback(_goal(5.0))
    _complete_request(node, 0)
    _executor_status(node, 'follow_path_near_goal')
    _executor_status(node, 'follow_path_succeeded')

    assert node._final_goal is None


def test_completion_window_abort_immediately_restores_replanning(node):
    node._goal_callback(_goal(5.0))
    _complete_request(node, 0)
    _executor_status(node, 'follow_path_near_goal')
    _executor_status(node, 'follow_path_aborted')

    assert node._final_goal is not None
    assert len(node._action_client.send_calls) == 2


def test_stale_success_from_old_final_goal_cannot_clear_new_goal(node):
    node._goal_callback(_goal(1.0))
    _complete_request(node, 0)
    old_signature = _published_path_signature(node)

    node._goal_callback(_goal(9.0))
    _executor_status(node, 'follow_path_succeeded', old_signature)

    assert node._final_goal.pose.position.x == 9.0
    assert node._generation == 2


def test_new_final_goal_success_completes_normally(node):
    node._goal_callback(_goal(1.0))
    _complete_request(node, 0)
    old_signature = _published_path_signature(node)
    node._goal_callback(_goal(9.0))
    _complete_request(node, 1)
    new_signature = _published_path_signature(node)

    _executor_status(node, 'follow_path_succeeded', old_signature)
    assert node._final_goal is not None
    _executor_status(node, 'follow_path_succeeded', new_signature)

    assert node._final_goal is None


def test_non_positive_period_preserves_single_plan_behavior(node):
    node._replan_period_sec = 0.0
    node._goal_callback(_goal())
    _complete_request(node, 0)

    node._replan_timer_callback()

    assert len(node._action_client.send_calls) == 1


def test_launch_chain_exposes_one_second_replan_parameter(node):
    package_root = FilePath(__file__).resolve().parents[1]
    bridge_launch = (
        package_root / 'launch' / 'cleannav_hybrid_planner_bridge.launch.py'
    ).read_text(encoding='utf-8')
    navigation_launch = (
        package_root.parent
        / 'cleannav_navigation'
        / 'launch'
        / 'cleannav_ackermann_navigation.launch.py'
    ).read_text(encoding='utf-8')
    executor_launch = (
        package_root.parent
        / 'cleannav_path_executor'
        / 'launch'
        / 'cleannav_path_executor.launch.py'
    ).read_text(encoding='utf-8')

    assert "'replan_period_sec'" in bridge_launch
    assert "default_value='1.0'" in bridge_launch
    assert "'replan_period_sec': replan_period_sec" in navigation_launch
    assert "default_value='0.25'" in executor_launch
    assert "'completion_window_xy_tolerance': completion_window_xy_tolerance" in navigation_launch
