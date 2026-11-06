"""Input adapter timeout and ownership tests, without navigation execution."""

import ast
from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
import yaml

ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location('rviz_input_test', ROOT/'scripts/rviz_goal_adapter.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def adapter():
    mod = module()
    rclpy.init()
    node = mod.RvizGoalAdapter()
    reports = []
    node.report = lambda event, cid, **kw: reports.append((event, cid, kw))
    try:
        yield mod, node, reports
    finally:
        node.destroy_node()
        rclpy.shutdown()


def goal():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 10
    msg.pose.position.x = .8
    msg.pose.orientation.w = 1.
    return msg


def test_same_click_event_sends_once(adapter):
    _, node, reports = adapter
    calls, future = [], Future()
    node.client = NS(service_is_ready=lambda: True,
                     call_async=lambda req: (calls.append(req), future)[1])
    node.goal(goal())
    node.goal(goal())
    node.poll()
    node.poll()
    assert len(calls) == 1 and reports[0][0] == 'CLICK'
    assert calls[0].goal.header.frame_id == 'map'


def test_unavailable_service_times_out_without_send(adapter, monkeypatch):
    mod, node, reports = adapter
    now = [1.]
    monkeypatch.setattr(mod.time, 'monotonic', lambda: now[0])
    node.client = NS(service_is_ready=lambda: False)
    node.goal(goal())
    now[0] = 5.
    node.poll()
    assert reports[-1][0] == 'SERVICE_UNAVAILABLE' and not node.pending


def test_sent_timeout_retains_late_response_and_does_not_retry(adapter, monkeypatch):
    mod, node, reports = adapter
    now = [1.]
    monkeypatch.setattr(mod.time, 'monotonic', lambda: now[0])
    calls, future = [], Future()
    node.client = NS(service_is_ready=lambda: True,
                     call_async=lambda req: (calls.append(req), future)[1])
    node.goal(goal())
    node.poll()
    now[0] = 5.
    node.poll()
    assert reports[-1][0] == 'SERVICE_TIMEOUT' and len(node.pending) == 1
    future.set_result(NS(accepted=True, task_id=40, execution_id='execution',
                         reason='COMMAND_QUEUED', reason_code=3))
    node.poll()
    assert reports[-1][0] == 'ACCEPTED' and reports[-1][2]['late']
    assert len(calls) == 1 and not node.pending


def test_service_exception_reports_failure(adapter):
    _, node, reports = adapter
    future = Future()
    node.client = NS(service_is_ready=lambda: True, call_async=lambda req: future)
    node.goal(goal())
    node.poll()
    future.set_exception(RuntimeError('transport failed'))
    node.poll()
    assert reports[-1][0] == 'SERVICE_ERROR' and not node.pending


def test_namespaced_input_and_no_control_or_lease_clients():
    mod = module()
    assert mod.TOPIC == '/cleannav/rviz/goal_pose'
    tree = ast.parse((ROOT/'scripts/rviz_goal_adapter.py').read_text())
    strings = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
               and isinstance(n.value, str)}
    assert strings.isdisjoint({'/goal_pose', '/follow_path', '/cmd_vel',
                              '/cleannav/cmd_vel_candidate', '/cleannav/navigate_to_pose'})
    assert 'SafetyLease' not in (ROOT/'scripts/rviz_goal_adapter.py').read_text()
    config = yaml.safe_load((ROOT/'config/simulation_v2_goals.rviz').read_text())
    tool = next(t for t in config['Visualization Manager']['Tools']
                if t['Class'] == 'rviz_default_plugins/SetGoal')
    assert tool['Topic'] == mod.TOPIC
    default = yaml.safe_load((ROOT/'config/simulation_v2.rviz').read_text())
    assert not any(t['Class'].endswith('/SetGoal')
                   for t in default['Visualization Manager']['Tools'])


def test_abandoned_transport_future_is_removed_without_retry(adapter, monkeypatch):
    mod, node, reports = adapter
    now = [1.]
    monkeypatch.setattr(mod.time, 'monotonic', lambda: now[0])
    future, removed = Future(), []
    node.client = NS(service_is_ready=lambda: True, call_async=lambda req: future,
                     remove_pending_request=removed.append)
    node.goal(goal())
    node.poll()
    now[0] = 5.
    node.poll()
    now[0] = 40.
    node.poll()
    assert removed == [future] and not node.pending
    assert sum(e[0] == 'SERVICE_TIMEOUT' for e in reports) == 1
