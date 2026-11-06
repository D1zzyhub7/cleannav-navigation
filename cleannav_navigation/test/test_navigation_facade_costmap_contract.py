"""Production Facade costmap snapshot and path admission contract regression."""
from __future__ import annotations

import hashlib
import importlib
import os
import sys
import time
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as PathMessage
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

SOURCE_DIR = Path(__file__).resolve().parents[1]
if os.environ.get('CLEAN_NAV_TEST_INSTALLED_MODULE') == '1':
    from ament_index_python.packages import get_package_prefix

    INSTALLED_FACADE_DIR = Path(get_package_prefix('cleannav_navigation')) / 'lib/cleannav_navigation'
    sys.path.insert(0, str(INSTALLED_FACADE_DIR))
else:
    sys.path.insert(0, str(SOURCE_DIR))

from cleannav_global_planner.path_safety import Grid  # noqa: E402
from boundary_runtime import PolygonNavigationMixin  # noqa: E402
_navigation_module = importlib.import_module('navigation_facade_node')
NavigationFacadeNode = _navigation_module.NavigationFacadeNode
if os.environ.get('CLEAN_NAV_TEST_INSTALLED_MODULE') == '1':
    assert Path(_navigation_module.__file__).resolve() == (
        INSTALLED_FACADE_DIR / 'navigation_facade_node.py').resolve()


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


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _ReadyCostmapClient:
    def __init__(self, response):
        self.response = response
        self.requests = []
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        assert isinstance(request, GetCostmap.Request)
        self.requests.append(request)
        future = Future()
        future.set_result(self.response)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class _FacadeHarness:
    """Provide only ROS node services; invoke production Facade methods unmodified."""

    def __init__(self, response):
        self._costmap_client = _ReadyCostmapClient(response)
        self._path_evidence = _Publisher()
        self._admitted_evidence = _Publisher()
        self._boundary = None  # GOTO_REGION has no active polygon boundary.
        self._boundary_token = ''

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_200_000_000))

    def create_timer(self, *_args, **_kwargs):
        return object()

    def destroy_timer(self, _timer):
        pass

    def get_parameter(self, name):
        assert name == 'm0_forward_only'
        return SimpleNamespace(value=False)



def _costmap(*, blocked_cost=None):
    from nav2_msgs.msg import Costmap

    value = Costmap()
    value.header.frame_id = 'map'
    value.metadata.update_time.sec = 100
    value.metadata.update_time.nanosec = 0
    value.metadata.size_x = 120
    value.metadata.size_y = 120
    value.metadata.resolution = 0.05
    value.metadata.origin.position.x = -3.0
    value.metadata.origin.position.y = -3.0
    value.metadata.origin.orientation.w = 1.0
    data = [0] * (value.metadata.size_x * value.metadata.size_y)
    if blocked_cost is not None:
        # A cell inside the swept vehicle footprint on the path.
        data[60 * value.metadata.size_x + 60] = blocked_cost
    value.data = data
    response = GetCostmap.Response()
    response.map = value
    return response


def _path():
    value = PathMessage()
    value.header.frame_id = 'map'
    for x in (-0.5, 0.5):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        value.poses.append(pose)
    return value


def _goal():
    value = PoseStamped()
    value.header.frame_id = 'map'
    value.pose.position.x = 0.5
    value.pose.orientation.w = 1.0
    return value


def _spin(executor, awaitable):
    task = executor.create_task(awaitable)
    deadline = time.monotonic() + 2.0
    while not task.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert task.done(), 'production Facade coroutine did not finish'
    return task.result()


def _harness(response):
    value = _FacadeHarness(response)
    value._request_costmap_snapshot = MethodType(
        NavigationFacadeNode._request_costmap_snapshot, value)
    value._validate_boundary_path = MethodType(
        PolygonNavigationMixin._validate_boundary_path, value)
    return value


def test_actual_grid_snapshot_and_complete_facade_path_admission(executor):
    response = _costmap()
    harness = _harness(response)

    grid, snapshot = _spin(
        executor, NavigationFacadeNode._request_costmap_snapshot(harness, 'map'))

    assert type(grid) is Grid
    assert grid.frame == 'map'
    assert not hasattr(grid, 'frame_id')
    assert snapshot['costmap_frame'] == 'map'
    assert snapshot['costmap_sha256'] == hashlib.sha256(bytes(response.map.data)).hexdigest()
    assert snapshot['costmap_age_s'] == pytest.approx(0.2)

    proof = _spin(
        executor,
        NavigationFacadeNode._validate_controller_path(harness, _path(), _goal()),
    )

    assert proof['mode'] == 'GEOMETRY'
    assert proof['rectangles'] > 0
    assert proof['path_sha256']
    assert proof['costmap_sha256'] == snapshot['costmap_sha256']
    assert len(harness._path_evidence.messages) == 1
    assert len(harness._admitted_evidence.messages) == 1


@pytest.mark.parametrize('blocked_cost', [254, 255], ids=['lethal', 'unknown'])
def test_facade_rejects_blocked_or_unknown_costmap_and_never_admits_path(executor, blocked_cost):
    harness = _harness(_costmap(blocked_cost=blocked_cost))

    with pytest.raises(ValueError, match=r'(footprint blocked|path center buffer blocked)'):
        _spin(
            executor,
            NavigationFacadeNode._validate_controller_path(harness, _path(), _goal()),
        )

    assert len(harness._path_evidence.messages) == 1
    assert harness._admitted_evidence.messages == []
