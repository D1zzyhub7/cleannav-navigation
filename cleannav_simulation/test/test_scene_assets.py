"""Static validation for the demo-only obstacle scene."""

from __future__ import annotations

import ast
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ElementTree

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = PACKAGE_ROOT / "models"
CONTROLLER_PATH = PACKAGE_ROOT / "scripts" / (
    "dynamic_obstacle_controller.py"
)
SCENE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / (
    "ackermann_mcity_obstacles_demo.launch.py"
)
BASE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "ackermann_mcity_demo.launch.py"
BASE_WORLD_PATH = Path(
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/worlds/mcity.world"
)


def _load_controller():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_dynamic_obstacle_controller_test",
        CONTROLLER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import controller source")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _model(path: Path):
    root = ElementTree.parse(path).getroot()
    model = root.find("model")
    assert model is not None
    return model


def test_static_obstacle_names_are_unique_and_have_scan_height():
    paths = [
        MODEL_ROOT / "cleannav_demo_static_block.sdf",
        MODEL_ROOT / "cleannav_demo_static_cylinder.sdf",
    ]
    models = [_model(path) for path in paths]
    names = [model.attrib["name"] for model in models]

    assert len(names) == len(set(names))
    assert all(model.find("static").text == "true" for model in models)
    for model in models:
        link = model.find("link")
        assert link is not None
        assert link.find("collision") is not None
        assert link.find("visual") is not None

    box_height = float(
        models[0].findtext("link/visual/geometry/box/size").split()[2]
    )
    cylinder_height = float(
        models[1].findtext(
            "link/visual/geometry/cylinder/length"
        )
    )
    assert box_height >= 0.40
    assert cylinder_height >= 0.40


def test_dynamic_sdf_and_endpoints_are_valid():
    dynamic = _model(MODEL_ROOT / "cleannav_demo_dynamic_obstacle.sdf")
    link = dynamic.find("link")
    assert dynamic.findtext("static") == "false"
    assert link is not None
    assert link.find("collision") is not None
    assert link.find("visual") is not None
    assert float(
        link.findtext("visual/geometry/box/size").split()[2]
    ) >= 0.40

    controller = _load_controller()
    endpoint_a = controller.Endpoint(4.0, -6.25, 0.40, 0.0)
    endpoint_b = controller.Endpoint(4.0, -3.75, 0.40, 0.0)
    assert controller.validate_controller_config(
        endpoint_a,
        endpoint_b,
        0.25,
        10.0,
    ) == pytest.approx(2.5)


def test_controller_reverses_between_a_and_b():
    controller = _load_controller()
    distance, direction = controller.advance_ping_pong(
        0.0,
        1,
        1.0,
        2.5,
        2.5,
    )
    assert distance == pytest.approx(2.5)
    assert direction == -1

    distance, direction = controller.advance_ping_pong(
        distance,
        direction,
        1.0,
        2.5,
        2.5,
    )
    assert distance == pytest.approx(0.0)
    assert direction == 1


def test_controller_does_not_create_cmd_vel_publisher():
    source = CONTROLLER_PATH.read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "/cmd_vel" not in source
    assert "SetEntityState" in source


def test_dynamic_controller_launch_is_independent_of_spawn_exit():
    source = SCENE_LAUNCH_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(SCENE_LAUNCH_PATH))
    call_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }
    assert "OnProcessExit" not in call_names
    assert "RegisterEventHandler" not in call_names

    same_timer_actions = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "TimerAction":
            continue
        actions = next(
            (
                keyword.value
                for keyword in node.keywords
                if keyword.arg == "actions"
            ),
            None,
        )
        if isinstance(actions, ast.List):
            same_timer_actions.append(
                {
                    item.id
                    for item in actions.elts
                    if isinstance(item, ast.Name)
                }
            )
    assert {"dynamic_spawn", "dynamic_controller"} in same_timer_actions


class _FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class _FakeFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        if isinstance(self._response, BaseException):
            raise self._response
        return self._response


class _FakeClient:
    def __init__(
        self,
        ready,
        response=None,
        responses=None,
        call_error=None,
    ):
        self.ready = ready
        self.response = response
        self.responses = list(responses or [])
        self.call_error = call_error
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        if self.call_error is not None:
            raise self.call_error
        response = self.responses.pop(0) if self.responses else self.response
        return _FakeFuture(response)


def _controller_for_timer_test(controller, client, elapsed_sec):
    instance = object.__new__(controller.DynamicObstacleController)
    instance._obstacle_name = "cleannav_demo_dynamic_obstacle"
    instance._endpoint_a = controller.Endpoint(4.0, -6.25, 0.40, 0.0)
    instance._endpoint_b = controller.Endpoint(4.0, -3.75, 0.40, 0.0)
    instance._speed = 0.25
    instance._update_rate = 10.0
    instance._segment_length = 2.5
    instance._distance = 0.0
    instance._direction = 1
    instance._pending_call = None
    instance._last_tick = time.monotonic() - elapsed_sec
    instance._last_warning_time = -float("inf")
    instance._client = client
    instance._logger = _FakeLogger()
    instance.get_logger = lambda: instance._logger
    return instance


def test_controller_retries_when_entity_is_unavailable():
    controller = _load_controller()
    client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=False),
    )
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._on_timer()
    instance._on_timer()

    assert len(client.calls) == 2
    assert instance._pending_call is None
    assert len(instance._logger.warnings) == 1
    assert "returned success=False" in instance._logger.warnings[0]


def test_controller_waits_for_service_without_exiting():
    controller = _load_controller()
    client = _FakeClient(
        ready=False,
        response=SimpleNamespace(success=True),
    )
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._on_timer()
    instance._on_timer()

    assert client.calls == []
    assert instance._pending_call is None
    assert len(instance._logger.warnings) == 1


def test_controller_recovers_after_failure_failure_success():
    controller = _load_controller()
    client = _FakeClient(
        ready=True,
        responses=[
            SimpleNamespace(success=False),
            SimpleNamespace(success=False),
            SimpleNamespace(success=True),
        ],
    )
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._on_timer()
    instance._on_timer()
    instance._on_timer()

    assert len(client.calls) == 3
    assert instance._pending_call is None
    assert instance._distance > 0.0
    assert client.calls[-1].state.pose.position.y > -6.25


def test_controller_recovers_after_future_exception():
    controller = _load_controller()
    client = _FakeClient(
        ready=True,
        responses=[
            RuntimeError("transport failure"),
            SimpleNamespace(success=True),
        ],
    )
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._on_timer()
    instance._on_timer()

    assert len(client.calls) == 2
    assert instance._pending_call is None
    assert any(
        "call failed" in message
        for message in instance._logger.warnings
    )


def test_controller_recovers_after_service_call_exception():
    controller = _load_controller()
    client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=True),
        call_error=RuntimeError("service call failure"),
    )
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._on_timer()
    client.call_error = None
    instance._on_timer()

    assert len(client.calls) == 2
    assert instance._pending_call is None
    assert any(
        "request failed" in message
        for message in instance._logger.warnings
    )


def test_controller_warning_throttle_allows_periodic_recovery_warning():
    controller = _load_controller()
    client = _FakeClient(ready=False)
    instance = _controller_for_timer_test(controller, client, 0.1)

    instance._warn_throttled("first")
    instance._warn_throttled("suppressed")
    assert instance._logger.warnings == ["first"]

    instance._last_warning_time -= controller.WARNING_THROTTLE_SEC
    instance._warn_throttled("after throttle")
    assert instance._logger.warnings == ["first", "after throttle"]


def test_controller_updates_motion_after_entity_becomes_available():
    controller = _load_controller()
    client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=True),
    )
    instance = _controller_for_timer_test(controller, client, 0.4)

    instance._on_timer()

    assert len(client.calls) == 1
    assert instance._pending_call is None
    assert instance._distance > 0.0
    assert client.calls[0].state.name == "cleannav_demo_dynamic_obstacle"
    assert client.calls[0].state.pose.position.y > -6.25


def test_scene_flags_and_launch_sources_are_valid_python():
    for path in (BASE_LAUNCH_PATH, SCENE_LAUNCH_PATH):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        compile(path.read_text(encoding="utf-8"), str(path), "exec")

    scene_source = SCENE_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"enable_static_obstacles"' in scene_source
    assert '"enable_dynamic_obstacle"' in scene_source
    assert '"dynamic_obstacle_speed"' in scene_source
    assert "ackermann_mcity_demo.launch.py" in scene_source
    assert "-slibgazebo_ros_state.so" not in scene_source


def test_world_overlay_injects_exactly_one_state_plugin_and_preserves_base(
    tmp_path,
):
    if not BASE_WORLD_PATH.is_file():
        pytest.skip("external mcity demo asset is not installed")

    base_bytes_before = BASE_WORLD_PATH.read_bytes()
    overlay_path = tmp_path / "mcity_overlay.world"
    second_overlay_path = tmp_path / "mcity_overlay_second.world"
    repeated_overlay_path = tmp_path / "mcity_overlay_repeated.world"

    module = _load_scene_launch()
    module.generate_mcity_overlay_world(BASE_WORLD_PATH, overlay_path)
    module.generate_mcity_overlay_world(
        BASE_WORLD_PATH,
        second_overlay_path,
    )
    module.generate_mcity_overlay_world(
        overlay_path,
        repeated_overlay_path,
    )

    assert BASE_WORLD_PATH.read_bytes() == base_bytes_before
    overlay_root = ElementTree.parse(overlay_path).getroot()
    second_root = ElementTree.parse(second_overlay_path).getroot()
    base_root = ElementTree.parse(BASE_WORLD_PATH).getroot()
    overlay_world = overlay_root.find("world")
    second_world = second_root.find("world")
    base_world = base_root.find("world")
    assert overlay_world is not None
    assert second_world is not None
    assert base_world is not None
    assert overlay_world.attrib["name"] == base_world.attrib["name"]
    assert {
        model.attrib.get("name")
        for model in base_world.findall("model")
    } == {
        model.attrib.get("name")
        for model in overlay_world.findall("model")
    }
    assert {
        include.findtext("uri")
        for include in base_world.findall("include")
    } <= {
        include.findtext("uri")
        for include in overlay_world.findall("include")
    }
    assert len(overlay_world.findall("plugin")) == (
        len(base_world.findall("plugin")) + 1
    )
    assert sum(
        Path(plugin.attrib.get("filename", "")).name
        == "libgazebo_ros_state.so"
        for plugin in overlay_world.findall("plugin")
    ) == 1
    state_plugin = next(
        plugin
        for plugin in overlay_world.findall("plugin")
        if Path(plugin.attrib.get("filename", "")).name
        == "libgazebo_ros_state.so"
    )
    assert state_plugin.attrib["name"] == "cleannav_gazebo_ros_state"
    assert state_plugin.find("ros") is None
    assert state_plugin.find("update_rate") is None
    assert overlay_path.read_bytes() == second_overlay_path.read_bytes()
    repeated_world = ElementTree.parse(repeated_overlay_path).getroot().find(
        "world"
    )
    assert repeated_world is not None
    assert sum(
        Path(plugin.attrib.get("filename", "")).name
        == "libgazebo_ros_state.so"
        for plugin in repeated_world.findall("plugin")
    ) == 1


def _load_scene_launch():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_mcity_obstacles_demo_launch_test",
        SCENE_LAUNCH_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import scene launch")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_scene_launch_imports_and_constructs():
    module = _load_scene_launch()
    description = module.generate_launch_description()
    assert description is not None
