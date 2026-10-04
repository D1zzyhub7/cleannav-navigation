"""Static validation for the demo-only obstacle scene."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import subprocess
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
BICYCLE_CONTROLLER_PATH = PACKAGE_ROOT / "scripts" / (
    "bicycle_patrol_controller.py"
)
CLEANUP_CONTROLLER_PATH = PACKAGE_ROOT / "scripts" / (
    "leaf_cleanup_demo_controller.py"
)
SHOWCASE_PERSON_CONTROLLER_PATH = PACKAGE_ROOT / "scripts" / (
    "showcase_person_controller.py"
)
SCENE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / (
    "ackermann_mcity_obstacles_demo.launch.py"
)
SHOWCASE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / (
    "ackermann_mcity_showcase_v2.launch.py"
)
BASE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "ackermann_mcity_demo.launch.py"
VEHICLE_XACRO_PATH = PACKAGE_ROOT / "urdf" / "cleannav_ackermann.urdf.xacro"
STATIC_NAV2_PARAMS_PATH = PACKAGE_ROOT.parent / "cleannav_navigation" / (
    "config/nav2_params_ackermann_static_demo.yaml"
)
BASE_WORLD_PATH = Path(
    "/home/d1zzy/code/cleannav_demo_assets/"
    "osrf_car_demo/car_demo/worlds/mcity.world"
)
VEHICLE_PHYSICAL_CONTRACT_SHA256 = (
    "8086600130de4f3fc8df405516be86dbe74a07d439080f92fce0e77e448c1496"
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


def _load_showcase_person_controller():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_showcase_person_controller_test",
        SHOWCASE_PERSON_CONTROLLER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import Showcase person controller")
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


def test_static_visuals_preserve_collision_proxies():
    block = _model(MODEL_ROOT / "cleannav_demo_static_block.sdf")
    block_link = block.find("link")
    assert block_link is not None
    block_collision = block_link.find("collision")
    assert block_collision is not None
    assert block_collision.attrib["name"] == "collision"
    assert block_collision.findtext("geometry/box/size") == "0.80 1.00 0.80"
    assert len(block_link.findall("visual")) >= 2
    assert not block_link.findall(".//mesh")

    bin_model = _model(MODEL_ROOT / "cleannav_demo_static_cylinder.sdf")
    bin_link = bin_model.find("link")
    assert bin_link is not None
    bin_collision = bin_link.find("collision")
    assert bin_collision is not None
    assert bin_collision.attrib["name"] == "collision"
    assert bin_collision.findtext("geometry/cylinder/radius") == "0.45"
    assert bin_collision.findtext("geometry/cylinder/length") == "0.80"
    assert len(bin_link.findall("visual")) >= 3
    assert not bin_link.findall(".//mesh")


def test_dynamic_sdf_and_endpoints_are_valid():
    dynamic = _model(MODEL_ROOT / "cleannav_demo_dynamic_obstacle.sdf")
    link = dynamic.find("link")
    assert dynamic.attrib["name"] == "cleannav_demo_dynamic_obstacle"
    assert dynamic.findtext("static") == "false"
    assert dynamic.findtext("allow_auto_disable") == "false"
    assert link is not None
    assert link.attrib["name"] == "link"
    assert link.findtext("gravity") == "false"
    assert link.findtext("kinematic") == "true"
    assert link.find("collision") is not None
    collision = link.find("collision")
    assert collision.attrib["name"] == "collision"
    assert collision.findtext("geometry/box/size") == "0.65 0.65 0.80"

    visuals = link.findall("visual")
    assert len(visuals) >= 5
    visual_geometry_types = {
        next(iter(visual.find("geometry"))).tag
        for visual in visuals
    }
    assert {"box", "cylinder", "sphere"} <= visual_geometry_types
    assert not link.findall(".//mesh")
    assert "model://" not in (
        MODEL_ROOT.joinpath(
            "cleannav_demo_dynamic_obstacle.sdf"
        ).read_text(encoding="utf-8")
    )

    controller = _load_controller()
    endpoint_a = controller.Endpoint(4.0, -6.25, 0.40, 0.0)
    endpoint_b = controller.Endpoint(4.0, -3.75, 0.40, 0.0)
    assert controller.validate_controller_config(
        endpoint_a,
        endpoint_b,
        0.25,
        10.0,
    ) == pytest.approx(2.5)


def test_showcase_person_is_visual_only_but_real_person_keeps_collision():
    real_person = _model(
        MODEL_ROOT / "cleannav_demo_dynamic_obstacle.sdf"
    )
    real_link = real_person.find("link")
    assert real_link is not None
    assert real_link.find("collision") is not None
    assert real_link.find("inertial") is not None

    showcase_person = _model(
        MODEL_ROOT / "cleannav_demo_showcase_person.sdf"
    )
    showcase_link = showcase_person.find("link")
    assert showcase_person.attrib["name"] == (
        "cleannav_demo_showcase_person"
    )
    assert showcase_person.findtext("static") == "false"
    assert showcase_link is not None
    assert showcase_link.findtext("gravity") == "false"
    assert showcase_link.findtext("kinematic") == "true"
    assert not showcase_link.findall("collision")
    assert not showcase_link.findall("inertial")
    assert len(showcase_link.findall("visual")) >= 5


def test_showcase_person_motion_ping_pongs_without_overshoot():
    controller = _load_showcase_person_controller()
    endpoint_a = controller.Endpoint(5.8, -3.85, 0.40)
    endpoint_b = controller.Endpoint(5.8, -3.35, 0.40)
    segment_length = controller.validate_controller_config(
        endpoint_a,
        endpoint_b,
        0.10,
        10.0,
    )
    assert segment_length == pytest.approx(0.5)

    distance, direction = controller.advance_ping_pong(
        0.0,
        1,
        0.10,
        20.0,
        segment_length,
    )
    assert distance == pytest.approx(0.0)
    assert direction == 1

    distance, direction = controller.advance_ping_pong(
        distance,
        direction,
        0.10,
        20.0,
        segment_length,
    )
    assert distance == pytest.approx(0.0)
    assert direction == 1
    endpoint = controller.interpolate_endpoint(
        endpoint_a,
        endpoint_b,
        segment_length,
        segment_length,
    )
    assert endpoint == endpoint_b


def test_showcase_person_controller_is_robot_independent():
    source = SHOWCASE_PERSON_CONTROLLER_PATH.read_text(encoding="utf-8")
    assert "SetEntityState" in source
    assert 'reference_frame = "world"' in source
    assert "mode=ping_pong_visual" in source
    assert "advance_ping_pong" in source
    assert "create_publisher" not in source
    assert "/cmd_vel" not in source
    assert "NavigateToPose" not in source
    assert "Safety" not in source


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
        self.infos = []

    def warning(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


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
    for path in (
        BASE_LAUNCH_PATH,
        SCENE_LAUNCH_PATH,
        SHOWCASE_LAUNCH_PATH,
    ):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        compile(path.read_text(encoding="utf-8"), str(path), "exec")

    scene_source = SCENE_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"enable_static_obstacles"' in scene_source
    assert '"enable_dynamic_obstacle"' in scene_source
    assert '"dynamic_obstacle_speed"' in scene_source
    assert "ackermann_mcity_demo.launch.py" in scene_source
    assert "cleannav_demo_dynamic_obstacle.sdf" in scene_source
    assert 'executable="dynamic_obstacle_controller"' in scene_source
    assert "cleannav_demo_showcase_person.sdf" not in scene_source
    assert "-slibgazebo_ros_state.so" not in scene_source

    showcase_source = SHOWCASE_LAUNCH_PATH.read_text(encoding="utf-8")
    assert "ackermann_mcity_obstacles_demo.launch.py" in showcase_source
    assert '"enable_static_obstacles"' in showcase_source
    assert '"enable_dynamic_person"' in showcase_source
    assert '"enable_bicycle"' in showcase_source
    assert '"enable_leaf_goal"' in showcase_source
    assert '"leaf_collect_distance"' in showcase_source
    assert '"leaf_cleanup_demo_controller"' in showcase_source
    assert "cleannav_demo_showcase_person.sdf" in showcase_source
    assert 'executable="showcase_person_controller"' in showcase_source
    assert '"enable_dynamic_obstacle": "false"' in showcase_source
    assert "cleannav_demo_bicycle.sdf" in showcase_source
    assert "bicycle_patrol_controller" in showcase_source
    assert "cleannav_demo_leaf_pile.sdf" in showcase_source
    assert '"leaf_entities": list(LEAF_ENTITY_NAMES)' in showcase_source


def _declared_argument_defaults(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        default = next(
            (
                keyword.value
                for keyword in node.keywords
                if keyword.arg == "default_value"
            ),
            None,
        )
        if isinstance(default, ast.Constant):
            defaults[node.args[0].value] = default.value
    return defaults


def test_obstacle_baseline_defaults_remain_unchanged():
    defaults = _declared_argument_defaults(SCENE_LAUNCH_PATH)
    assert defaults["spawn_x"] == "0.0"
    assert defaults["spawn_y"] == "-5.0"
    assert defaults["spawn_z"] == "0.02"
    assert defaults["spawn_yaw"] == "0.0"
    assert defaults["static_block_x"] == "2.20"
    assert defaults["static_block_y"] == "-5.00"
    assert defaults["static_block_z"] == "0.40"
    assert defaults["static_cylinder_x"] == "2.20"
    assert defaults["static_cylinder_y"] == "-2.50"
    assert defaults["static_cylinder_z"] == "0.40"
    assert defaults["dynamic_obstacle_a_x"] == "4.0"
    assert defaults["dynamic_obstacle_a_y"] == "-6.25"
    assert defaults["dynamic_obstacle_a_z"] == "0.40"
    assert defaults["dynamic_obstacle_b_x"] == "4.0"
    assert defaults["dynamic_obstacle_b_y"] == "-3.75"
    assert defaults["dynamic_obstacle_b_z"] == "0.40"
    assert defaults["dynamic_obstacle_speed"] == "0.25"


def test_showcase_v2_defaults_match_recording_layout():
    defaults = _declared_argument_defaults(SHOWCASE_LAUNCH_PATH)
    assert defaults["spawn_x"] == "0.0"
    assert defaults["spawn_y"] == "-5.0"
    assert defaults["spawn_z"] == "0.02"
    assert defaults["spawn_yaw"] == "0.0"
    assert defaults["static_block_x"] == "4.0"
    assert defaults["static_block_y"] == "-4.4"
    assert defaults["static_cylinder_x"] == "8.0"
    assert defaults["static_cylinder_y"] == "-5.8"
    assert defaults["showcase_person_a_x"] == "5.8"
    assert defaults["showcase_person_b_x"] == "5.8"
    assert defaults["showcase_person_a_y"] == "-3.85"
    assert defaults["showcase_person_b_y"] == "-3.35"
    assert defaults["showcase_person_speed"] == "0.10"
    assert defaults["showcase_person_update_rate"] == "10.0"
    assert defaults["bicycle_a_x"] == "1.0"
    assert defaults["bicycle_a_y"] == "-3.0"
    assert defaults["bicycle_b_x"] == "10.0"
    assert defaults["bicycle_b_y"] == "-3.0"
    assert defaults["bicycle_speed"] == "0.80"
    assert defaults["leaf_collect_distance"] == "0.35"
    assert defaults["enable_dynamic_person"] == "true"
    assert defaults["enable_bicycle"] == "true"
    assert defaults["enable_leaf_goal"] == "true"


def test_showcase_leaf_field_has_five_visual_only_piles():
    source = SHOWCASE_LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'range(5)' in source
    for y in ("-5.40", "-5.20", "-5.00", "-4.80", "-4.60"):
        assert y in source
    assert '"cleannav_demo_leaf_pile.sdf"' in source
    assert '"leaf_entities": list(LEAF_ENTITY_NAMES)' in source
    assert 'actions=[*leaf_spawns, leaf_controller]' in source


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


def _load_showcase_launch():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_mcity_showcase_demo_launch_test",
        SHOWCASE_LAUNCH_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import Showcase launch")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_bicycle_controller():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_bicycle_patrol_controller_test",
        BICYCLE_CONTROLLER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import bicycle controller")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_cleanup_controller():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/cleannav-ros-log")
    spec = importlib.util.spec_from_file_location(
        "cleannav_leaf_cleanup_demo_controller_test",
        CLEANUP_CONTROLLER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to import leaf cleanup controller")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_scene_launch_imports_and_constructs():
    module = _load_scene_launch()
    description = module.generate_launch_description()
    assert description is not None


def test_showcase_launch_imports_and_constructs():
    module = _load_showcase_launch()
    description = module.generate_launch_description()
    assert description is not None


def test_bicycle_sdf_is_visual_only_and_uses_primitives():
    bicycle = _model(MODEL_ROOT / "cleannav_demo_bicycle.sdf")
    link = bicycle.find("link")
    assert bicycle.attrib["name"] == "cleannav_demo_bicycle"
    assert bicycle.findtext("static") == "false"
    assert bicycle.findtext("allow_auto_disable") == "false"
    assert link is not None
    assert link.attrib["name"] == "link"
    assert link.findtext("gravity") == "false"
    assert link.findtext("kinematic") == "true"
    assert not link.findall("collision")

    visuals = link.findall("visual")
    assert len(visuals) >= 7
    geometry_types = {
        next(iter(visual.find("geometry"))).tag
        for visual in visuals
    }
    assert {"cylinder", "box", "sphere"} <= geometry_types
    assert sum(
        next(iter(visual.find("geometry"))).tag == "cylinder"
        for visual in visuals
    ) >= 2
    source = (
        MODEL_ROOT / "cleannav_demo_bicycle.sdf"
    ).read_text(encoding="utf-8")
    assert "model://" not in source
    assert "fuel.gazebosim.org" not in source
    assert "mesh" not in source


def test_bicycle_patrol_endpoints_heading_and_ping_pong():
    controller = _load_bicycle_controller()
    endpoint_a = controller.Endpoint(-3.0, -1.5, 0.0)
    endpoint_b = controller.Endpoint(-19.0, -1.5, 0.0)
    assert controller.validate_controller_config(
        endpoint_a,
        endpoint_b,
        0.80,
        10.0,
    ) == pytest.approx(16.0)
    assert controller.heading_for_direction(endpoint_a, endpoint_b, 1) == (
        pytest.approx(math.pi)
    )
    assert controller.heading_for_direction(endpoint_a, endpoint_b, -1) == (
        pytest.approx(0.0)
    )

    distance, direction = controller.advance_ping_pong(
        0.0,
        1,
        0.80,
        20.0,
        16.0,
    )
    assert distance == pytest.approx(16.0)
    assert direction == -1
    distance, direction = controller.advance_ping_pong(
        distance,
        direction,
        0.80,
        20.0,
        16.0,
    )
    assert distance == pytest.approx(0.0)
    assert direction == 1


def test_showcase_bicycle_uses_visible_background_patrol():
    source = SHOWCASE_LAUNCH_PATH.read_text(encoding="utf-8")
    assert (
        'DeclareLaunchArgument("bicycle_a_x", default_value="1.0")'
        in source
    )
    assert (
        'DeclareLaunchArgument("bicycle_a_y", default_value="-3.0")'
        in source
    )
    assert (
        'DeclareLaunchArgument("bicycle_b_x", default_value="10.0")'
        in source
    )
    assert (
        'DeclareLaunchArgument("bicycle_b_y", default_value="-3.0")'
        in source
    )


def test_leaf_sdf_is_visual_only_and_uses_primitives():
    leaf = _model(MODEL_ROOT / "cleannav_demo_leaf_pile.sdf")
    link = leaf.find("link")
    assert leaf.attrib["name"] == "cleannav_demo_leaf_pile"
    assert leaf.findtext("static") == "true"
    assert link is not None
    assert not link.findall("collision")
    assert not link.findall("inertial")
    visuals = link.findall("visual")
    assert len(visuals) >= 5
    assert {next(iter(v.find("geometry"))).tag for v in visuals} == {"box"}
    assert not any(v.attrib.get("name") == "leaf_pile_center" for v in visuals)
    assert "<sphere><radius>0.12</radius></sphere>" not in (
        MODEL_ROOT.joinpath("cleannav_demo_leaf_pile.sdf")
        .read_text(encoding="utf-8")
    )


def test_brush_centers_transform_and_distance_boundaries():
    controller = _load_cleanup_controller()
    robot = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    assert controller.cleaning_point_xy(robot, 0.37, 0.0) == (
        pytest.approx(1.37),
        pytest.approx(2.0),
    )

    robot.orientation = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(math.pi / 4.0),
        w=math.cos(math.pi / 4.0),
    )
    clean_x, clean_y = controller.cleaning_point_xy(robot, 0.37, 0.0)
    assert clean_x == pytest.approx(1.0)
    assert clean_y == pytest.approx(2.37)

    left, right = controller.brush_centers_xy(robot)
    assert left == pytest.approx((clean_x - 0.16, clean_y))
    assert right == pytest.approx((clean_x + 0.16, clean_y))

    leaf = SimpleNamespace(
        position=SimpleNamespace(x=left[0], y=left[1] + 0.35, z=0.0),
    )
    distance = controller.cleaning_distance(robot, leaf, 0.37, 0.0)
    assert distance == pytest.approx(0.35)
    assert distance <= 0.35 + controller.DISTANCE_COMPARISON_EPSILON_M

    leaf.position.y = clean_y + 0.350001
    distance = controller.cleaning_distance(robot, leaf, 0.37, 0.0)
    assert distance > 0.35


def test_brush_centers_rotate_at_ninety_degrees():
    controller = _load_cleanup_controller()
    robot = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(math.pi / 4.0),
            w=math.cos(math.pi / 4.0),
        ),
    )
    left, right = controller.brush_centers_xy(robot)
    assert left == pytest.approx((0.84, 2.37))
    assert right == pytest.approx((1.16, 2.37))


def test_one_brush_inside_radius_completes_when_midpoint_is_outside():
    controller = _load_cleanup_controller()
    robot = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    leaf = SimpleNamespace(
        position=SimpleNamespace(x=0.37, y=0.45, z=0.0),
    )
    assert controller.cleaning_distance(robot, leaf) == pytest.approx(0.29)
    assert math.hypot(0.37 - 0.37, 0.45) > 0.35


def test_right_brush_inside_radius_completes_too():
    controller = _load_cleanup_controller()
    robot = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    leaf = SimpleNamespace(
        position=SimpleNamespace(x=0.37, y=-0.45, z=0.0),
    )
    assert controller.cleaning_distance(robot, leaf) == pytest.approx(0.29)


def test_both_brushes_outside_radius_do_not_complete():
    controller = _load_cleanup_controller()
    robot = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    leaf = SimpleNamespace(
        position=SimpleNamespace(x=0.37, y=0.52, z=0.0),
    )
    assert controller.cleaning_distance(robot, leaf) > 0.35


def test_cleaning_point_translation_and_yaw_combination():
    controller = _load_cleanup_controller()
    yaw = math.radians(30.0)
    robot = SimpleNamespace(
        position=SimpleNamespace(x=4.0, y=-1.0, z=0.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        ),
    )
    clean_x, clean_y = controller.cleaning_point_xy(robot, 0.37, 0.16)
    assert clean_x == pytest.approx(
        4.0 + 0.37 * math.cos(yaw) - 0.16 * math.sin(yaw)
    )
    assert clean_y == pytest.approx(
        -1.0 + 0.37 * math.sin(yaw) + 0.16 * math.cos(yaw)
    )


def test_leaf_cleanup_distance_threshold_is_horizontal_only():
    controller = _load_cleanup_controller()
    first = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=5.0))
    second = SimpleNamespace(position=SimpleNamespace(x=0.3, y=0.16, z=-4.0))
    assert controller.horizontal_distance(first, second) == pytest.approx(
        0.34
    )


def _cleanup_for_timer_test(
    controller,
    state_client,
    delete_client,
    leaf_entities=None,
):
    instance = object.__new__(controller.LeafCleanupDemoController)
    instance._robot_entity = "cleannav_ackermann"
    instance._leaf_entity = "cleannav_demo_leaf_pile"
    instance._leaf_entities = tuple(leaf_entities or (instance._leaf_entity,))
    instance._threshold = 0.35
    instance._cleaning_point_offset_x = 0.37
    instance._cleaning_point_offset_y = 0.0
    instance._left_brush_offset = (0.37, 0.16)
    instance._right_brush_offset = (0.37, -0.16)
    instance._update_rate = 10.0
    instance._state_client = state_client
    instance._delete_client = delete_client
    instance._pending_state = None
    instance._pending_leaf_states = {}
    instance._pending_delete = {}
    instance._collected_entities = set()
    instance._completion_latched_entities = set()
    instance._collected = False
    instance._completion_latched = False
    instance._last_warning_time = -float("inf")
    instance._logger = _FakeLogger()
    instance.get_logger = lambda: instance._logger
    return instance


def test_leaf_cleanup_deletes_once_after_threshold():
    controller = _load_cleanup_controller()
    pose = SimpleNamespace(
        position=SimpleNamespace(x=10.8, y=-5.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    state_client = _FakeClient(
        ready=True,
        responses=[
            SimpleNamespace(success=True, state=SimpleNamespace(pose=pose)),
            SimpleNamespace(
                success=True,
                state=SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=11.0, y=-5.0, z=3.0),
                        orientation=SimpleNamespace(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        )
                    )
                ),
            ),
        ],
    )
    delete_client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=True),
    )
    instance = _cleanup_for_timer_test(
        controller,
        state_client,
        delete_client,
    )

    instance._on_timer()
    instance._on_timer()

    assert len(delete_client.calls) == 1
    assert delete_client.calls[0].name == "cleannav_demo_leaf_pile"
    assert instance.collected is True


def test_leaf_cleanup_deletes_only_the_piles_reached_by_a_brush():
    controller = _load_cleanup_controller()
    leaf_entities = tuple(
        f"cleannav_demo_leaf_pile_{index}" for index in range(5)
    )
    robot_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    far_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.37, y=0.52, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    near_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.37, y=-0.45, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    leaf_responses = []
    for index in range(5):
        leaf_responses.append(
            SimpleNamespace(
                success=True,
                state=SimpleNamespace(
                    pose=near_pose if index in (1, 3) else far_pose,
                ),
            )
        )
    state_client = _FakeClient(
        ready=True,
        responses=[
            SimpleNamespace(success=True, state=SimpleNamespace(pose=robot_pose)),
            *leaf_responses,
        ],
    )
    delete_client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=True),
    )
    instance = _cleanup_for_timer_test(
        controller,
        state_client,
        delete_client,
        leaf_entities=leaf_entities,
    )

    instance._on_timer()

    assert [request.name for request in delete_client.calls] == [
        leaf_entities[1],
        leaf_entities[3],
    ]
    assert instance.collected is True


def test_leaf_cleanup_waits_when_state_service_is_unavailable():
    controller = _load_cleanup_controller()
    state_client = _FakeClient(ready=False)
    delete_client = _FakeClient(
        ready=True,
        response=SimpleNamespace(success=True),
    )
    instance = _cleanup_for_timer_test(
        controller,
        state_client,
        delete_client,
    )

    instance._on_timer()

    assert state_client.calls == []
    assert delete_client.calls == []
    assert instance.collected is False


def test_leaf_cleanup_controller_is_navigation_independent():
    source = CLEANUP_CONTROLLER_PATH.read_text(encoding="utf-8")
    assert "DeleteEntity" in source
    assert "GetEntityState" in source
    assert "SHOWCASE_LEAF_COLLECTED" in source
    assert "create_publisher" not in source
    assert "/cmd_vel" not in source
    assert "NavigateToPose" not in source
    assert "cleaning_point_offset_x" in source
    assert "cleaning_point_offset_y" in source
    assert "CLEANING_DISTANCE=" in source
    assert "LEFT_BRUSH_DISTANCE=" in source
    assert "RIGHT_BRUSH_DISTANCE=" in source


def _expanded_vehicle_root():
    result = subprocess.run(
        ["xacro", str(VEHICLE_XACRO_PATH)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


def _physical_contract_hash(root):
    copy = ElementTree.fromstring(ElementTree.tostring(root))
    for parent in list(copy.iter()):
        for child in list(parent):
            local_name = child.tag.rsplit("}", 1)[-1]
            if local_name in {"visual", "material"}:
                parent.remove(child)
    for parent in list(copy.iter()):
        for child in list(parent):
            local_name = child.tag.rsplit("}", 1)[-1]
            if local_name != "gazebo":
                continue
            if not list(child) and not (child.text or "").strip():
                parent.remove(child)
    for element in copy.iter():
        element.text = (element.text or "").strip()
        element.tail = ""
    return hashlib.sha256(ElementTree.tostring(copy)).hexdigest()


def test_vehicle_visual_upgrade_preserves_physical_contract():
    source_root = ElementTree.parse(VEHICLE_XACRO_PATH).getroot()
    base_link = next(
        link for link in source_root.findall("link")
        if link.attrib.get("name") == "base_link"
    )
    assert len(base_link.findall("collision")) == 1
    assert base_link.find("collision/geometry/box").attrib["size"] == (
        "${chassis_length} ${chassis_width} ${chassis_height}"
    )
    inertial = base_link.find("inertial")
    assert inertial is not None
    assert inertial.find("mass").attrib["value"] == "${chassis_mass}"
    assert inertial.find("inertia").attrib == {
        "ixx": "0.348",
        "ixy": "0",
        "ixz": "0",
        "iyy": "1.258",
        "iyz": "0",
        "izz": "1.498",
    }

    properties = {
        item.attrib["name"]: item.attrib["value"]
        for item in source_root
        if item.tag.endswith("property")
    }
    assert properties["wheelbase"] == "0.60"
    assert properties["front_wheel_track"] == "0.48"
    assert properties["rear_wheel_track"] == "0.48"
    assert properties["front_wheel_radius"] == "0.10"
    assert properties["rear_wheel_radius"] == "0.10"
    assert properties["base_scan_x"] == "0.50"
    assert properties["base_scan_y"] == "0.0"
    assert properties["base_scan_z"] == "0.28"

    expanded = _expanded_vehicle_root()
    link_names = {link.attrib["name"] for link in expanded.findall("link")}
    assert link_names == {
        "base_footprint",
        "base_link",
        "base_scan",
        "front_left_steering_link",
        "front_right_steering_link",
        "front_left_wheel_link",
        "front_right_wheel_link",
        "rear_left_wheel_link",
        "rear_right_wheel_link",
    }
    joint_names = {
        joint.attrib["name"] for joint in expanded.findall("joint")
    }
    assert joint_names == {
        "base_footprint_to_base_link",
        "base_link_to_base_scan",
        "front_left_steering_joint",
        "front_right_steering_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    }
    scan_joint = next(
        joint for joint in expanded.findall("joint")
        if joint.attrib["name"] == "base_link_to_base_scan"
    )
    assert scan_joint.find("origin").attrib == {
        "xyz": "0.5 0.0 0.28",
        "rpy": "0 0 0",
    }
    assert expanded.find(
        "gazebo/plugin[@name='gazebo_ros2_control']"
    ) is not None
    assert expanded.find(
        "gazebo[@reference='base_scan']/sensor/"
        "plugin[@name='cleannav_laser_scan']"
    ) is not None
    assert _physical_contract_hash(expanded) == (
        VEHICLE_PHYSICAL_CONTRACT_SHA256
    )


def test_vehicle_visuals_are_inline_multicolor_and_within_envelope():
    root = ElementTree.parse(VEHICLE_XACRO_PATH).getroot()
    base_link = next(
        link for link in root.findall("link")
        if link.attrib.get("name") == "base_link"
    )
    visuals = base_link.findall("visual")
    assert len(visuals) >= 20
    assert all(visual.find("material/color") is not None for visual in visuals)
    colors = {
        visual.find("material/color").attrib["rgba"]
        for visual in visuals
    }
    assert {
        "0.16 0.18 0.19 1.0",
        "0.25 0.27 0.28 1.0",
        "0.07 0.08 0.09 1.0",
        "0.08 0.25 0.48 1.0",
        "0.95 0.65 0.05 1.0",
        "0.72 0.47 0.10 1.0",
    } <= colors

    source = VEHICLE_XACRO_PATH.read_text(encoding="utf-8")
    assert "<material>Gazebo/DarkGrey</material>" in source
    assert "<material>Gazebo/Black</material>" in source
    assert '<gazebo reference="base_link">' in source
    assert '<gazebo reference="base_scan">' in source

    named_visuals = {
        visual.attrib.get("name"): visual for visual in visuals
    }
    assert named_visuals["cleaner_main_body_shell"].find(
        "geometry/box"
    ).attrib["size"] == "0.76 0.52 0.24"
    assert named_visuals["cleaner_upper_equipment_compartment"].find(
        "geometry/box"
    ).attrib["size"] == "0.48 0.40 0.18"
    assert named_visuals["cleaner_side_panel_left"].find(
        "origin"
    ).attrib["xyz"] == "-0.03 0.266 0.37"
    assert named_visuals["cleaner_side_panel_right"].find(
        "origin"
    ).attrib["xyz"] == "-0.03 -0.266 0.37"

    arm_names = {
        "cleaner_arm_base_pedestal",
        "cleaner_arm_shoulder_joint",
        "cleaner_arm_upper_arm",
        "cleaner_arm_elbow_joint",
        "cleaner_arm_forearm",
        "cleaner_arm_wrist",
        "cleaner_arm_gripper_palm",
        "cleaner_arm_gripper_finger_left",
        "cleaner_arm_gripper_finger_right",
    }
    assert arm_names <= set(named_visuals)


def test_cleaning_point_defaults_match_front_brush_midpoint():
    source = VEHICLE_XACRO_PATH.read_text(encoding="utf-8")
    assert '<origin xyz="0.37 0.16 0.105"' in source
    assert '<origin xyz="0.37 -0.16 0.105"' in source
    controller = _load_cleanup_controller()
    assert controller.DEFAULT_CLEANING_POINT_OFFSET_X == pytest.approx(0.37)
    assert controller.DEFAULT_CLEANING_POINT_OFFSET_Y == pytest.approx(0.0)
    assert controller.DEFAULT_LEFT_BRUSH_OFFSET_Y == pytest.approx(0.16)
    assert controller.DEFAULT_RIGHT_BRUSH_OFFSET_Y == pytest.approx(-0.16)


def test_nav2_footprint_remains_the_demo_physical_contract():
    source = STATIC_NAV2_PARAMS_PATH.read_text(encoding="utf-8")
    assert (
        'footprint: "[[-0.45, -0.29], [-0.45, 0.29], '
        '[0.45, 0.29], [0.45, -0.29]]"'
    ) in source


def test_showcase_bicycle_controller_is_independent_and_configured():
    source = SHOWCASE_LAUNCH_PATH.read_text(encoding="utf-8")
    controller_source = BICYCLE_CONTROLLER_PATH.read_text(encoding="utf-8")
    assert "bicycle_patrol_controller" in source
    assert "enable_bicycle" in source
    assert "bicycle_speed" in source
    assert "bicycle_update_rate" in source
    assert '"cleaning_point_offset_x": 0.37' in source
    assert '"cleaning_point_offset_y": 0.0' in source
    assert "create_publisher" not in controller_source
    assert "/cmd_vel" not in controller_source
    assert "SetEntityState" in controller_source
