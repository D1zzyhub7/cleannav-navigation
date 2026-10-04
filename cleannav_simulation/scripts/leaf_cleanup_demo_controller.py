#!/usr/bin/env python3
"""Remove the visual-only demo leaf pile when the robot reaches it."""

from __future__ import annotations

import math
import time

from gazebo_msgs.srv import DeleteEntity, GetEntityState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


WARNING_THROTTLE_SEC = 1.5
DISTANCE_COMPARISON_EPSILON_M = 1e-9

# These are the exact centers of the two front brush visuals in
# cleannav_ackermann.urdf.xacro.  They are deliberately demo-only
# parameters, not vehicle collision geometry.
DEFAULT_CLEANING_POINT_OFFSET_X = 0.37
DEFAULT_CLEANING_POINT_OFFSET_Y = 0.0
DEFAULT_LEFT_BRUSH_OFFSET_X = 0.37
DEFAULT_LEFT_BRUSH_OFFSET_Y = 0.16
DEFAULT_RIGHT_BRUSH_OFFSET_X = 0.37
DEFAULT_RIGHT_BRUSH_OFFSET_Y = -0.16
DEFAULT_LEAF_ENTITIES = tuple(
    f"cleannav_demo_leaf_pile_{index}" for index in range(5)
)


def _yaw_from_quaternion(orientation) -> float:
    """Return finite planar yaw from a pose-like quaternion."""
    values = tuple(float(getattr(orientation, name)) for name in (
        "x", "y", "z", "w",
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("robot orientation must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("robot orientation must be non-zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def cleaning_point_xy(robot_pose, offset_x, offset_y) -> tuple[float, float]:
    """Transform one brush point from base_link into world XY."""
    robot_x = float(robot_pose.position.x)
    robot_y = float(robot_pose.position.y)
    offset_x = float(offset_x)
    offset_y = float(offset_y)
    if not all(math.isfinite(value) for value in (
        robot_x, robot_y, offset_x, offset_y,
    )):
        raise ValueError("cleaning point inputs must be finite")
    yaw = _yaw_from_quaternion(robot_pose.orientation)
    clean_x = robot_x + offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
    clean_y = robot_y + offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
    if not all(math.isfinite(value) for value in (clean_x, clean_y)):
        raise ValueError("cleaning point must be finite")
    return clean_x, clean_y


def brush_centers_xy(
    robot_pose,
    left_offset_x=DEFAULT_LEFT_BRUSH_OFFSET_X,
    left_offset_y=DEFAULT_LEFT_BRUSH_OFFSET_Y,
    right_offset_x=DEFAULT_RIGHT_BRUSH_OFFSET_X,
    right_offset_y=DEFAULT_RIGHT_BRUSH_OFFSET_Y,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return left and right brush centers in Gazebo world XY."""
    return (
        cleaning_point_xy(robot_pose, left_offset_x, left_offset_y),
        cleaning_point_xy(robot_pose, right_offset_x, right_offset_y),
    )


def brush_distances(
    robot_pose,
    leaf_pose,
    left_offset_x=DEFAULT_LEFT_BRUSH_OFFSET_X,
    left_offset_y=DEFAULT_LEFT_BRUSH_OFFSET_Y,
    right_offset_x=DEFAULT_RIGHT_BRUSH_OFFSET_X,
    right_offset_y=DEFAULT_RIGHT_BRUSH_OFFSET_Y,
) -> tuple[float, float]:
    """Return left/right brush-center distances to the leaf in XY."""
    leaf_x = float(leaf_pose.position.x)
    leaf_y = float(leaf_pose.position.y)
    if not all(math.isfinite(value) for value in (leaf_x, leaf_y)):
        raise ValueError("leaf position must be finite")
    left, right = brush_centers_xy(
        robot_pose,
        left_offset_x,
        left_offset_y,
        right_offset_x,
        right_offset_y,
    )
    distances = (
        math.hypot(left[0] - leaf_x, left[1] - leaf_y),
        math.hypot(right[0] - leaf_x, right[1] - leaf_y),
    )
    if not all(math.isfinite(value) for value in distances):
        raise ValueError("brush distances must be finite")
    return distances


def cleaning_distance(
    robot_pose,
    leaf_pose,
    offset_x=DEFAULT_CLEANING_POINT_OFFSET_X,
    offset_y=DEFAULT_CLEANING_POINT_OFFSET_Y,
) -> float:
    """Return the minimum left/right brush-center distance to the leaf."""
    left_distance, right_distance = brush_distances(
        robot_pose,
        leaf_pose,
        offset_x,
        offset_y + DEFAULT_LEFT_BRUSH_OFFSET_Y,
        offset_x,
        offset_y + DEFAULT_RIGHT_BRUSH_OFFSET_Y,
    )
    return min(left_distance, right_distance)


def horizontal_distance(first, second) -> float:
    """Return XY distance between two pose-like objects.

    Kept as a small compatibility helper for existing scene tests; the
    cleanup trigger itself uses the two-brush ``brush_distances`` helper.
    """
    dx = float(first.position.x) - float(second.position.x)
    dy = float(first.position.y) - float(second.position.y)
    return math.hypot(dx, dy)


class LeafCleanupDemoController(Node):
    """Demo-only distance trigger; it never controls navigation."""

    def __init__(self) -> None:
        super().__init__("cleannav_leaf_cleanup_demo_controller")
        self.declare_parameter("robot_entity", "cleannav_ackermann")
        self.declare_parameter("leaf_entity", "cleannav_demo_leaf_pile")
        self.declare_parameter("leaf_entities", list(DEFAULT_LEAF_ENTITIES))
        self.declare_parameter("distance_threshold", 0.35)
        self.declare_parameter(
            "cleaning_point_offset_x",
            DEFAULT_CLEANING_POINT_OFFSET_X,
        )
        self.declare_parameter(
            "cleaning_point_offset_y",
            DEFAULT_CLEANING_POINT_OFFSET_Y,
        )
        self.declare_parameter("update_rate", 10.0)

        self._robot_entity = str(self.get_parameter("robot_entity").value)
        configured_leaf_entities = tuple(
            str(entity)
            for entity in self.get_parameter("leaf_entities").value
        )
        if not configured_leaf_entities:
            configured_leaf_entities = (
                str(self.get_parameter("leaf_entity").value),
            )
        self._leaf_entities = configured_leaf_entities
        self._leaf_entity = self._leaf_entities[0]
        self._threshold = float(
            self.get_parameter("distance_threshold").value
        )
        self._cleaning_point_offset_x = float(
            self.get_parameter("cleaning_point_offset_x").value
        )
        self._cleaning_point_offset_y = float(
            self.get_parameter("cleaning_point_offset_y").value
        )
        self._left_brush_offset = (
            self._cleaning_point_offset_x,
            self._cleaning_point_offset_y + DEFAULT_LEFT_BRUSH_OFFSET_Y,
        )
        self._right_brush_offset = (
            self._cleaning_point_offset_x,
            self._cleaning_point_offset_y + DEFAULT_RIGHT_BRUSH_OFFSET_Y,
        )
        self._update_rate = float(self.get_parameter("update_rate").value)
        if not math.isfinite(self._threshold) or self._threshold <= 0.0:
            raise ValueError("distance_threshold must be finite and > 0")
        if not all(math.isfinite(value) for value in (
            self._cleaning_point_offset_x,
            self._cleaning_point_offset_y,
        )):
            raise ValueError("cleaning point offsets must be finite")
        if not math.isfinite(self._update_rate) or self._update_rate <= 0.0:
            raise ValueError("update_rate must be finite and > 0")

        self._state_client = self.create_client(
            GetEntityState,
            "/get_entity_state",
        )
        self._delete_client = self.create_client(
            DeleteEntity,
            "/delete_entity",
        )
        self._pending_state = None
        self._pending_leaf_states = {}
        self._pending_delete = {}
        self._collected_entities = set()
        self._completion_latched_entities = set()
        self._collected = False
        self._last_warning_time = -math.inf
        self._timer = self.create_timer(
            1.0 / self._update_rate,
            self._on_timer,
        )

    @property
    def collected(self) -> bool:
        """Return whether at least one leaf pile was deleted."""
        return self._collected

    def _on_timer(self) -> None:
        leaf_entities = getattr(
            self,
            "_leaf_entities",
            (getattr(self, "_leaf_entity", "cleannav_demo_leaf_pile"),),
        )
        collected_entities = getattr(self, "_collected_entities", set())
        if (
            set(leaf_entities).issubset(collected_entities)
            or self._pending_state is not None
        ):
            return
        if not self._state_client.service_is_ready():
            self._warn_throttled(
                "GetEntityState unavailable; waiting safely"
            )
            return
        request = GetEntityState.Request()
        request.name = self._robot_entity
        request.reference_frame = "world"
        try:
            self._pending_state = self._state_client.call_async(request)
            self._pending_state.add_done_callback(self._on_state_result)
        except Exception as exc:
            self._pending_state = None
            self._warn_throttled(f"GetEntityState request failed: {exc}")

    def _on_state_result(self, future) -> None:
        self._pending_state = None
        try:
            response = future.result()
        except Exception as exc:
            self._warn_throttled(f"GetEntityState call failed: {exc}")
            return
        if not bool(response.success):
            self._warn_throttled("GetEntityState returned success=False")
            return

        for leaf_entity in self._leaf_entities:
            if (
                leaf_entity in self._collected_entities
                or leaf_entity in self._completion_latched_entities
                or leaf_entity in self._pending_leaf_states
            ):
                continue
            self._request_leaf_state(leaf_entity, response.state.pose)

    def _request_leaf_state(self, leaf_entity, robot_pose) -> None:
        leaf_request = GetEntityState.Request()
        leaf_request.name = leaf_entity
        leaf_request.reference_frame = "world"
        if not self._state_client.service_is_ready():
            return
        try:
            future = self._state_client.call_async(leaf_request)
            self._pending_leaf_states[leaf_entity] = future
            future.add_done_callback(
                lambda leaf_future, entity=leaf_entity: self._on_leaf_state_result(
                    entity,
                    robot_pose,
                    leaf_future,
                )
            )
        except Exception as exc:
            self._warn_throttled(
                f"GetEntityState leaf request failed entity={leaf_entity}: {exc}"
            )

    def _on_leaf_state_result(self, leaf_entity, robot_pose, future) -> None:
        self._pending_leaf_states.pop(leaf_entity, None)
        try:
            response = future.result()
        except Exception as exc:
            self._warn_throttled(f"GetEntityState leaf call failed: {exc}")
            return
        if not bool(response.success):
            self._warn_throttled("Leaf entity unavailable; waiting safely")
            return

        leaf_pose = response.state.pose
        try:
            left_distance, right_distance = brush_distances(
                robot_pose,
                leaf_pose,
                self._left_brush_offset[0],
                self._left_brush_offset[1],
                self._right_brush_offset[0],
                self._right_brush_offset[1],
            )
            distance = min(left_distance, right_distance)
        except (AttributeError, TypeError, ValueError) as exc:
            self._warn_throttled(
                f"Ignoring invalid cleaning pose: {exc}"
            )
            return
        if distance - self._threshold > DISTANCE_COMPARISON_EPSILON_M:
            return
        if not self._delete_client.service_is_ready():
            self._warn_throttled("DeleteEntity unavailable; waiting safely")
            return

        # Latch only when a valid threshold crossing is ready to issue the
        # one-shot delete request.  This prevents repeated DeleteEntity calls
        # while preserving retry behavior if the service is not yet ready.
        self._completion_latched_entities.add(leaf_entity)
        request = DeleteEntity.Request()
        request.name = leaf_entity
        try:
            delete_future = self._delete_client.call_async(request)
            self._pending_delete[leaf_entity] = delete_future
            delete_future.add_done_callback(
                lambda delete_future: self._on_delete_result(
                    leaf_entity,
                    distance,
                    left_distance,
                    right_distance,
                    robot_pose,
                    leaf_pose,
                    delete_future,
                )
            )
        except Exception as exc:
            self._pending_delete.pop(leaf_entity, None)
            self._completion_latched_entities.discard(leaf_entity)
            self._warn_throttled(
                f"DeleteEntity request failed entity={leaf_entity}: {exc}"
            )

    def _on_delete_result(
        self,
        leaf_entity: str,
        distance: float,
        left_distance: float,
        right_distance: float,
        robot_pose,
        leaf_pose,
        future,
    ) -> None:
        self._pending_delete.pop(leaf_entity, None)
        try:
            response = future.result()
        except Exception as exc:
            self._warn_throttled(f"DeleteEntity call failed: {exc}")
            return
        if not bool(response.success):
            self._completion_latched_entities.discard(leaf_entity)
            self._warn_throttled("DeleteEntity returned success=False")
            return
        self._collected_entities.add(leaf_entity)
        self._collected = bool(self._collected_entities)
        self.get_logger().info(
            "SHOWCASE_LEAF_COLLECTED "
            f"LEFT_BRUSH_DISTANCE={left_distance:.3f}m "
            f"RIGHT_BRUSH_DISTANCE={right_distance:.3f}m "
            f"CLEANING_DISTANCE={distance:.3f}m "
            f"robot=({robot_pose.position.x:.3f},{robot_pose.position.y:.3f}) "
            f"leaf=({leaf_pose.position.x:.3f},"
            f"{leaf_pose.position.y:.3f}) "
            f"entity={leaf_entity}"
        )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time < WARNING_THROTTLE_SEC:
            return
        self.get_logger().warning(message)
        self._last_warning_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeafCleanupDemoController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
