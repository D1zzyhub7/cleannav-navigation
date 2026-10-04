#!/usr/bin/env python3
"""Move the visual-only Showcase person back and forth between A and B."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


WARNING_THROTTLE_SEC = 1.5


@dataclass(frozen=True)
class Endpoint:
    """One finite world-frame endpoint."""

    x: float
    y: float
    z: float


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_controller_config(
    endpoint_a: Endpoint,
    endpoint_b: Endpoint,
    speed: float,
    update_rate: float,
) -> float:
    """Validate the one-way motion configuration and return path length."""
    for endpoint_name, endpoint in (
        ("endpoint_a", endpoint_a),
        ("endpoint_b", endpoint_b),
    ):
        for field_name in ("x", "y", "z"):
            _finite(
                getattr(endpoint, field_name),
                f"{endpoint_name}.{field_name}",
            )

    speed = _finite(speed, "speed")
    update_rate = _finite(update_rate, "update_rate")
    if speed <= 0.0:
        raise ValueError("speed must be greater than zero")
    if update_rate <= 0.0:
        raise ValueError("update_rate must be greater than zero")

    segment_length = math.hypot(
        endpoint_b.x - endpoint_a.x,
        endpoint_b.y - endpoint_a.y,
    )
    if segment_length <= 1e-6:
        raise ValueError("showcase person endpoints must be distinct")
    return segment_length


def advance_ping_pong(
    distance: float,
    direction: int,
    speed: float,
    elapsed_sec: float,
    segment_length: float,
) -> tuple[float, int]:
    """Advance along A--B, reversing exactly at either endpoint."""
    distance = _finite(distance, "distance")
    if direction not in (-1, 1):
        raise ValueError("direction must be either -1 or 1")
    speed = _finite(speed, "speed")
    elapsed_sec = _finite(elapsed_sec, "elapsed_sec")
    segment_length = _finite(segment_length, "segment_length")
    if distance < 0.0 or speed <= 0.0 or elapsed_sec < 0.0:
        raise ValueError("distance/time inputs must be non-negative")
    if segment_length <= 0.0:
        raise ValueError("segment_length must be greater than zero")

    if distance > segment_length:
        raise ValueError("distance must not exceed segment_length")

    remaining = speed * elapsed_sec
    while remaining > 0.0:
        distance_to_endpoint = (
            segment_length - distance if direction == 1 else distance
        )
        if distance_to_endpoint <= 1e-12:
            distance = segment_length if direction == 1 else 0.0
            direction = -direction
            continue
        step = min(remaining, distance_to_endpoint)
        distance += direction * step
        remaining -= step
        if (
            distance <= 1e-12 or distance >= segment_length - 1e-12
        ):
            distance = min(max(distance, 0.0), segment_length)
            direction = -direction

    return min(max(distance, 0.0), segment_length), direction


def interpolate_endpoint(
    endpoint_a: Endpoint,
    endpoint_b: Endpoint,
    distance: float,
    segment_length: float,
) -> Endpoint:
    """Interpolate A to B, clamped at both ends."""
    ratio = min(max(distance / segment_length, 0.0), 1.0)
    return Endpoint(
        x=endpoint_a.x + ratio * (endpoint_b.x - endpoint_a.x),
        y=endpoint_a.y + ratio * (endpoint_b.y - endpoint_a.y),
        z=endpoint_a.z + ratio * (endpoint_b.z - endpoint_a.z),
    )


class ShowcasePersonController(Node):
    """Drive only the Showcase person entity through SetEntityState."""

    def __init__(self) -> None:
        super().__init__("cleannav_showcase_person_controller")
        self.declare_parameter(
            "person_name",
            "cleannav_demo_showcase_person",
        )
        self.declare_parameter("service_name", "/set_entity_state")
        self.declare_parameter("a_x", 5.8)
        self.declare_parameter("a_y", -3.85)
        self.declare_parameter("a_z", 0.40)
        self.declare_parameter("b_x", 5.8)
        self.declare_parameter("b_y", -3.35)
        self.declare_parameter("b_z", 0.40)
        self.declare_parameter("speed", 0.10)
        self.declare_parameter("update_rate", 10.0)

        self._person_name = str(self.get_parameter("person_name").value)
        service_name = str(self.get_parameter("service_name").value)
        self._endpoint_a = Endpoint(
            x=float(self.get_parameter("a_x").value),
            y=float(self.get_parameter("a_y").value),
            z=float(self.get_parameter("a_z").value),
        )
        self._endpoint_b = Endpoint(
            x=float(self.get_parameter("b_x").value),
            y=float(self.get_parameter("b_y").value),
            z=float(self.get_parameter("b_z").value),
        )
        self._speed = float(self.get_parameter("speed").value)
        self._update_rate = float(self.get_parameter("update_rate").value)
        self._segment_length = validate_controller_config(
            self._endpoint_a,
            self._endpoint_b,
            self._speed,
            self._update_rate,
        )
        self._heading_forward = math.atan2(
            self._endpoint_b.y - self._endpoint_a.y,
            self._endpoint_b.x - self._endpoint_a.x,
        )
        self._distance = 0.0
        self._direction = 1
        self._pending_call = None
        self._last_tick = time.monotonic()
        self._last_warning_time = -math.inf
        self._client = self.create_client(SetEntityState, service_name)
        self._timer = self.create_timer(
            1.0 / self._update_rate,
            self._on_timer,
        )

        self.get_logger().info(
            "SHOWCASE_PERSON mode=ping_pong_visual "
            f"A=({self._endpoint_a.x:.2f},{self._endpoint_a.y:.2f},"
            f"{self._endpoint_a.z:.2f}) "
            f"B=({self._endpoint_b.x:.2f},{self._endpoint_b.y:.2f},"
            f"{self._endpoint_b.z:.2f}) "
            f"speed={self._speed:.2f}m/s "
            f"rate={self._update_rate:.1f}Hz"
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._pending_call is not None:
            self._last_tick = now
            return
        if not self._client.service_is_ready():
            self._last_tick = now
            self._warn_throttled(
                "SetEntityState service unavailable; waiting safely"
            )
            return

        elapsed_sec = min(max(now - self._last_tick, 0.0), 0.5)
        self._last_tick = now
        self._distance, self._direction = advance_ping_pong(
            self._distance,
            self._direction,
            self._speed,
            elapsed_sec,
            self._segment_length,
        )
        endpoint = interpolate_endpoint(
            self._endpoint_a,
            self._endpoint_b,
            self._distance,
            self._segment_length,
        )
        heading = self._heading_forward
        if self._direction == -1:
            heading += math.pi
        self._send_pose(endpoint, heading)

    def _send_pose(self, endpoint: Endpoint, heading: float) -> None:
        request = SetEntityState.Request()
        request.state.name = self._person_name
        request.state.pose = self._pose(endpoint, heading)
        request.state.twist = Twist()
        request.state.reference_frame = "world"
        try:
            self._pending_call = self._client.call_async(request)
            self._pending_call.add_done_callback(self._on_service_result)
        except Exception as exc:
            self._pending_call = None
            self._warn_throttled(f"SetEntityState request failed: {exc}")

    @staticmethod
    def _pose(endpoint: Endpoint, heading: float) -> Pose:
        pose = Pose()
        pose.position.x = endpoint.x
        pose.position.y = endpoint.y
        pose.position.z = endpoint.z
        pose.orientation.z = math.sin(heading / 2.0)
        pose.orientation.w = math.cos(heading / 2.0)
        return pose

    def _on_service_result(self, future) -> None:
        self._pending_call = None
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as exc:  # pragma: no cover - middleware-specific
            self._warn_throttled(f"SetEntityState call failed: {exc}")
            return
        if not success:
            self._warn_throttled(
                "SetEntityState returned success=False; "
                "entity may not be ready"
            )
            return

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time < WARNING_THROTTLE_SEC:
            return
        self.get_logger().warning(message)
        self._last_warning_time = now


def main(args=None) -> None:
    """Run the visual-only ping-pong person controller."""
    rclpy.init(args=args)
    node = None
    try:
        node = ShowcasePersonController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
