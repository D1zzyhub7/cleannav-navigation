#!/usr/bin/env python3
"""Move a visual-only bicycle deterministically along an X-road patrol."""

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
    """One world-frame bicycle patrol endpoint."""

    x: float
    y: float
    z: float


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_controller_config(
    endpoint_a: Endpoint,
    endpoint_b: Endpoint,
    speed: float,
    update_rate: float,
) -> float:
    """Validate values and return the endpoint distance."""
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
    distance = math.hypot(
        endpoint_b.x - endpoint_a.x,
        endpoint_b.y - endpoint_a.y,
    )
    if distance <= 1e-6:
        raise ValueError("endpoint_a and endpoint_b must be distinct")
    return distance


def advance_ping_pong(
    distance: float,
    direction: int,
    speed: float,
    elapsed_sec: float,
    segment_length: float,
) -> tuple[float, int]:
    """Advance path distance and reverse exactly at each endpoint."""
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if not 0.0 <= distance <= segment_length:
        raise ValueError("distance must be within the segment")
    if speed < 0.0 or elapsed_sec < 0.0 or segment_length <= 0.0:
        raise ValueError("speed, elapsed_sec and segment_length are invalid")

    remaining = speed * elapsed_sec
    epsilon = 1e-9
    while remaining > epsilon:
        endpoint = segment_length if direction > 0 else 0.0
        to_endpoint = abs(endpoint - distance)
        if to_endpoint > epsilon and remaining < to_endpoint:
            distance += direction * remaining
            return distance, direction
        distance = endpoint
        remaining -= to_endpoint
        direction = -direction
    return distance, direction


def interpolate_endpoint(
    endpoint_a: Endpoint,
    endpoint_b: Endpoint,
    distance: float,
    segment_length: float,
) -> Endpoint:
    """Return the world point at a scalar distance from endpoint A."""
    ratio = min(max(distance / segment_length, 0.0), 1.0)
    return Endpoint(
        x=endpoint_a.x + ratio * (endpoint_b.x - endpoint_a.x),
        y=endpoint_a.y + ratio * (endpoint_b.y - endpoint_a.y),
        z=endpoint_a.z + ratio * (endpoint_b.z - endpoint_a.z),
    )


def heading_for_direction(
    endpoint_a: Endpoint,
    endpoint_b: Endpoint,
    direction: int,
) -> float:
    """Return the bicycle yaw for the current patrol direction."""
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    angle = math.atan2(
        endpoint_b.y - endpoint_a.y,
        endpoint_b.x - endpoint_a.x,
    )
    if direction < 0:
        angle += math.pi
    return angle % (2.0 * math.pi)


class BicyclePatrolController(Node):
    """Set a visual-only bicycle pose at approximately the configured rate."""

    def __init__(self) -> None:
        super().__init__("cleannav_bicycle_patrol_controller")
        self.declare_parameter("bicycle_name", "cleannav_demo_bicycle")
        self.declare_parameter("service_name", "/set_entity_state")
        self.declare_parameter("a_x", -3.0)
        self.declare_parameter("a_y", -1.5)
        self.declare_parameter("a_z", 0.0)
        self.declare_parameter("b_x", -19.0)
        self.declare_parameter("b_y", -1.5)
        self.declare_parameter("b_z", 0.0)
        self.declare_parameter("speed", 0.80)
        self.declare_parameter("update_rate", 10.0)

        self._bicycle_name = str(self.get_parameter("bicycle_name").value)
        service_name = str(self.get_parameter("service_name").value)
        self._endpoint_a = Endpoint(
            float(self.get_parameter("a_x").value),
            float(self.get_parameter("a_y").value),
            float(self.get_parameter("a_z").value),
        )
        self._endpoint_b = Endpoint(
            float(self.get_parameter("b_x").value),
            float(self.get_parameter("b_y").value),
            float(self.get_parameter("b_z").value),
        )
        self._speed = float(self.get_parameter("speed").value)
        self._update_rate = float(self.get_parameter("update_rate").value)
        self._segment_length = validate_controller_config(
            self._endpoint_a,
            self._endpoint_b,
            self._speed,
            self._update_rate,
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
            "SHOWCASE_BICYCLE "
            f"name={self._bicycle_name} "
            f"A=({self._endpoint_a.x:.2f},{self._endpoint_a.y:.2f}) "
            f"B=({self._endpoint_b.x:.2f},{self._endpoint_b.y:.2f}) "
            f"speed={self._speed:.2f}m/s rate={self._update_rate:.1f}Hz"
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._pending_call is not None:
            self._last_tick = now
            return
        if not self._client.service_is_ready():
            self._last_tick = now
            self._warn_throttled(
                "SetEntityState service unavailable; bicycle waiting safely"
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
        point = interpolate_endpoint(
            self._endpoint_a,
            self._endpoint_b,
            self._distance,
            self._segment_length,
        )
        self._send_pose(point, self._direction)

    def _send_pose(self, point: Endpoint, direction: int) -> None:
        request = SetEntityState.Request()
        request.state.name = self._bicycle_name
        request.state.pose = self._pose(
            point,
            heading_for_direction(
                self._endpoint_a,
                self._endpoint_b,
                direction,
            ),
        )
        request.state.twist = Twist()
        request.state.reference_frame = "world"
        try:
            self._pending_call = self._client.call_async(request)
            self._pending_call.add_done_callback(self._on_service_result)
        except Exception as exc:
            self._pending_call = None
            self._warn_throttled(f"SetEntityState request failed: {exc}")

    @staticmethod
    def _pose(point: Endpoint, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = point.z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
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
                "bicycle may not be ready"
            )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time < WARNING_THROTTLE_SEC:
            return
        self.get_logger().warning(message)
        self._last_warning_time = now


def main(args=None) -> None:
    """Run the bicycle controller until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = BicyclePatrolController()
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
