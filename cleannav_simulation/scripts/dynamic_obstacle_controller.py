#!/usr/bin/env python3
"""Move one demo obstacle deterministically between two world poses."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


WARNING_THROTTLE_SEC = 1.5
SERVICE_CALL_TIMEOUT_SEC = 1.0


@dataclass(frozen=True)
class Endpoint:
    """One world-frame pose used by the ping-pong controller."""

    x: float
    y: float
    z: float
    yaw: float


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
    """Validate controller values and return the A-to-B distance."""
    for endpoint_name, endpoint in (
        ("endpoint_a", endpoint_a),
        ("endpoint_b", endpoint_b),
    ):
        for field_name in ("x", "y", "z", "yaw"):
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
    """Advance scalar path distance and reverse exactly at each endpoint."""
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
    """Return the world pose at a scalar distance from endpoint A."""
    ratio = min(max(distance / segment_length, 0.0), 1.0)
    return Endpoint(
        x=endpoint_a.x + ratio * (endpoint_b.x - endpoint_a.x),
        y=endpoint_a.y + ratio * (endpoint_b.y - endpoint_a.y),
        z=endpoint_a.z + ratio * (endpoint_b.z - endpoint_a.z),
        yaw=endpoint_a.yaw + ratio * (endpoint_b.yaw - endpoint_a.yaw),
    )


class DynamicObstacleController(Node):
    """Set one Gazebo entity pose at approximately the configured rate."""

    def __init__(self) -> None:
        super().__init__("cleannav_dynamic_obstacle_controller")

        self.declare_parameter(
            "obstacle_name",
            "cleannav_demo_dynamic_obstacle",
        )
        self.declare_parameter("service_name", "/set_entity_state")
        self.declare_parameter("a_x", 4.0)
        self.declare_parameter("a_y", -6.25)
        self.declare_parameter("a_z", 0.40)
        self.declare_parameter("a_yaw", 0.0)
        self.declare_parameter("b_x", 4.0)
        self.declare_parameter("b_y", -3.75)
        self.declare_parameter("b_z", 0.40)
        self.declare_parameter("b_yaw", 0.0)
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("update_rate", 10.0)

        self._obstacle_name = str(
            self.get_parameter("obstacle_name").value
        )
        service_name = str(self.get_parameter("service_name").value)
        self._endpoint_a = Endpoint(
            x=float(self.get_parameter("a_x").value),
            y=float(self.get_parameter("a_y").value),
            z=float(self.get_parameter("a_z").value),
            yaw=float(self.get_parameter("a_yaw").value),
        )
        self._endpoint_b = Endpoint(
            x=float(self.get_parameter("b_x").value),
            y=float(self.get_parameter("b_y").value),
            z=float(self.get_parameter("b_z").value),
            yaw=float(self.get_parameter("b_yaw").value),
        )
        self._speed = float(self.get_parameter("speed").value)
        self._update_rate = float(
            self.get_parameter("update_rate").value
        )
        self._segment_length = validate_controller_config(
            self._endpoint_a,
            self._endpoint_b,
            self._speed,
            self._update_rate,
        )

        self._distance = 0.0
        self._direction = 1
        self._pending_call = None
        self._pending_call_started = None
        self._last_tick = time.monotonic()
        self._last_warning_time = -math.inf
        self._callback_group = ReentrantCallbackGroup()
        self._client = self.create_client(
            SetEntityState,
            service_name,
            callback_group=self._callback_group,
        )
        self._timer = self.create_timer(
            1.0 / self._update_rate,
            self._on_timer,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "DEMO_ONLY_DYNAMIC_OBSTACLE "
            f"name={self._obstacle_name} "
            f"A=({self._endpoint_a.x:.2f},{self._endpoint_a.y:.2f}) "
            f"B=({self._endpoint_b.x:.2f},{self._endpoint_b.y:.2f}) "
            f"speed={self._speed:.2f}m/s rate={self._update_rate:.1f}Hz"
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._pending_call is not None:
            if (
                self._pending_call_started is not None
                and now - self._pending_call_started
                > SERVICE_CALL_TIMEOUT_SEC
            ):
                stale_call = self._pending_call
                self._pending_call = None
                self._pending_call_started = None
                try:
                    stale_call.cancel()
                except Exception:
                    pass
                self._warn_throttled(
                    "SetEntityState call timed out; retrying"
                )
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
        self._send_pose(endpoint)

    def _send_pose(self, endpoint: Endpoint) -> None:
        request = SetEntityState.Request()
        request.state.name = self._obstacle_name
        request.state.pose = self._pose(endpoint)
        request.state.twist = Twist()
        request.state.reference_frame = "world"
        try:
            self._pending_call = self._client.call_async(request)
            self._pending_call_started = time.monotonic()
            self._pending_call.add_done_callback(self._on_service_result)
        except Exception as exc:
            self._pending_call = None
            self._pending_call_started = None
            self._warn_throttled(f"SetEntityState request failed: {exc}")

    @staticmethod
    def _pose(endpoint: Endpoint) -> Pose:
        pose = Pose()
        pose.position.x = endpoint.x
        pose.position.y = endpoint.y
        pose.position.z = endpoint.z
        pose.orientation.z = math.sin(endpoint.yaw / 2.0)
        pose.orientation.w = math.cos(endpoint.yaw / 2.0)
        return pose

    def _on_service_result(self, future) -> None:
        # A timed-out request may complete after a newer call was submitted.
        # Never let that stale callback clear the active request state.
        if future is not self._pending_call:
            return
        self._pending_call = None
        self._pending_call_started = None
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

    def _warn_throttled(self, message: str) -> None:
        """Warn at most once per throttle interval while retrying."""
        now = time.monotonic()
        if now - self._last_warning_time < WARNING_THROTTLE_SEC:
            return
        self.get_logger().warning(message)
        self._last_warning_time = now


def main(args=None) -> None:
    """Run the demo-only controller until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DynamicObstacleController()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
