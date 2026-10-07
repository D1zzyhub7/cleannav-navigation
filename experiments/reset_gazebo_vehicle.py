#!/usr/bin/env python3
"""Reset the experiment vehicle pose and verify it from Gazebo model states."""

from __future__ import annotations

import argparse
import math
import time

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.node import Node


class VehicleReset(Node):
    def __init__(self, name: str, x: float, y: float, yaw: float) -> None:
        super().__init__('cleannav_experiment_vehicle_reset')
        self._name = name
        self._x = x
        self._y = y
        self._yaw = yaw
        self._verified = False
        self._last_observation = 'no model state received'
        self._client = self.create_client(
            SetEntityState, '/set_entity_state')
        self.create_subscription(ModelStates, '/model_states',
                                 self._on_models, 10)

    def _on_models(self, message: ModelStates) -> None:
        try:
            index = message.name.index(self._name)
        except ValueError:
            return
        pose = message.pose[index]
        twist = message.twist[index]
        siny_cosp = 2.0 * (
            pose.orientation.w * pose.orientation.z
            + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_error = abs(math.atan2(
            math.sin(yaw - self._yaw), math.cos(yaw - self._yaw)))
        speed = math.hypot(twist.linear.x, twist.linear.y)
        self._last_observation = (
            f'pose=({pose.position.x:.3f},{pose.position.y:.3f},{yaw:.3f}) '
            f'speed={speed:.3f} yaw_rate={twist.angular.z:.3f}')
        self._verified = (
            math.hypot(pose.position.x - self._x,
                       pose.position.y - self._y) <= 0.03
            and yaw_error <= 0.03
            and speed <= 0.10
            and abs(twist.angular.z) <= 0.10
        )

    def reset(self) -> bool:
        for service_attempt in range(1, 4):
            if self._client.wait_for_service(timeout_sec=10.0):
                break
            print(f'set_entity_state discovery attempt={service_attempt} '
                  'timed out', flush=True)
        else:
            return False
        for attempt in range(1, 4):
            request = SetEntityState.Request()
            request.state.name = self._name
            request.state.pose.position.x = self._x
            request.state.pose.position.y = self._y
            request.state.pose.position.z = 0.02
            request.state.pose.orientation.z = math.sin(self._yaw / 2.0)
            request.state.pose.orientation.w = math.cos(self._yaw / 2.0)
            request.state.reference_frame = 'world'
            self._verified = False
            self._client.call_async(request)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._verified:
                    print(f'vehicle reset verified attempt={attempt}')
                    return True
            print(f'vehicle reset attempt={attempt} unverified: '
                  f'{self._last_observation}', flush=True)
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='cleannav_ackermann')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = VehicleReset(args.name, args.x, args.y, args.yaw)
    try:
        return 0 if node.reset() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
