#!/usr/bin/env python3
"""Record velocity and odometry samples for stop/resume verification."""

from __future__ import annotations

import csv
import sys
import time

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MotionMonitor(Node):
    def __init__(self, output_path: str) -> None:
        super().__init__('cleannav_dynamic_recovery_monitor')
        self._started = time.monotonic()
        self._file = open(output_path, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._file)
        self._writer.writerow(['elapsed_sec', 'event', 'linear_x', 'angular_z', 'x', 'y'])
        self._file.flush()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 20)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(ModelStates, '/model_states', self._on_models, 10)

    def _elapsed(self) -> str:
        return f'{time.monotonic() - self._started:.6f}'

    def _on_cmd(self, msg: Twist) -> None:
        self._writer.writerow([
            self._elapsed(), 'cmd', msg.linear.x, msg.angular.z, '', ''
        ])
        self._file.flush()

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self._writer.writerow([
            self._elapsed(), 'odom', '', '', position.x, position.y
        ])
        self._file.flush()

    def _on_models(self, msg: ModelStates) -> None:
        try:
            index = msg.name.index('cleannav_demo_dynamic_obstacle')
        except ValueError:
            return
        position = msg.pose[index].position
        self._writer.writerow([
            self._elapsed(), 'obstacle', '', '', position.x, position.y
        ])
        self._file.flush()

    def destroy_node(self) -> None:
        self._file.close()
        super().destroy_node()


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit('usage: dynamic_recovery_monitor.py OUTPUT.csv')
    rclpy.init(args=sys.argv[2:])
    node = MotionMonitor(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
