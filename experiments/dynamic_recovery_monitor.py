#!/usr/bin/env python3
"""Record stop/resume data and a timestamped navigation command chain."""

from __future__ import annotations

import csv
import math
import sys
import time

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


CHAIN_COLUMNS = [
    'elapsed_sec', 'monotonic_ns', 'wall_time_ns', 'ros_time_ns',
    'source_stamp_ns', 'event', 'frame_id', 'linear_x', 'angular_z',
    'x', 'y', 'yaw_rad', 'scan_front_min_m', 'scan_front_angle_rad',
    'scan_valid_count', 'scan_total_valid_count',
    'scan_front_beam_count', 'scan_range_min_m', 'scan_range_max_m',
    'costmap_nearest_lethal_m',
    'costmap_front_lethal_m', 'costmap_front_x_m', 'costmap_front_y_m',
    'costmap_lethal_count', 'costmap_unknown_count', 'odom_age_sec',
    'status',
]


def _stamp_ns(header) -> int:
    stamp = header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


class MotionMonitor(Node):
    def __init__(self, output_path: str, chain_path: str | None = None) -> None:
        super().__init__('cleannav_dynamic_recovery_monitor')
        self._started_ns = time.monotonic_ns()
        self._file = open(output_path, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._file)
        self._writer.writerow(['elapsed_sec', 'event', 'linear_x', 'angular_z', 'x', 'y'])
        self._file.flush()
        self._chain_file = None
        self._chain_writer = None
        self._last_odom = None
        if chain_path is not None:
            self._chain_file = open(chain_path, 'w', newline='', buffering=1)
            self._chain_writer = csv.DictWriter(
                self._chain_file, fieldnames=CHAIN_COLUMNS)
            self._chain_writer.writeheader()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 20)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(ModelStates, '/model_states', self._on_models, 10)
        if chain_path is not None:
            self.create_subscription(
                Twist, '/cleannav/cmd_vel_candidate', self._on_candidate, 20)
            self.create_subscription(LaserScan, '/scan', self._on_scan, 20)
            self.create_subscription(
                OccupancyGrid, '/local_costmap/costmap', self._on_costmap, 5)
            self.create_subscription(
                String, '/cleannav/safety_supervisor_status',
                self._on_safety_status, 10)

    def _times(self) -> tuple[str, int, int, int]:
        monotonic_ns = time.monotonic_ns()
        return (
            f'{(monotonic_ns - self._started_ns) / 1e9:.6f}',
            monotonic_ns,
            time.time_ns(),
            self.get_clock().now().nanoseconds,
        )

    def _record(self, event: str, times, **fields) -> None:
        if self._chain_writer is None:
            return
        elapsed, monotonic_ns, wall_time_ns, ros_time_ns = times
        self._chain_writer.writerow({
            'elapsed_sec': elapsed,
            'monotonic_ns': monotonic_ns,
            'wall_time_ns': wall_time_ns,
            'ros_time_ns': ros_time_ns,
            'event': event,
            **fields,
        })

    def _on_cmd(self, msg: Twist) -> None:
        times = self._times()
        self._writer.writerow([
            times[0], 'cmd', msg.linear.x, msg.angular.z, '', ''
        ])
        self._record('cmd_vel', times, linear_x=msg.linear.x,
                     angular_z=msg.angular.z)

    def _on_candidate(self, msg: Twist) -> None:
        self._record('cmd_vel_candidate', self._times(),
                     linear_x=msg.linear.x, angular_z=msg.angular.z)

    def _on_odom(self, msg: Odometry) -> None:
        times = self._times()
        position = msg.pose.pose.position
        yaw = _yaw(msg.pose.pose.orientation)
        self._last_odom = (times[1], msg.header.frame_id,
                           position.x, position.y, yaw)
        self._writer.writerow([
            times[0], 'odom', '', '', position.x, position.y
        ])
        self._record('odom', times, source_stamp_ns=_stamp_ns(msg.header),
                     frame_id=msg.header.frame_id, x=position.x,
                     y=position.y, yaw_rad=yaw)

    def _on_models(self, msg: ModelStates) -> None:
        try:
            index = msg.name.index('cleannav_demo_dynamic_obstacle')
        except ValueError:
            return
        times = self._times()
        position = msg.pose[index].position
        self._writer.writerow([
            times[0], 'obstacle', '', '', position.x, position.y
        ])
        self._record('obstacle_world', times, frame_id='world',
                     x=position.x, y=position.y)

    def _on_scan(self, msg: LaserScan) -> None:
        nearest = None
        angle_nearest = None
        front_valid_count = total_valid_count = front_beam_count = 0
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            is_front = abs(angle) <= math.pi / 6
            if is_front:
                front_beam_count += 1
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            total_valid_count += 1
            if not is_front:
                continue
            front_valid_count += 1
            if nearest is None or distance < nearest:
                nearest, angle_nearest = distance, angle
        self._record('scan', self._times(),
                     source_stamp_ns=_stamp_ns(msg.header),
                     frame_id=msg.header.frame_id,
                     scan_front_min_m=nearest,
                     scan_front_angle_rad=angle_nearest,
                     scan_valid_count=front_valid_count,
                     scan_total_valid_count=total_valid_count,
                     scan_front_beam_count=front_beam_count,
                     scan_range_min_m=msg.range_min,
                     scan_range_max_m=msg.range_max,
                     status=(
                         'front_return' if front_valid_count else
                         'no_front_return'))

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        times = self._times()
        info = msg.info
        fields = {
            'source_stamp_ns': _stamp_ns(msg.header),
            'frame_id': msg.header.frame_id,
            'costmap_lethal_count': 0,
            'costmap_unknown_count': 0,
        }
        odom = self._last_odom
        if odom is not None:
            fields['odom_age_sec'] = (times[1] - odom[0]) / 1e9
        if info.width * info.height != len(msg.data) or info.resolution <= 0:
            fields['status'] = 'invalid_grid'
        elif odom is None or odom[1] != msg.header.frame_id:
            fields['status'] = 'odom_frame_unavailable'
        else:
            _, _, robot_x, robot_y, robot_yaw = odom
            origin = info.origin.position
            origin_yaw = _yaw(info.origin.orientation)
            co, so = math.cos(origin_yaw), math.sin(origin_yaw)
            cr, sr = math.cos(robot_yaw), math.sin(robot_yaw)
            nearest = float('inf')
            front_nearest = float('inf')
            front_x = front_y = None
            lethal_count = unknown_count = 0
            resolution = info.resolution
            for index, value in enumerate(msg.data):
                if value < 0:
                    unknown_count += 1
                    continue
                if value < 90:
                    continue
                lethal_count += 1
                gx = ((index % info.width) + 0.5) * resolution
                gy = ((index // info.width) + 0.5) * resolution
                dx = origin.x + co * gx - so * gy - robot_x
                dy = origin.y + so * gx + co * gy - robot_y
                forward = cr * dx + sr * dy
                lateral = -sr * dx + cr * dy
                distance = math.hypot(forward, lateral)
                nearest = min(nearest, distance)
                if forward > 0 and abs(lateral) <= 0.75 \
                        and distance < front_nearest:
                    front_nearest = distance
                    front_x, front_y = forward, lateral
            fields.update({
                'costmap_nearest_lethal_m': (
                    nearest if math.isfinite(nearest) else None),
                'costmap_front_lethal_m': (
                    front_nearest if math.isfinite(front_nearest) else None),
                'costmap_front_x_m': front_x,
                'costmap_front_y_m': front_y,
                'costmap_lethal_count': lethal_count,
                'costmap_unknown_count': unknown_count,
                'status': 'ok',
            })
        self._record('local_costmap', times, **fields)

    def _on_safety_status(self, msg: String) -> None:
        self._record('safety_status', self._times(), status=msg.data)

    def destroy_node(self) -> None:
        self._file.close()
        if self._chain_file is not None:
            self._chain_file.close()
        super().destroy_node()


def main() -> None:
    arguments = sys.argv[1:]
    if '--ros-args' in arguments:
        split = arguments.index('--ros-args')
        files, ros_arguments = arguments[:split], arguments[split:]
    else:
        files, ros_arguments = arguments, []
    if len(files) not in (1, 2):
        raise SystemExit(
            'usage: dynamic_recovery_monitor.py MOTION.csv [CHAIN.csv] '
            '[--ros-args -p use_sim_time:=true]')
    rclpy.init(args=ros_arguments)
    node = MotionMonitor(files[0], files[1] if len(files) == 2 else None)
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
