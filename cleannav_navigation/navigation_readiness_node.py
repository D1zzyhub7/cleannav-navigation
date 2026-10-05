#!/usr/bin/env python3
"""Wait for navigation inputs before starting the lifecycle manager."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener

class Readiness(Node):
    def __init__(self):
        super().__init__('cleannav_navigation_readiness')
        for name, value in [('timeout_sec', 180.0), ('stable_sec', 3.0),
                            ('map_topic', '/rtabmap/map'), ('map_frame', 'map'),
                            ('base_frame', 'base_link')]:
            self.declare_parameter(name, value)
        self.map = self.scan = self.odom = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned = [
            self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                     lambda m: setattr(self, 'map', m), map_qos),
            self.create_subscription(LaserScan, '/scan',
                                     lambda m: setattr(self, 'scan', m), qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom',
                                     lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)]
    def missing(self):
        missing = []
        if self.map is None or self.map.info.width == 0 or self.map.info.height == 0 or not self.map.data:
            missing.append('valid map')
        now = self.get_clock().now().nanoseconds
        for label, msg in [('scan', self.scan), ('odom', self.odom)]:
            if msg is None:
                missing.append(label)
                continue
            stamp = Time.from_msg(msg.header.stamp)
            age = (now - stamp.nanoseconds) / 1e9
            if stamp.nanoseconds == 0 or age < -0.1 or age > 1.0:
                missing.append(f'{label} stamp age={age:.3f}s')
            target = self.get_parameter('map_frame').value if label == 'odom' else self.get_parameter('base_frame').value
            if not msg.header.frame_id or not self.tf.can_transform(target, msg.header.frame_id, stamp):
                missing.append(f'TF {target} <- {msg.header.frame_id} at {label} stamp')
        if not self.tf.can_transform(self.get_parameter('map_frame').value,
                                     self.get_parameter('base_frame').value, Time()):
            missing.append('map->base TF')
        return missing

def main(args=None):
    rclpy.init(args=args)
    node = Readiness()
    start = time.monotonic()
    stable = None
    next_log = 0.0
    code = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            missing = node.missing()
            if missing:
                stable = None
            elif stable is None:
                stable = time.monotonic()
            elif time.monotonic() - stable >= node.get_parameter('stable_sec').value:
                node.get_logger().info(f'READY after {elapsed:.3f}s: map, fresh scan/odom and timestamped TF stable')
                code = 0
                break
            if elapsed >= node.get_parameter('timeout_sec').value:
                node.get_logger().error('READINESS TIMEOUT: ' + ', '.join(missing or ['stability window']))
                break
            if elapsed >= next_log:
                node.get_logger().info('Waiting: ' + ', '.join(missing or ['stability window']))
                next_log = elapsed + 5.0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code

if __name__ == '__main__':
    raise SystemExit(main())
