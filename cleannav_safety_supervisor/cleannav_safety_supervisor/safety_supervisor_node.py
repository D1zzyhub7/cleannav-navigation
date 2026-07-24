#!/usr/bin/env python3
"""
cleannav_safety_supervisor — Safety Supervisor Node.

默认阻断所有 candidate 速度，只发布零速度到 /cmd_vel。
不转发 candidate，不调用 FollowPath，不规划路径。
"""

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class SafetySupervisorNode(Node):
    """安全门控节点 — 默认 block_all，仅输出零速度。
    支持 SetBool service 显式授权 autonomous 放行。
    """

    def __init__(self):
        super().__init__('cleannav_safety_supervisor')

        self._block_all = self.declare_parameter('block_all', True).value
        self._candidate_timeout_sec = self.declare_parameter(
            'candidate_timeout_sec', 0.5).value
        publish_freq = self.declare_parameter('publish_frequency', 10.0).value
        status_freq = self.declare_parameter(
            'status_publish_frequency', 2.0).value

        self._max_forward_linear_x = self.declare_parameter(
            'max_forward_linear_x', 0.05).value
        self._max_reverse_linear_x = self.declare_parameter(
            'max_reverse_linear_x', 0.0).value
        self._max_angular_z = self.declare_parameter(
            'max_angular_z', 0.3).value
        self._autonomous_timeout_sec = self.declare_parameter(
            'autonomous_timeout_sec', 5.0).value

        self._emergency_stop = False
        self._last_candidate_time = self.get_clock().now()
        self._candidate_count = 0
        self._nonzero_candidate_count = 0
        self._last_candidate_twist = None
        self._last_output_limited = False

        self._autonomous_enabled = False
        self._autonomous_start_time = None

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self._status_pub = self.create_publisher(
            String, '/cleannav/safety_supervisor_status', 1)

        self._candidate_sub = self.create_subscription(
            Twist, '/cleannav/cmd_vel_candidate', self._candidate_cb, 1)
        self._estop_sub = self.create_subscription(
            Bool, '/cleannav/safety/emergency_stop', self._estop_cb, 1)

        self._autonomous_srv = self.create_service(
            SetBool, '/cleannav/safety/set_autonomous_enabled',
            self._autonomous_srv_cb)

        self._cmd_timer = self.create_timer(
            1.0 / publish_freq, self._cmd_timer_cb)
        self._status_timer = self.create_timer(
            1.0 / status_freq, self._status_timer_cb)

        self.get_logger().info(
            "Safety Supervisor started — default mode: block_all")

    def _candidate_cb(self, msg: Twist):
        self._last_candidate_time = self.get_clock().now()
        self._last_candidate_twist = msg
        self._candidate_count += 1
        if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0 \
                or msg.angular.x != 0.0 or msg.angular.y != 0.0 \
                or msg.angular.z != 0.0:
            self._nonzero_candidate_count += 1

    def _estop_cb(self, msg: Bool):
        if self._autonomous_enabled and msg.data:
            self._autonomous_enabled = False
            self._autonomous_start_time = None
        self._emergency_stop = msg.data

    def _autonomous_srv_cb(self, request, response):
        if request.data:
            if self._emergency_stop:
                response.success = False
                response.message = "rejected by emergency_stop"
            else:
                self._autonomous_enabled = True
                self._autonomous_start_time = time.monotonic()
                response.success = True
                response.message = (
                    f"autonomous enabled for "
                    f"{self._autonomous_timeout_sec} seconds")
        else:
            self._autonomous_enabled = False
            self._autonomous_start_time = None
            response.success = True
            response.message = "autonomous disabled, block_all"
        return response

    def _limit_twist(self, twist: Twist) -> Twist:
        out = Twist()
        if twist.linear.x > 0:
            out.linear.x = min(twist.linear.x, self._max_forward_linear_x)
        else:
            rev = -self._max_reverse_linear_x
            out.linear.x = max(twist.linear.x, rev)
        out.angular.z = max(
            min(twist.angular.z, self._max_angular_z),
            -self._max_angular_z)
        return out

    def _cmd_timer_cb(self):
        # Priority 1: emergency_stop → zero
        if self._emergency_stop:
            if self._autonomous_enabled:
                self._autonomous_enabled = False
                self._autonomous_start_time = None
            self._cmd_vel_pub.publish(Twist())
            return

        # Priority 2: autonomous timeout (checked before stale, so timeout
        # works even without candidate)
        if self._autonomous_enabled and self._autonomous_start_time is not None:
            elapsed = time.monotonic() - self._autonomous_start_time
            if elapsed >= self._autonomous_timeout_sec:
                self._autonomous_enabled = False
                self._autonomous_start_time = None
                self._last_output_limited = False
                self.get_logger().info(
                    "autonomous timeout — back to block_all")
                self._cmd_vel_pub.publish(Twist())
                return

        # Priority 3: stale / no candidate → zero
        now = self.get_clock().now()
        age = (now - self._last_candidate_time).nanoseconds * 1e-9
        stale = (self._candidate_count == 0) or (age > self._candidate_timeout_sec)
        if stale or self._last_candidate_twist is None:
            self._cmd_vel_pub.publish(Twist())
            return

        # Priority 4: not autonomous → zero (default block_all)
        if not self._autonomous_enabled:
            self._cmd_vel_pub.publish(Twist())
            return

        # Priority 5: forward limited candidate
        limited = self._limit_twist(self._last_candidate_twist)
        self._last_output_limited = (
            limited.linear.x != self._last_candidate_twist.linear.x
            or limited.angular.z != self._last_candidate_twist.angular.z)
        self._cmd_vel_pub.publish(limited)

    def _status_timer_cb(self):
        now = self.get_clock().now()
        age = (now - self._last_candidate_time).nanoseconds * 1e-9
        if self._candidate_count == 0:
            freshness = "no_candidate"
        elif age > self._candidate_timeout_sec:
            freshness = "candidate_stale"
        else:
            freshness = "candidate_fresh"

        remaining = 0.0
        if self._autonomous_enabled and self._autonomous_start_time is not None:
            remaining = max(
                0.0, self._autonomous_timeout_sec
                - (time.monotonic() - self._autonomous_start_time))

        if self._emergency_stop:
            base = "emergency_stop"
        elif self._autonomous_enabled:
            base = "autonomous"
        else:
            base = "block_all"

        status = (
            f"{base} "
            f"{freshness} "
            f"candidate_count={self._candidate_count} "
            f"nonzero={self._nonzero_candidate_count} "
            f"last_age_sec={age:.3f} "
            f"block_all={self._block_all} "
            f"emergency_stop={self._emergency_stop} "
            f"autonomous_enabled={self._autonomous_enabled} "
            f"autonomous_remaining_sec={remaining:.1f} "
            f"last_output_limited={self._last_output_limited}"
        )
        self._status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    shutdown_requested = False

    def _handle_shutdown_signal(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_shutdown_signal)
    signal.signal(signal.SIGTERM, _handle_shutdown_signal)

    try:
        node = SafetySupervisorNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        while rclpy.ok() and not shutdown_requested:
            executor.spin_once(timeout_sec=0.1)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
