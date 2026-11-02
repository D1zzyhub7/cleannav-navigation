#!/usr/bin/env python3
"""Publish Gazebo ground truth as a run-local, odom-anchored diagnostic source."""
from __future__ import annotations

import math


def normalize_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(float(v)) for v in q):
        raise ValueError("orientation must contain four finite xyzw values")
    norm = math.sqrt(sum(float(v) * float(v) for v in q))
    if norm < 1e-12:
        raise ValueError("orientation quaternion has zero norm")
    return tuple(float(v) / norm for v in q)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_inverse(q):
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def rotate_vector(q, vector):
    q = normalize_quaternion(q)
    vx, vy, vz = (float(v) for v in vector)
    rotated = quaternion_multiply(
        quaternion_multiply(q, (vx, vy, vz, 0.0)), quaternion_inverse(q))
    return rotated[:3]


def world_pose_to_odom(position, orientation, anchor_position, anchor_orientation):
    """Express a world pose relative to the first valid base_footprint sample."""
    anchor_inverse = quaternion_inverse(anchor_orientation)
    delta = tuple(float(p) - float(a) for p, a in zip(position, anchor_position))
    local_position = rotate_vector(anchor_inverse, delta)
    local_orientation = normalize_quaternion(
        quaternion_multiply(anchor_inverse, normalize_quaternion(orientation)))
    return local_position, local_orientation


def world_twist_to_base(linear, angular, orientation):
    """GetEntityState expresses twist in its requested world reference frame."""
    inverse = quaternion_inverse(orientation)
    return rotate_vector(inverse, linear), rotate_vector(inverse, angular)


def main():
    import rclpy
    from gazebo_msgs.srv import GetEntityState
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    class OracleOdometry(Node):
        def __init__(self):
            super().__init__("gazebo_oracle_odometry")
            output = self.declare_parameter("truth_output", "").value
            self.truth_stream = open(output, "w") if output else None
            self._entity = "cleannav_ackermann::base_footprint"
            self._client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
            self._publisher = self.create_publisher(Odometry, "/odom", 10)
            self._tf = TransformBroadcaster(self)
            self._future = None
            self._anchor_position = None
            self._anchor_orientation = None
            self._last_warning = 0.0
            self.create_timer(0.1, self._tick)
            self.get_logger().info(
                "Odometry uses GetEntityState response.header.stamp from Gazebo simulation time")

        def _warn(self, message):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warning >= 5.0:
                self.get_logger().warning(message)
                self._last_warning = now

        def _publish_response(self, response):
            if response.success is not True:
                self._warn("Gazebo ground-truth service returned success=false; no /odom published")
                return
            state = response.state
            # gazebo_ros_state binds the requested entity in the service call;
            # its implementation populates response.header.frame_id/stamp but
            # leaves state.name and state.reference_frame empty.
            if response.header.frame_id != "world":
                self._warn(
                    "Gazebo ground-truth entity/reference frame mismatch; no /odom published")
                return

            stamp = response.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                self._warn("Gazebo source timestamp is zero; no /odom published")
                return
            position = (state.pose.position.x, state.pose.position.y, state.pose.position.z)
            orientation = normalize_quaternion((state.pose.orientation.x,
                state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w))
            if self._anchor_position is None:
                self._anchor_position = position
                self._anchor_orientation = orientation
            if self.truth_stream is not None:
                import json
                self.truth_stream.write(json.dumps({
                    "stamp_ns": stamp.sec * 1000000000 + stamp.nanosec,
                    "position": position, "orientation": orientation,
                    "linear": [state.twist.linear.x, state.twist.linear.y, state.twist.linear.z],
                    "angular": [state.twist.angular.x, state.twist.angular.y, state.twist.angular.z],
                }) + "\n")
                self.truth_stream.flush()

            odom_position, odom_orientation = world_pose_to_odom(
                position, orientation, self._anchor_position, self._anchor_orientation)
            linear, angular = world_twist_to_base(
                (state.twist.linear.x, state.twist.linear.y, state.twist.linear.z),
                (state.twist.angular.x, state.twist.angular.y, state.twist.angular.z),
                orientation)

            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_footprint"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = odom_position
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = odom_orientation
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = linear
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
            self._publisher.publish(msg)

            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = odom_position[0]
            transform.transform.translation.y = odom_position[1]
            transform.transform.translation.z = odom_position[2]
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = odom_orientation
            self._tf.sendTransform(transform)

        def _tick(self):
            if self._future is not None:
                if not self._future.done():
                    return
                future, self._future = self._future, None
                try:
                    self._publish_response(future.result())
                except Exception as error:
                    self._warn(f"Gazebo ground-truth request failed; no /odom published: {error}")
            if not self._client.service_is_ready():
                self._warn("waiting for /gazebo/get_entity_state; no /odom published")
                return
            request = GetEntityState.Request()
            request.name = self._entity
            request.reference_frame = "world"
            self._future = self._client.call_async(request)

    rclpy.init()
    node = OracleOdometry()
    try:
        rclpy.spin(node)
    finally:
        if node.truth_stream is not None:
            node.truth_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
