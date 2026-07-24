#!/usr/bin/env python3
"""
cleannav_path_executor — Path Bridge Node.

订阅 A* 全局路径并以 FollowPath Action 交接给 controller_server。
不发布 Twist，不发布 /cmd_vel，不发布 candidate 速度，不发布 /goal_pose。
"""

import copy
import hashlib
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from std_msgs.msg import String


CANCEL_TIMEOUT_SEC = 0.5


def _path_signature(path_msg: Path) -> str:
    """确定性路径签名：frame_id + poses 数量 + 首尾坐标 + 中点位坐标。"""
    if not path_msg.poses:
        return ""
    first = path_msg.poses[0].pose.position
    last = path_msg.poses[-1].pose.position
    mid = path_msg.poses[len(path_msg.poses) // 2].pose.position
    raw = (
        f"{path_msg.header.frame_id}|{len(path_msg.poses)}|"
        f"{first.x:.6f},{first.y:.6f}|"
        f"{last.x:.6f},{last.y:.6f}|"
        f"{mid.x:.6f},{mid.y:.6f}"
    )
    return hashlib.sha256(raw.encode()).hexdigest()


def _path_is_valid(path_msg: Path, logger) -> bool:
    """路径有效性检查。"""
    if not path_msg.header.frame_id:
        return False
    if len(path_msg.poses) < 2:
        return False
    for i, pose_stamped in enumerate(path_msg.poses):
        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        if not (math.isfinite(px) and math.isfinite(py)):
            logger.warning(f"pose[{i}] position non-finite: ({px},{py})")
            return False
        if pose_stamped.header.frame_id and pose_stamped.header.frame_id != path_msg.header.frame_id:
            logger.warning(
                f"pose[{i}] frame_id mismatch: {pose_stamped.header.frame_id} "
                f"!= {path_msg.header.frame_id}"
            )
            return False
    return True


class PathExecutorNode(Node):
    """Path Bridge 节点 — A* Path → FollowPath Action Goal。"""

    def __init__(self):
        super().__init__('cleannav_path_executor')
        self._controller_id = self.declare_parameter(
            'controller_id', 'FollowPath').value
        self._goal_checker_id = self.declare_parameter(
            'goal_checker_id', '').value

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self._path_sub = self.create_subscription(
            Path, '/cleannav/global_path', self._path_cb, 1)
        self._status_sub = self.create_subscription(
            String, '/cleannav/global_planner_status', self._status_cb, 1)
        self._status_pub = self.create_publisher(
            String, '/cleannav/path_executor_status', 1)

        self._last_sent_signature: str = ""
        self._active_goal_handle = None
        self._cancel_requested: bool = False

        self._publish_status('waiting_for_action_server')

    # ------------------------------------------------------------------ #
    #  状态发布（dedup）                                                    #
    # ------------------------------------------------------------------ #

    _last_pub_status: str = ""

    def _publish_status(self, text: str):
        if text != self._last_pub_status:
            self._last_pub_status = text
            msg = String(data=text)
            self._status_pub.publish(msg)
            self.get_logger().info(f"status: {text}")

    # ------------------------------------------------------------------ #
    #  A* status 订阅                                                      #
    # ------------------------------------------------------------------ #

    _DANGER_STATUS_PREFIXES = (
        "goal in inflation zone",
        "goal occupied in raw map",
        "start in inflation zone",
        "start occupied in raw map",
        "A* fail",
        "invalid map",
        "invalid inflation_radius",
        "map data len",
        "map frame",
        "goal frame",
        "map dims invalid",
        "map origin invalid",
        "goal position non-finite",
        "goal cell",
        "start cell",
        "no localization pose",
        "loc frame",
        "loc x/y non-finite",
        "loc quaternion invalid",
        "loc cov invalid",
        "TF fail",
        "TF trans non-finite",
        "TF quaternion invalid",
        "inflation data not ready",
    )

    def _status_cb(self, msg: String):
        text = msg.data
        is_danger = any(text.startswith(pfx) for pfx in self._DANGER_STATUS_PREFIXES)
        if is_danger and self._active_goal_handle is not None:
            self.get_logger().warn(
                f"planner danger status → cancelling FollowPath: {text}")
            self._cancel_active_goal()
            self._publish_status('cancel_requested_due_to_planner_status')

    # ------------------------------------------------------------------ #
    #  Path 订阅                                                          #
    # ------------------------------------------------------------------ #

    def _path_cb(self, msg: Path):
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self._publish_status('waiting_for_action_server')
            return

        if not _path_is_valid(msg, self.get_logger()):
            self._publish_status('invalid_path')
            return

        sig = _path_signature(msg)
        if sig == self._last_sent_signature:
            self.get_logger().debug('duplicate path ignored')
            self._publish_status('duplicate_path_ignored')
            return

        self._send_follow_path(msg, sig)

    # ------------------------------------------------------------------ #
    #  Action 发送与生命周期                                               #
    # ------------------------------------------------------------------ #

    def _cancel_active_goal(self):
        if self._active_goal_handle is None:
            return
        self._cancel_requested = True
        try:
            self._active_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"cancel_goal_async exception: {exc}")
        self._active_goal_handle = None
        self._cancel_requested = False

    def _send_follow_path(self, path_msg: Path, signature: str):
        self._publish_status('sending_follow_path')

        if self._active_goal_handle is not None:
            self._cancel_active_goal()
            deadline = time.monotonic() + CANCEL_TIMEOUT_SEC
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._active_goal_handle is None:
                    break

        goal = FollowPath.Goal()
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        controller_path = copy.deepcopy(path_msg)
        controller_path.header.stamp.sec = 0
        controller_path.header.stamp.nanosec = 0
        for ps in controller_path.poses:
            ps.header.stamp.sec = 0
            ps.header.stamp.nanosec = 0
        goal.path = controller_path

        self.get_logger().info(
            "Sending FollowPath with zero-stamped controller path "
            "for TF latest transform")

        self._last_sent_signature = signature

        try:
            future = self._action_client.send_goal_async(goal)
        except Exception as exc:
            self.get_logger().error(f"send_goal_async exception: {exc}")
            self._publish_status('follow_path_rejected')
            self._last_sent_signature = ""
            return

        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected")
            self._publish_status('follow_path_rejected')
            self._active_goal_handle = None
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info("FollowPath goal accepted")
        self._publish_status('follow_path_accepted')

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            self.get_logger().error(f"get_result_async exception: {exc}")
            self._publish_status('follow_path_aborted')
            self._active_goal_handle = None
            return

        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self._active_goal_handle = None
        if self._cancel_requested:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"get_result exception: {exc}")
            self._publish_status('follow_path_aborted')
            return

        code = result.status
        self.get_logger().info(f"FollowPath result status={code}")

        if code == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('follow_path_succeeded')
        elif code == GoalStatus.STATUS_CANCELED:
            self._publish_status(f'follow_path_canceled (result_status={code})')
        elif code == GoalStatus.STATUS_ABORTED:
            self._publish_status(f'follow_path_aborted (result_status={code})')
        else:
            self._publish_status(f'follow_path_unknown_result_status_{code}')

    # ------------------------------------------------------------------ #
    #  关闭                                                               #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        if self._active_goal_handle is not None:
            try:
                self.get_logger().info("shutdown: cancelling active goal")
                self._active_goal_handle.cancel_goal_async()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
