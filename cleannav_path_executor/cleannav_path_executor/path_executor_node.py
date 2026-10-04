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
from rclpy.time import Time
import tf2_ros

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from std_msgs.msg import String


def _path_signature(path_msg: Path) -> str:
    """确定性路径签名：位置采样加最终朝向，作为 terminal 身份。"""
    if not path_msg.poses:
        return ""
    first = path_msg.poses[0].pose.position
    last = path_msg.poses[-1].pose.position
    mid = path_msg.poses[len(path_msg.poses) // 2].pose.position
    last_orientation = path_msg.poses[-1].pose.orientation
    raw = (
        f"{path_msg.header.frame_id}|{len(path_msg.poses)}|"
        f"{first.x:.6f},{first.y:.6f}|"
        f"{last.x:.6f},{last.y:.6f}|"
        f"{mid.x:.6f},{mid.y:.6f}|"
        f"{last_orientation.x:.6f},{last_orientation.y:.6f},"
        f"{last_orientation.z:.6f},{last_orientation.w:.6f}"
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
        self._completion_window_xy_tolerance = float(
            self.declare_parameter(
                'completion_window_xy_tolerance', 0.25).value)
        self._robot_base_frame = self.declare_parameter(
            'robot_base_frame', 'base_link').value

        self._action_client = ActionClient(self, FollowPath, '/follow_path')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path_sub = self.create_subscription(
            Path, '/cleannav/global_path', self._path_cb, 1)
        self._status_sub = self.create_subscription(
            String, '/cleannav/global_planner_status', self._status_cb, 1)
        self._status_pub = self.create_publisher(
            String, '/cleannav/path_executor_status', 1)

        self._send_future = None
        self._sending_signature: str = ""
        self._send_sequence: int = 0
        self._sending_sequence = None
        self._active_goal_handle = None
        self._active_signature: str = ""
        self._active_path = None
        self._active_sequence = None
        self._near_goal_reported = False
        self._last_tf_warning_time = 0.0
        self._pending_path = None
        self._cancel_future = None
        self._cancel_requested: bool = False
        self._cancel_when_accepted: bool = False
        self._last_pub_status: str = ""

        self._publish_status('waiting_for_action_server')

    def _feedback_callback(self, feedback_msg):
        """按 active path 的真实 XY 距离发布 completion-window 状态。"""
        if (
                not self._active_signature
                or self._active_path is None
                or self._active_goal_handle is None
                or self._near_goal_reported):
            return

        # rclpy delivers FollowPath.Impl.FeedbackMessage with goal_id. Ignore
        # late feedback from a replaced goal before evaluating the new path.
        feedback_goal_id = getattr(feedback_msg, 'goal_id', None)
        active_goal_id = getattr(self._active_goal_handle, 'goal_id', None)
        if feedback_goal_id is None or active_goal_id is None:
            return
        if bytes(feedback_goal_id.uuid) != bytes(active_goal_id.uuid):
            return

        path_frame = self._active_path.header.frame_id
        if not path_frame or not self._active_path.poses:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                path_frame, self._robot_base_frame, Time())
        except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self._warn_tf_unavailable(path_frame)
            return

        robot = transform.transform.translation
        final = self._active_path.poses[-1].pose.position
        distance_xy = math.hypot(final.x - robot.x, final.y - robot.y)
        if (
                math.isfinite(distance_xy)
                and distance_xy <= self._completion_window_xy_tolerance):
            self._near_goal_reported = True
            self._publish_status(
                f'follow_path_near_goal:{self._active_signature}')

    def _warn_tf_unavailable(self, path_frame):
        now = time.monotonic()
        if now - self._last_tf_warning_time < 2.0:
            return
        self._last_tf_warning_time = now
        self.get_logger().warning(
            f'completion-window TF unavailable: {path_frame} '
            f'<- {self._robot_base_frame}')

    # ------------------------------------------------------------------ #
    #  状态发布（dedup）                                                    #
    # ------------------------------------------------------------------ #

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
        if not is_danger:
            return

        # hard danger 会撤销待替换路径；replan fail 不在 danger 前缀中。
        self._pending_path = None
        if self._send_future is not None:
            self._cancel_when_accepted = True
        if self._active_goal_handle is not None:
            self.get_logger().warn(
                f"planner danger status → cancelling FollowPath: {text}")
            self._request_cancel_active_goal()
            self._publish_status('cancel_requested_due_to_planner_status')

    # ------------------------------------------------------------------ #
    #  Path 订阅                                                          #
    # ------------------------------------------------------------------ #

    def _path_cb(self, msg: Path):
        if not _path_is_valid(msg, self.get_logger()):
            self._publish_status('invalid_path')
            return

        sig = _path_signature(msg)
        current_signatures = {
            self._sending_signature,
            self._active_signature,
            self._pending_path[1] if self._pending_path is not None else '',
        }
        if sig in current_signatures:
            self.get_logger().debug('duplicate path ignored')
            self._publish_status('duplicate_path_ignored')
            return

        self._pending_path = (copy.deepcopy(msg), sig)
        if self._active_goal_handle is not None:
            self._request_cancel_active_goal()
            return
        if self._send_future is not None:
            return
        self._try_send_pending_path()

    # ------------------------------------------------------------------ #
    #  Action 发送与生命周期                                               #
    # ------------------------------------------------------------------ #

    def _request_cancel_active_goal(self):
        goal_handle = self._active_goal_handle
        if goal_handle is None or self._cancel_requested:
            return

        self._cancel_requested = True
        active_sequence = self._active_sequence
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as exc:
            # 安全优先：cancel 异常时仍保留 active，禁止发送新 goal。
            self._cancel_requested = False
            self.get_logger().error(f"cancel_goal_async exception: {exc}")
            self._publish_status('follow_path_cancel_failed')
            return

        self._cancel_future = future
        future.add_done_callback(
            lambda completed,
            request_sequence=active_sequence,
            active_handle=goal_handle:
            self._cancel_response_callback(
                completed, request_sequence, active_handle))

    def _cancel_response_callback(self, future, sequence, goal_handle):
        if (
                future is not self._cancel_future
                or sequence != self._active_sequence
                or goal_handle is not self._active_goal_handle):
            return

        self._cancel_future = None
        try:
            response = future.result()
        except Exception as exc:
            self._cancel_requested = False
            self.get_logger().error(f"cancel response exception: {exc}")
            self._publish_status('follow_path_cancel_failed')
            return

        if not getattr(response, 'goals_canceling', []):
            self._cancel_requested = False
            self.get_logger().error("FollowPath cancel request rejected")
            self._publish_status('follow_path_cancel_rejected')

    def _try_send_pending_path(self):
        if (
                self._pending_path is None
                or self._send_future is not None
                or self._active_goal_handle is not None):
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self._publish_status('waiting_for_action_server')
            return

        path_msg, signature = self._pending_path
        self._pending_path = None
        self._send_follow_path(path_msg, signature)

    def _send_follow_path(self, path_msg: Path, signature: str):
        self._publish_status('sending_follow_path')

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

        self._send_sequence += 1
        sequence = self._send_sequence
        try:
            future = self._action_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback)
        except Exception as exc:
            self.get_logger().error(f"send_goal_async exception: {exc}")
            self._publish_status('follow_path_rejected')
            return

        self._send_future = future
        self._sending_signature = signature
        self._sending_sequence = sequence
        future.add_done_callback(
            lambda completed,
            sent_sequence=sequence,
            sent_signature=signature,
            sent_path=copy.deepcopy(path_msg):
            self._goal_response_callback(
                completed, sent_sequence, sent_signature, sent_path))

    def _goal_response_callback(self, future, sequence, signature, path_msg):
        if (
                future is not self._send_future
                or sequence != self._sending_sequence):
            self._cancel_stale_response(future)
            return

        self._send_future = None
        self._sending_signature = ""
        self._sending_sequence = None
        cancel_when_accepted = self._cancel_when_accepted
        self._cancel_when_accepted = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"FollowPath goal response exception: {exc}")
            self._publish_status('follow_path_rejected')
            self._try_send_pending_path()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected")
            self._publish_status('follow_path_rejected')
            self._try_send_pending_path()
            return

        self._active_goal_handle = goal_handle
        self._active_signature = signature
        self._active_path = path_msg
        self._active_sequence = sequence
        self._cancel_requested = False
        self._near_goal_reported = False
        self._cancel_future = None
        self.get_logger().info("FollowPath goal accepted")
        self._publish_status('follow_path_accepted')

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            self.get_logger().error(f"get_result_async exception: {exc}")
            self._publish_status(f'follow_path_aborted:{signature}')
            # 无法观察 terminal 时保留 active，避免并发发送第二个 goal。
            self._request_cancel_active_goal()
            return

        result_future.add_done_callback(
            lambda completed,
            active_sequence=sequence,
            active_handle=goal_handle:
            self._result_callback(
                completed, active_sequence, active_handle))

        if cancel_when_accepted or self._pending_path is not None:
            self._request_cancel_active_goal()

    def _cancel_stale_response(self, future):
        try:
            goal_handle = future.result()
        except Exception:
            return
        if goal_handle is None or not goal_handle.accepted:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - best-effort cancel
            self.get_logger().error(
                f"stale FollowPath cancel exception: {exc}")

    def _result_callback(self, future, sequence, goal_handle):
        if (
                sequence != self._active_sequence
                or goal_handle is not self._active_goal_handle):
            return

        replacement_pending = self._pending_path is not None
        active_signature = self._active_signature
        self._active_goal_handle = None
        self._active_signature = ""
        self._active_path = None
        self._active_sequence = None
        self._cancel_future = None
        self._cancel_requested = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"get_result exception: {exc}")
            self._publish_status(f'follow_path_aborted:{active_signature}')
            self._try_send_pending_path()
            return

        code = result.status
        self.get_logger().info(f"FollowPath result status={code}")

        if code == GoalStatus.STATUS_SUCCEEDED and replacement_pending:
            # cancel race 中旧 path 已成功，不把内部替换误报为最终目标完成。
            self._publish_status(
                f'follow_path_replaced (result_status={code})')
        elif code == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f'follow_path_succeeded:{active_signature}')
        elif code == GoalStatus.STATUS_CANCELED:
            self._publish_status(
                f'follow_path_canceled:{active_signature} '
                f'(result_status={code})')
        elif code == GoalStatus.STATUS_ABORTED:
            self._publish_status(
                f'follow_path_aborted:{active_signature} '
                f'(result_status={code})')
        else:
            self._publish_status(f'follow_path_unknown_result_status_{code}')

        # 只有 terminal result 才会清 active 并推进 latest pending path。
        self._try_send_pending_path()

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
