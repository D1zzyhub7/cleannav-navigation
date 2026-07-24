#!/usr/bin/env python3
"""A* 全局规划器节点 V2——只生成路径，不发布 /cmd_vel。"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, LookupException
from tf2_ros.transform_listener import TransformListener

from .a_star_core import plan as a_star_plan
from .inflation_core import inflate_strict_binary


def _finite_norm_quat(q):
    return (math.isfinite(q.x) and math.isfinite(q.y) and
            math.isfinite(q.z) and math.isfinite(q.w) and
            math.hypot(q.x, q.y, q.z, q.w) > 1e-9)


def _yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def _compute_path_orientations(positions, target_orientation=None, epsilon=1e-6):
    n = len(positions)
    if n == 0:
        return []

    orientations = []
    for i in range(n):
        x, y = positions[i]
        yaw = None
        # 向后寻找第一个不重复点
        for j in range(i + 1, n):
            if math.hypot(positions[j][0] - x, positions[j][1] - y) > epsilon:
                yaw = math.atan2(positions[j][1] - y, positions[j][0] - x)
                break
        # 向后找不到则向前寻找
        if yaw is None:
            for k in range(i - 1, -1, -1):
                if math.hypot(x - positions[k][0], y - positions[k][1]) > epsilon:
                    yaw = math.atan2(y - positions[k][1], x - positions[k][0])
                    break
        # 单点路径：尝试使用目标 orientation
        if yaw is None and target_orientation is not None and _finite_norm_quat(target_orientation):
            q = target_orientation
            q_norm = math.hypot(q.x, q.y, q.z, q.w)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y) / q_norm,
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z) / (q_norm * q_norm))
        # 最终兜底
        if yaw is None:
            yaw = 0.0
        orientations.append(_yaw_to_quaternion(yaw))
    return orientations


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('cleannav_global_planner',
                         parameter_overrides=[
                             Parameter('use_sim_time', Parameter.Type.BOOL, True)],
                         automatically_declare_parameters_from_overrides=True)
        sim_val = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time={sim_val}')

        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('prevent_corner_cutting', True)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('inflation_radius_m', 0.25)

        self._map_msg = None
        self._loc_msg = None
        self._goal_msg = None
        self._pending = False
        self._last_status = ''
        self._last_completed_key = None
        self._map_fingerprint = 0
        self._raw_binary_data = None
        self._inflated_data = None
        self._inflation_radius_cells = 0

        map_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        reliable_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                  reliability=QoSReliabilityPolicy.RELIABLE,
                                  durability=QoSDurabilityPolicy.VOLATILE)

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/rtabmap/map', self._map_cb, map_qos)
        self._loc_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose',
            self._loc_cb, reliable_qos)
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, reliable_qos)

        self._path_pub = self.create_publisher(Path, '/cleannav/global_path', reliable_qos)
        self._status_pub = self.create_publisher(
            String, '/cleannav/global_planner_status', reliable_qos)
        self._inflated_map_pub = self.create_publisher(
            OccupancyGrid, '/cleannav/inflated_map', map_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(0.25, self._timer_cb)

        self.add_on_set_parameters_callback(self._on_param_change)

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'inflation_radius_m':
                return SetParametersResult(
                    successful=False,
                    reason='inflation_radius_m is fixed after startup; '
                           'restart with launch override')
        return SetParametersResult(successful=True)

    def _reset_inflation(self):
        self._raw_binary_data = None
        self._inflated_data = None
        self._inflation_radius_cells = 0

    def _map_cb(self, msg):
        old_is_none = self._map_msg is None
        mg = msg.info
        w, h, res = mg.width, mg.height, mg.resolution

        # 异常地图检查
        if w <= 0 or h <= 0:
            self._reset_inflation()
            self._note_fail(f'invalid map dims: {w}×{h}')
            return
        if not math.isfinite(res) or res <= 0:
            self._reset_inflation()
            self._note_fail(f'invalid map resolution: {res}')
            return
        if len(msg.data) != w * h:
            self._reset_inflation()
            self._note_fail(f'map data len {len(msg.data)} != {w}×{h}')
            return

        ox = mg.origin.position.x
        oy = mg.origin.position.y
        oq = mg.origin.orientation
        oyaw = math.atan2(2.0 * (oq.w * oq.z + oq.x * oq.y),
                          1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))

        new_fingerprint = (
            msg.header.frame_id,
            w, h, res,
            ox, oy, oyaw,
            tuple(msg.data),
        )
        map_changed = (old_is_none or new_fingerprint != self._map_fingerprint)
        self._map_msg = msg
        self._map_fingerprint = new_fingerprint

        if map_changed:
            if old_is_none:
                self.get_logger().info(f'map: {w}×{h}')

            # 验证膨胀半径
            irm = self.get_parameter('inflation_radius_m').value
            if not math.isfinite(irm) or irm < 0:
                self._reset_inflation()
                self._note_fail(f'invalid inflation_radius_m: {irm}')
                return

            rc = int(math.ceil(irm / res))
            self._inflation_radius_cells = rc
            raw = list(msg.data)
            self._raw_binary_data = [100 if v != 0 else 0 for v in raw]
            self._inflated_data = inflate_strict_binary(
                raw, w, h, rc)
            inflated_msg = OccupancyGrid()
            inflated_msg.header = msg.header
            inflated_msg.info = mg
            inflated_msg.data = self._inflated_data
            self._inflated_map_pub.publish(inflated_msg)
            raw_obs = sum(1 for v in raw if v != 0)
            inf_obs = sum(1 for v in self._inflated_data if v == 100)
            self.get_logger().info(
                f'inflated map: radius={irm:.3f}m cells={rc} '
                f'raw_obstacles={raw_obs} inflated_obstacles={inf_obs}')
            self._pending = True
            self._last_status = ''

    def _loc_cb(self, msg):
        self._loc_msg = msg
        self._pending = True

    def _goal_cb(self, msg):
        self._goal_msg = msg
        self.get_logger().info(
            f'goal: ({msg.pose.position.x:.3f},{msg.pose.position.y:.3f})')
        self._pending = True
        self._last_status = ''

    def _timer_cb(self):
        if not self._pending:
            return
        self._try_plan()

    def _try_plan(self):
        if self._map_msg is None or self._goal_msg is None:
            return

        map_frame = self.get_parameter('map_frame').value
        robot_frame = self.get_parameter('robot_frame').value
        mg = self._map_msg.info
        mf = self._map_msg.header.frame_id

        if mf != map_frame:
            self._note_fail(f'map frame [{mf}] != [{map_frame}]')
            return
        if self._goal_msg.header.frame_id != map_frame:
            self._note_fail(
                f'goal frame [{self._goal_msg.header.frame_id}] != [{map_frame}]')
            return
        gp = self._goal_msg.pose.position
        if not (math.isfinite(gp.x) and math.isfinite(gp.y)):
            self._note_fail(f'goal position non-finite')
            return

        if self._loc_msg is None:
            self._note_wait('no localization pose')
            return
        lf = self._loc_msg.header.frame_id
        if lf != map_frame:
            self._note_wait(f'loc frame [{lf}] != [{map_frame}]')
            return
        lx, ly = self._loc_msg.pose.pose.position.x, self._loc_msg.pose.pose.position.y
        if not (math.isfinite(lx) and math.isfinite(ly)):
            self._note_wait('loc x/y non-finite')
            return
        lq = self._loc_msg.pose.pose.orientation
        if not _finite_norm_quat(lq):
            self._note_wait('loc quaternion invalid')
            return
        lcov = self._loc_msg.pose.covariance
        if not (math.isfinite(lcov[0]) and lcov[0] >= 0 and
                math.isfinite(lcov[7]) and lcov[7] >= 0):
            self._note_wait('loc cov invalid')
            return

        w, h, res = mg.width, mg.height, mg.resolution
        if w <= 0 or h <= 0 or res <= 0:
            self._note_fail(f'map dims invalid: {w}×{h}, {res}')
            return

        unk = self.get_parameter('unknown_is_obstacle').value
        if not unk:
            self._note_fail('strict-free V2 requires unknown_is_obstacle=True')
            return

        ox, oy = mg.origin.position.x, mg.origin.position.y
        oq = mg.origin.orientation
        oyaw = math.atan2(2.0 * (oq.w * oq.z + oq.x * oq.y),
                          1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))
        if not (math.isfinite(ox) and math.isfinite(oy) and
                _finite_norm_quat(oq) and math.isfinite(oyaw)):
            self._note_fail('map origin invalid')
            return

        if not (self._raw_binary_data is not None and
                len(self._raw_binary_data) == w * h and
                self._inflated_data is not None and
                len(self._inflated_data) == w * h):
            self._note_fail('inflation data not ready')
            return

        try:
            t = self._tf_buffer.lookup_transform(
                map_frame, robot_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except (TransformException, LookupException) as e:
            self._note_wait(f'TF fail: {e}')
            return

        tx, ty = t.transform.translation.x, t.transform.translation.y
        if not (math.isfinite(tx) and math.isfinite(ty)):
            self._note_wait('TF trans non-finite')
            return
        tq = t.transform.rotation
        if not _finite_norm_quat(tq):
            self._note_wait('TF quaternion invalid')
            return

        cos_o = math.cos(-oyaw)
        sin_o = math.sin(-oyaw)

        def _map_to_cell(mx, my):
            dx, dy = mx - ox, my - oy
            gx_f = (dx * cos_o - dy * sin_o) / res
            gy_f = (dx * sin_o + dy * cos_o) / res
            return int(math.floor(gx_f)), int(math.floor(gy_f))

        sc, sr = _map_to_cell(tx, ty)
        gc, gr = _map_to_cell(gp.x, gp.y)
        if not (0 <= sc < w and 0 <= sr < h):
            self._note_fail(f'start cell ({sc},{sr}) out of bounds')
            return
        if not (0 <= gc < w and 0 <= gr < h):
            self._note_fail(f'goal cell ({gc},{gr}) out of bounds')
            return

        request_key = (
            mf,
            self._map_fingerprint,
            sc, sr, gc, gr,
            bool(self.get_parameter('allow_diagonal').value),
            bool(self.get_parameter('prevent_corner_cutting').value),
            bool(self.get_parameter('unknown_is_obstacle').value),
            self._inflation_radius_cells,
        )
        if request_key == self._last_completed_key:
            self._pending = False
            return

        # 起终点安全检查
        rb = self._raw_binary_data
        infl = self._inflated_data
        start_idx = sr * w + sc
        goal_idx = gr * w + gc

        if rb[start_idx] == 100:
            self._finish_terminal(request_key, 'start occupied in raw map')
            return
        if rb[goal_idx] == 100:
            self._finish_terminal(request_key, 'goal occupied in raw map')
            return
        if infl[start_idx] == 100:
            self._finish_terminal(request_key, 'start in inflation zone')
            return
        if infl[goal_idx] == 100:
            self._finish_terminal(request_key, 'goal in inflation zone')
            return

        try:
            path_cells = a_star_plan(
                w, h, infl, (sc, sr), (gc, gr),
                allow_diagonal=self.get_parameter('allow_diagonal').value,
                prevent_corner_cutting=self.get_parameter('prevent_corner_cutting').value)
        except ValueError as e:
            self._finish_terminal(
                request_key, f'A* fail after inflation: {e}')
            return

        def _cell_to_map(col, row):
            lx = (col + 0.5) * res
            ly = (row + 0.5) * res
            cos_a, sin_a = math.cos(oyaw), math.sin(oyaw)
            mx = ox + lx * cos_a - ly * sin_a
            my = oy + lx * sin_a + ly * cos_a
            return mx, my

        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = map_frame
        path_msg.header.stamp = stamp

        # 先计算所有世界坐标位置
        positions = [_cell_to_map(col, row) for col, row in path_cells]

        # 从位置计算切线朝向
        orientations = _compute_path_orientations(
            positions,
            target_orientation=self._goal_msg.pose.orientation
            if hasattr(self, '_goal_msg') and self._goal_msg is not None
            else None,
        )

        for (mx, my), orientation in zip(positions, orientations):
            ps = PoseStamped()
            ps.header.frame_id = map_frame
            ps.header.stamp = stamp
            ps.pose.position = Point(x=mx, y=my, z=0.0)
            ps.pose.orientation = orientation
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)
        self._last_completed_key = request_key
        self._pending = False
        self._last_status = ''
        self._publish_status(f'path: {len(path_cells)} cells')

    def _publish_status_once(self, text):
        if text == self._last_status:
            return
        self._last_status = text
        self._publish_status(text)

    def _note_wait(self, text):
        self._publish_status_once(text)

    def _finish_terminal(self, request_key, text):
        self._last_completed_key = request_key
        self._pending = False
        self._publish_status_once(text)

    def _note_fail(self, text):
        self._publish_status_once(text)
        self._pending = False

    def _publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
