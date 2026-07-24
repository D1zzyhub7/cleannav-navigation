"""纯 Python 静态测试：_compute_path_orientations 路径 orientation 正确性验证。

不初始化 ROS Node，仅在模块级别验证几何逻辑。
"""

import math
import pytest
from geometry_msgs.msg import Quaternion
from cleannav_global_planner.global_planner_node import (
    _compute_path_orientations,
    _finite_norm_quat,
)


def _yaw_from_quat(q):
    """从平面 quaternion 提取 yaw（弧度）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _check_orientations(orientations, expected_yaws, tolerance=1e-9):
    """验证 orientations 长度、有限性、范数与 yaw 值。"""
    assert len(orientations) == len(expected_yaws), (
        f"length mismatch: {len(orientations)} != {len(expected_yaws)}")
    for i, (q, exp) in enumerate(zip(orientations, expected_yaws)):
        # 有限性
        assert math.isfinite(q.x)
        assert math.isfinite(q.y)
        assert math.isfinite(q.z)
        assert math.isfinite(q.w)
        # 范数
        norm = math.hypot(q.x, q.y, q.z, q.w)
        assert abs(norm - 1.0) < 1e-9, f"[{i}] norm={norm}（应为 1）"
        # 全零检测
        assert not (q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0), (
            f"[{i}] all-zero quaternion")
        # yaw 值
        yaw = _yaw_from_quat(q)
        assert abs(yaw - exp) < tolerance, (
            f"[{i}] yaw={yaw} expected={exp}（差 {abs(yaw - exp)}）")


# -------- 场景 --------


def test_scenario_a_horizontal_forward():
    """场景 A：水平正向路径 [(0,0),(1,0),(2,0)]，yaw 应为 0。"""
    positions = [(0, 0), (1, 0), (2, 0)]
    orientations = _compute_path_orientations(positions)
    expected = [0.0, 0.0, 0.0]
    _check_orientations(orientations, expected)


def test_scenario_b_vertical_downward():
    """场景 B：竖直负向路径 [(0,0),(0,-1),(0,-2)]，yaw 应为 -pi/2。"""
    positions = [(0, 0), (0, -1), (0, -2)]
    orientations = _compute_path_orientations(positions)
    expected = [-math.pi / 2, -math.pi / 2, -math.pi / 2]
    _check_orientations(orientations, expected)


def test_scenario_c_diagonal():
    """场景 C：斜向路径 [(0,0),(1,1),(2,2)]，yaw 应为 pi/4。"""
    positions = [(0, 0), (1, 1), (2, 2)]
    orientations = _compute_path_orientations(positions)
    expected = [math.pi / 4, math.pi / 4, math.pi / 4]
    _check_orientations(orientations, expected)


def test_scenario_d_turn():
    """场景 D：转弯路径 [(0,0),(1,0),(1,-1)]。"""
    positions = [(0, 0), (1, 0), (1, -1)]
    orientations = _compute_path_orientations(positions)
    expected = [0.0, -math.pi / 2, -math.pi / 2]
    _check_orientations(orientations, expected)


def test_scenario_e_duplicates():
    """场景 E：含重复点 [(0,0),(0,0),(1,0)]，所有点应有方向且前两个指向正X。"""
    positions = [(0, 0), (0, 0), (1, 0)]
    orientations = _compute_path_orientations(positions)
    expected = [0.0, 0.0, 0.0]
    _check_orientations(orientations, expected)


def test_scenario_f_single_point():
    """场景 F：单点路径 [(0,0)]。使用兜底 yaw=0（单位 quaternion）。"""
    positions = [(0, 0)]
    orientations = _compute_path_orientations(positions)
    expected = [0.0]
    _check_orientations(orientations, expected)


def test_scenario_f_single_point_with_goal_orientation():
    """场景 F 变体：单点路径 + 用户目标 orientation (yaw=pi/2)。"""
    positions = [(0, 0)]
    target = Quaternion(x=0.0, y=0.0,
                        z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
    orientations = _compute_path_orientations(
        positions, target_orientation=target)
    expected = [math.pi / 2]
    _check_orientations(orientations, expected)


def test_scenario_f_single_point_invalid_goal_orientation():
    """场景 F 变体：单点路径 + 无效目标 orientation（全零）→ 兜底 yaw=0。"""
    positions = [(0, 0)]
    target = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    orientations = _compute_path_orientations(
        positions, target_orientation=target)
    expected = [0.0]
    _check_orientations(orientations, expected)
