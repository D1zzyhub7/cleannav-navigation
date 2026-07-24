# cleannav_global_planner/inflation_core.py
# 纯 Python 欧氏圆形障碍膨胀——不导入 rclpy

import math


def inflate_strict_binary(data, width, height, radius_cells):
    """严格二值化 + 欧氏圆形膨胀。

    输入：原始 OccupancyGrid 一维 data。
    规则：仅值 0 为自由，其余均为障碍种子。
    输出：长度不变、仅含 0 或 100 的膨胀图。
    半径 0 时只做严格二值化。
    radius_cells 必须 >=0 整数，否则抛出 ValueError。
    不连锁扩张——仅从原始种子出发做 mask。
    """
    if not isinstance(radius_cells, int) or radius_cells < 0:
        raise ValueError(f'radius_cells must be int >= 0, got {radius_cells}')
    if width <= 0 or height <= 0:
        raise ValueError(f'invalid dims {width}×{height}')
    if len(data) != width * height:
        raise ValueError(f'data len {len(data)} != {width}×{height}')

    # 步骤 1：严格二值化——仅 0 为自由
    seeds = []
    binary = [0] * len(data)
    for idx, val in enumerate(data):
        if val != 0:
            binary[idx] = 100
            seeds.append(idx)

    # 步骤 2：欧氏圆形膨胀（radius=0 时跳过）
    if radius_cells == 0:
        return binary

    result = list(binary)  # 从二值图继承障碍
    r2 = radius_cells * radius_cells

    for seed_idx in seeds:
        sc = seed_idx % width
        sr = seed_idx // width
        # 遍历圆内候选格（边界裁剪）
        col_min = max(0, sc - radius_cells)
        col_max = min(width - 1, sc + radius_cells)
        row_min = max(0, sr - radius_cells)
        row_max = min(height - 1, sr + radius_cells)
        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                dc = c - sc
                dr = r - sr
                if dc * dc + dr * dr <= r2:
                    idx = r * width + c
                    result[idx] = 100

    return result
