# cleannav_global_planner/a_star_core.py
# 纯 Python A* 全局路径规划核心——不导入 rclpy

import heapq
import math


SQRT2 = math.sqrt(2)


def plan(width, height, data, start_cell, goal_cell,
         allow_diagonal=True, prevent_corner_cutting=True):
    """A* 搜索。

    Args:
        width:  地图宽度（列数）
        height: 地图高度（行数）
        data:   一维 occupancy 列表，idx = row * width + col
        start_cell: (col, row)
        goal_cell:  (col, row)
        allow_diagonal:      是否允许对角移动
        prevent_corner_cutting: 是否禁止对角穿角（仅 allow_diagonal=True 时生效）

    Returns:
        [(col, row), ...] — 有序路径（含起终点）

    Raises:
        ValueError: 起点/终点越界或被占据、无可达路径、width/height ≤ 0
    """
    if width <= 0 or height <= 0:
        raise ValueError(f'invalid dims {width}×{height}')
    if len(data) != width * height:
        raise ValueError(f'data length {len(data)} != {width}×{height}')

    sc, sr = start_cell
    gc, gr = goal_cell

    if not (0 <= sc < width and 0 <= sr < height):
        raise ValueError(f'start ({sc},{sr}) out of bounds')
    if not (0 <= gc < width and 0 <= gr < height):
        raise ValueError(f'goal ({gc},{gr}) out of bounds')

    if not _traversable(data, width, sc, sr):
        raise ValueError(f'start ({sc},{sr}) occupied')
    if not _traversable(data, width, gc, gr):
        raise ValueError(f'goal ({gc},{gr}) occupied')

    neighbors = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1)]
    if allow_diagonal:
        neighbors += [(1, 1, SQRT2), (1, -1, SQRT2),
                      (-1, 1, SQRT2), (-1, -1, SQRT2)]

    def _h(col, row):
        dc = abs(col - gc)
        dr = abs(row - gr)
        if allow_diagonal:
            return min(dc, dr) * SQRT2 + abs(dc - dr)
        return dc + dr  # Manhattan

    start_idx = sr * width + sc
    goal_idx = gr * width + gc

    g_score = {start_idx: 0.0}
    came_from = {}
    open_set = [(0.0, start_idx, sc, sr)]
    closed = set()

    while open_set:
        _, cur_idx, cx, cy = heapq.heappop(open_set)
        if cur_idx in closed:
            continue
        closed.add(cur_idx)

        if (cx, cy) == (gc, gr):
            path = []
            idx = cur_idx
            while idx in came_from:
                path.append((idx % width, idx // width))
                idx = came_from[idx]
            path.append(start_cell)
            path.reverse()
            return path

        for dc, dr, cost in neighbors:
            nx, ny = cx + dc, cy + dr
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            nidx = ny * width + nx
            if nidx in closed:
                continue
            if not _traversable(data, width, nx, ny):
                continue

            if dc != 0 and dr != 0 and prevent_corner_cutting:
                if not _traversable(data, width, cx + dc, cy):
                    continue
                if not _traversable(data, width, cx, cy + dr):
                    continue

            new_g = g_score[cur_idx] + cost
            if nidx in g_score and new_g >= g_score[nidx]:
                continue

            g_score[nidx] = new_g
            f = new_g + _h(nx, ny)
            came_from[nidx] = cur_idx
            heapq.heappush(open_set, (f, nidx, nx, ny))

    raise ValueError(f'no path from {start_cell} to {goal_cell}')


def _traversable(data, width, col, row):
    if not (0 <= col < width and 0 <= row < len(data) // width):
        return False
    return data[row * width + col] == 0


# ============== self-check ==============
def _self_check():
    w, h = 5, 5
    empty = [0] * (w * h)

    # 1: normal
    path = plan(w, h, empty, (0, 0), (4, 4))
    assert len(path) > 1

    # 2: start blocked
    blocked = list(empty); blocked[0] = 100
    try:
        plan(w, h, blocked, (0, 0), (4, 4))
        assert False
    except ValueError:
        pass

    # 3: corner cutting (2×2, prevent enabled → no path)
    grid2 = [0, 100, 100, 0]  # 2×2: (1,0) blocked, (0,1) blocked
    try:
        plan(2, 2, grid2, (0, 0), (1, 1), prevent_corner_cutting=True)
        assert False, 'should be no path'
    except ValueError:
        pass

    # 4: corner cutting disabled → direct diagonal path
    path2 = plan(2, 2, grid2, (0, 0), (1, 1), prevent_corner_cutting=False)
    has_diag = any(abs(b[0]-a[0]) == 1 and abs(b[1]-a[1]) == 1
                   for a, b in zip(path2, path2[1:]))
    assert has_diag, "should allow diagonal when prevent=False"

    # 5: allow_diagonal=False (no diagonals in path)
    path3 = plan(w, h, empty, (0, 0), (4, 3), allow_diagonal=False)
    for a, b in zip(path3, path3[1:]):
        dc, dr = b[0] - a[0], b[1] - a[1]
        assert dc == 0 or dr == 0, f'diagonal found: {a}→{b}'

    print('a_star_core self-check passed')


if __name__ == '__main__':
    _self_check()
