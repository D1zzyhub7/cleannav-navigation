"""控制器交付前的保守栅格包络检查；不发布速度，不承担 Safety 授权。

检查闭合 footprint 内部以及段间 SE2 插值。每个小段用中点矩形加
位移/旋转上界扩张覆盖整个插值扫掠体，而非仅检查离散采样点。
这只证明指定栅格快照中的几何安全，不证明实际控制器轨迹或动态安全。
"""
from dataclasses import dataclass
import math

from .path_identity import path_signature


@dataclass(frozen=True)
class Grid:
    frame: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    costs: bytes

    def validate(self):
        if (not self.frame or not 0 < self.width <= 10000 or not 0 < self.height <= 10000
                or len(self.costs) != self.width * self.height
                or not all(math.isfinite(v) for v in
                           (self.resolution, self.origin_x, self.origin_y))
                or self.resolution <= 0):
            raise ValueError('invalid costmap')


def planar_poses(path):
    if not path_signature(path):
        raise ValueError('empty path')
    if len(path.poses) > 5000:
        raise ValueError('path too large')
    poses = []
    for p in path.poses:
        q = p.pose.orientation
        if (abs(p.pose.position.z) > 1e-3 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or abs(q.z*q.z + q.w*q.w - 1) > 1e-3):
            raise ValueError('non-planar or non-unit pose')
        poses.append((p.pose.position.x, p.pose.position.y, 2*math.atan2(q.z, q.w)))
    return poses


def _rectangle(x, y, yaw, bounds, expand):
    xmin, xmax, half_width = bounds
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in
            [(xmin-expand, -half_width-expand), (xmax+expand, -half_width-expand),
             (xmax+expand, half_width+expand), (xmin-expand, half_width+expand)]]


def _overlaps_cell(poly, x, y, size):
    # SAT 的闭合边界：接触也拒绝；包含车体内部的单个障碍栅格。
    axes = [(1., 0.), (0., 1.)]
    for a, b in zip(poly[:2], poly[1:3]):
        axes.append((-(b[1]-a[1]), b[0]-a[0]))
    corners = [(x, y), (x+size, y), (x+size, y+size), (x, y+size)]
    for nx, ny in axes:
        p = [nx*a+ny*b for a,b in poly]
        q = [nx*a+ny*b for a,b in corners]
        if max(p) < min(q)-1e-12 or max(q) < min(p)-1e-12:
            return False
    return True


def _check_rectangle(grid, polygon, *, minimum_cost=254):
    r = grid.resolution
    xs = [(x-grid.origin_x)/r for x, _ in polygon]
    ys = [(y-grid.origin_y)/r for _, y in polygon]
    if min(xs) <= 0 or min(ys) <= 0 or max(xs) >= grid.width or max(ys) >= grid.height:
        raise ValueError('footprint outside costmap')
    # Include the adjacent cell when the footprint lies exactly on a grid line.
    for iy in range(max(0, math.floor(min(ys)-1e-10)), min(grid.height, math.floor(max(ys))+1)):
        for ix in range(max(0, math.floor(min(xs)-1e-10)), min(grid.width, math.floor(max(xs))+1)):
            if grid.costs[iy*grid.width+ix] >= minimum_cost and _overlaps_cell(
                    polygon, grid.origin_x+ix*r, grid.origin_y+iy*r, r):
                raise ValueError(f'footprint blocked cost={grid.costs[iy*grid.width+ix]} cell={ix},{iy}')


def _check_center_clearance(grid, x, y, radius, *, minimum_cost=253):
    """Check the reference-point buffer against inscribed/lethal/unknown cost.

    Nav2's 253 inscribed-inflation cost already represents the robot's
    inscribed radius around a hard obstacle.  Expanding that cost by the full
    vehicle footprint a second time rejects otherwise clear paths.  The full
    oriented footprint is instead checked against hard/unknown cells (254/255)
    while this reference-point buffer preserves the 253 collision boundary.
    """
    r = grid.resolution
    if (x-radius < grid.origin_x or y-radius < grid.origin_y
            or x+radius > grid.origin_x+grid.width*r
            or y+radius > grid.origin_y+grid.height*r):
        raise ValueError('path center clearance outside costmap')
    min_ix = max(0, math.floor((x-radius-grid.origin_x)/r))
    max_ix = min(grid.width-1, math.floor((x+radius-grid.origin_x)/r))
    min_iy = max(0, math.floor((y-radius-grid.origin_y)/r))
    max_iy = min(grid.height-1, math.floor((y+radius-grid.origin_y)/r))
    radius_sq = radius*radius
    for iy in range(min_iy, max_iy+1):
        cell_y0 = grid.origin_y + iy*r
        cell_y1 = cell_y0 + r
        dy = max(cell_y0-y, 0., y-cell_y1)
        for ix in range(min_ix, max_ix+1):
            cost = grid.costs[iy*grid.width+ix]
            if cost < minimum_cost:
                continue
            cell_x0 = grid.origin_x + ix*r
            cell_x1 = cell_x0 + r
            dx = max(cell_x0-x, 0., x-cell_x1)
            if dx*dx + dy*dy <= radius_sq + 1e-12:
                raise ValueError(f'path center buffer blocked cost={cost} cell={ix},{iy}')


def validate_path(path, grid, *, bounds=(-.55, .55, .34), clearance=None,
                  m0=False, goal=None):
    """返回审计数据；任何非法/碰撞/未知/越界都抛 ValueError，不隐式回退。

    bounds 采用 Campus 正式 footprint，覆盖 chassis、轮胎与 lidar 碰撞几何。
    clearance 为交付路径的误差余量；与 Guard/Keepout/Unknown 阈值无关。
    """
    grid.validate()
    if clearance is None:
        # M0: 0.08m tracking + 0.10rad body rotation + registration/stop allowance.
        clearance = .30 if m0 else .10
    if (len(bounds) != 3 or not all(math.isfinite(v) for v in (*bounds, clearance))
            or bounds[0] >= bounds[1] or bounds[2] <= 0 or clearance < 0):
        raise ValueError('invalid footprint bounds')
    if path.header.frame_id != grid.frame:
        raise ValueError('path/costmap frame mismatch')
    poses = planar_poses(path)
    if m0:
        if goal is None or goal.header.frame_id != grid.frame:
            raise ValueError('M0 needs same-frame goal')
        x0,y0,yaw0 = poses[0]
        length = 0.
        for a,b in zip(poses, poses[1:]):
            dx,dy = b[0]-a[0],b[1]-a[1]
            ds = math.hypot(dx,dy); length += ds
            da=abs(math.remainder(b[2]-a[2],2*math.pi))
            if da > ds/1.12+1e-5:
                raise ValueError('M0 segment curvature exceeds vehicle radius')
            if abs(-dx*math.sin(a[2])+dy*math.cos(a[2])) > .05*ds+1e-5:
                raise ValueError('M0 lateral segment incompatible with Ackermann')
            if dx*math.cos(a[2])+dy*math.sin(a[2]) < -1e-5:
                raise ValueError('M0 reverse/cusp')
        along = (poses[-1][0]-x0)*math.cos(yaw0)+(poses[-1][1]-y0)*math.sin(yaw0)
        if not .35 <= along <= .80 or length > .85:
            raise ValueError('M0 path distance')
        for x,y,yaw in poses:
            if (abs(-(x-x0)*math.sin(yaw0)+(y-y0)*math.cos(yaw0)) > .03
                    or abs(math.remainder(yaw-yaw0,2*math.pi)) > .05):
                raise ValueError('M0 not straight')
        g = goal.pose.position
        if math.hypot(poses[-1][0]-g.x, poses[-1][1]-g.y) > .06:
            raise ValueError('M0 planner endpoint misses requested goal')
    radius = math.hypot(max(abs(bounds[0]), abs(bounds[1])), bounds[2])
    checks = 0
    center_checks = 0
    if len(poses) == 1:
        x, y, yaw = poses[0]
        _check_rectangle(grid, _rectangle(x, y, yaw, bounds, clearance))
        _check_center_clearance(grid, x, y, clearance)
        checks = center_checks = 1
    for a,b in zip(poses, poses[1:]):
        dx,dy = b[0]-a[0],b[1]-a[1]
        da = math.remainder(b[2]-a[2],2*math.pi)
        distance = math.hypot(dx,dy)
        n = max(1, math.ceil(distance/.02), math.ceil(abs(da)/.02))
        if n > 10000:
            raise ValueError('path segment too large')
        # Each original body point differs from its midpoint by <= bound.
        expansion = clearance + distance/(2*n) + 2*radius*math.sin(abs(da)/(4*n))
        for i in range(n):
            t = (i+.5)/n
            x, y = a[0]+t*dx, a[1]+t*dy
            poly = _rectangle(x, y, a[2]+t*da, bounds, expansion)
            # Compare the whole footprint with lethal and unknown cells.  A
            # 253 cell is already inflated by Nav2 for the inscribed radius,
            # so test the base reference-point sweep against that threshold.
            _check_rectangle(grid, poly, minimum_cost=254)
            _check_center_clearance(grid, x, y,
                                    clearance + distance/(2*n), minimum_cost=253)
            checks += 1
            center_checks += 1
    return dict(path_sha256=path_signature(path), rectangles=checks,
                center_clearance_checks=center_checks, clearance_m=clearance,
                footprint_bounds=list(bounds), mode='M0_FORWARD' if m0 else 'GEOMETRY',
                proof='bounded SE2 interpolation sweep in supplied costmap snapshot')
