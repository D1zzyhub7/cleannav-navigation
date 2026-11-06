"""Generic planar task constraints; independent of Mission task identifiers.

The same bounded SE(2) interpolation proof as collision admission is used for
polygon containment. Checking just centers or just polygon/footprint vertices
would miss a concave notch and is deliberately insufficient here.
"""
import math
from cleannav_global_planner.path_safety import planar_poses, _rectangle


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    n = dx*dx+dy*dy
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/n)) if n else 0.
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def _intersect(a, b, c, d):
    if min(_distance(a,c,d), _distance(b,c,d), _distance(c,a,b), _distance(d,a,b)) <= 1e-9:
        return True
    return (_cross(a,b,c)>0) != (_cross(a,b,d)>0) and (_cross(c,d,a)>0) != (_cross(c,d,b)>0)


def validate_boundary(points):
    points = tuple(tuple(float(x) for x in p) for p in points)
    if not 3 <= len(points) <= 256 or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in points):
        raise ValueError('boundary needs 3..256 finite planar vertices')
    edges = tuple(zip(points, points[1:]+points[:1]))
    area = 0.
    for i, (a,b) in enumerate(edges):
        if math.dist(a,b) < 1e-6:
            raise ValueError('duplicate boundary vertices')
        area += a[0]*b[1]-a[1]*b[0]
        for j in range(i+1,len(edges)):
            if j == i+1 or (i == 0 and j == len(edges)-1):
                continue
            if _intersect(a,b,*edges[j]):
                raise ValueError('boundary is not simple')
    if abs(area) < .0001:
        raise ValueError('boundary has no usable area')
    return points


def contains_point(p, boundary):
    inside = False
    for a,b in zip(boundary, boundary[1:]+boundary[:1]):
        if _distance(p,a,b) <= 1e-9:
            return False
        if (a[1]>p[1]) != (b[1]>p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return inside


def contains_rectangle(rectangle, boundary):
    if not all(contains_point(p,boundary) for p in rectangle):
        return False
    for a,b in zip(rectangle, rectangle[1:]+rectangle[:1]):
        if any(_intersect(a,b,c,d) for c,d in zip(boundary,boundary[1:]+boundary[:1])):
            return False
    return True


def contains_pose(pose, boundary, *, bounds=(-.55,.55,.34), margin=0.):
    if len(pose)!=3 or not all(math.isfinite(x) for x in pose) or not math.isfinite(margin) or margin<0:
        return False
    return contains_rectangle(_rectangle(*pose,bounds,margin),boundary)


def validate_swept_path(path, boundary, *, bounds=(-.55,.55,.34), margin=.15):
    boundary = validate_boundary(boundary)
    if not math.isfinite(margin) or margin < .1:
        raise ValueError('invalid task boundary margin')
    poses = planar_poses(path)
    radius = math.hypot(max(abs(bounds[0]),abs(bounds[1])),bounds[2])
    checks = 0
    if len(poses)==1 and not contains_pose(poses[0],boundary,bounds=bounds,margin=margin):
        raise ValueError('footprint outside task polygon')
    for a,b in zip(poses,poses[1:]):
        distance = math.dist(a[:2],b[:2]); da = math.remainder(b[2]-a[2],2*math.pi)
        n = max(1,math.ceil(distance/.02),math.ceil(abs(da)/.02))
        if n>10000:
            raise ValueError('constraint path segment too large')
        expand = margin + distance/(2*n) + 2*radius*math.sin(abs(da)/(4*n))
        for i in range(n):
            t = (i+.5)/n
            pose = (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*da)
            if not contains_pose(pose,boundary,bounds=bounds,margin=expand):
                raise ValueError('swept footprint leaves task polygon')
            checks += 1
    return dict(boundary_checks=max(checks,1),boundary_margin_m=margin,
                boundary_vertices=len(boundary),boundary_proof='bounded SE2 full footprint sweep inside simple polygon')
