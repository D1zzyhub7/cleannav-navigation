"""Split a closed offline path into FollowPath goals the current checker can finish.

Nav2 SimpleGoalChecker treats the last pose as the goal. A closed patrol path
starts on that same pose, so one controller goal would succeed before the robot
moves. Prefer the farthest sample preceded by an approximately straight metre.
If none qualifies, retain the farthest-sample fallback. The second goal
continues from that same sample and closes the loop.
"""

from __future__ import annotations

import math


def _straight_before(points, index):
    """Check the preceding 1 m against the endpoint tangent, within 1 cm."""
    run = 0.0
    back = index
    while back > 0 and run < 1.0:
        run += math.dist(points[back - 1], points[back])
        back -= 1
    if run < 1.0:
        return False
    bx, by = points[index]
    ax, ay = points[index - 1]
    span = math.hypot(bx - ax, by - ay)
    if span < 1e-9:
        return False
    return all(
        abs((bx - ax) * (py - by) - (by - ay) * (px - bx)) / span <= 0.01
        for px, py in points[back:index + 1]
    )


def closed_loop_splits(points, *, closure_m=0.05, minimum_far_m=1.0):
    """Return inclusive-exclusive index pairs over an (x, y) sequence."""
    if len(points) < 2:
        raise ValueError('path needs at least two poses')
    start = points[0]
    end = points[-1]
    if math.dist(start, end) > closure_m:
        return ((0, len(points)),)
    if len(points) < 3:
        raise ValueError('closed path needs a far point')
    far = None
    reach = 0.0
    for index in range(1, len(points) - 1):
        distance = math.dist(start, points[index])
        if distance >= minimum_far_m and distance > reach and _straight_before(points, index):
            far, reach = index, distance
    if far is None:
        far = max(range(1, len(points) - 1),
                  key=lambda index: math.dist(start, points[index]))
    if math.dist(start, points[far]) < minimum_far_m:
        raise ValueError('closed path has no point far from the start')
    return ((0, far + 1), (far, len(points)))
