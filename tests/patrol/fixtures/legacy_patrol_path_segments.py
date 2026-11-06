"""Split a closed offline path into FollowPath goals the current checker can finish.

Nav2 SimpleGoalChecker treats the last pose as the goal. A closed patrol path
starts on that same pose, so one controller goal would succeed before the robot
moves. The first goal ends at the farthest sample. The second goal continues
from there and closes the loop.
"""

from __future__ import annotations

import math


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
    far = max(range(1, len(points) - 1), key=lambda index: math.dist(start, points[index]))
    if math.dist(start, points[far]) < minimum_far_m:
        raise ValueError('closed path has no point far from the start')
    return ((0, far + 1), (far, len(points)))
