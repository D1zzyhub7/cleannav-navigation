#!/usr/bin/env python3
"""Require a forward-motion, sustained-stop, resumed-motion sequence."""

from __future__ import annotations

import csv
import json
import math
import sys


MOVE_THRESHOLD = 0.03
STOP_THRESHOLD = 0.015
MIN_STOP_SEC = 0.50
MIN_DISPLACEMENT_M = 0.25


def analyze(path: str) -> dict[str, object]:
    commands: list[tuple[float, float]] = []
    odometry: list[tuple[float, float]] = []
    obstacle: list[tuple[float, float]] = []
    with open(path, newline='') as stream:
        for row in csv.DictReader(stream):
            if row['event'] == 'cmd':
                commands.append((float(row['elapsed_sec']), float(row['linear_x'])))
            elif row['event'] == 'odom':
                odometry.append((float(row['x']), float(row['y'])))
            elif row['event'] == 'obstacle':
                obstacle.append((float(row['x']), float(row['y'])))

    first_move = next((i for i, (_, vx) in enumerate(commands)
                       if abs(vx) >= MOVE_THRESHOLD), None)
    stop_start = None
    stop_end = None
    resumed_at = None
    if first_move is not None:
        candidate = None
        for index in range(first_move + 1, len(commands)):
            stamp, vx = commands[index]
            if abs(vx) <= STOP_THRESHOLD:
                if candidate is None:
                    candidate = stamp
                if stamp - candidate >= MIN_STOP_SEC:
                    stop_start = candidate
                    stop_end = stamp
            elif stop_end is not None:
                # Preserve a confirmed stop while velocity ramps through the
                # dead band between STOP_THRESHOLD and MOVE_THRESHOLD.
                if abs(vx) >= MOVE_THRESHOLD:
                    resumed_at = stamp
                    break
            else:
                candidate = None

    displacement = 0.0
    if len(odometry) >= 2:
        displacement = math.hypot(
            odometry[-1][0] - odometry[0][0],
            odometry[-1][1] - odometry[0][1],
        )
    passed = (
        first_move is not None
        and stop_start is not None
        and resumed_at is not None
        and displacement >= MIN_DISPLACEMENT_M
    )
    return {
        'passed': passed,
        'command_samples': len(commands),
        'odometry_samples': len(odometry),
        'obstacle_samples': len(obstacle),
        'first_move_sec': commands[first_move][0] if first_move is not None else None,
        'stop_start_sec': stop_start,
        'stop_end_sec': stop_end,
        'resumed_sec': resumed_at,
        'displacement_m': round(displacement, 4),
        'obstacle_y_min': round(min((p[1] for p in obstacle), default=0.0), 4),
        'obstacle_y_max': round(max((p[1] for p in obstacle), default=0.0), 4),
    }


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit('usage: analyze_dynamic_recovery.py MOTION.csv')
    result = analyze(sys.argv[1])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
