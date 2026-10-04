#!/usr/bin/env python3
"""Create a map matching the empty Gazebo world used by headless tests."""

from __future__ import annotations

from pathlib import Path
import sys


WIDTH = 300
HEIGHT = 300
RESOLUTION = 0.05
ORIGIN_X = -7.5
ORIGIN_Y = -7.5


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit('usage: create_headless_empty_map.py OUTPUT.yaml')
    yaml_path = Path(sys.argv[1]).resolve()
    pgm_path = yaml_path.with_suffix('.pgm')
    pgm_path.write_bytes(
        f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii')
        + bytes([254]) * WIDTH * HEIGHT
    )
    yaml_path.write_text(
        '\n'.join([
            f'image: {pgm_path}',
            'mode: trinary',
            f'resolution: {RESOLUTION}',
            f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.25',
            '',
        ]),
        encoding='utf-8',
    )


if __name__ == '__main__':
    main()
