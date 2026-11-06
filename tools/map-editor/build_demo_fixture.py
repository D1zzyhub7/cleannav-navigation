#!/usr/bin/env python3
"""Create the editor's isolated synthetic OccupancyGrid fixture."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path

from PIL import Image
import numpy as np

OUT = Path(__file__).resolve().parent/'fixtures/perimeter_demo'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    # 15 x 15 m, 5 cm cells, with a boundary and an interior obstacle.
    # This is deliberately a geometry fixture, not RTAB data or a Gazebo map.
    w = h = 300
    grid = np.zeros((h, w), dtype=np.int16)
    grid[:5, :] = 100
    grid[-5:, :] = 100
    grid[:, :5] = 100
    grid[:, -5:] = 100
    grid[120:180, 140:160] = 100
    grid[200:210, 80:90] = -1
    record = dict(frame_id='map', width=w, height=h, resolution=0.05,
                  origin=[0.0, 0.0, 0.0], stamp=dict(sec=0, nanosec=0),
                  data=grid.ravel().tolist())
    encoded = (json.dumps(record, separators=(',', ':'))+'\n').encode()
    (OUT/'occupancy_grid.json').write_bytes(encoded)
    # PGM follows OccupancyGrid display convention: row zero at the image bottom.
    pixels = np.where(grid < 0, 205, np.where(grid <= 25, 254, 0)).astype('uint8')
    Image.fromarray(np.flipud(pixels)).save(OUT/'map.pgm')
    (OUT/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    sha = hashlib.sha256(encoded).hexdigest()
    manifest = dict(name='DEMO-perimeter-15m', created_at='fixture-2026-10-14-v1',
                    sha256={'occupancy_grid.json': sha}, editor_fixture=True,
                    simulation_only=True, test_only=True,
                    note='Synthetic map fixture; contains no RTAB-Map database or physical survey.')
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Wrote TEST FIXTURE only: {OUT} sha256={sha}')


if __name__ == '__main__':
    main()
