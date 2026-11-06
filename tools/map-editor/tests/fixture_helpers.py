"""Test-only exact captures; never bypass MapModel validation for production assets."""
import gzip
import hashlib
import io
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from core import MapModel, grid_geometry, vehicle

FIXTURES = Path(__file__).resolve().parent / 'fixtures'
SOURCES = json.loads((FIXTURES / 'SOURCES.json').read_text())['files']


def captured(relative):
    path = FIXTURES / relative
    record = next(x for x in SOURCES if x['fixture'] == str(path.relative_to(FIXTURES.parents[1])))
    raw = gzip.decompress(path.read_bytes())
    assert hashlib.sha256(raw).hexdigest() == record['sha256'], relative
    return raw


def load_site(label):
    # Derived unit fixture without RTAB DB, explicitly test_only; not a MapModel file loader.
    g = json.loads(captured(label + '/occupancy_grid.json.gz'))
    metadata = json.loads((FIXTURES / label / 'metadata.json').read_text())
    mask = yaml.safe_load(captured(label + '/keepout.yaml.gz'))
    pixels = np.flipud(np.array(Image.open(io.BytesIO(captured(label + '/keepout.pgm.gz'))).convert('L')))
    model = MapModel.__new__(MapModel)
    model.width, model.height = g['width'], g['height']
    model.resolution, model.origin = g['resolution'], g['origin']
    model.data = np.asarray(g['data'], dtype=np.int16).reshape(g['height'], g['width'])
    model.keepout = grid_geometry((1-pixels.astype(float)/255) > mask['occupied_thresh'], mask['resolution'], mask['origin'])
    model.free = grid_geometry((model.data >= 0) & (model.data <= 25), model.resolution, model.origin).difference(model.keepout)
    model.identity = metadata['identity']
    model.vehicle = vehicle()
    model.directory = FIXTURES / label
    model.profile_sha256 = metadata['source_profile_sha256']
    model.test_only = True
    model.provenance = 'Derived real-map offline TEST FIXTURE; no RTAB DB or motion authorization'
    return model


def unpack_projection(directory):
    directory.mkdir()
    for name in ['manifest.json', 'map.yaml', 'map.pgm', 'occupancy_grid.json']:
        (directory / name).write_bytes(captured('projection/' + name + '.gz'))
    (directory / 'source.world').write_bytes(captured('projection/source.world.gz'))
    manifest = json.loads((directory / 'manifest.json').read_text())
    manifest['source_world'] = 'source.world'  # only private test binding changes
    (directory / 'manifest.json').write_text(json.dumps(manifest))
    return directory
