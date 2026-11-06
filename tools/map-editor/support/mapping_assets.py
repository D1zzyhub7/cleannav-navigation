"""Offline persistent map validation and Linux no-overwrite atomic publish."""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sqlite3

import yaml


DEFAULT_ROOT = '/home/d1zzy/code/cleannav_modularization/cleannav-maps'
REQUIRED = ('rtabmap.db', 'map.yaml', 'map.pgm', 'rtabmap_config.ini', 'occupancy_grid.json', 'rtabmap_params.yaml')


def map_name(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('map name must be 1..80 letters/digits/underscore/hyphen')
    return name


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def prepare_database_continuation(maps_root, source_name, world_path,
                                  expected_world_sha256, destination_db):
    """Copy a validated, closed RTAB map DB for a new mapping session.

    The published map remains immutable. The new session receives a distinct
    SQLite file and the caller must bind the source world by its audited SHA.
    RTAB can then load the previous session and append a new one to this copy.
    """
    root = Path(maps_root).expanduser().resolve(strict=True)
    name = map_name(source_name)
    source = root / name
    if source.is_symlink() or source.resolve(strict=True).parent != root:
        raise ValueError('continuation source must be a direct, non-symlink map under maps_root')
    validate_map(source)
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    if (manifest.get('state') != 'ready' or manifest.get('simulation_only') is not True
            or manifest.get('database_closed_before_copy') is not True
            or manifest.get('rtabmap_exit_code') != 0):
        raise ValueError('continuation source is not a closed, ready simulation map')

    world = Path(world_path).expanduser().resolve(strict=True)
    source_world = Path(manifest.get('world_path', '')).expanduser().resolve(strict=True)
    if world != source_world:
        raise ValueError('continuation map was recorded from a different Gazebo world path')
    expected = str(expected_world_sha256).lower()
    if not re.fullmatch(r'[0-9a-f]{64}', expected):
        raise ValueError('continuation requires the audited 64-character world SHA-256')
    actual_world_sha = digest(world)
    if actual_world_sha != expected:
        raise ValueError('Gazebo world SHA-256 differs from the continuation attestation')

    source_db = source/'rtabmap.db'
    for suffix in ('-wal', '-journal'):
        sidecar = Path(str(source_db)+suffix)
        if sidecar.exists() and sidecar.stat().st_size:
            raise ValueError('continuation source database has a non-empty SQLite sidecar')
    destination = Path(destination_db).resolve()
    if destination.exists() or destination.parent.resolve() == source.resolve():
        raise FileExistsError('continuation destination must be new and separate from the published map')
    if destination.parent.resolve() == source_db.parent.resolve():
        raise ValueError('continuation database must live in a new mapping session directory')
    source_sha_before = digest(source_db)
    if source_sha_before != manifest['sha256']['rtabmap.db']:
        raise ValueError('continuation source database changed after map validation')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source_db.open('rb') as source_file, destination.open('xb') as destination_file:
        shutil.copyfileobj(source_file, destination_file, length=1024*1024)
        destination_file.flush()
        os.fsync(destination_file.fileno())
    source_sha_after = digest(source_db)
    destination_sha = digest(destination)
    if source_sha_before != source_sha_after or destination_sha != source_sha_before:
        raise ValueError('continuation database copy failed byte-for-byte verification')
    if source_db.stat().st_ino == destination.stat().st_ino and source_db.stat().st_dev == destination.stat().st_dev:
        raise ValueError('continuation database is not copy-on-write isolated')
    source_integrity = database_integrity(source_db)
    destination_integrity = database_integrity(destination)
    if source_integrity != destination_integrity or source_integrity != manifest['database']:
        raise ValueError('continuation database integrity or node metadata mismatch')
    return dict(source_map_id=manifest['name'],
        source_map_revision=manifest['created_at'],
        source_map_sha256=manifest['sha256']['occupancy_grid.json'],
        source_manifest_sha256=digest(source/'manifest.json'),
        source_database_sha256=source_sha_before,
        source_database_node_count=source_integrity['node_count'],
        source_database_version=source_integrity['database_version'],
        world_path=str(world), world_sha256=actual_world_sha,
        copy_on_write=True, destination_database=str(destination))


def database_integrity(path):
    path = Path(path).resolve(strict=True)
    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1', uri=True) as database:
        rows = database.execute('PRAGMA integrity_check').fetchall()
        if rows != [('ok',)]:
            raise ValueError(f'SQLite integrity failed: {rows}')
        nodes = database.execute('SELECT count(*) FROM Node').fetchone()[0]
        if nodes <= 0:
            raise ValueError('RTAB database has no mapping nodes')
        version = database.execute('SELECT version FROM Admin LIMIT 1').fetchone()[0]
        columns = {row[1] for row in database.execute('PRAGMA table_info(Admin)')}
        stored_grid = None
        if {'opt_map_x_min', 'opt_map_y_min', 'opt_map_resolution'}.issubset(columns):
            x, y, resolution = database.execute('SELECT opt_map_x_min,opt_map_y_min,opt_map_resolution FROM Admin LIMIT 1').fetchone()
            if resolution is not None and resolution > 0:
                stored_grid = dict(origin=[x, y, 0.], resolution=resolution)
    return dict(integrity_check='ok', node_count=nodes, database_version=version, stored_grid=stored_grid)


def validate_mapping_session_graph(path):
    """Reject copied RTAB sessions that have no cross-session graph edge.

    A copied database is not a merged map by itself. This check deliberately
    rejects the observed multi-map/no-cross-link failure while preserving the
    normal single-session map contract.
    """
    path = Path(path).resolve(strict=True)
    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1', uri=True) as db:
        node_columns = {row[1] for row in db.execute('PRAGMA table_info(Node)')}
        link_columns = {row[1] for row in db.execute('PRAGMA table_info(Link)')}
        if 'map_id' not in node_columns or not {'from_id', 'to_id'} <= link_columns:
            raise ValueError('RTAB database lacks map/link identity columns; refusing executable map publication')
        maps = db.execute('SELECT map_id,count(*) FROM Node GROUP BY map_id ORDER BY map_id').fetchall()
        if not maps:
            raise ValueError('RTAB database has no map sessions')
        cross_rows = db.execute(
            'SELECT a.map_id,b.map_id FROM Link l JOIN Node a ON a.id=l.from_id '
            'JOIN Node b ON b.id=l.to_id WHERE a.map_id<>b.map_id').fetchall()
    cross_links = len(cross_rows)
    if len(maps) > 1:
        # A single cross-session edge does not prove that every copied map is
        # part of the same graph. Require connectivity across all map_id groups.
        adjacency = {int(mid): set() for mid, _count in maps}
        for left, right in cross_rows:
            adjacency[int(left)].add(int(right))
            adjacency[int(right)].add(int(left))
        reached, pending = set(), [next(iter(adjacency))]
        while pending:
            current = pending.pop()
            if current in reached:
                continue
            reached.add(current)
            pending.extend(adjacency[current] - reached)
        if len(reached) != len(adjacency):
            raise ValueError('RTAB database contains multiple unconnected map sessions; refusing executable map publication')
    return dict(verified=True,
                map_sessions=[dict(map_id=int(mid), node_count=int(count)) for mid, count in maps],
                cross_session_link_count=int(cross_links))


def restore_export_precision(directory, source):
    """Humble Nav2 writes origin with three significant digits. Restore source metadata only.

    The native image stays untouched. Larger discrepancies are real mismatch,
    not quantization, and must fail rather than being corrected.
    """
    directory = Path(directory)
    original = (directory/'map.yaml').read_text()
    metadata = yaml.safe_load(original)
    if len(metadata['origin']) != 3 or any(not math.isclose(a, float(format(b, '.3g')), abs_tol=1e-9, rel_tol=1e-9)
        for a, b in zip(metadata['origin'], source['origin'])):
        raise ValueError('native YAML origin mismatch exceeds rounding precision (expected three significant digits)')
    if not math.isclose(metadata['resolution'], source['resolution'], abs_tol=1e-8):
        raise ValueError('native YAML resolution mismatch')
    with (directory/'map_saver_original.yaml').open('x') as saved:
        saved.write(original)
    metadata['origin'], metadata['resolution'] = source['origin'], source['resolution']
    (directory/'map.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))


def encode_trinary_pgm(directory, source):
    """Rewrite map_saver's PGM so Nav2 preserves OccupancyGrid unknown cells.

    map_saver_cli writes gray 205 for unknown. With negate=0 and the common
    free_thresh=.25, that pixel has occupancy probability 50/255 (< .25), so
    MapServer classifies it as free. Use a gray value inside the configured
    trinary unknown interval, then let validate_map verify a threshold roundtrip.
    """
    directory = Path(directory)
    metadata = yaml.safe_load((directory/'map.yaml').read_text())
    free_thresh = float(metadata.get('free_thresh', .25))
    occupied_thresh = float(metadata.get('occupied_thresh', .65))
    if not 0 < free_thresh < occupied_thresh < 1:
        raise ValueError('invalid trinary map thresholds')
    unknown = 127
    probability = (255 - unknown) / 255
    if not free_thresh <= probability <= occupied_thresh:
        unknown = round(255 * (1 - (free_thresh + occupied_thresh) / 2))
        probability = (255 - unknown) / 255
    if not free_thresh <= probability <= occupied_thresh:
        raise ValueError('no grayscale PGM value represents unknown at these thresholds')
    width, height = int(source['width']), int(source['height'])
    values = source['data']
    if len(values) != width * height:
        raise ValueError('OccupancyGrid size does not match data while encoding PGM')
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        for value in values[row * width:(row + 1) * width]:
            value = int(value)
            pixels.append(255 if 0 <= value <= 25 else 0 if value >= 65 else unknown)
    (directory/'map.pgm').write_bytes(
        f'P5\n# Nav2 trinary; unknown gray={unknown}\n{width} {height}\n255\n'.encode('ascii')
        + pixels)
    return dict(width=width, height=height, unknown_gray=unknown,
                unknown_probability=probability, free_thresh=free_thresh,
                occupied_thresh=occupied_thresh)


def pgm(path):
    data = Path(path).read_bytes()
    tokens, index = [], 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord('#'):
            index = data.index(b'\n', index)+1
            continue
        end = index
        while end < len(data) and not chr(data[end]).isspace():
            end += 1
        tokens.append(data[index:end])
        index = end
    if tokens[0] != b'P5' or tokens[3] != b'255':
        raise ValueError('expected 8-bit binary PGM')
    width, height = int(tokens[1]), int(tokens[2])
    index += 2 if data[index:index+2] == b'\r\n' else 1
    pixels = data[index:]
    if len(pixels) != width*height:
        raise ValueError('PGM pixel count mismatch')
    return width, height, pixels


def validate_map(directory, require_ready=True):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest.get('schema_version') != 1:
        raise ValueError('unsupported map manifest version')
    if require_ready and manifest.get('state') != 'ready':
        raise ValueError('map has not been atomically published as ready')
    if manifest.get('rtabmap_exit_code') != 0 or not manifest.get('database_closed_before_copy'):
        raise ValueError('no normal RTAB database closure evidence')
    for name in REQUIRED:
        path = directory/name
        if not path.is_file() or path.is_symlink():
            raise ValueError(f'missing or unsafe map asset: {name}')
        if manifest['sha256'].get(name) != digest(path):
            raise ValueError(f'checksum mismatch: {name}')
    info = yaml.safe_load((directory/'map.yaml').read_text())
    if info.get('image') != 'map.pgm' or info.get('negate') != 0 or info.get('mode') != 'trinary':
        raise ValueError('map YAML image/mode contract mismatch')
    source = json.loads((directory/'occupancy_grid.json').read_text())
    free_thresh = float(info.get('free_thresh', .25))
    occupied_thresh = float(info.get('occupied_thresh', .65))
    if not 0 < free_thresh < occupied_thresh < 1:
        raise ValueError('map YAML trinary thresholds are invalid')
    width, height, pixels = pgm(directory/'map.pgm')
    if (width, height) != (source['width'], source['height']):
        raise ValueError('image and OccupancyGrid dimensions differ')
    if manifest['grid']['width'] != width or manifest['grid']['height'] != height:
        raise ValueError('manifest map dimensions differ')
    if not math.isclose(float(info['resolution']), source['resolution'], abs_tol=1e-8):
        raise ValueError('YAML and source resolution differ')
    if any(not math.isclose(float(a), float(b), abs_tol=1e-5)
           for a, b in zip(info['origin'], source['origin'])) or len(info['origin']) != 3:
        raise ValueError('YAML and source origin differ')
    if source['frame_id'] != 'map' or not manifest['grid']['usable']:
        raise ValueError('map frame/quality gate failed')
    unknown = 127
    if not free_thresh <= (255 - unknown) / 255 <= occupied_thresh:
        unknown = round(255 * (1 - (free_thresh + occupied_thresh) / 2))
    expected = bytearray()
    for row in range(height-1, -1, -1):
        for value in source['data'][row*width:(row+1)*width]:
            expected.append(unknown if value < 0 or 25 < value < 65 else
                            255 if value <= 25 else 0)
    if bytes(expected) != pixels:
        raise ValueError('exported image differs from frozen OccupancyGrid')
    decoded = [0] * (width * height)
    for row in range(height):
        grid_row = height - 1 - row
        for col in range(width):
            probability = (255 - pixels[row * width + col]) / 255
            decoded[grid_row * width + col] = (
                100 if probability > occupied_thresh else
                0 if probability < free_thresh else -1)
    original = [100 if value >= 65 else 0 if 0 <= value <= 25 else -1
                for value in source['data']]
    if decoded != original:
        raise ValueError('PGM/YAML thresholds do not roundtrip OccupancyGrid unknown cells')
    database = database_integrity(directory/'rtabmap.db')
    validate_mapping_session_graph(directory/'rtabmap.db')
    if database != manifest['database']:
        raise ValueError('database metadata differs from manifest')
    if database['stored_grid']:
        saved = database['stored_grid']
        if (not math.isclose(saved['resolution'], source['resolution'], abs_tol=1e-8)
                or any(not math.isclose(a,b,abs_tol=1e-5) for a,b in zip(saved['origin'], source['origin']))):
            raise ValueError('RTAB stored grid geometry differs from exported grid')
    return dict(available=True, name=manifest['name'], grid=manifest['grid'], database=database)


def atomic_publish(staging, destination):
    """RENAME_NOREPLACE: an existing directory is never overwritten, even empty."""
    staging, destination = Path(staging), Path(destination)
    for path in staging.iterdir():
        if path.is_file():
            with path.open('rb') as source:
                os.fsync(source.fileno())
    fd = os.open(staging, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
    libc = ctypes.CDLL(None, use_errno=True)
    rename = libc.renameat2
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    if rename(-100, os.fsencode(staging), -100, os.fsencode(destination), 1) != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number), str(destination))
    fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
