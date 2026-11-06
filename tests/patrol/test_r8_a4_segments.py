"""Offline regression against the actual new installation; no ROS imports."""
import copy
import importlib.util
import inspect
import json
import math
from pathlib import Path

import pytest

CANDIDATE = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / 'fixtures'
MODULE = CANDIDATE / 'cleannav_navigation/patrol_path_segments.py'
PARENT = FIXTURES / 'legacy_patrol_path_segments.py'
AUDIT = FIXTURES
R8 = FIXTURES
ROUTE = FIXTURES / 'route.json'


def load(path):
    spec = importlib.util.spec_from_file_location('segments_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


NEW = load(MODULE)
OLD = load(PARENT)
DOCUMENT = json.loads(ROUTE.read_text())
POSES = [(p['x'], p['y'], p['yaw']) for p in DOCUMENT['dense_path_map']]
TURNS = json.loads((AUDIT / 'evidence/geometry.json').read_text())['true_turns_from_pose_tangent']


def outcome(module, xy, **kwargs):
    try:
        return module.closed_loop_splits(xy, **kwargs)
    except ValueError as error:
        return (ValueError, str(error))


def check_preserved_poses(poses, splits):
    before = copy.deepcopy(poses)
    assert type(splits) is tuple and all(type(pair) is tuple for pair in splits)
    first, second = [poses[a:b] for a, b in splits]
    assert first[-1] is second[0]
    assert first + second[1:] == before
    assert poses == before


def test_signature_and_original_route():
    assert str(inspect.signature(NEW.closed_loop_splits)) == str(inspect.signature(OLD.closed_loop_splits))
    poses = copy.deepcopy(POSES)
    before = copy.deepcopy(poses)
    result = NEW.closed_loop_splits([p[:2] for p in poses])
    assert result == ((0, 933), (932, 1649))
    check_preserved_poses(poses, result)
    assert poses == before
    assert Path(NEW.__file__).resolve() == MODULE


def test_reverse_route():
    poses = [(x, y, yaw + math.pi) for x, y, yaw in reversed(POSES)]
    result = NEW.closed_loop_splits([p[:2] for p in poses])
    assert result == ((0, 752), (751, 1649))
    original_index = len(POSES) - 1 - result[0][1] + 1
    assert not any(t['i0'] < original_index < t['i1'] for t in TURNS)
    check_preserved_poses(poses, result)


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('start', range(0, 1648, 25))
def test_rephased_start(start, reverse):
    base = POSES[:-1]
    indices = list(range(start, len(base))) + list(range(start))
    if reverse:
        indices.reverse()
    indices.append(indices[0])
    poses = [(base[i][0], base[i][1], base[i][2] + math.pi) if reverse else base[i] for i in indices]
    xy = [p[:2] for p in poses]
    before = copy.deepcopy(xy)
    splits = NEW.closed_loop_splits(xy)
    selected = indices[splits[0][1] - 1]
    assert not any(t['i0'] < selected < t['i1'] for t in TURNS)
    assert xy == before
    check_preserved_poses(poses, splits)


@pytest.mark.parametrize('label', ['B_smooth', 'C_1232mm', 'C_1288mm', 'C_1344mm',
                                  'D_1344_smooth', 'S_1500mm', 'S_2000mm',
                                  'S_2400mm', 'S_3000mm'])
def test_optimized_route(label):
    poses = json.loads((R8 / 'candidates' / (label + '.json')).read_text())['dense']
    before = copy.deepcopy(poses)
    splits = NEW.closed_loop_splits([p[:2] for p in poses])
    i = splits[0][1] - 1
    # Independent local circumcircle: the selected points lie on a straight.
    a, b, c = poses[i-4], poses[i], poses[i+4]
    u = (b[0]-a[0], b[1]-a[1])
    v = (c[0]-b[0], c[1]-b[1])
    denominator = math.hypot(*u) * math.hypot(*v) * math.dist(a[:2], c[:2])
    curvature = 2 * (u[0]*v[1]-u[1]*v[0]) / denominator
    assert abs(curvature) < .05
    check_preserved_poses(poses, splits)
    assert poses == before


@pytest.mark.parametrize('xy', [[], [(0, 0)], [(0, 0), (0, 0)],
                              [(0, 0), (.1, 0), (0, 0)],
                              [(0, 0), (0, 0), (0, 0)]])
def test_original_error_text(xy):
    assert outcome(NEW, xy) == outcome(OLD, xy)


@pytest.mark.parametrize('closure', [.05, .0500001])
def test_closure_boundary(closure):
    xy = [(0, 0), (2, 0), (closure, 0)]
    assert outcome(NEW, xy) == outcome(OLD, xy)


def test_minimum_distance_equality_and_rejection():
    xy = [(0, 0), (1, 0), (0, 0)]
    assert NEW.closed_loop_splits(xy) == ((0, 2), (1, 3))
    assert outcome(NEW, xy, minimum_far_m=1.01) == outcome(OLD, xy, minimum_far_m=1.01)


def test_qualified_near_start_does_not_replace_valid_far():
    circle = [(.6*math.cos(t), .6+.6*math.sin(t))
              for t in [-math.pi/2 + i*2*math.pi/200 for i in range(201)]]
    line = [(-.8*i/40, 0.) for i in range(41)]
    xy = circle + line[1:] + line[-2::-1]
    assert NEW.closed_loop_splits(xy) == OLD.closed_loop_splits(xy)


@pytest.mark.parametrize('radius', [.6, 1., 1.1536, 2., 20.])
def test_circle_fallback(radius):
    count = max(200, math.ceil(2*math.pi*radius/.025))
    xy = [(radius*math.cos(i*2*math.pi/count), radius*math.sin(i*2*math.pi/count))
          for i in range(count+1)]
    assert not any(NEW._straight_before(xy, i) for i in range(1, len(xy)-1))
    assert NEW.closed_loop_splits(xy) == OLD.closed_loop_splits(xy)


def test_continuous_ellipse_fallback():
    xy = [(2*math.cos(i*2*math.pi/800), math.sin(i*2*math.pi/800)) for i in range(801)]
    assert not any(NEW._straight_before(xy, i) for i in range(1, len(xy)-1))
    assert NEW.closed_loop_splits(xy) == OLD.closed_loop_splits(xy)


@pytest.mark.parametrize('xy', [[(0, 0), (2, 0), (2, 0)],
                              [(0, 0), (0, 0), (2, 0), (2, 0), (0, 0)],
                              [(0, 0), (2, 0), (0, 0), (0, 0)]])
def test_duplicate_points(xy):
    before = copy.deepcopy(xy)
    assert NEW.closed_loop_splits(xy) == OLD.closed_loop_splits(xy)
    assert xy == before


def test_zero_length_terminal_segment_is_not_qualified():
    assert not NEW._straight_before([(0, 0), (2, 0), (2, 0)], 2)
    assert not NEW._straight_before([(0, 0), (.2, 0)], 1)


def test_open_path_is_unchanged():
    xy = [(0, 0), (.1, 0)]
    assert NEW.closed_loop_splits(xy) == ((0, 2),)


def test_last_tangent_rejects_chord_counterexample():
    poses = list(reversed(POSES))
    # Historical chord reference selected reversed index 757 inside the bend.
    assert not NEW._straight_before([p[:2] for p in poses], 757)
