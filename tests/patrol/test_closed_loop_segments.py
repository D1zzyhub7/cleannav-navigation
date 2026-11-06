"""The closed offline path becomes two controller goals, not one immediate success."""
import importlib.util
from pathlib import Path

MODULE = Path(__file__).resolve().parents[2] / 'cleannav_navigation/patrol_path_segments.py'


def load():
    spec = importlib.util.spec_from_file_location('patrol_path_segments', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_closed_loop_ends_away_from_the_start_then_returns():
    splits = load().closed_loop_splits([(0.0, 0.0), (10.0, 0.0), (10.0, 8.0), (0.0, 0.0)])
    assert splits == ((0, 3), (2, 4))


def test_open_path_stays_one_goal():
    splits = load().closed_loop_splits([(0.0, 0.0), (5.0, 0.0)])
    assert splits == ((0, 2),)
