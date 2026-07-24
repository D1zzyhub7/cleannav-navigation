"""Self-tests for a_star_core — run via pytest."""

import pytest
from cleannav_global_planner.a_star_core import plan


def _make_grid(w, h):
    return [0] * (w * h)


def test_normal_straight():
    grid = _make_grid(10, 10)
    path = plan(10, 10, grid, (0, 0), (9, 9))
    assert len(path) >= 2
    assert path[0] == (0, 0)
    assert path[-1] == (9, 9)


def test_start_blocked():
    grid = _make_grid(5, 5)
    grid[0] = 100
    with pytest.raises(ValueError, match='start'):
        plan(5, 5, grid, (0, 0), (4, 4))


def test_goal_blocked():
    grid = _make_grid(5, 5)
    grid[4 * 5 + 4] = 100
    with pytest.raises(ValueError, match='goal'):
        plan(5, 5, grid, (0, 0), (4, 4))


def test_corner_cutting():
    """2×2 grid, (1,0)+(0,1) blocked → prevent=True → no path."""
    grid = [0, 100, 100, 0]
    with pytest.raises(ValueError, match='no path'):
        plan(2, 2, grid, (0, 0), (1, 1), prevent_corner_cutting=True)


def test_corner_cutting_disabled():
    """Same grid, prevent=False → diagonal path exists."""
    grid = [0, 100, 100, 0]
    path = plan(2, 2, grid, (0, 0), (1, 1), prevent_corner_cutting=False)
    has_diag = any(abs(b[0] - a[0]) == 1 and abs(b[1] - a[1]) == 1
                   for a, b in zip(path, path[1:]))
    assert has_diag


def test_no_path():
    w, h = 5, 5
    grid = _make_grid(w, h)
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = 2 + dx, 2 + dy
        grid[ny * w + nx] = 100
    with pytest.raises(ValueError, match='no path'):
        plan(w, h, grid, (0, 0), (2, 2))


def test_allow_diagonal_false():
    """allow_diagonal=False → path must contain no diagonal steps."""
    w, h = 5, 5
    grid = _make_grid(w, h)
    path = plan(w, h, grid, (0, 0), (4, 3), allow_diagonal=False)
    for a, b in zip(path, path[1:]):
        dc, dr = b[0] - a[0], b[1] - a[1]
        assert dc == 0 or dr == 0, f'diagonal at {a}→{b}'


def test_corner_cutting_disabled():
    """prevent_corner_cutting=False should allow diagonal."""
    w, h = 5, 5
    grid = _make_grid(w, h)
    grid[2] = 100
    path = plan(w, h, grid, (1, 0), (3, 1), prevent_corner_cutting=False)
    has_diag = any(abs(b[0] - a[0]) == 1 and abs(b[1] - a[1]) == 1
                   for a, b in zip(path, path[1:]))
    assert has_diag
