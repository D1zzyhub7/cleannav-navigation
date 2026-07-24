"""Tests for inflation_core.py — run via pytest."""

import pytest
from cleannav_global_planner.inflation_core import inflate_strict_binary


def test_radius_zero():
    """radius=0: only binarization, no expansion."""
    w, h = 4, 4
    data = [0, 0, 0, 0,
            0, 50, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 100]
    out = inflate_strict_binary(data, w, h, 0)
    expected = [0] * 16
    expected[5] = 100   # idx 5 = row1 col1 (50)
    expected[10] = 100  # idx 10 = row2 col2 (-1)
    expected[15] = 100  # idx 15 = row3 col3 (100)
    assert out == expected


def test_euclidean_circle():
    """Center obstacle, radius=2: (±2,0) and (0,±2) inflated, (±2,±1) not."""
    w, h = 7, 7
    data = [0] * (w * h)
    data[3 * w + 3] = 100  # center
    out = inflate_strict_binary(data, w, h, 2)

    def occ(col, row):
        return out[row * w + col]

    # (±2,0) and (0,±2): must be inflated
    assert occ(1, 3) == 100  # -2,0
    assert occ(5, 3) == 100  # +2,0
    assert occ(3, 1) == 100  # 0,-2
    assert occ(3, 5) == 100  # 0,+2
    # (±2,±1): should NOT be inflated (2^2+1^2=5 > 4)
    assert occ(1, 2) == 0
    assert occ(1, 4) == 0
    assert occ(5, 2) == 0
    assert occ(5, 4) == 0


def test_no_chain():
    """Single obstacle, radius=1: only 4 neighbors, no further expansion."""
    w, h = 5, 5
    data = [0] * (w * h)
    data[2 * w + 2] = 100  # center
    out = inflate_strict_binary(data, w, h, 1)
    inflated = sum(1 for v in out if v == 100)
    # center + 4 orth neighbors = 5
    assert inflated == 5, f'inflated count {inflated}'


def test_corner_clip():
    """Corner obstacle must not underflow indices."""
    w, h = 5, 5
    data = [0] * (w * h)
    data[0] = 100  # top-left corner
    out = inflate_strict_binary(data, w, h, 2)
    # Just ensure no crash and cell (0,0) is inflated
    assert out[0] == 100


def test_unknown_and_other():
    """-1, 1, 49, 50, 100: all must become seeds."""
    for val in [-1, 1, 49, 50, 100]:
        data = [0, 0, 0, val]
        out = inflate_strict_binary(data, 4, 1, 0)
        assert out[3] == 100, f'{val} not treated as obstacle'


def test_invalid_dims():
    with pytest.raises(ValueError, match='dims'):
        inflate_strict_binary([], 0, 5, 1)


def test_data_length_mismatch():
    with pytest.raises(ValueError, match='data len'):
        inflate_strict_binary([0, 0], 3, 3, 1)


def test_negative_radius():
    with pytest.raises(ValueError, match='radius'):
        inflate_strict_binary([0], 1, 1, -1)


def test_float_radius():
    """radius_cells=1.0 must raise ValueError (only int accepted)."""
    with pytest.raises(ValueError, match='radius'):
        inflate_strict_binary([0], 1, 1, 1.0)


def test_radius_zero_output():
    """radius=0: output must contain only 0 and 100."""
    w, h = 5, 5
    data = [0, 50, -1, 0, 100] * 5
    out = inflate_strict_binary(data, w, h, 0)
    assert all(v in (0, 100) for v in out)
