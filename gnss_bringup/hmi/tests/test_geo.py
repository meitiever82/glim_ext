"""Tests for the shared geodesy helper."""
from gnss_hmi.geo import horiz_dist_m


def test_horiz_dist_zero():
    assert horiz_dist_m(44.5, 90.28, 44.5, 90.28) == 0.0


def test_horiz_dist_one_metre_north():
    # 1 m north ≈ 1/111320 deg latitude
    d = horiz_dist_m(44.5, 90.28, 44.5 + 1 / 111320.0, 90.28)
    assert abs(d - 1.0) < 0.02


def test_horiz_dist_symmetric():
    a = horiz_dist_m(44.50, 90.28, 44.51, 90.29)
    b = horiz_dist_m(44.51, 90.29, 44.50, 90.28)
    assert abs(a - b) < 1e-9
