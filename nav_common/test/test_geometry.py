"""Testes para nav_common.geometry."""

import math
import pytest
from nav_common.geometry import (
    wrap_to_pi, distance, angle_diff, angle_to_target
)


class TestWrapToPi:
    def test_already_in_range(self):
        assert wrap_to_pi(0.5) == pytest.approx(0.5)
        assert wrap_to_pi(-0.5) == pytest.approx(-0.5)

    def test_positive_overflow(self):
        assert wrap_to_pi(2 * math.pi) == pytest.approx(0.0, abs=1e-10)

    def test_negative_overflow(self):
        assert wrap_to_pi(-2 * math.pi) == pytest.approx(0.0, abs=1e-10)

    def test_just_above_pi(self):
        result = wrap_to_pi(math.pi + 0.1)
        assert -math.pi < result <= math.pi
        assert result == pytest.approx(-math.pi + 0.1, abs=1e-10)

    def test_large_angle(self):
        result = wrap_to_pi(10 * math.pi + 0.5)
        assert -math.pi < result <= math.pi


class TestDistance:
    def test_same_point(self):
        assert distance((0, 0), (0, 0)) == pytest.approx(0.0)

    def test_unit(self):
        assert distance((0, 0), (1, 0)) == pytest.approx(1.0)
        assert distance((0, 0), (0, 1)) == pytest.approx(1.0)

    def test_diagonal(self):
        assert distance((0, 0), (3, 4)) == pytest.approx(5.0)

    def test_negative_coords(self):
        assert distance((-1, -1), (2, 3)) == pytest.approx(5.0)


class TestAngleDiff:
    def test_zero(self):
        assert angle_diff(1.0, 1.0) == pytest.approx(0.0)

    def test_positive(self):
        assert angle_diff(1.0, 0.5) == pytest.approx(0.5)

    def test_wrap_around(self):
        # 3.1 - (-3.1) = 6.2, mas deve dar ~0.08 (não 6.2)
        result = angle_diff(3.1, -3.1)
        assert abs(result) < 0.2

    def test_opposite_directions(self):
        result = angle_diff(math.pi, 0)
        assert abs(result) == pytest.approx(math.pi, abs=1e-10)


class TestAngleToTarget:
    def test_east(self):
        assert angle_to_target((0, 0), (1, 0)) == pytest.approx(0.0)

    def test_north(self):
        assert angle_to_target((0, 0), (0, 1)) == pytest.approx(math.pi / 2)

    def test_west(self):
        assert angle_to_target((0, 0), (-1, 0)) == pytest.approx(math.pi)

    def test_south(self):
        assert angle_to_target((0, 0), (0, -1)) == pytest.approx(-math.pi / 2)