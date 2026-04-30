"""Testes para nav_common.laser_utils."""

import math
import pytest
from sensor_msgs.msg import LaserScan
from nav_common.laser_utils import (
    scan_to_points, find_discontinuities, closest_obstacle, is_path_clear
)


def _make_scan(ranges, angle_min=-math.pi, angle_max=math.pi):
    """Cria um LaserScan fake para testes."""
    scan = LaserScan()
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    n = len(ranges)
    scan.angle_increment = (angle_max - angle_min) / n if n > 1 else 0.0
    scan.range_min = 0.12
    scan.range_max = 8.0
    scan.ranges = [float(r) for r in ranges]
    return scan


class TestScanToPoints:
    def test_single_beam_forward(self):
        scan = _make_scan([2.0], angle_min=0.0, angle_max=0.0)
        pts = scan_to_points(scan)
        assert len(pts) == 1
        assert pts[0][0] == pytest.approx(2.0, abs=0.01)
        assert pts[0][1] == pytest.approx(0.0, abs=0.01)

    def test_inf_filtered(self):
        scan = _make_scan([1.0, float('inf'), 2.0])
        pts = scan_to_points(scan)
        assert len(pts) == 2

    def test_below_min_filtered(self):
        scan = _make_scan([0.05, 1.0])
        pts = scan_to_points(scan)
        assert len(pts) == 1


class TestFindDiscontinuities:
    def test_no_discontinuity(self):
        scan = _make_scan([3.0, 3.1, 3.05, 2.95])
        discs = find_discontinuities(scan, threshold=0.5)
        assert len(discs) == 0

    def test_clear_discontinuity(self):
        # Parede a 2m, depois vazio (8m) — quina do obstáculo
        scan = _make_scan([2.0, 2.0, 2.0, 7.0, 7.0])
        discs = find_discontinuities(scan, threshold=0.5)
        assert len(discs) >= 1
        # O ponto deve estar a ~2m (o mais próximo)
        assert discs[0]['point'][0] != 0  # não é origem

    def test_inf_transition(self):
        scan = _make_scan([2.0, float('inf')])
        discs = find_discontinuities(scan, threshold=0.5)
        assert len(discs) == 1


class TestClosestObstacle:
    def test_basic(self):
        scan = _make_scan([5.0, 2.0, 7.0, 3.0])
        result = closest_obstacle(scan)
        assert result is not None
        assert result[0] == pytest.approx(2.0)

    def test_all_inf(self):
        scan = _make_scan([float('inf'), float('inf')])
        assert closest_obstacle(scan) is None


class TestIsPathClear:
    def test_clear(self):
        # Tudo a 5m, distância segura 0.5m → livre
        scan = _make_scan([5.0] * 360)
        assert is_path_clear(scan, direction=0.0, safe_distance=0.5) is True

    def test_blocked(self):
        # Feixes na frente a 0.3m
        ranges = [5.0] * 360
        # Coloca obstáculo na frente (índices do meio ≈ 180 para angle_min=-pi)
        for i in range(175, 185):
            ranges[i] = 0.3
        scan = _make_scan(ranges)
        # Direção 0 (frente) deve estar bloqueada
        assert is_path_clear(scan, direction=0.0, safe_distance=0.5) is False