"""Processamento de sensor_msgs/LaserScan para os algoritmos de navegação."""

import math
from typing import List, Tuple, Optional
from sensor_msgs.msg import LaserScan


def scan_to_points(scan: LaserScan) -> List[Tuple[float, float]]:
    """Converte as leituras do LaserScan em pontos (x, y) no frame do laser.

    Cada feixe i tem ângulo angle_min + i * angle_increment e distância
    ranges[i]. Feixes com leitura fora do intervalo [range_min, range_max]
    são descartados (inf, nan, ou muito perto).

    Returns:
        Lista de (x, y) no frame do laser (laser na origem, x pra frente).
    """
    points = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        points.append((r * math.cos(angle), r * math.sin(angle)))
    return points


def scan_to_polar(scan: LaserScan) -> List[Tuple[float, float]]:
    """Retorna lista de (distância, ângulo) para feixes válidos.

    Útil quando você quer trabalhar diretamente em coordenadas polares
    (ex: cálculo do potencial repulsivo).
    """
    polar = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        polar.append((r, angle))
    return polar


def find_discontinuities(scan: LaserScan,
                         threshold: float = 0.5) -> List[dict]:
    """Detecta descontinuidades no array de ranges do LaserScan.

    Uma descontinuidade é onde |ranges[i+1] - ranges[i]| > threshold.
    Esses pontos correspondem às "quinas" dos obstáculos visíveis — são
    exatamente os pontos O_i que o Tangent Bug usa para decidir por onde
    contornar um obstáculo.

    Args:
        scan: mensagem LaserScan
        threshold: diferença mínima de distância para considerar descontinuidade

    Returns:
        Lista de dicts com:
            'index': índice do feixe
            'angle': ângulo do feixe (rad)
            'range_before': distância do feixe i
            'range_after': distância do feixe i+1
            'point': (x, y) do ponto mais próximo (o de menor range) no frame do laser
    """
    disconts = []
    ranges = scan.ranges

    for i in range(len(ranges) - 1):
        r1 = ranges[i]
        r2 = ranges[i + 1]

        # Ignora feixes inválidos
        valid1 = math.isfinite(r1) and scan.range_min <= r1 <= scan.range_max
        valid2 = math.isfinite(r2) and scan.range_min <= r2 <= scan.range_max

        # Transição válido→inválido ou vice-versa conta como descontinuidade
        if valid1 and valid2:
            if abs(r2 - r1) > threshold:
                # O ponto de interesse é o mais próximo (menor range)
                if r1 < r2:
                    idx, r = i, r1
                else:
                    idx, r = i + 1, r2
                angle = scan.angle_min + idx * scan.angle_increment
                disconts.append({
                    'index': idx,
                    'angle': angle,
                    'range_before': r1,
                    'range_after': r2,
                    'point': (r * math.cos(angle), r * math.sin(angle))
                })
        elif valid1 and not valid2:
            angle = scan.angle_min + i * scan.angle_increment
            disconts.append({
                'index': i,
                'angle': angle,
                'range_before': r1,
                'range_after': float('inf'),
                'point': (r1 * math.cos(angle), r1 * math.sin(angle))
            })
        elif not valid1 and valid2:
            angle = scan.angle_min + (i + 1) * scan.angle_increment
            disconts.append({
                'index': i + 1,
                'angle': angle,
                'range_before': float('inf'),
                'range_after': r2,
                'point': (r2 * math.cos(angle), r2 * math.sin(angle))
            })

    return disconts


def closest_obstacle(scan: LaserScan) -> Optional[Tuple[float, float, float]]:
    """Retorna (distância, ângulo, índice) do obstáculo mais próximo.

    Returns:
        Tupla (range, angle, index) ou None se não houver leituras válidas.
    """
    min_range = float('inf')
    min_angle = 0.0
    min_idx = -1

    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            if r < min_range:
                min_range = r
                min_angle = scan.angle_min + i * scan.angle_increment
                min_idx = i

    if min_idx < 0:
        return None
    return (min_range, min_angle, min_idx)


def is_path_clear(scan: LaserScan, direction: float,
                  cone_half_angle: float = 0.3,
                  safe_distance: float = 0.5) -> bool:
    """Verifica se há caminho livre em uma direção (no frame do laser).

    Útil para o Tangent Bug decidir se pode ir direto à meta:
    se o cone na direção da meta está livre, usa MOTION_TO_GOAL.

    Args:
        scan: LaserScan
        direction: ângulo desejado no frame do laser (rad)
        cone_half_angle: meio-ângulo do cone de checagem (rad, ~17°)
        safe_distance: distância mínima para considerar "livre" (m)

    Returns:
        True se todos os feixes no cone estão acima de safe_distance.
    """
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        # Verifica se o feixe está dentro do cone
        angle_diff = abs(math.atan2(math.sin(angle - direction),
                                    math.cos(angle - direction)))
        if angle_diff <= cone_half_angle:
            if math.isfinite(r) and r < safe_distance:
                return False
    return True