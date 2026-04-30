"""Potencial repulsivo calculado a partir do LaserScan.

Cada feixe do laser com leitura abaixo de d0 (distância de influência)
contribui com uma força repulsiva empurrando o robô para longe do obstáculo.

A fórmula clássica para um obstáculo pontual a distância d:
    |F_rep| = k_rep * (1/d - 1/d0) * (1/d²)

A direção é do obstáculo para o robô (sentido oposto ao feixe).

O resultado total é a SOMA de todas as contribuições, retornada
no frame do mundo (transformada pelo yaw do robô).
"""

import math
from typing import Tuple
from sensor_msgs.msg import LaserScan


def compute_repulsive(scan: LaserScan,
                      robot_yaw: float,
                      k_rep: float = 0.5,
                      d0: float = 1.5
                      ) -> Tuple[float, float]:
    """Calcula a força repulsiva total a partir do LaserScan.

    Args:
        scan: mensagem LaserScan
        robot_yaw: heading do robô no frame do mundo (rad).
                   Necessário para transformar as forças do frame do laser
                   para o frame do mundo.
        k_rep: ganho repulsivo
        d0: distância de influência — feixes acima de d0 são ignorados

    Returns:
        (Fx, Fy) no frame do mundo, apontando PARA LONGE dos obstáculos
    """
    fx_total = 0.0
    fy_total = 0.0

    for i, d in enumerate(scan.ranges):
        # Ignora feixes inválidos ou fora do alcance
        if not math.isfinite(d):
            continue
        if d < scan.range_min or d > scan.range_max:
            continue
        # Ignora feixes fora da zona de influência
        if d >= d0:
            continue

        # Ângulo do feixe no frame do laser
        angle_laser = scan.angle_min + i * scan.angle_increment

        # Magnitude da força repulsiva (fórmula clássica)
        magnitude = k_rep * (1.0 / d - 1.0 / d0) / (d * d)

        # Direção: do obstáculo para o robô = oposta ao feixe
        # No frame do laser, o feixe aponta para (cos(angle), sin(angle))
        # A repulsão aponta para (-cos(angle), -sin(angle))
        fx_laser = -magnitude * math.cos(angle_laser)
        fy_laser = -magnitude * math.sin(angle_laser)

        fx_total += fx_laser
        fy_total += fy_laser

    # Transforma do frame do laser para o frame do mundo
    # Rotação 2D pelo yaw do robô
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    fx_world = cos_yaw * fx_total - sin_yaw * fy_total
    fy_world = sin_yaw * fx_total + cos_yaw * fy_total

    return (fx_world, fy_world)


def compute_repulsive_from_points(obstacles: list,
                                  robot_x: float, robot_y: float,
                                  k_rep: float = 0.5,
                                  d0: float = 2.0
                                  ) -> Tuple[float, float]:
    """Calcula repulsão a partir de posições conhecidas (para multi-robô).

    No exercício 4, cada robô sabe a posição dos outros. Essa função
    calcula o potencial repulsivo usando essas posições diretamente,
    sem precisar do laser.

    Args:
        obstacles: lista de (x, y) no frame do mundo
        robot_x, robot_y: posição do robô
        k_rep: ganho repulsivo
        d0: distância de influência

    Returns:
        (Fx, Fy) no frame do mundo
    """
    fx_total = 0.0
    fy_total = 0.0

    for ox, oy in obstacles:
        dx = robot_x - ox
        dy = robot_y - oy
        d = math.hypot(dx, dy)

        if d < 0.01:
            d = 0.01  # evita divisão por zero
        if d >= d0:
            continue

        magnitude = k_rep * (1.0 / d - 1.0 / d0) / (d * d)
        fx_total += magnitude * dx / d
        fy_total += magnitude * dy / d

    return (fx_total, fy_total)