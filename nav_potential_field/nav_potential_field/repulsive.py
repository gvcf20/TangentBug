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


def compute_repulsive(
    scan: LaserScan,
    k_rep: float = 1.2,
    d0: float = 1.5,
    robot_radius: float = 0.22,
    max_force: float = 8.0,
    front_angle_deg: float = 140.0,
    smoothing_window: int = 2,
) -> Tuple[float, float]:
    """
    Calcula força repulsiva no FRAME DO ROBÔ.

    Args:
        scan:
            Mensagem LaserScan

        k_rep:
            Ganho repulsivo

        d0:
            Distância de influência

        robot_radius:
            Distância mínima de segurança

        max_force:
            Saturação da força total

        front_angle_deg:
            Campo angular considerado

        smoothing_window:
            Quantidade de vizinhos usados para suavização

    Returns:
        (Fx, Fy) no frame do robô
    """

    fx_total = 0.0
    fy_total = 0.0

    ranges = list(scan.ranges)
    n = len(ranges)

    front_limit = math.radians(front_angle_deg / 2.0)

    for i in range(n):

        # ============================================================
        # 1. SUAVIZAÇÃO LOCAL
        # ============================================================

        values = []

        for j in range(
            max(0, i - smoothing_window),
            min(n, i + smoothing_window + 1),
        ):
            d = ranges[j]

            if (
                math.isfinite(d)
                and scan.range_min < d < scan.range_max
            ):
                values.append(d)

        if not values:
            continue

        d = sum(values) / len(values)

        # ============================================================
        # 2. ÂNGULO DO FEIXE
        # ============================================================

        angle = scan.angle_min + i * scan.angle_increment

        # ignora traseira
        if abs(angle) > front_limit:
            continue

        # ============================================================
        # 3. ZONA DE INFLUÊNCIA
        # ============================================================

        if d >= d0:
            continue

        # distância efetiva considerando tamanho do robô
        d_eff = max(d - robot_radius, 0.05)

        # ============================================================
        # 4. MAGNITUDE REPULSIVA
        # ============================================================

        # Fórmula clássica:
        #
        # F = k * (1/d - 1/d0) / d²
        #
        magnitude = (
            k_rep
            * (1.0 / d_eff - 1.0 / d0)
            / (d_eff * d_eff)
        )

        # ============================================================
        # 5. PESO ANGULAR
        # ============================================================

        # Obstáculos à frente têm peso maior.
        #
        # cos(angle):
        #   1.0  -> frente
        #   0.0  -> lateral
        #
        angular_weight = max(math.cos(angle), 0.0)

        magnitude *= angular_weight

        # ============================================================
        # 6. SATURAÇÃO SUAVE
        # ============================================================

        magnitude = min(magnitude, max_force)

        # ============================================================
        # 7. VETOR REPULSIVO
        # ============================================================

        # Feixe aponta para obstáculo.
        # Repulsão aponta no sentido oposto.
        fx = -magnitude * math.cos(angle)
        fy = -magnitude * math.sin(angle)

        fx_total += fx
        fy_total += fy

    # ================================================================
    # 8. NORMALIZAÇÃO OPCIONAL
    # ================================================================

    norm = math.hypot(fx_total, fy_total)

    if norm > max_force:
        scale = max_force / norm
        fx_total *= scale
        fy_total *= scale

    return fx_total, fy_total


# ===================================================================
# EXEMPLO DE USO EM ROBÔ DIFERENCIAL
# ===================================================================

def force_to_cmd_vel(
    fx: float,
    fy: float,
    max_linear: float = 0.4,
    max_angular: float = 1.5,
) -> Tuple[float, float]:
    """
    Converte força potencial em cmd_vel.

    Ideal para robô diferencial.

    Returns:
        (linear_x, angular_z)
    """

    # direção desejada
    desired_heading = math.atan2(fy, fx)

    # intensidade da força
    magnitude = math.hypot(fx, fy)

    # ============================================================
    # Controle angular
    # ============================================================

    angular_z = 2.0 * desired_heading

    angular_z = max(
        -max_angular,
        min(max_angular, angular_z),
    )

    # ============================================================
    # Controle linear
    # ============================================================

    # anda mais quando alinhado
    heading_factor = max(math.cos(desired_heading), 0.0)

    linear_x = 0.25 * magnitude * heading_factor

    linear_x = min(linear_x, max_linear)

    return linear_x, angular_z