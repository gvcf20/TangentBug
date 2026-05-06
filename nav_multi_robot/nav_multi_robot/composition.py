"""Composição dos campos vetoriais para o exercício 4.

F_total = alpha * F_curva + beta * F_rep_obstáculos + gamma * F_rep_robôs

Importa diretamente de nav_parametric_curve e nav_potential_field.
"""

import math
from typing import List, Tuple
from sensor_msgs.msg import LaserScan

from nav_parametric_curve.curves import ParametricCurve
from nav_parametric_curve.vector_field import compute_field as compute_curve_field
from nav_potential_field.repulsive import (
    compute_repulsive,
    compute_repulsive_from_points
)


def compute_composed_field(
    robot_x: float, robot_y: float, robot_yaw: float,
    curve: ParametricCurve,
    scan: LaserScan,
    other_robots: List[Tuple[float, float]],
    alpha: float = 1.0,
    beta: float = 1.0,
    gamma: float = 1.5,
    k_normal: float = 1.5,
    k_tangent: float = 1.0,
    convergence_radius: float = 3.0,
    k_rep_obs: float = 1.0,
    d0_obs: float = 1.2,
    k_rep_robot: float = 2.0,
    d0_robot: float = 1.5,
) -> Tuple[float, float]:
    """Calcula o campo vetorial composto para um robô.

    Args:
        robot_x, robot_y, robot_yaw: pose do robô
        curve: curva paramétrica alvo
        scan: LaserScan do próprio robô
        other_robots: lista de (x, y) dos outros robôs
        alpha: peso do campo da curva
        beta: peso da repulsão de obstáculos
        gamma: peso da repulsão entre robôs
        (demais parâmetros: ganhos dos sub-campos)

    Returns:
        (Fx, Fy) no frame do mundo
    """
    # Campo da curva (convergir + circular)
    fc_x, fc_y = compute_curve_field(
        robot_x, robot_y, curve,
        k_normal=k_normal,
        k_tangent=k_tangent,
        convergence_radius=convergence_radius)

    # Repulsão de obstáculos (via laser)
    fr_obs_x, fr_obs_y = compute_repulsive(
        scan, robot_yaw,
        k_rep=k_rep_obs,
        d0=d0_obs)

    # Repulsão entre robôs (via posições conhecidas)
    fr_rob_x, fr_rob_y = compute_repulsive_from_points(
        other_robots,
        robot_x, robot_y,
        k_rep=k_rep_robot,
        d0=d0_robot)

    # Soma ponderada
    fx = alpha * fc_x + beta * fr_obs_x + gamma * fr_rob_x
    fy = alpha * fc_y + beta * fr_obs_y + gamma * fr_rob_y

    return (fx, fy)