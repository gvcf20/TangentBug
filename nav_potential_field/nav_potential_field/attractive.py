import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normaliza ângulo para [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def compute_attractive(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    k_att: float = 1.2,
    max_force: float = 2.0,
    slow_radius: float = 1.5,
    goal_tolerance: float = 0.08,
    angular_weight: float = 0.8,
) -> Tuple[float, float, float, float]:
    """
    Calcula força atrativa otimizada para robô diferencial.

    Args:
        robot_x, robot_y:
            posição atual do robô

        robot_yaw:
            orientação atual do robô

        goal_x, goal_y:
            posição da meta

        k_att:
            ganho atrativo

        max_force:
            força máxima permitida

        slow_radius:
            raio onde começa desaceleração

        goal_tolerance:
            tolerância de chegada

        angular_weight:
            penalização angular
            reduz avanço quando desalinhado

    Returns:
        (
            fx_world,
            fy_world,
            desired_heading,
            distance_to_goal
        )
    """

    # ============================================================
    # 1. ERRO DE POSIÇÃO
    # ============================================================

    dx = goal_x - robot_x
    dy = goal_y - robot_y

    distance = math.hypot(dx, dy)

    # ============================================================
    # 2. META ATINGIDA
    # ============================================================

    if distance < goal_tolerance:
        return 0.0, 0.0, robot_yaw, distance

    # ============================================================
    # 3. DIREÇÃO DA META
    # ============================================================

    desired_heading = math.atan2(dy, dx)

    heading_error = normalize_angle(
        desired_heading - robot_yaw
    )

    # ============================================================
    # 4. GANHO ANGULAR
    # ============================================================

    # Robô diferencial:
    # não faz sentido acelerar forte
    # quando está olhando para longe da meta.
    #
    # cos():
    #   1.0 -> alinhado
    #   0.0 -> perpendicular
    #  -1.0 -> contrário
    #
    heading_factor = max(
        math.cos(heading_error),
        0.0
    )

    heading_factor = (
        angular_weight
        + (1.0 - angular_weight) * heading_factor
    )

    # ============================================================
    # 5. PERFIL DE VELOCIDADE
    # ============================================================

    # Perto da meta:
    #   força linear
    #
    # Longe:
    #   força saturada
    #
    if distance <= slow_radius:

        # parabólico
        magnitude = k_att * distance

    else:

        # saturação
        magnitude = k_att * slow_radius

    # ============================================================
    # 6. MODULAÇÃO ANGULAR
    # ============================================================

    magnitude *= heading_factor

    # ============================================================
    # 7. SATURAÇÃO
    # ============================================================

    magnitude = min(magnitude, max_force)

    # ============================================================
    # 8. VETOR ATRATIVO
    # ============================================================

    ux = dx / distance
    uy = dy / distance

    fx = magnitude * ux
    fy = magnitude * uy

    return (
        fx,
        fy,
        desired_heading,
        distance,
    )


# ===================================================================
# CONVERSÃO PARA cmd_vel
# ===================================================================

def attractive_to_cmd_vel(
    fx: float,
    fy: float,
    robot_yaw: float,
    max_linear: float = 0.5,
    max_angular: float = 1.8,
    k_linear: float = 0.8,
    k_angular: float = 2.5,
) -> Tuple[float, float]:
    """
    Converte força atrativa em cmd_vel.

    Ideal para robô diferencial.

    Returns:
        (linear_x, angular_z)
    """

    # ============================================================
    # 1. HEADING DESEJADO
    # ============================================================

    desired_heading = math.atan2(fy, fx)

    heading_error = normalize_angle(
        desired_heading - robot_yaw
    )

    # ============================================================
    # 2. VELOCIDADE ANGULAR
    # ============================================================

    angular_z = k_angular * heading_error

    angular_z = max(
        -max_angular,
        min(max_angular, angular_z)
    )

    # ============================================================
    # 3. VELOCIDADE LINEAR
    # ============================================================

    force_magnitude = math.hypot(fx, fy)

    # reduz avanço quando desalinhado
    alignment = max(math.cos(heading_error), 0.0)

    linear_x = (
        k_linear
        * force_magnitude
        * alignment
    )

    linear_x = min(linear_x, max_linear)

    return linear_x, angular_z