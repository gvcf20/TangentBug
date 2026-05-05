"""Heurísticas do Tangent Bug: d_reach e d_followed.

d_reach: a menor distância à meta que o robô pode alcançar por uma
         linha reta tangente a partir da posição atual, considerando
         o que o sensor vê. É calculada usando os pontos de
         descontinuidade do laser (as "quinas" dos obstáculos).

d_followed: a menor distância à meta já observada ao longo do contorno
            atual durante boundary-following. Se d_reach < d_followed,
            vale a pena sair do contorno e ir direto.
"""

import math
from typing import List, Tuple, Optional
from sensor_msgs.msg import LaserScan
from nav_common.geometry import distance


def compute_d_reach(robot_x: float, robot_y: float,
                    robot_yaw: float,
                    goal_x: float, goal_y: float,
                    scan: LaserScan) -> float:
    """Calcula d_reach: menor distância à meta alcançável por reta livre.

    Verifica se o caminho direto até a meta está livre.
    Se estiver, d_reach = distância direta.
    Se não, calcula d_reach como o mínimo de:
        d(robô, O_i) + d(O_i, meta)
    para cada ponto de descontinuidade O_i visível.

    Também considera todos os pontos do laser como possíveis
    pontos intermediários — isso é mais conservador mas mais robusto.
    """
    dist_direct = distance((robot_x, robot_y), (goal_x, goal_y))

    # Ângulo do robô até a meta no frame do mundo
    angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
    # Converte para frame do laser (relativo ao heading do robô)
    angle_in_laser = angle_to_goal - robot_yaw
    # Normaliza
    angle_in_laser = math.atan2(math.sin(angle_in_laser),
                                math.cos(angle_in_laser))

    # Verifica se o caminho direto está livre
    path_blocked = False
    cone_half = math.radians(15)  # cone de 30° em torno da direção da meta

    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        beam_angle = scan.angle_min + i * scan.angle_increment
        angle_diff = abs(math.atan2(
            math.sin(beam_angle - angle_in_laser),
            math.cos(beam_angle - angle_in_laser)))

        if angle_diff < cone_half:
            # Feixe está na direção da meta
            if r < dist_direct:
                # Tem obstáculo entre o robô e a meta
                path_blocked = True
                break

    if not path_blocked:
        return dist_direct

    # Caminho bloqueado — calcula d_reach via pontos visíveis
    d_reach = float('inf')

    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue

        beam_angle = scan.angle_min + i * scan.angle_increment
        # Posição do ponto no frame do mundo
        px = robot_x + r * math.cos(robot_yaw + beam_angle)
        py = robot_y + r * math.sin(robot_yaw + beam_angle)

        # d(robô, ponto) + d(ponto, meta)
        d_candidate = r + distance((px, py), (goal_x, goal_y))
        if d_candidate < d_reach:
            d_reach = d_candidate

    return d_reach


def find_best_tangent_point(robot_x: float, robot_y: float,
                            robot_yaw: float,
                            goal_x: float, goal_y: float,
                            scan: LaserScan,
                            discontinuities: List[dict]
                            ) -> Optional[Tuple[float, float, float]]:
    """Encontra o melhor ponto de descontinuidade para contornar.

    Retorna o O_i que minimiza d(robô, O_i) + d(O_i, meta).

    Returns:
        (angle_in_laser, dist_to_point, heuristic_value) ou None se não há candidatos.
    """
    if not discontinuities:
        return None

    best = None
    best_heuristic = float('inf')

    for disc in discontinuities:
        # Ponto no frame do laser
        lx, ly = disc['point']
        # Transforma para frame do mundo
        wx = robot_x + lx * math.cos(robot_yaw) - ly * math.sin(robot_yaw)
        wy = robot_y + lx * math.sin(robot_yaw) + ly * math.cos(robot_yaw)

        d_robot_to_point = math.hypot(lx, ly)
        d_point_to_goal = distance((wx, wy), (goal_x, goal_y))
        heuristic = d_robot_to_point + d_point_to_goal

        if heuristic < best_heuristic:
            best_heuristic = heuristic
            best = (disc['angle'], d_robot_to_point, heuristic)

    return best