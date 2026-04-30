"""Potencial atrativo para navegação por campo potencial.

Dois regimes:
- Perto da meta (d < d_threshold): potencial PARABÓLICO → força linear.
  F = -k_att * (posição - meta)
  O robô desacelera suavemente ao se aproximar.

- Longe da meta (d >= d_threshold): potencial CÔNICO → força constante.
  F = -k_att * d_threshold * (posição - meta) / d
  Evita que a força cresça indefinidamente com a distância, o que
  saturaria o controlador e tornaria os ganhos repulsivos irrelevantes.
"""

import math
from typing import Tuple


def compute_attractive(robot_x: float, robot_y: float,
                       goal_x: float, goal_y: float,
                       k_att: float = 1.0,
                       d_threshold: float = 2.0
                       ) -> Tuple[float, float]:
    """Calcula a força atrativa no frame do mundo.

    Args:
        robot_x, robot_y: posição atual do robô
        goal_x, goal_y: posição da meta
        k_att: ganho atrativo
        d_threshold: distância de transição parabólico→cônico

    Returns:
        (Fx, Fy) no frame do mundo, apontando do robô para a meta
    """
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist = math.hypot(dx, dy)

    if dist < 1e-6:
        return (0.0, 0.0)

    if dist <= d_threshold:
        # Regime parabólico: força proporcional à distância
        fx = k_att * dx
        fy = k_att * dy
    else:
        # Regime cônico: força de magnitude constante
        fx = k_att * d_threshold * dx / dist
        fy = k_att * d_threshold * dy / dist

    return (fx, fy)