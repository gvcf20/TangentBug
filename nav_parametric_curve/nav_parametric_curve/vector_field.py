"""Campo vetorial para convergência e circulação de curvas paramétricas.

A abordagem:
1. Encontra o ponto da curva mais próximo do robô (parâmetro t*).
2. Calcula o vetor tangente T unitário no ponto t* (direção de circulação).
3. Calcula o vetor normal N apontando do robô para o ponto mais próximo
   da curva (direção de convergência).
4. O campo é a soma ponderada: F = T + k_n * N

Quando o robô está longe, N domina → robô vai em direção à curva.
Quando o robô está em cima, N ≈ 0 → robô segue tangencialmente.

A transição é suave e automática: a magnitude de N é proporcional
à distância, então a convergência enfraquece gradualmente até zero.
"""

import math
from typing import Tuple
from nav_parametric_curve.curves import ParametricCurve


def compute_field(robot_x: float, robot_y: float,
                  curve: ParametricCurve,
                  k_normal: float = 1.5,
                  k_tangent: float = 1.0,
                  convergence_radius: float = 5.0
                  ) -> Tuple[float, float]:
    """Calcula o vetor de campo (Fx, Fy) para a posição do robô.

    Args:
        robot_x, robot_y: posição atual do robô no frame do mundo
        curve: objeto ParametricCurve
        k_normal: ganho do componente normal (convergência)
        k_tangent: ganho do componente tangente (circulação)
        convergence_radius: distância a partir da qual o ganho normal satura

    Returns:
        (Fx, Fy): vetor de campo no frame do mundo
    """
    # 1. Encontra o ponto mais próximo na curva
    t_closest = curve.find_closest_t(robot_x, robot_y)
    cx, cy = curve.evaluate(t_closest)

    # 2. Vetor normal: do robô para o ponto da curva
    nx = cx - robot_x
    ny = cy - robot_y
    dist = math.hypot(nx, ny)

    # Normaliza e escala o componente normal
    # Usa tanh para saturar suavemente: longe → 1, perto → ~0
    if dist > 1e-6:
        # tanh(dist / convergence_radius) satura em ~1 quando dist >> radius
        # e vai linearmente a 0 quando dist → 0
        normal_gain = k_normal * math.tanh(dist / convergence_radius)
        nx_scaled = normal_gain * nx / dist
        ny_scaled = normal_gain * ny / dist
    else:
        nx_scaled = 0.0
        ny_scaled = 0.0

    # 3. Vetor tangente: direção de circulação
    tx, ty = curve.tangent(t_closest)
    t_mag = math.hypot(tx, ty)
    if t_mag > 1e-6:
        tx_unit = tx / t_mag
        ty_unit = ty / t_mag
    else:
        tx_unit = 0.0
        ty_unit = 0.0

    # Escala tangente: máximo quando em cima da curva, reduz quando longe
    # Complementar ao normal: 1 - tanh(...)
    tangent_weight = k_tangent * (1.0 - 0.5 * math.tanh(dist / convergence_radius))
    tx_scaled = tangent_weight * tx_unit
    ty_scaled = tangent_weight * ty_unit

    # 4. Campo total
    fx = tx_scaled + nx_scaled
    fy = ty_scaled + ny_scaled

    return (fx, fy)