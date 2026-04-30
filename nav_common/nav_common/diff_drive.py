"""Conversão de vetor de força desejado para comandos (v, ω) de um robô diferencial."""

import math
from geometry_msgs.msg import Twist
from nav_common.geometry import wrap_to_pi


def force_to_twist(fx: float, fy: float, robot_yaw: float,
                   v_max: float = 0.5, omega_max: float = 1.5,
                   k_omega: float = 2.0) -> Twist:
    """Converte um vetor de força (Fx, Fy) no frame do mundo para um Twist.

    A lógica:
    1. Calcula a direção desejada θ_d = atan2(Fy, Fx).
    2. O erro angular é θ_d - yaw_atual.
    3. ω = k_omega * erro_angular (proporcional, saturado).
    4. v = v_max * cos(erro_angular) quando |erro| < π/2,
       senão v = 0 (não anda de costas, só gira).

    O cos(erro) faz o robô desacelerar automaticamente quando está
    desalinhado e acelerar quando está apontando certo. Isso evita
    trajetórias em arco largo e prioriza alinhar antes de andar.

    Args:
        fx, fy: componentes do vetor de força/gradiente no frame do mundo
        robot_yaw: heading atual do robô (radianos)
        v_max: velocidade linear máxima (m/s)
        omega_max: velocidade angular máxima (rad/s)
        k_omega: ganho proporcional do controlador angular

    Returns:
        Twist com linear.x e angular.z preenchidos
    """
    msg = Twist()

    # Magnitude do vetor — se for ~zero, não faz nada (evita jitter)
    magnitude = math.hypot(fx, fy)
    if magnitude < 1e-6:
        return msg

    # Direção desejada no frame do mundo
    desired_yaw = math.atan2(fy, fx)

    # Erro angular
    yaw_error = wrap_to_pi(desired_yaw - robot_yaw)

    # Velocidade angular: proporcional ao erro, saturada
    omega = k_omega * yaw_error
    omega = max(-omega_max, min(omega_max, omega))

    # Velocidade linear: proporcional ao cosseno do erro
    # Se |erro| > 90°, para e só gira
    if abs(yaw_error) < math.pi / 2:
        v = v_max * math.cos(yaw_error)
    else:
        v = 0.0

    msg.linear.x = v
    msg.angular.z = omega
    return msg


def saturate_twist(twist: Twist, v_max: float = 0.5,
                   omega_max: float = 1.5) -> Twist:
    """Aplica saturação a um Twist já montado.

    Útil quando você monta o Twist manualmente em vez de usar force_to_twist.
    """
    twist.linear.x = max(-v_max, min(v_max, twist.linear.x))
    twist.angular.z = max(-omega_max, min(omega_max, twist.angular.z))
    return twist