"""Funções de geometria 2D usadas por todos os exercícios."""

import math
from transforms3d.euler import quat2euler


def wrap_to_pi(angle: float) -> float:
    """Normaliza um ângulo para o intervalo (-pi, pi].

    Sem isso, a diferença entre dois ângulos pode dar 350° em vez de -10°,
    e o controlador manda o robô girar a volta inteira ao invés de corrigir
    o mínimo necessário.
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    """Extrai o yaw (rotação em torno de z) de um quaternion.

    Aceita qualquer objeto com atributos x, y, z, w
    (como geometry_msgs/Quaternion) ou uma tupla/lista (x, y, z, w).

    O robô diferencial se move no plano, então só o yaw importa.
    Roll e pitch devem ser ~0 em operação normal.
    """
    if hasattr(q, 'x'):
        w, x, y, z = q.w, q.x, q.y, q.z
    else:
        x, y, z, w = q[0], q[1], q[2], q[3]
    # transforms3d usa convenção w-first: (w, x, y, z)
    # quat2euler retorna (ai, aj, ak) para os eixos escolhidos
    ai, aj, ak = quat2euler([w, x, y, z], axes='sxyz')
    return ak  # ak = yaw (rotação em z)


def distance(p1, p2) -> float:
    """Distância euclidiana entre dois pontos 2D.

    Aceita tuplas (x, y) ou objetos com atributos .x, .y
    (como geometry_msgs/Point).
    """
    x1, y1 = _unpack(p1)
    x2, y2 = _unpack(p2)
    return math.hypot(x2 - x1, y2 - y1)


def angle_to_target(source, target) -> float:
    """Ângulo absoluto (em radianos) de source até target no plano.

    Retorno no intervalo (-pi, pi].
    """
    x1, y1 = _unpack(source)
    x2, y2 = _unpack(target)
    return math.atan2(y2 - y1, x2 - x1)


def angle_diff(a: float, b: float) -> float:
    """Diferença angular mínima (a - b), normalizada para (-pi, pi].

    Útil para calcular o erro angular do controlador:
    erro = angle_diff(yaw_desejado, yaw_atual)
    """
    return wrap_to_pi(a - b)


def _unpack(p):
    """Extrai (x, y) de um ponto — tupla ou objeto com atributos."""
    if hasattr(p, 'x'):
        return p.x, p.y
    return p[0], p[1]