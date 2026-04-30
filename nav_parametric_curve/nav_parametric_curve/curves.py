"""Definição de curvas paramétricas para o exercício 2.

Cada curva é definida por equações paramétricas x(t), y(t) onde t ∈ [0, 2π).
A classe base fornece métodos para avaliar a curva, calcular tangentes,
e encontrar o ponto mais próximo de uma posição arbitrária.
"""

import math
import numpy as np
from typing import Tuple


class ParametricCurve:
    """Classe base para curvas paramétricas 2D."""

    def evaluate(self, t: float) -> Tuple[float, float]:
        """Retorna (x, y) para o parâmetro t."""
        raise NotImplementedError

    def tangent(self, t: float) -> Tuple[float, float]:
        """Retorna o vetor tangente (dx/dt, dy/dt) no parâmetro t."""
        raise NotImplementedError

    def find_closest_t(self, x: float, y: float,
                       n_samples: int = 360) -> float:
        """Encontra o parâmetro t cujo ponto da curva é mais próximo de (x, y).

        Amostra a curva em n_samples pontos e retorna o t do mais próximo.
        É uma busca bruta mas para 360 amostras roda em <1ms — suficiente
        para um loop de controle a 20 Hz.
        """
        t_values = np.linspace(0, 2 * math.pi, n_samples, endpoint=False)
        min_dist_sq = float('inf')
        best_t = 0.0

        for t in t_values:
            cx, cy = self.evaluate(t)
            dist_sq = (cx - x) ** 2 + (cy - y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_t = t

        return best_t

    def sample(self, n_points: int = 200) -> list:
        """Retorna lista de (x, y) para visualização da curva."""
        points = []
        for t in np.linspace(0, 2 * math.pi, n_points, endpoint=True):
            points.append(self.evaluate(t))
        return points


class Lemniscate(ParametricCurve):
    """Lemniscata de Bernoulli.

    Equações paramétricas:
        x(t) = a * cos(t) / (1 + sin²(t))
        y(t) = a * sin(t) * cos(t) / (1 + sin²(t))

    Forma de ∞ (infinito). O parâmetro 'a' controla o tamanho (~metade
    da extensão em x). Com a=3, a curva vai de -3 a +3 em x.

    A lemniscata NÃO é um círculo nem uma elipse (requisito do enunciado)
    e tem propriedades interessantes: curvatura variável, cruzamento na
    origem, e simetria.
    """

    def __init__(self, a: float = 3.0):
        self.a = a

    def evaluate(self, t: float) -> Tuple[float, float]:
        sin_t = math.sin(t)
        cos_t = math.cos(t)
        denom = 1.0 + sin_t ** 2
        x = self.a * cos_t / denom
        y = self.a * sin_t * cos_t / denom
        return (x, y)

    def tangent(self, t: float) -> Tuple[float, float]:
        """Derivada analítica dx/dt e dy/dt.

        Calculadas aplicando a regra do quociente nas equações paramétricas.
        Usar derivada analítica em vez de diferença finita evita ruído
        numérico e é mais eficiente.
        """
        sin_t = math.sin(t)
        cos_t = math.cos(t)
        sin2 = sin_t ** 2
        denom = 1.0 + sin2
        denom2 = denom ** 2

        # d/dt [cos(t) / (1 + sin²(t))]
        # = [-sin(t)(1+sin²t) - cos(t)*2*sin(t)*cos(t)] / (1+sin²t)²
        # = [-sin(t) - sin³(t) - 2*sin(t)*cos²(t)] / (1+sin²t)²
        # = [-sin(t) - sin(t)*(sin²t + 2cos²t)] / (1+sin²t)²
        # = [-sin(t)*(1 + sin²t + 2cos²t)] / (1+sin²t)²
        # = [-sin(t)*(1 + 1 + cos²t)] / (1+sin²t)²  [pois sin²+2cos² = 1+cos²]
        # Simplificando: -sin(t)*(2 + cos²t) / (1+sin²t)²

        dx_dt = self.a * (-sin_t * (2.0 + cos_t ** 2)) / denom2

        # d/dt [sin(t)*cos(t) / (1 + sin²(t))]
        # Usando regra do quociente com u = sin(t)cos(t), v = 1+sin²(t)
        # u' = cos²t - sin²t = cos(2t)
        # v' = 2*sin(t)*cos(t) = sin(2t)
        # dy/dt = (u'v - uv') / v²
        u = sin_t * cos_t
        u_prime = cos_t ** 2 - sin2
        v_prime = 2.0 * sin_t * cos_t

        dy_dt = self.a * (u_prime * denom - u * v_prime) / denom2

        return (dx_dt, dy_dt)


class Cardioid(ParametricCurve):
    """Cardioide — alternativa caso queira trocar a curva.

    x(t) = a * (2cos(t) - cos(2t))
    y(t) = a * (2sin(t) - sin(2t))

    Forma de coração. Não é círculo nem elipse.
    """

    def __init__(self, a: float = 1.5):
        self.a = a

    def evaluate(self, t: float) -> Tuple[float, float]:
        x = self.a * (2 * math.cos(t) - math.cos(2 * t))
        y = self.a * (2 * math.sin(t) - math.sin(2 * t))
        return (x, y)

    def tangent(self, t: float) -> Tuple[float, float]:
        dx = self.a * (-2 * math.sin(t) + 2 * math.sin(2 * t))
        dy = self.a * (2 * math.cos(t) - 2 * math.cos(2 * t))
        return (dx, dy)