"""Estados da máquina de estados do Tangent Bug."""

from enum import Enum, auto


class TBState(Enum):
    """Estados do algoritmo Tangent Bug.

    MOTION_TO_GOAL:
        O robô vai em linha reta na direção da meta.
        Transição para BOUNDARY_FOLLOWING quando um obstáculo bloqueia
        o caminho direto.

    BOUNDARY_FOLLOWING:
        O robô segue o contorno do obstáculo mantendo distância fixa.
        Monitora d_reach e d_followed para decidir quando sair.
        Transição para MOTION_TO_GOAL quando d_reach < d_followed
        (encontrou um ponto onde pode ir direto à meta por caminho mais curto).
        Transição para NO_PATH se completar uma volta ao redor do obstáculo
        sem encontrar saída.

    GOAL_REACHED:
        Estado terminal — a meta foi alcançada.

    NO_PATH:
        Estado terminal — não existe caminho entre o robô e a meta.
        O algoritmo detectou isso em tempo finito (requisito do enunciado).
    """
    MOTION_TO_GOAL = auto()
    BOUNDARY_FOLLOWING = auto()
    GOAL_REACHED = auto()
    NO_PATH = auto()