"""Funções de conveniência para consultar transformações via tf2."""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from typing import Optional, Tuple
from nav_common.geometry import yaw_from_quaternion


class TFHelper:
    """Wrapper que simplifica o uso do tf2 para obter a pose do robô.

    Uso típico dentro de um nó:
        self.tf = TFHelper(self)
        ...
        pose = self.tf.get_pose()  # retorna (x, y, yaw) ou None
    """

    def __init__(self, node: Node,
                 target_frame: str = 'odom',
                 source_frame: str = 'base_footprint'):
        self.node = node
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Retorna (x, y, yaw) do robô no frame alvo, ou None se indisponível.

        O tf2 pode não ter a transformação disponível ainda (ex: nos primeiros
        segundos de simulação). Por isso retornamos None em vez de crashar.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(),  # tempo mais recente disponível
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = yaw_from_quaternion(t.transform.rotation)
            return (x, y, yaw)
        except Exception:
            return None

    def get_transform(self, target: str, source: str):
        """Retorna o TransformStamped completo entre dois frames, ou None."""
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            return None