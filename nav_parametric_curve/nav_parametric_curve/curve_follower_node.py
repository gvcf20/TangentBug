"""Nó ROS 2 que segue uma curva paramétrica usando campo vetorial.

Pipeline:
    /odom → extrai pose → compute_field() → force_to_twist() → /cmd_vel

O nó também publica a curva alvo como Marker no RViz para visualização.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from nav_common.geometry import yaw_from_quaternion
from nav_common.diff_drive import force_to_twist
from nav_common.plotting import TrajectoryLogger

from nav_parametric_curve.curves import Lemniscate, Cardioid
from nav_parametric_curve.vector_field import compute_field


class CurveFollowerNode(Node):

    def __init__(self):
        super().__init__('curve_follower')

        # Parâmetros declarados — podem ser alterados via YAML ou linha de comando
        self.declare_parameter('curve_type', 'lemniscate')
        self.declare_parameter('curve_scale', 3.0)
        self.declare_parameter('k_normal', 1.5)
        self.declare_parameter('k_tangent', 1.0)
        self.declare_parameter('convergence_radius', 5.0)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('k_omega', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('log_trajectory', True)
        self.declare_parameter('log_file', '/tmp/curve_trajectory.csv')

        # Lê parâmetros
        curve_type = self.get_parameter('curve_type').value
        curve_scale = self.get_parameter('curve_scale').value
        self.k_normal = self.get_parameter('k_normal').value
        self.k_tangent = self.get_parameter('k_tangent').value
        self.conv_radius = self.get_parameter('convergence_radius').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.k_omega = self.get_parameter('k_omega').value
        control_rate = self.get_parameter('control_rate').value

        # Cria a curva
        if curve_type == 'cardioid':
            self.curve = Cardioid(a=curve_scale)
            self.get_logger().info(f'Curva: Cardioide (a={curve_scale})')
        else:
            self.curve = Lemniscate(a=curve_scale)
            self.get_logger().info(f'Curva: Lemniscata (a={curve_scale})')

        # Estado
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Subscribers e publishers
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, '/curve_marker', 10)

        # Timer de controle
        period = 1.0 / control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        # Timer para publicar a curva no RViz (uma vez por segundo basta)
        self.marker_timer = self.create_timer(1.0, self.publish_curve_marker)

        # Logger de trajetória
        self.logger_traj = None
        if self.get_parameter('log_trajectory').value:
            log_file = self.get_parameter('log_file').value
            self.logger_traj = TrajectoryLogger(
                log_file, extra_fields=['dist_to_curve'])
            self.get_logger().info(f'Logging trajetória em: {log_file}')

        self.get_logger().info('Curve follower pronto. Aguardando odometria...')

    def odom_callback(self, msg: Odometry):
        """Callback da odometria — só guarda o estado mais recente.

        Não faz controle aqui. O controle roda no timer a taxa fixa,
        usando o último estado recebido. Isso desacopla a taxa do sensor
        da taxa do controlador.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def control_loop(self):
        """Loop de controle — roda a taxa fixa (ex: 20 Hz)."""
        if not self.odom_received:
            return

        # 1. Calcula o campo vetorial
        fx, fy = compute_field(
            self.robot_x, self.robot_y,
            self.curve,
            k_normal=self.k_normal,
            k_tangent=self.k_tangent,
            convergence_radius=self.conv_radius
        )

        # 2. Converte para (v, ω) do robô diferencial
        twist = force_to_twist(
            fx, fy, self.robot_yaw,
            v_max=self.v_max,
            omega_max=self.omega_max,
            k_omega=self.k_omega
        )

        # 3. Publica
        self.pub_cmd.publish(twist)

        # 4. Log (opcional)
        if self.logger_traj:
            t_closest = self.curve.find_closest_t(self.robot_x, self.robot_y)
            cx, cy = self.curve.evaluate(t_closest)
            dist = math.hypot(cx - self.robot_x, cy - self.robot_y)
            self.logger_traj.log(
                self.robot_x, self.robot_y, self.robot_yaw,
                dist_to_curve=round(dist, 4)
            )

    def publish_curve_marker(self):
        """Publica a curva como LINE_STRIP no RViz para visualização."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_curve'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # espessura da linha

        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        # Amostra a curva
        marker.pose.orientation.w = 1.0
        for x, y in self.curve.sample(n_points=300):
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.01  # ligeiramente acima do chão
            marker.points.append(p)

        self.pub_marker.publish(marker)

    def destroy_node(self):
        """Limpeza ao fechar."""
        # Para o robô
        self.pub_cmd.publish(Twist())
        if self.logger_traj:
            self.logger_traj.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CurveFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()