"""Nó ROS 2 de navegação por campo potencial.

Pipeline:
    /odom + /scan → atrativo + repulsivo → force_to_twist → /cmd_vel

Anti-mínimo-local: wall-following orientado à meta (Bug-like escape).
Quando detecta que está preso, segue a parede do obstáculo até encontrar
um ponto com distância à meta menor que a distância onde ficou preso.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from nav_common.geometry import yaw_from_quaternion, distance, wrap_to_pi
from nav_common.diff_drive import force_to_twist
from nav_common.laser_utils import closest_obstacle
from nav_common.plotting import TrajectoryLogger

from nav_potential_field.attractive import compute_attractive
from nav_potential_field.repulsive import compute_repulsive


class PotentialFieldNode(Node):

    STATE_POTENTIAL = 'potential_field'
    STATE_WALL_FOLLOW = 'wall_follow'
    STATE_REACHED = 'reached'

    def __init__(self):
        super().__init__('potential_field')

        # Parâmetros
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 1.0)
        self.declare_parameter('d0', 1.2)
        self.declare_parameter('d_threshold', 1.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        self.declare_parameter('k_omega', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('wall_follow_distance', 0.35)
        self.declare_parameter('wall_follow_max_duration', 30.0)
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('stuck_movement_threshold', 0.05)
        self.declare_parameter('log_trajectory', True)
        self.declare_parameter('log_file', '/tmp/potential_field_trajectory.csv')

        # Lê parâmetros
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.d0 = self.get_parameter('d0').value
        self.d_threshold = self.get_parameter('d_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.k_omega = self.get_parameter('k_omega').value
        control_rate = self.get_parameter('control_rate').value
        self.wall_follow_dist = self.get_parameter('wall_follow_distance').value
        self.wall_follow_max = self.get_parameter('wall_follow_max_duration').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_move_thresh = self.get_parameter('stuck_movement_threshold').value

        # Estado do robô
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.last_scan = None

        # Meta
        self.goal_x = None
        self.goal_y = None
        self.goal_active = False

        # Máquina de estados
        self.state = self.STATE_POTENTIAL

        # Anti mínimo local
        self.last_move_x = 0.0
        self.last_move_y = 0.0
        self.last_move_time = 0.0
        self.dist_at_stuck = float('inf')       # distância à meta quando ficou preso
        self.best_dist_in_wf = float('inf')     # melhor distância durante wall-follow
        self.wall_follow_start = 0.0
        self.wall_follow_side = 1.0             # +1 esquerda, -1 direita
        self.wf_start_x = 0.0                   # posição onde entrou em wall-follow
        self.wf_start_y = 0.0
        self.wf_traveled_enough = False         # evita sair imediatamente
        self.escape_attempts = 0

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            rclpy.qos.QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, '/potential_markers', 10)

        # Timer
        period = 1.0 / control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        # Logger
        self.logger_traj = None
        if self.get_parameter('log_trajectory').value:
            log_file = self.get_parameter('log_file').value
            self.logger_traj = TrajectoryLogger(
                log_file, extra_fields=['dist_to_goal', 'state'])
            self.get_logger().info(f'Logging em: {log_file}')

        self.get_logger().info(
            'Potential field pronto. Envie meta via RViz "2D Goal Pose".')

    # ─── Callbacks ───────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_active = True
        self.state = self.STATE_POTENTIAL
        self.escape_attempts = 0
        self._reset_stuck_timer()
        self.get_logger().info(
            f'Nova meta: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.publish_goal_marker()

    # ─── Loop principal ──────────────────────────────────────────

    def control_loop(self):
        if not self.odom_received or self.last_scan is None:
            return
        if not self.goal_active:
            return

        dist_to_goal = distance(
            (self.robot_x, self.robot_y),
            (self.goal_x, self.goal_y))

        # Chegou?
        if dist_to_goal < self.goal_tolerance:
            self.pub_cmd.publish(Twist())
            self.goal_active = False
            self.state = self.STATE_REACHED
            self.get_logger().info(
                f'Meta alcançada! Distância final: {dist_to_goal:.3f} m')
            return

        # Dispatch por estado
        if self.state == self.STATE_POTENTIAL:
            twist = self._do_potential_field(dist_to_goal)
        elif self.state == self.STATE_WALL_FOLLOW:
            twist = self._do_wall_follow(dist_to_goal)
        else:
            twist = Twist()

        self.pub_cmd.publish(twist)

        # Log
        if self.logger_traj:
            self.logger_traj.log(
                self.robot_x, self.robot_y, self.robot_yaw,
                dist_to_goal=round(dist_to_goal, 4),
                state=self.state)

    # ─── Campo potencial ─────────────────────────────────────────

    def _do_potential_field(self, dist_to_goal: float) -> Twist:
        """Controle por campo potencial com detecção de mínimo local."""
        now = self._now()

        # Checa se está parado
        moved = distance(
            (self.robot_x, self.robot_y),
            (self.last_move_x, self.last_move_y))

        if moved > self.stuck_move_thresh:
            self.last_move_x = self.robot_x
            self.last_move_y = self.robot_y
            self.last_move_time = now

        stuck_elapsed = now - self.last_move_time

        if stuck_elapsed > self.stuck_timeout:
            # Preso! Entra em wall-following
            return self._enter_wall_follow(dist_to_goal)

        # Campo potencial normal
        f_att_x, f_att_y = compute_attractive(
            self.robot_x, self.robot_y,
            self.goal_x, self.goal_y,
            k_att=self.k_att,
            d_threshold=self.d_threshold)

        f_rep_x, f_rep_y = compute_repulsive(
            self.last_scan, self.robot_yaw,
            k_rep=self.k_rep, d0=self.d0)

        fx = f_att_x + f_rep_x
        fy = f_att_y + f_rep_y

        # Se a força resultante é muito pequena, provavelmente está quase preso
        mag = math.hypot(fx, fy)
        if mag < 0.05 and dist_to_goal > self.goal_tolerance * 2:
            return self._enter_wall_follow(dist_to_goal)

        return force_to_twist(
            fx, fy, self.robot_yaw,
            v_max=self.v_max,
            omega_max=self.omega_max,
            k_omega=self.k_omega)

    def _enter_wall_follow(self, dist_to_goal: float) -> Twist:
        """Transição para wall-following."""
        self.escape_attempts += 1
        self.state = self.STATE_WALL_FOLLOW
        self.dist_at_stuck = dist_to_goal
        self.best_dist_in_wf = dist_to_goal
        self.wall_follow_start = self._now()
        self.wf_start_x = self.robot_x
        self.wf_start_y = self.robot_y
        self.wf_traveled_enough = False

        # Alterna o lado a cada tentativa
        self.wall_follow_side = 1.0 if self.escape_attempts % 2 == 1 else -1.0

        self.get_logger().warn(
            f'Mínimo local detectado (tentativa {self.escape_attempts}). '
            f'Wall-follow lado {"esquerdo" if self.wall_follow_side > 0 else "direito"}, '
            f'd_meta={dist_to_goal:.2f} m')

        return self._do_wall_follow(dist_to_goal)

    # ─── Wall-following orientado à meta ─────────────────────────

    def _do_wall_follow(self, dist_to_goal: float) -> Twist:
        """Segue a parede até encontrar ponto com distância à meta menor.

        Condições de saída (qualquer uma):
        1. dist_to_goal < dist_at_stuck - margem → encontrou caminho melhor
        2. Tempo máximo esgotado → desiste e volta ao potencial
        3. Caminho para meta está livre e mais perto → sai direto
        """
        now = self._now()
        elapsed = now - self.wall_follow_start

        # Atualiza melhor distância durante wall-follow
        if dist_to_goal < self.best_dist_in_wf:
            self.best_dist_in_wf = dist_to_goal

        # Precisa ter andado um pouco antes de considerar sair
        dist_from_start = distance(
            (self.robot_x, self.robot_y),
            (self.wf_start_x, self.wf_start_y))
        if dist_from_start > 0.5:
            self.wf_traveled_enough = True

        # Condição de saída 1: encontrou ponto melhor
        margin = 0.3  # precisa ser pelo menos 30cm melhor que onde ficou preso
        if self.wf_traveled_enough and dist_to_goal < (self.dist_at_stuck - margin):
            self.get_logger().info(
                f'Escape bem-sucedido! d_meta: {self.dist_at_stuck:.2f} → {dist_to_goal:.2f} m')
            self.state = self.STATE_POTENTIAL
            self._reset_stuck_timer()
            return Twist()  # pausa breve antes de retomar

        # Condição de saída 2: tempo máximo
        if elapsed > self.wall_follow_max:
            self.get_logger().warn(
                f'Wall-follow timeout ({self.wall_follow_max:.0f}s). '
                f'Voltando ao potencial.')
            self.state = self.STATE_POTENTIAL
            self._reset_stuck_timer()
            return Twist()

        # Condição de saída 3: voltou ao ponto de partida (deu volta completa)
        if self.wf_traveled_enough and dist_from_start < 0.3:
            self.get_logger().warn(
                'Deu volta completa no obstáculo sem encontrar escape.')
            self.state = self.STATE_POTENTIAL
            self._reset_stuck_timer()
            return Twist()

        # ─── Controle de wall-following ───
        return self._wall_follow_twist()

    def _wall_follow_twist(self) -> Twist:
        """Gera o Twist para seguir a parede.

        Usa as leituras do laser para manter distância fixa da parede.
        O robô anda para frente e ajusta o ângulo baseado em:
        - Distância lateral ao obstáculo (manter wall_follow_dist)
        - Se tem obstáculo na frente (precisa girar)
        """
        scan = self.last_scan
        twist = Twist()
        side = self.wall_follow_side  # +1=esquerda, -1=direita

        n = len(scan.ranges)
        if n == 0:
            return twist

        # Divide o laser em regiões
        # Front: -30° a +30° (índices centrais)
        # Side: ±60° a ±120° (dependendo do lado)
        front_min = float('inf')
        side_min = float('inf')
        side_angle = 0.0

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # Frente: |angle| < 30°
            if abs(angle) < math.radians(30):
                if r < front_min:
                    front_min = r

            # Lado de contorno
            if side > 0:
                # Esquerda: 30° a 120°
                if math.radians(30) < angle < math.radians(120):
                    if r < side_min:
                        side_min = r
                        side_angle = angle
            else:
                # Direita: -120° a -30°
                if math.radians(-120) < angle < math.radians(-30):
                    if r < side_min:
                        side_min = r
                        side_angle = angle

        # Lógica de controle
        # 1. Se obstáculo na frente → gira para o lado oposto
        if front_min < self.wall_follow_dist * 1.5:
            twist.linear.x = 0.05
            twist.angular.z = self.omega_max * 0.6 * side
            return twist

        # 2. Se não vê parede lateral → gira em direção à parede
        #    (acabou o obstáculo, precisa continuar contornando)
        if side_min == float('inf') or side_min > self.wall_follow_dist * 3:
            twist.linear.x = self.v_max * 0.5
            twist.angular.z = -0.8 * side  # gira na direção da parede
            return twist

        # 3. Mantém distância da parede lateral
        dist_error = side_min - self.wall_follow_dist
        # PD controller: P no erro de distância
        angular_correction = -side * 1.5 * dist_error

        twist.linear.x = self.v_max * 0.6
        twist.angular.z = angular_correction
        twist.angular.z = max(-self.omega_max, min(self.omega_max, twist.angular.z))

        return twist

    # ─── Utilidades ──────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _reset_stuck_timer(self):
        now = self._now()
        self.last_move_x = self.robot_x
        self.last_move_y = self.robot_y
        self.last_move_time = now

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.pub_marker.publish(marker)

    def destroy_node(self):
        self.pub_cmd.publish(Twist())
        if self.logger_traj:
            self.logger_traj.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()