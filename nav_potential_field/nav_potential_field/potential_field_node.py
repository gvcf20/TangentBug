"""Nó ROS 2 de navegação por campo potencial.

Pipeline:
    /odom + /scan → atrativo + repulsivo → force_to_twist → /cmd_vel

Inclui mecanismo anti-mínimo-local em 3 níveis:
  1. Perturbação lateral leve
  2. Perturbação forte com alternância de direção
  3. Wall-following temporário
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from nav_common.geometry import yaw_from_quaternion, distance, wrap_to_pi
from nav_common.diff_drive import force_to_twist
from nav_common.laser_utils import closest_obstacle
from nav_common.plotting import TrajectoryLogger

from nav_potential_field.attractive import compute_attractive
from nav_potential_field.repulsive import compute_repulsive


class PotentialFieldNode(Node):

    # Estados do anti-stuck
    STATE_NORMAL = 'normal'
    STATE_PERTURB_1 = 'perturb_light'
    STATE_PERTURB_2 = 'perturb_strong'
    STATE_WALL_FOLLOW = 'wall_follow'

    def __init__(self):
        super().__init__('potential_field')

        # Parâmetros
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.5)
        self.declare_parameter('d0', 1.5)
        self.declare_parameter('d_threshold', 2.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('k_omega', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('wall_follow_distance', 0.6)
        self.declare_parameter('wall_follow_duration', 5.0)
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
        self.wall_follow_duration = self.get_parameter('wall_follow_duration').value

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
        self.goal_reached = False

        # Anti mínimo local
        self.stuck_state = self.STATE_NORMAL
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0
        self.last_progress_time = 0.0
        self.stuck_start_time = 0.0
        self.perturb_direction = 1.0  # +1 ou -1 (esquerda/direita)
        self.wall_follow_start_time = 0.0
        self.best_dist_to_goal = float('inf')

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
        self.pub_cmd = self.create_publisher(
        TwistStamped,
        '/cmd_vel',
        10
        )
        self.pub_marker = self.create_publisher(Marker, '/potential_markers', 10)

        # Timer de controle
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
            'Potential field pronto. Envie meta via RViz "2D Goal Pose" '
            'ou: ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '
            '"{header: {frame_id: odom}, pose: {position: {x: 5.0, y: 3.0}}}"')

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
        self.goal_reached = False

        # zera estados internos
        self.stuck_state = self.STATE_NORMAL
        self.best_dist_to_goal = float('inf')

        # para movimento anterior
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.twist = Twist()

        self.pub_cmd.publish(stop_msg)
        self.pub_cmd.publish(TwistStamped())
        
        self._reset_stuck()

        self.get_logger().info(
            f'Nova meta: ({self.goal_x:.2f}, {self.goal_y:.2f})')

        self.publish_goal_marker()

    def control_loop(self):
        if not self.odom_received or self.last_scan is None:
            return
        if not self.goal_active or self.goal_reached:
            return

        dist_to_goal = distance(
            (self.robot_x, self.robot_y),
            (self.goal_x, self.goal_y))

        # Atualiza melhor distância (para detectar progresso real)
        if dist_to_goal < self.best_dist_to_goal - 0.05:
            self.best_dist_to_goal = dist_to_goal
            self._reset_stuck()

        # Chegou?
        if dist_to_goal < self.goal_tolerance:

            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.twist = Twist()

            self.pub_cmd.publish(stop_msg)
            self.goal_reached = True
            self.goal_active = False
            self.stuck_state = self.STATE_NORMAL
            self.get_logger().info(
                f'Meta alcançada! Distância final: {dist_to_goal:.3f} m')
            return

        # Escolhe comportamento baseado no estado
        if self.stuck_state == self.STATE_WALL_FOLLOW:
            twist = self._wall_follow_control()
        else:
            twist = self._potential_field_control(dist_to_goal)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = twist

        self.pub_cmd.publish(msg)

        # Log
        if self.logger_traj:
            self.logger_traj.log(
                self.robot_x, self.robot_y, self.robot_yaw,
                dist_to_goal=round(dist_to_goal, 4),
                state=self.stuck_state)

    def _potential_field_control(self, dist_to_goal: float) -> Twist:
        """Controle normal por campo potencial, com perturbações se preso."""
        now = self.get_clock().now().nanoseconds / 1e9

        # Força atrativa
        f_att_x, f_att_y = compute_attractive(
            self.robot_x, self.robot_y,
            self.goal_x, self.goal_y,
            k_att=self.k_att,
            d_threshold=self.d_threshold)

        # Força repulsiva
        f_rep_x, f_rep_y = compute_repulsive(
            self.last_scan, self.robot_yaw,
            k_rep=self.k_rep, d0=self.d0)

        fx = f_att_x + f_rep_x
        fy = f_att_y + f_rep_y

        # Checa progresso
        moved = distance(
            (self.robot_x, self.robot_y),
            (self.last_progress_x, self.last_progress_y))

        if moved > 0.1:
            self.last_progress_x = self.robot_x
            self.last_progress_y = self.robot_y
            self.last_progress_time = now
            self.stuck_state = self.STATE_NORMAL
        else:
            elapsed = now - self.last_progress_time
            if elapsed < 0.01:
                elapsed = 0.01

            if elapsed > 10.0:
                # Nível 3: wall-following
                self.stuck_state = self.STATE_WALL_FOLLOW
                self.wall_follow_start_time = now
                self.get_logger().warn(
                    'Mínimo local persistente. Iniciando wall-following temporário.')
                return self._wall_follow_control()

            elif elapsed > 6.0:
                # Nível 2: perturbação forte, alternando direção
                if self.stuck_state != self.STATE_PERTURB_2:
                    self.stuck_state = self.STATE_PERTURB_2
                    self.perturb_direction *= -1.0  # alterna lado
                    self.get_logger().warn(
                        f'Perturbação forte (dir={self.perturb_direction:+.0f})')

                dx = self.goal_x - self.robot_x
                dy = self.goal_y - self.robot_y
                d = math.hypot(dx, dy)
                if d > 1e-6:
                    fx += self.perturb_direction * (-dy / d) * 1.5
                    fy += self.perturb_direction * (dx / d) * 1.5

            elif elapsed > 3.0:
                # Nível 1: perturbação leve
                if self.stuck_state != self.STATE_PERTURB_1:
                    self.stuck_state = self.STATE_PERTURB_1
                    self.get_logger().warn('Perturbação leve ativada.')

                dx = self.goal_x - self.robot_x
                dy = self.goal_y - self.robot_y
                d = math.hypot(dx, dy)
                if d > 1e-6:
                    fx += self.perturb_direction * (-dy / d) * 0.7
                    fy += self.perturb_direction * (dx / d) * 0.7

        return force_to_twist(
            fx, fy, self.robot_yaw,
            v_max=self.v_max,
            omega_max=self.omega_max,
            k_omega=self.k_omega)

    def _wall_follow_control(self) -> Twist:
        """Segue a parede do obstáculo mais próximo por tempo limitado.

        Mantém uma distância fixa da parede usando as leituras laterais
        do laser. Depois de wall_follow_duration segundos, volta ao
        campo potencial normal.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.wall_follow_start_time

        # Tempo esgotado → volta ao normal
        if elapsed > self.wall_follow_duration:
            self.get_logger().info('Wall-following encerrado. Voltando ao campo potencial.')
            self._reset_stuck()
            self.perturb_direction *= -1.0  # alterna para próxima vez
            return Twist()

        scan = self.last_scan
        twist = Twist()

        # Encontra obstáculo mais próximo
        obs = closest_obstacle(scan)
        if obs is None:
            # Sem obstáculo detectado → vai para a meta
            self._reset_stuck()
            return Twist()

        obs_range, obs_angle, _ = obs

        # Estratégia: andar para frente e ajustar ângulo para manter
        # distância wall_follow_dist do obstáculo mais próximo.
        # A direção de contorno depende de perturb_direction.

        # Ângulo desejado: perpendicular ao obstáculo
        if self.perturb_direction > 0:
            # Contorna pela esquerda: obstáculo deve ficar à direita
            desired_angle = obs_angle + math.pi / 2
        else:
            # Contorna pela direita: obstáculo deve ficar à esquerda
            desired_angle = obs_angle - math.pi / 2

        # Correção de distância: se muito perto, afasta; se muito longe, aproxima
        dist_error = obs_range - self.wall_follow_dist
        angle_correction = -0.5 * dist_error * self.perturb_direction

        desired_angle += angle_correction

        # Converte para velocidades
        # desired_angle está no frame do laser (= frame do robô para nós)
        twist.linear.x = self.v_max * 0.6  # mais devagar no wall-follow
        twist.angular.z = 1.5 * wrap_to_pi(desired_angle)
        twist.angular.z = max(-self.omega_max, min(self.omega_max, twist.angular.z))

        # Se muito perto de qualquer obstáculo, para e só gira
        if obs_range < 0.3:
            twist.linear.x = 0.0
            twist.angular.z = self.omega_max * self.perturb_direction

        return twist

    def _reset_stuck(self):
        """Reseta o estado anti-mínimo-local."""
        now = self.get_clock().now().nanoseconds / 1e9
        self.stuck_state = self.STATE_NORMAL
        self.last_progress_x = self.robot_x
        self.last_progress_y = self.robot_y
        self.last_progress_time = now

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
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
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.twist = Twist()

        self.pub_cmd.publish(stop_msg)
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