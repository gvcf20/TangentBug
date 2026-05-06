"""Nó de controle para um robô no exercício 4 (multi-robô).

Cada instância controla UM robô. O launch sobe N instâncias,
cada uma com robot_id diferente.

Pipeline:
    /robotN/odom + /robotN/scan + /robotM/odom (outros) → composição → /robotN/cmd_vel
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from nav_common.geometry import yaw_from_quaternion
from nav_common.diff_drive import force_to_twist

from nav_parametric_curve.curves import Lemniscate, Cardioid
from nav_multi_robot.composition import compute_composed_field


class MultiRobotNode(Node):

    def __init__(self):
        super().__init__('multi_robot_node')

        # Parâmetros
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('n_robots', 2)
        self.declare_parameter('curve_type', 'lemniscate')
        self.declare_parameter('curve_scale', 2.0)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta', 1.0)
        self.declare_parameter('gamma', 1.5)
        self.declare_parameter('k_normal', 1.5)
        self.declare_parameter('k_tangent', 1.0)
        self.declare_parameter('convergence_radius', 3.0)
        self.declare_parameter('k_rep_obs', 1.0)
        self.declare_parameter('d0_obs', 1.2)
        self.declare_parameter('k_rep_robot', 2.0)
        self.declare_parameter('d0_robot', 1.5)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        self.declare_parameter('k_omega', 2.0)
        self.declare_parameter('control_rate', 20.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.n_robots = self.get_parameter('n_robots').value
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.gamma = self.get_parameter('gamma').value
        self.k_normal = self.get_parameter('k_normal').value
        self.k_tangent = self.get_parameter('k_tangent').value
        self.conv_radius = self.get_parameter('convergence_radius').value
        self.k_rep_obs = self.get_parameter('k_rep_obs').value
        self.d0_obs = self.get_parameter('d0_obs').value
        self.k_rep_robot = self.get_parameter('k_rep_robot').value
        self.d0_robot = self.get_parameter('d0_robot').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.k_omega = self.get_parameter('k_omega').value
        control_rate = self.get_parameter('control_rate').value

        # Curva
        curve_type = self.get_parameter('curve_type').value
        curve_scale = self.get_parameter('curve_scale').value
        if curve_type == 'cardioid':
            self.curve = Cardioid(a=curve_scale)
        else:
            self.curve = Lemniscate(a=curve_scale)

        self.ns = f'robot_{self.robot_id}'

        # Estado próprio
        self.my_x = 0.0
        self.my_y = 0.0
        self.my_yaw = 0.0
        self.my_odom_received = False
        self.my_scan = None

        # Estado dos outros robôs
        self.other_poses = {}  # {robot_id: (x, y)}

        # Subscriber do próprio odom
        self.sub_odom = self.create_subscription(
            Odometry, f'/{self.ns}/odom', self.my_odom_cb, 10)

        # Subscriber do próprio scan (Best Effort)
        self.sub_scan = self.create_subscription(
            LaserScan, f'/{self.ns}/scan', self.my_scan_cb,
            rclpy.qos.QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        # Subscribers dos outros robôs
        for i in range(self.n_robots):
            if i == self.robot_id:
                continue
            other_ns = f'robot_{i}'
            self.create_subscription(
                Odometry, f'/{other_ns}/odom',
                lambda msg, rid=i: self.other_odom_cb(msg, rid), 10)

        # Publisher
        self.pub_cmd = self.create_publisher(
            Twist, f'/{self.ns}/cmd_vel', 10)

        # Timer
        period = 1.0 / control_rate
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'Multi-robot node [{self.ns}] iniciado. '
            f'Curva: {curve_type} (a={curve_scale})')

    def my_odom_cb(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        self.my_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.my_odom_received = True

    def my_scan_cb(self, msg):
        self.my_scan = msg

    def other_odom_cb(self, msg, robot_id):
        self.other_poses[robot_id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def control_loop(self):
        if not self.my_odom_received or self.my_scan is None:
            return

        # Lista de posições dos outros robôs
        others = list(self.other_poses.values())

        # Campo composto
        fx, fy = compute_composed_field(
            self.my_x, self.my_y, self.my_yaw,
            self.curve,
            self.my_scan,
            others,
            alpha=self.alpha,
            beta=self.beta,
            gamma=self.gamma,
            k_normal=self.k_normal,
            k_tangent=self.k_tangent,
            convergence_radius=self.conv_radius,
            k_rep_obs=self.k_rep_obs,
            d0_obs=self.d0_obs,
            k_rep_robot=self.k_rep_robot,
            d0_robot=self.d0_robot)

        twist = force_to_twist(
            fx, fy, self.my_yaw,
            v_max=self.v_max,
            omega_max=self.omega_max,
            k_omega=self.k_omega)

        self.pub_cmd.publish(twist)

    def destroy_node(self):
        self.pub_cmd.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()