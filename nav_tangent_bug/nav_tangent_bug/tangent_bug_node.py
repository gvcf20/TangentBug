"""Servidor de action NavigateToGoal implementando o algoritmo Tangent Bug.

O Tangent Bug usa um sensor laser para navegar de forma reativa até uma meta,
contornando obstáculos quando necessário, com garantia de completude:
se existe caminho, encontra; se não existe, detecta em tempo finito.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from nav_msgs_custom.action import NavigateToGoal

from nav_common.geometry import (
    yaw_from_quaternion, distance, angle_to_target, angle_diff, wrap_to_pi
)
from nav_common.laser_utils import (
    find_discontinuities, closest_obstacle, is_path_clear
)
from nav_common.diff_drive import force_to_twist
from nav_common.plotting import TrajectoryLogger

from nav_tangent_bug.states import TBState
from nav_tangent_bug.heuristic import compute_d_reach, find_best_tangent_point


class TangentBugNode(Node):

    def __init__(self):
        super().__init__('tangent_bug')

        # Parâmetros
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        self.declare_parameter('k_omega', 2.5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('safe_distance', 0.30)
        self.declare_parameter('wall_follow_distance', 0.35)
        self.declare_parameter('discontinuity_threshold', 0.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('loop_closure_dist', 0.3)
        self.declare_parameter('loop_closure_min_travel', 1.5)
        self.declare_parameter('bf_stagnation_timeout', 5.0)
        self.declare_parameter('log_trajectory', True)
        self.declare_parameter('log_file', '/tmp/tangent_bug_trajectory.csv')

        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.k_omega = self.get_parameter('k_omega').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.wf_dist = self.get_parameter('wall_follow_distance').value
        self.disc_thresh = self.get_parameter('discontinuity_threshold').value
        self.control_rate = self.get_parameter('control_rate').value
        self.loop_closure_dist = self.get_parameter('loop_closure_dist').value
        self.loop_closure_min_travel = self.get_parameter('loop_closure_min_travel').value
        self.bf_stagnation_timeout = self.get_parameter('bf_stagnation_timeout').value

        # Estado do robô
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.last_scan = None

        # Callback groups
        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10,
            callback_group=cb_group)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb,
            rclpy.qos.QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT),
            callback_group=cb_group)

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_markers = self.create_publisher(Marker, '/tangent_bug_markers', 10)

        # Action server
        self.action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=cb_group)

        # Logger
        self.logger_traj = None
        if self.get_parameter('log_trajectory').value:
            log_file = self.get_parameter('log_file').value
            self.logger_traj = TrajectoryLogger(
                log_file, extra_fields=['state', 'd_reach', 'd_followed'])

        self.get_logger().info('Tangent Bug pronto. Envie meta via tangent_bug_client.')

    # ─── Callbacks ───────────────────────────────────────────────

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def scan_cb(self, msg):
        self.last_scan = msg

    def goal_cb(self, goal_request):
        self.get_logger().info(
            f'Meta recebida: ({goal_request.target.x:.2f}, '
            f'{goal_request.target.y:.2f})')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancelamento solicitado.')
        return CancelResponse.ACCEPT

    # ─── Execute ─────────────────────────────────────────────────

    def execute_cb(self, goal_handle):
        target = goal_handle.request.target
        goal_x, goal_y = target.x, target.y

        self.get_logger().info(f'Navegando para ({goal_x:.2f}, {goal_y:.2f})')
        self.publish_goal_marker(goal_x, goal_y)

        # Inicialização
        state = TBState.MOTION_TO_GOAL
        d_followed = float('inf')
        d_reach = float('inf')

        # Boundary-following tracking
        bf_start_x = 0.0
        bf_start_y = 0.0
        bf_total_travel = 0.0
        bf_prev_x = 0.0
        bf_prev_y = 0.0
        wf_side = 1.0

        # Detecção de estagnação
        bf_best_dist = float('inf')
        bf_best_dist_time = 0.0

        rate = self.create_rate(self.control_rate)
        feedback_msg = NavigateToGoal.Feedback()
        result_msg = NavigateToGoal.Result()

        while rclpy.ok():
            try:
                # Cancelamento
                if goal_handle.is_cancel_requested:
                    self.pub_cmd.publish(Twist())
                    goal_handle.canceled()
                    result_msg.success = False
                    result_msg.message = 'cancelled'
                    return result_msg

                # Espera dados
                if not self.odom_received or self.last_scan is None:
                    rate.sleep()
                    continue

                dist_to_goal = distance(
                    (self.robot_x, self.robot_y), (goal_x, goal_y))

                # ─── GOAL REACHED ────────────────────────────
                if dist_to_goal <= self.goal_tol:
                    self.pub_cmd.publish(Twist())
                    result_msg.success = True
                    result_msg.message = 'goal_reached'
                    self.get_logger().info(
                        f'Meta alcançada! d={dist_to_goal:.3f} m')
                    try:
                        goal_handle.succeed()
                    except Exception as e:
                        self.get_logger().warn(f'Erro ao finalizar action: {e}')
                    return result_msg

                scan = self.last_scan
                d_reach = compute_d_reach(
                    self.robot_x, self.robot_y, self.robot_yaw,
                    goal_x, goal_y, scan)
                disconts = find_discontinuities(scan, self.disc_thresh)

                # ─── MOTION TO GOAL ──────────────────────────
                if state == TBState.MOTION_TO_GOAL:
                    angle_to_goal_laser = angle_diff(
                        angle_to_target(
                            (self.robot_x, self.robot_y), (goal_x, goal_y)),
                        self.robot_yaw)

                    path_free = is_path_clear(
                        scan, angle_to_goal_laser,
                        cone_half_angle=math.radians(20),
                        safe_distance=self.safe_dist)

                    if path_free:
                        twist = self._go_toward(goal_x, goal_y)
                    else:
                        # Entra em boundary-following
                        state = TBState.BOUNDARY_FOLLOWING
                        d_followed = dist_to_goal

                        # Escolhe lado pelo produto vetorial
                        obs = closest_obstacle(scan)
                        if obs is not None:
                            obs_range, obs_angle, _ = obs
                            ox = self.robot_x + obs_range * math.cos(self.robot_yaw + obs_angle)
                            oy = self.robot_y + obs_range * math.sin(self.robot_yaw + obs_angle)
                            to_obs_x = ox - self.robot_x
                            to_obs_y = oy - self.robot_y
                            to_goal_x = goal_x - self.robot_x
                            to_goal_y = goal_y - self.robot_y
                            cross = to_obs_x * to_goal_y - to_obs_y * to_goal_x
                            wf_side = 1.0 if cross > 0 else -1.0
                        else:
                            wf_side = 1.0

                        bf_start_x = self.robot_x
                        bf_start_y = self.robot_y
                        bf_prev_x = self.robot_x
                        bf_prev_y = self.robot_y
                        bf_total_travel = 0.0
                        bf_best_dist = dist_to_goal
                        bf_best_dist_time = self.get_clock().now().nanoseconds / 1e9

                        self.get_logger().info(
                            f'Obstáculo detectado. BF '
                            f'lado {"esq" if wf_side > 0 else "dir"}, '
                            f'd={d_followed:.2f}')

                        twist = self._wall_follow(scan, wf_side)

                # ─── BOUNDARY FOLLOWING ──────────────────────
                elif state == TBState.BOUNDARY_FOLLOWING:
                    if dist_to_goal < d_followed:
                        d_followed = dist_to_goal

                    step = distance(
                        (self.robot_x, self.robot_y),
                        (bf_prev_x, bf_prev_y))
                    bf_total_travel += step
                    bf_prev_x = self.robot_x
                    bf_prev_y = self.robot_y

                    # Detecção de estagnação
                    now_ts = self.get_clock().now().nanoseconds / 1e9
                    if dist_to_goal < bf_best_dist - 0.05:
                        bf_best_dist = dist_to_goal
                        bf_best_dist_time = now_ts
                    elif (now_ts - bf_best_dist_time) > self.bf_stagnation_timeout:
                        state = TBState.MOTION_TO_GOAL
                        d_followed = float('inf')
                        bf_best_dist = float('inf')
                        self.get_logger().warn(
                            'Estagnação no BF. Voltando para MTG.')
                        twist = self._go_toward(goal_x, goal_y)
                        self.pub_cmd.publish(twist)
                        rate.sleep()
                        continue

                    # Caminho direto livre?
                    angle_to_goal_laser = angle_diff(
                        angle_to_target(
                            (self.robot_x, self.robot_y), (goal_x, goal_y)),
                        self.robot_yaw)
                    path_free = is_path_clear(
                        scan, angle_to_goal_laser,
                        cone_half_angle=math.radians(20),
                        safe_distance=self.safe_dist)

                    if path_free and bf_total_travel > 0.3:
                        state = TBState.MOTION_TO_GOAL
                        self.get_logger().info(
                            f'Caminho livre! d={dist_to_goal:.2f}')
                        twist = self._go_toward(goal_x, goal_y)

                    elif d_reach < d_followed - 0.05:
                        state = TBState.MOTION_TO_GOAL
                        self.get_logger().info(
                            f'Atalho! d_reach={d_reach:.2f} < '
                            f'd_followed={d_followed:.2f}')
                        twist = self._go_toward(goal_x, goal_y)

                    elif (bf_total_travel > self.loop_closure_min_travel and
                          distance((self.robot_x, self.robot_y),
                                   (bf_start_x, bf_start_y)) < self.loop_closure_dist):
                        self.pub_cmd.publish(Twist())
                        result_msg.success = False
                        result_msg.message = 'no_path_found'
                        self.get_logger().warn(
                            f'Sem caminho! Volta completa após '
                            f'{bf_total_travel:.1f} m.')
                        try:
                            goal_handle.succeed()
                        except Exception:
                            pass
                        return result_msg

                    else:
                        twist = self._wall_follow(scan, wf_side)

                self.pub_cmd.publish(twist)

                # Feedback
                feedback_msg.distance_to_goal = float(dist_to_goal)
                feedback_msg.current_state = state.name.lower()
                feedback_msg.d_reach = float(d_reach)
                feedback_msg.d_followed = float(d_followed)
                goal_handle.publish_feedback(feedback_msg)

                if self.logger_traj:
                    self.logger_traj.log(
                        self.robot_x, self.robot_y, self.robot_yaw,
                        state=state.name,
                        d_reach=round(d_reach, 4),
                        d_followed=round(d_followed, 4))

            except Exception as e:
                self.get_logger().error(f'Erro: {e}')
                self.pub_cmd.publish(Twist())

            rate.sleep()

        self.pub_cmd.publish(Twist())
        result_msg.success = False
        result_msg.message = 'shutdown'
        goal_handle.abort()
        return result_msg

    # ─── Helpers ─────────────────────────────────────────────────

    def _go_toward(self, goal_x, goal_y) -> Twist:
        fx = goal_x - self.robot_x
        fy = goal_y - self.robot_y
        mag = math.hypot(fx, fy)
        if mag > 1e-6:
            fx /= mag
            fy /= mag
        return force_to_twist(
            fx, fy, self.robot_yaw,
            v_max=self.v_max,
            omega_max=self.omega_max,
            k_omega=self.k_omega)

    def _wall_follow(self, scan: LaserScan, side: float) -> Twist:
        twist = Twist()
        n = len(scan.ranges)
        if n == 0:
            return twist

        front_min = float('inf')
        side_min = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment

            if abs(angle) < math.radians(30):
                if r < front_min:
                    front_min = r

            if side > 0:
                if math.radians(30) < angle < math.radians(120):
                    if r < side_min:
                        side_min = r
            else:
                if math.radians(-120) < angle < math.radians(-30):
                    if r < side_min:
                        side_min = r

        if front_min < self.wf_dist * 1.5:
            twist.linear.x = 0.05
            twist.angular.z = self.omega_max * 0.5 * side
            return twist

        if side_min == float('inf') or side_min > self.wf_dist * 3:
            twist.linear.x = self.v_max * 0.4
            twist.angular.z = -0.8 * side
            return twist

        dist_error = side_min - self.wf_dist
        angular_correction = -side * 1.5 * dist_error

        twist.linear.x = self.v_max * 0.6
        twist.angular.z = max(-self.omega_max,
                              min(self.omega_max, angular_correction))
        return twist

    def publish_goal_marker(self, gx, gy):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'tb_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        self.pub_markers.publish(marker)

    def destroy_node(self):
        self.pub_cmd.publish(Twist())
        if self.logger_traj:
            self.logger_traj.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TangentBugNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()