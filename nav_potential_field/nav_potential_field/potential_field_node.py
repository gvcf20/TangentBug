"""Nó ROS 2 de navegação por campo potencial.

Pipeline:
    /odom + /scan → atrativo + repulsivo → force_to_twist → /cmd_vel

Inclui mecanismo anti-mínimo-local em 3 níveis:
  1. Perturbação lateral leve
  2. Perturbação forte com alternância de direção
  3. Wall-following temporário
"""

import math
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

from nav_common.geometry import (
    yaw_from_quaternion,
    wrap_to_pi,
)

from nav_potential_field.attractive import (
    compute_attractive,
)

from nav_potential_field.repulsive import (
    compute_repulsive,
)


# ============================================================
# FSM
# ============================================================

class NavState(Enum):
    IDLE = 0
    GO_TO_GOAL = 1
    ESCAPE = 2
    WALL_FOLLOW = 3
    GOAL_REACHED = 4


# ============================================================
# NODE
# ============================================================

class PotentialFieldNode(Node):

    def __init__(self):

        super().__init__("potential_field")

        # ========================================================
        # PARAMETERS
        # ========================================================

        self.declare_parameter("control_rate", 20.0)

        self.declare_parameter("k_att", 1.2)
        self.declare_parameter("k_rep", 0.9)

        self.declare_parameter("d0", 1.2)

        self.declare_parameter("goal_tolerance", 0.15)

        self.declare_parameter("max_linear", 0.45)
        self.declare_parameter("max_angular", 1.8)

        self.declare_parameter("escape_gain", 1.5)

        self.declare_parameter("enable_tangential", True)
        self.declare_parameter("tangential_gain", 0.35)

        self.declare_parameter("smoothing_alpha", 0.75)

        self.declare_parameter("stuck_timeout", 4.0)

        self.declare_parameter("wall_follow_time", 4.0)

        self.declare_parameter("front_safety_distance", 0.22)

        # ========================================================
        # LOAD
        # ========================================================

        self.control_rate = self.get_parameter(
            "control_rate").value

        self.k_att = self.get_parameter(
            "k_att").value

        self.k_rep = self.get_parameter(
            "k_rep").value

        self.d0 = self.get_parameter(
            "d0").value

        self.goal_tolerance = self.get_parameter(
            "goal_tolerance").value

        self.max_linear = self.get_parameter(
            "max_linear").value

        self.max_angular = self.get_parameter(
            "max_angular").value

        self.escape_gain = self.get_parameter(
            "escape_gain").value

        self.enable_tangential = self.get_parameter(
            "enable_tangential").value

        self.tangential_gain = self.get_parameter(
            "tangential_gain").value

        self.smoothing_alpha = self.get_parameter(
            "smoothing_alpha").value

        self.stuck_timeout = self.get_parameter(
            "stuck_timeout").value

        self.wall_follow_time = self.get_parameter(
            "wall_follow_time").value

        self.front_safety_distance = self.get_parameter(
            "front_safety_distance").value

        # ========================================================
        # ROBOT STATE
        # ========================================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.scan = None

        self.goal_x = None
        self.goal_y = None

        self.has_odom = False
        self.has_scan = False
        self.has_goal = False

        # ========================================================
        # FSM
        # ========================================================

        self.state = NavState.IDLE

        # ========================================================
        # FORCE FILTER
        # ========================================================

        self.filtered_fx = 0.0
        self.filtered_fy = 0.0

        # ========================================================
        # STUCK DETECTION
        # ========================================================

        self.progress_history = deque(maxlen=60)

        self.escape_direction = 1.0

        self.escape_start_time = 0.0

        # ========================================================
        # SUBSCRIBERS
        # ========================================================

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            rclpy.qos.QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10,
        )

        # ========================================================
        # PUBLISHERS
        # ========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/pf_debug",
            10,
        )

        # ========================================================
        # TIMER
        # ========================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
        )

        self.get_logger().info(
            "Potential Field Improved Node Started"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        self.robot_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        self.has_odom = True

    def scan_callback(self, msg):

        self.scan = msg
        self.has_scan = True

    def goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.has_goal = True

        self.state = NavState.GO_TO_GOAL

        self.progress_history.clear()

        self.get_logger().info(
            f"Goal received: "
            f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    # ============================================================
    # MAIN LOOP
    # ============================================================

    def control_loop(self):

        if not (
            self.has_odom
            and self.has_scan
            and self.has_goal
        ):
            return

        # ========================================================
        # GOAL CHECK
        # ========================================================

        goal_dist = math.hypot(
            self.goal_x - self.robot_x,
            self.goal_y - self.robot_y,
        )

        if goal_dist < self.goal_tolerance:

            self.stop_robot()

            self.state = NavState.GOAL_REACHED

            self.get_logger().info(
                "Goal reached."
            )

            return

        # ========================================================
        # STUCK CHECK
        # ========================================================

        self.update_progress(goal_dist)

        # ========================================================
        # FSM
        # ========================================================

        if self.state == NavState.GO_TO_GOAL:

            twist = self.go_to_goal_controller()

        elif self.state == NavState.ESCAPE:

            twist = self.escape_controller()

        elif self.state == NavState.WALL_FOLLOW:

            twist = self.wall_follow_controller()

        else:

            twist = Twist()

        self.cmd_pub.publish(twist)

    # ============================================================
    # GO TO GOAL
    # ============================================================

    def go_to_goal_controller(self):

        # ========================================================
        # ATTRACTIVE
        # ========================================================

        f_att_x, f_att_y, _, _ = compute_attractive(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            self.goal_x,
            self.goal_y,
            k_att=self.k_att,
        )

        # ========================================================
        # REPULSIVE
        # ========================================================

        f_rep_x, f_rep_y = compute_repulsive(
            self.scan,
            k_rep=self.k_rep,
            d0=self.d0,
        )

        # ========================================================
        # TOTAL FORCE
        # ========================================================

        fx = f_att_x + f_rep_x
        fy = f_att_y + f_rep_y

        # ========================================================
        # TANGENTIAL FIELD
        # ========================================================

        if self.enable_tangential:

            tx = -f_rep_y
            ty = f_rep_x

            tx *= self.tangential_gain
            ty *= self.tangential_gain

            fx += tx
            fy += ty

        # ========================================================
        # FILTER
        # ========================================================

        fx = (
            self.smoothing_alpha * self.filtered_fx
            + (1.0 - self.smoothing_alpha) * fx
        )

        fy = (
            self.smoothing_alpha * self.filtered_fy
            + (1.0 - self.smoothing_alpha) * fy
        )

        self.filtered_fx = fx
        self.filtered_fy = fy

        # ========================================================
        # SAFETY STOP
        # ========================================================

        if self.front_collision_risk():

            twist = Twist()

            twist.angular.z = (
                0.8 * self.escape_direction
            )

            return twist

        # ========================================================
        # DEBUG
        # ========================================================

        self.publish_force_marker(fx, fy)

        # ========================================================
        # CONTROL
        # ========================================================

        return self.force_to_cmd_vel(fx, fy)

    # ============================================================
    # ESCAPE
    # ============================================================

    def escape_controller(self):

        twist = Twist()

        twist.linear.x = 0.0

        twist.angular.z = (
            self.escape_gain
            * self.escape_direction
        )

        elapsed = (
            self.get_clock().now().nanoseconds / 1e9
            - self.escape_start_time
        )

        if elapsed > 2.0:

            self.state = NavState.WALL_FOLLOW

        return twist

    # ============================================================
    # WALL FOLLOW
    # ============================================================

    def wall_follow_controller(self):

        twist = Twist()

        twist.linear.x = 0.15

        twist.angular.z = (
            0.5 * self.escape_direction
        )

        elapsed = (
            self.get_clock().now().nanoseconds / 1e9
            - self.escape_start_time
        )

        if elapsed > self.wall_follow_time:

            self.state = NavState.GO_TO_GOAL

            self.progress_history.clear()

        return twist

    # ============================================================
    # FORCE → CMD_VEL
    # ============================================================

    def force_to_cmd_vel(self, fx, fy):

        twist = Twist()

        desired_heading = math.atan2(fy, fx)

        heading_error = wrap_to_pi(
            desired_heading - self.robot_yaw
        )

        force_norm = math.hypot(fx, fy)

        # ========================================================
        # ANGULAR
        # ========================================================

        twist.angular.z = (
            2.2 * heading_error
        )

        twist.angular.z = max(
            -self.max_angular,
            min(
                self.max_angular,
                twist.angular.z,
            )
        )

        # ========================================================
        # LINEAR
        # ========================================================

        alignment = max(
            math.cos(heading_error),
            0.0,
        )

        twist.linear.x = (
            0.4
            * force_norm
            * alignment
        )

        twist.linear.x = min(
            twist.linear.x,
            self.max_linear,
        )

        return twist

    # ============================================================
    # FRONT COLLISION
    # ============================================================

    def front_collision_risk(self):

        if self.scan is None:
            return False

        ranges = self.scan.ranges

        center = len(ranges) // 2

        width = 20

        for i in range(center - width, center + width):

            if i < 0 or i >= len(ranges):
                continue

            d = ranges[i]

            if not math.isfinite(d):
                continue

            if d < self.front_safety_distance:
                return True

        return False

    # ============================================================
    # PROGRESS
    # ============================================================

    def update_progress(self, goal_dist):

        now = (
            self.get_clock().now().nanoseconds / 1e9
        )

        self.progress_history.append(
            (now, goal_dist)
        )

        if len(self.progress_history) < 10:
            return

        old_time, old_dist = self.progress_history[0]

        progress = old_dist - goal_dist

        elapsed = now - old_time

        if elapsed < self.stuck_timeout:
            return

        # sem progresso suficiente
        if progress < 0.15:

            self.get_logger().warn(
                "Robot appears stuck."
            )

            self.state = NavState.ESCAPE

            self.escape_start_time = now

            self.escape_direction *= -1.0

            self.progress_history.clear()

    # ============================================================
    # DEBUG MARKER
    # ============================================================

    def publish_force_marker(self, fx, fy):

        marker = Marker()

        marker.header.frame_id = "base_link"

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = "forces"

        marker.id = 0

        marker.type = Marker.ARROW

        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color.a = 1.0
        marker.color.r = 1.0

        marker.points = []

        p0 = Vector3()
        p1 = Vector3()

        p1.x = fx
        p1.y = fy

        marker_pub = marker

        self.marker_pub.publish(marker_pub)

    # ============================================================
    # STOP
    # ============================================================

    def stop_robot(self):

        self.cmd_pub.publish(Twist())


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PotentialFieldNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()