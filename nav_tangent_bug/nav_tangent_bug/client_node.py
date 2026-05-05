"""Cliente de action para enviar metas ao Tangent Bug.

Uso:
    ros2 run nav_tangent_bug tangent_bug_client -- --x 5.0 --y 3.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs_custom.action import NavigateToGoal
import argparse
import sys


class TangentBugClient(Node):

    def __init__(self, goal_x, goal_y):
        super().__init__('tangent_bug_client')
        self.client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y

    def send_goal(self):
        self.get_logger().info('Esperando servidor Tangent Bug...')
        self.client.wait_for_server()

        goal_msg = NavigateToGoal.Goal()
        goal_msg.target.x = self.goal_x
        goal_msg.target.y = self.goal_y
        goal_msg.target.z = 0.0

        self.get_logger().info(
            f'Enviando meta: ({self.goal_x:.2f}, {self.goal_y:.2f})')

        future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Meta rejeitada!')
            return
        self.get_logger().info('Meta aceita.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.current_state}] d_goal={fb.distance_to_goal:.2f} '
            f'd_reach={fb.d_reach:.2f} d_followed={fb.d_followed:.2f}')

    def result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'SUCESSO: {result.message}')
        else:
            self.get_logger().warn(f'FALHA: {result.message}')
        rclpy.shutdown()


def main(args=None):
    # Parse argumentos da linha de comando
    parser = argparse.ArgumentParser(description='Tangent Bug Client')
    parser.add_argument('--x', type=float, default=5.0, help='Meta X')
    parser.add_argument('--y', type=float, default=3.0, help='Meta Y')

    # Filtra argumentos do ROS
    argv = sys.argv[1:]
    if '--' in sys.argv:
        idx = sys.argv.index('--')
        argv = sys.argv[idx + 1:]

    parsed = parser.parse_args(argv)

    rclpy.init(args=args)
    client = TangentBugClient(parsed.x, parsed.y)
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()