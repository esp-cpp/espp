# rclpy Fibonacci action server for the espp interop matrix: the espp action
# CLIENT (rtps_action_interop_client) drives this and checks the result.
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciServer(Node):
    def __init__(self):
        super().__init__("fibonacci_server")
        self._server = ActionServer(self, Fibonacci, "fibonacci", self.execute)

    def execute(self, goal_handle):
        order = goal_handle.request.order
        seq = [0, 1]
        for i in range(1, order):
            seq.append(seq[i] + seq[i - 1])
            fb = Fibonacci.Feedback()
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
            time.sleep(0.2)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = seq
        self.get_logger().info(f"goal order={order} -> {seq}")
        return result


def main():
    rclpy.init()
    rclpy.spin(FibonacciServer())


main()
