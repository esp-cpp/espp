"""ROS 2 (rclpy) Fibonacci action client for the interop matrix: the goal paths
`ros2 action send_goal` cannot drive against the espp action server.

usage: ros2_fibonacci_client.py cancel [order]   send a goal, cancel it at the
                                                 first feedback, expect CANCELED
                                                 with a partial sequence
       ros2_fibonacci_client.py reject [order]   send a goal the server refuses
                                                 (order <= 0), expect it rejected

Prints one of "CANCELED seq_len=<n>", "REJECTED" or "FAIL: <why>" and exits 0
iff the expected outcome happened. Works on every RMW the environment selects.
"""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node


class Client(Node):
    def __init__(self, action_name):
        super().__init__("fibonacci_interop_client")
        self.client = ActionClient(self, Fibonacci, action_name)
        self.goal_handle = None
        self.cancel_sent = False
        self.feedback_count = 0

    def spin_until(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def on_feedback(self, msg):
        self.feedback_count += 1
        if self.mode == "cancel" and not self.cancel_sent and self.goal_handle is not None:
            self.cancel_sent = True
            self.get_logger().info("first feedback: cancelling the goal")
            self.goal_handle.cancel_goal_async()

    def run(self, mode, order, timeout_s):
        self.mode = mode
        if not self.client.wait_for_server(timeout_sec=timeout_s):
            return "FAIL: action server not available"
        goal = Fibonacci.Goal()
        goal.order = order
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        if not self.spin_until(send, timeout_s):
            return "FAIL: no goal response"
        self.goal_handle = send.result()
        if mode == "reject":
            return "REJECTED" if not self.goal_handle.accepted else "FAIL: goal accepted, expected a rejection"
        if not self.goal_handle.accepted:
            return "FAIL: goal rejected"
        result = self.goal_handle.get_result_async()
        if not self.spin_until(result, timeout_s):
            return "FAIL: no result (cancel_sent=%s feedback=%d)" % (self.cancel_sent, self.feedback_count)
        status = result.result().status
        seq = list(result.result().result.sequence)
        if mode == "cancel":
            if status != GoalStatus.STATUS_CANCELED:
                return "FAIL: status %d, expected CANCELED (%d); seq_len=%d" % (
                    status, GoalStatus.STATUS_CANCELED, len(seq))
            if not self.cancel_sent:
                return "FAIL: canceled without a cancel request"
            return "CANCELED seq_len=%d" % len(seq)
        return "FAIL: unknown mode %s" % mode


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "cancel"
    default_order = 12 if mode == "cancel" else 0
    order = int(sys.argv[2]) if len(sys.argv) > 2 else default_order
    rclpy.init()
    node = Client("/fibonacci")
    try:
        outcome = node.run(mode, order, timeout_s=25.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(outcome, flush=True)
    sys.exit(0 if not outcome.startswith("FAIL") else 1)


if __name__ == "__main__":
    main()
