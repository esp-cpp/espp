"""ROS 2 (rclpy) Fibonacci action client for the interop matrix: the goal paths
`ros2 action send_goal` cannot drive against the espp action server.

usage: ros2_fibonacci_client.py cancel [order]   send a goal, cancel it at the
                                                 first feedback, expect CANCELED
                                                 with a partial sequence, and the
                                                 cancel answered ERROR_NONE with
                                                 the goal in goals_canceling
       ros2_fibonacci_client.py reject [order]   send a goal the server refuses
                                                 (order <= 0), expect it rejected
       ros2_fibonacci_client.py cancel-unknown   cancel a goal id the server never
                                                 saw, expect ERROR_UNKNOWN_GOAL_ID
       ros2_fibonacci_client.py cancel-finished [order]
                                                 run a goal to completion, then
                                                 cancel it, expect
                                                 ERROR_GOAL_TERMINATED

Prints one of "CANCELED seq_len=<n>", "REJECTED", "CANCEL_UNKNOWN ...",
"CANCEL_FINISHED ..." or "FAIL: <why>" and exits 0 iff the expected outcome
happened. Works on every RMW the environment selects.
"""

import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node


class Client(Node):
    def __init__(self, action_name):
        super().__init__("fibonacci_interop_client")
        self.client = ActionClient(self, Fibonacci, action_name)
        # A CancelGoal client of our own on the action's cancel service: the
        # cancel-unknown mode sends an id the action client never handed out,
        # which rclpy's ActionClient has no public path for (and no stable
        # private one: Jazzy has no `_cancel_client`).
        self.cancel_client = self.create_client(CancelGoal, action_name + "/_action/cancel_goal")
        self.goal_handle = None
        self.cancel_sent = False
        self.cancel_future = None
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
            self.cancel_future = self.goal_handle.cancel_goal_async()

    def check_cancel_response(self, timeout_s):
        """The server's CancelGoal verdict for the cancel sent on this goal:
        ERROR_NONE and the goal listed in goals_canceling (rclpy's canonical
        client judges a cancel by that list, rclcpp's by the code)."""
        if self.cancel_future is None or not self.spin_until(self.cancel_future, timeout_s):
            return "FAIL: no cancel response"
        response = self.cancel_future.result()
        if response.return_code != CancelGoal.Response.ERROR_NONE:
            return "FAIL: cancel return_code %d, expected ERROR_NONE" % response.return_code
        ids = [bytes(info.goal_id.uuid) for info in response.goals_canceling]
        if bytes(self.goal_handle.goal_id.uuid) not in ids:
            return "FAIL: the goal is not in goals_canceling (%d listed)" % len(ids)
        return None

    def cancel_by_id(self, uuid_bytes, timeout_s):
        """A CancelGoal request for an arbitrary id, through our own client
        on the action's cancel service; returns the response or None."""
        if not self.cancel_client.wait_for_service(timeout_sec=timeout_s):
            return None
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = list(uuid_bytes)
        future = self.cancel_client.call_async(request)
        if not self.spin_until(future, timeout_s):
            return None
        return future.result()

    def run(self, mode, order, timeout_s):
        self.mode = mode
        if not self.client.wait_for_server(timeout_sec=timeout_s):
            return "FAIL: action server not available"
        if mode == "cancel-unknown":
            response = self.cancel_by_id(os.urandom(16), timeout_s)
            if response is None:
                return "FAIL: no cancel response"
            if response.return_code != CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
                return "FAIL: cancel of an unknown goal answered %d, expected ERROR_UNKNOWN_GOAL_ID (%d)" % (
                    response.return_code, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)
            if len(response.goals_canceling) != 0:
                return "FAIL: %d goals canceling for an unknown id" % len(response.goals_canceling)
            return "CANCEL_UNKNOWN return_code=%d" % response.return_code
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
        if mode == "cancel" and self.feedback_count > 0 and not self.cancel_sent:
            # Feedback comes over its own topic and can land before the goal
            # response resolves, when on_feedback() had no handle to cancel
            # with: send the pending cancellation now rather than wait for a
            # later feedback that a short goal may never produce.
            self.cancel_sent = True
            self.get_logger().info("feedback arrived before the goal response: cancelling the goal")
            self.cancel_future = self.goal_handle.cancel_goal_async()
        result = self.goal_handle.get_result_async()
        if not self.spin_until(result, timeout_s):
            return "FAIL: no result (cancel_sent=%s feedback=%d)" % (self.cancel_sent, self.feedback_count)
        status = result.result().status
        seq = list(result.result().result.sequence)
        if mode == "cancel-finished":
            if status != GoalStatus.STATUS_SUCCEEDED:
                return "FAIL: status %d, expected SUCCEEDED (%d)" % (status, GoalStatus.STATUS_SUCCEEDED)
            # The goal is done and its result fetched; the server still knows it
            # (ActionConfig::result_timeout) and must say so.
            cancel = self.goal_handle.cancel_goal_async()
            if not self.spin_until(cancel, timeout_s):
                return "FAIL: no cancel response for the finished goal"
            response = cancel.result()
            if response.return_code != CancelGoal.Response.ERROR_GOAL_TERMINATED:
                return "FAIL: cancel of a finished goal answered %d, expected ERROR_GOAL_TERMINATED (%d)" % (
                    response.return_code, CancelGoal.Response.ERROR_GOAL_TERMINATED)
            return "CANCEL_FINISHED return_code=%d seq_len=%d" % (response.return_code, len(seq))
        if mode == "cancel":
            if status != GoalStatus.STATUS_CANCELED:
                return "FAIL: status %d, expected CANCELED (%d); seq_len=%d" % (
                    status, GoalStatus.STATUS_CANCELED, len(seq))
            if not self.cancel_sent:
                return "FAIL: canceled without a cancel request"
            verdict = self.check_cancel_response(timeout_s)
            if verdict is not None:
                return verdict
            # The partial-sequence contract: the result is a proper Fibonacci
            # prefix (the server starts at [0, 1] and appends one element per
            # step, order + 1 elements when it runs to completion) and shorter
            # than the complete sequence, so an empty, corrupt or finished
            # result cannot pass as a cancellation.
            full = order + 1
            expected = [0, 1]
            while len(expected) < len(seq):
                expected.append(expected[-1] + expected[-2])
            if len(seq) < 2 or seq != expected[:len(seq)]:
                return "FAIL: CANCELED result is not a Fibonacci prefix: %s" % seq
            if len(seq) >= full:
                return "FAIL: CANCELED result is the complete sequence (%d elements): nothing was cut short" % len(seq)
            return "CANCELED seq_len=%d (of %d)" % (len(seq), full)
        return "FAIL: unknown mode %s" % mode


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "cancel"
    default_order = {"cancel": 12, "cancel-finished": 5}.get(mode, 0)
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
