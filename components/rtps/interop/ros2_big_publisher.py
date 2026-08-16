#!/usr/bin/env python3
"""Publish a deterministic 200 KB std_msgs/String from a ROS 2 node.

Used by the interop harness for the ROS 2 -> espp large-payload (DATA_FRAG) leg.
The pattern is generated in-process (not passed on the command line) so it does
not hit the OS ARG_MAX limit that `ros2 topic pub "{data: '<200KB>'}"` does.
Best-effort QoS to match the v1 (best-effort) fragmentation scope.
"""

import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/bignum2"
    size = int(sys.argv[2]) if len(sys.argv) > 2 else 200000
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0

    # Shared pattern the espp subscriber verifies byte-exact: 'A' + (i % 26).
    pattern = "".join(chr(65 + (i % 26)) for i in range(size))

    rclpy.init()
    node = rclpy.create_node("bigpub")
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    pub = node.create_publisher(String, topic, qos)

    msg = String()
    msg.data = pattern
    deadline = time.time() + seconds
    while time.time() < deadline:
        pub.publish(msg)  # FastDDS fragments this 200 KB sample into DATA_FRAG
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
