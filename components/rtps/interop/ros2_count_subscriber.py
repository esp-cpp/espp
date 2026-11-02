#!/usr/bin/env python3
"""Count std_msgs/String samples from a ROS 2 subscriber for a fixed time.

Used by the interop harness as the ROS 2 side of the liveliness cases:
`ros2 topic echo --once` proves one sample arrived, this proves samples KEEP
arriving - samples received earlier than `count_after_s` seconds after start
are reported but not counted, so a peer that stops being served once its
participant lease has expired cannot pass.

Usage: ros2_count_subscriber.py TOPIC SECONDS REQUIRED COUNT_AFTER_S [reliable|best_effort]
Exits 0 iff at least REQUIRED samples arrived after COUNT_AFTER_S seconds.
"""

import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/chatter"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    required = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    count_after_s = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    reliable = (sys.argv[5] if len(sys.argv) > 5 else "reliable") != "best_effort"

    rclpy.init()
    node = rclpy.create_node("countsub")
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
    start = time.time()
    counts = {"early": 0, "counted": 0}

    def on_sample(_msg: String) -> None:
        if time.time() - start < count_after_s:
            counts["early"] += 1
        else:
            counts["counted"] += 1
            if counts["counted"] <= 3 or counts["counted"] % 25 == 0:
                print(f"received {counts['counted']} (t={time.time() - start:.1f}s)", flush=True)

    node.create_subscription(String, topic, on_sample, qos)
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = counts["counted"] >= required
    print(
        f"{'PASS' if ok else 'FAIL'} received={counts['counted']} required={required} "
        f"(not counted, before {count_after_s:.0f}s: {counts['early']})",
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
