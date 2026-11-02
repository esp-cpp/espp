#!/usr/bin/env python3
"""Count std_msgs/String samples from a ROS 2 subscriber for a fixed time.

Used by the interop harness as the ROS 2 side of the liveliness cases:
`ros2 topic echo --once` proves one sample arrived, this proves samples KEEP
arriving - samples received earlier than `count_after_s` seconds after start
are reported but not counted, so a peer that stops being served once its
participant lease has expired cannot pass.

Usage: ros2_count_subscriber.py TOPIC [--seconds S] [--required N] [--count-after S]
       [--reliability reliable|best_effort]
Exits 0 iff at least REQUIRED samples arrived after COUNT_AFTER seconds, 2 on
bad arguments.
"""

import argparse
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def non_negative(text: str) -> float:
    value = float(text)
    if value < 0:
        raise argparse.ArgumentTypeError(f"must be >= 0, got {text}")
    return value


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Count std_msgs/String samples on a topic for a fixed time; "
        "samples before --count-after are reported but not counted."
    )
    parser.add_argument("topic", nargs="?", default="/chatter")
    parser.add_argument("--seconds", type=non_negative, default=25.0, help="how long to listen")
    parser.add_argument("--required", type=int, default=10, help="counted samples needed to pass")
    parser.add_argument(
        "--count-after", type=non_negative, default=0.0, help="seconds before counting starts"
    )
    parser.add_argument(
        "--reliability", choices=("reliable", "best_effort"), default="reliable"
    )
    args = parser.parse_args(argv)
    if args.required < 0:
        parser.error(f"--required must be >= 0, got {args.required}")
    return args


def main() -> int:
    args = parse_args(sys.argv[1:])
    topic = args.topic
    seconds = args.seconds
    required = args.required
    count_after_s = args.count_after
    reliable = args.reliability == "reliable"

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
