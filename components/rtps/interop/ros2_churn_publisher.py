#!/usr/bin/env python3
"""Create, use and destroy a std_msgs/String publisher in rounds.

Used by the interop harness as the ROS 2 side of the endpoint-churn case: a
peer that deletes a publisher (or leaves the domain) tells the others with an
SEDP (SPDP) dispose, and espp must drop the matched writer right then, not
when the peer's lease would have expired. Each round creates a publisher on
TOPIC, publishes at 10 Hz for --on seconds, destroys it (rclpy sends the
dispose) and waits --off seconds. The last round's publisher is destroyed
before the node and the context go down, so the final dispose is an endpoint
dispose as well; the participant dispose follows from rclpy.shutdown().

Usage: ros2_churn_publisher.py TOPIC [--rounds N] [--on S] [--off S]
Exits 0 after the rounds, 2 on bad arguments.
"""

import argparse
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def positive(text: str) -> float:
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {text}")
    return value


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create, publish on and destroy a publisher in rounds (endpoint churn)."
    )
    parser.add_argument("topic", nargs="?", default="/churn")
    parser.add_argument("--rounds", type=int, default=5, help="publisher create/destroy rounds")
    parser.add_argument("--on", type=positive, default=3.0, help="seconds each publisher lives")
    parser.add_argument("--off", type=positive, default=2.0, help="seconds between publishers")
    args = parser.parse_args(argv)
    if args.rounds <= 0:
        parser.error(f"--rounds must be > 0, got {args.rounds}")
    return args


def main() -> int:
    args = parse_args(sys.argv[1:])

    rclpy.init()
    node = rclpy.create_node("churnpub")
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    start = time.time()
    for round_no in range(1, args.rounds + 1):
        pub = node.create_publisher(String, args.topic, qos)
        print(f"round {round_no}: publisher created (t={time.time() - start:.1f}s)", flush=True)
        deadline = time.time() + args.on
        msg = String()
        count = 0
        while time.time() < deadline:
            msg.data = f"round {round_no} sample {count}"
            pub.publish(msg)
            count += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
        node.destroy_publisher(pub)
        print(
            f"round {round_no}: publisher destroyed after {count} samples "
            f"(t={time.time() - start:.1f}s)",
            flush=True,
        )
        if round_no < args.rounds:
            time.sleep(args.off)

    node.destroy_node()
    rclpy.shutdown()
    print(f"done: {args.rounds} rounds (t={time.time() - start:.1f}s)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
