#!/usr/bin/env python3
"""Standalone RTPS subscriber using the espp Python library (embeddedRTPS engine).

Subscribes to CDR string samples; defaults follow the ROS 2 conventions for
std_msgs/String on /chatter, so `ros2 topic pub /chatter std_msgs/msg/String ...`
(with rmw_fastrtps) is received. Pair it with rtps_publisher.py or any DDS peer.

Usage: python rtps_subscriber.py [topic] [type] [interface_ipv4] [run_seconds]
"""

import sys
import time

import espp


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "rt/chatter"
    type_name = sys.argv[2] if len(sys.argv) > 2 else "std_msgs::msg::dds_::String_"
    interface = sys.argv[3] if len(sys.argv) > 3 else ""  # "" -> auto-detect
    run_seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

    stats = {"received": 0}

    def on_sample(data: bytes) -> None:
        text = espp.CdrReader(data).read_string()
        stats["received"] += 1
        print(f"received {stats['received']}: {text!r}")

    R = espp.RtpsParticipant
    participant = R(R.Config(interface_address=interface, log_level=espp.Logger.Verbosity.info))
    if not participant.start():
        print("failed to start participant")
        return 1
    if not participant.add_reader(
        topic=topic, type_name=type_name, reliable=True, on_sample=on_sample
    ):
        print("failed to add reader")
        return 1
    print(f"subscribed to '{topic}' ({type_name}) for {run_seconds}s")

    try:
        time.sleep(run_seconds)
    except KeyboardInterrupt:
        pass
    participant.stop()
    print(f"done; received={stats['received']}")
    return stats["received"] == 0
    # exit 0 when samples arrived, 1 otherwise


if __name__ == "__main__":
    sys.exit(main())
