#!/usr/bin/env python3
"""Standalone RTPS subscriber using the espp Python library (embeddedRTPS engine).

Subscribes to the complex nested message published by rtps_publisher.py (or any
DDS/ROS 2 peer publishing espp_examples/msg/SensorSample), using pycdr2 for CDR
deserialization and the light espp.rtps.Subscriber wrapper -- the callback
receives fully-decoded message objects, not raw bytes.

Requires pycdr2 (see python/requirements.txt).

Usage: python rtps_subscriber.py [topic] [interface_ipv4] [run_seconds]
"""

import sys
import time

import espp

from rtps_messages import SensorSample


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "rt/imu"
    interface = sys.argv[2] if len(sys.argv) > 2 else ""  # "" -> auto-detect
    run_seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

    stats = {"received": 0}

    def on_message(msg) -> None:  # msg is a fully-decoded SensorSample
        stats["received"] += 1
        print(
            f"received {stats['received']}: seq={msg.seq} frame={msg.frame_id!r} "
            f"wz={msg.angular_velocity.z:.3f} samples={list(msg.samples)}"
        )

    R = espp.RtpsParticipant
    participant = R(R.Config(interface_address=interface, log_level=espp.Logger.Verbosity.info))
    if not participant.start():
        print("failed to start participant")
        return 1
    sub = espp.rtps.Subscriber(participant, topic, SensorSample, on_message=on_message, reliable=True)
    if not sub.valid:
        print("failed to add reader")
        return 1
    print(f"subscribed to '{topic}' (espp_examples/msg/SensorSample) for {run_seconds}s")

    try:
        time.sleep(run_seconds)
    except KeyboardInterrupt:
        pass
    participant.stop()
    print(f"done; received={stats['received']}")
    return 0 if stats["received"] else 1


if __name__ == "__main__":
    sys.exit(main())
