#!/usr/bin/env python3
"""Standalone RTPS publisher using the espp Python library (embeddedRTPS engine).

Publishes CDR string samples on a topic; defaults follow the ROS 2 conventions for
std_msgs/String on /chatter, so `ros2 topic echo /chatter std_msgs/msg/String`
(with rmw_fastrtps) receives them. Pair it with rtps_subscriber.py or any DDS peer.

Usage: python rtps_publisher.py [topic] [type] [interface_ipv4] [period_seconds]
"""

import sys
import time

import espp


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "rt/chatter"
    type_name = sys.argv[2] if len(sys.argv) > 2 else "std_msgs::msg::dds_::String_"
    interface = sys.argv[3] if len(sys.argv) > 3 else ""  # "" -> auto-detect
    period = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5

    R = espp.RtpsParticipant
    participant = R(R.Config(interface_address=interface, log_level=espp.Logger.Verbosity.info))
    if not participant.start():
        print("failed to start participant")
        return 1
    if not participant.add_writer(topic=topic, type_name=type_name, reliable=True):
        print("failed to add writer")
        return 1
    print(f"publishing '{topic}' ({type_name}) every {period}s; ctrl-c to stop")

    count = 0
    try:
        while True:
            writer = espp.CdrWriter()  # little-endian CDR with encapsulation header
            writer.write_string(f"espp python {count}")
            if participant.publish(topic, bytes(writer.take_buffer())):
                count += 1
                print(f"sent {count}")
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        participant.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
