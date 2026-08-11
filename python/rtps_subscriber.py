#!/usr/bin/env python3
"""Standalone RTPS subscriber using the espp Python library.

Announces a reader and prints received std_msgs/msg/UInt32 samples. Pair it with rtps_publisher.py,
the C++ rtps_publisher, or rtps_host.py.

Usage: python rtps_subscriber.py [topic] [advertised_ipv4]
"""

import datetime
import socket
import struct
import sys
import time

import espp


def guess_local_ipv4() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 53))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def deserialize_uint32(data: bytes):
    # Accepts either endianness; the encapsulation identifier's low bit selects little-endian.
    if len(data) < 8 or data[0] != 0x00 or data[1] not in (0x00, 0x01):
        return None  # int, or None on failure
    endian = "<" if data[1] & 1 else ">"
    return struct.unpack_from(endian + "I", data, 4)[0]


def main() -> int:
    R = espp.RtpsParticipant
    topic = sys.argv[1] if len(sys.argv) > 1 else "espp/test/counter"
    address = sys.argv[2] if len(sys.argv) > 2 else guess_local_ipv4()

    count = {"n": 0}

    cfg = R.Config()
    cfg.node_name = "py_subscriber"
    cfg.participant_id = 22
    cfg.advertised_address = address
    cfg.announce_period = datetime.timedelta(milliseconds=500)
    cfg.on_participant_discovered = lambda p: print(
        f"discovered participant '{p.name}' at {p.address}")
    participant = R(cfg)

    def on_sample(cdr: bytes):
        value = deserialize_uint32(cdr)
        if value is not None:
            count["n"] += 1
            print(f"received {value} (#{count['n']})")

    rc = R.ReaderConfig()
    rc.topic_name = topic
    rc.on_sample = on_sample
    participant.add_reader(rc)

    if not participant.start():
        print("Failed to start participant (is multicast networking available?)")
        return 1
    print(f"subscribed to '{topic}' on {address} (Ctrl-C to stop)")

    try:
        while True:
            time.sleep(5)
            print(f"status: {count['n']} samples received, "
                  f"{len(participant.discovered_writers())} known publisher(s)")
    except KeyboardInterrupt:
        participant.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
