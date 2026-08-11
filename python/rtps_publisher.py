#!/usr/bin/env python3
"""Standalone RTPS publisher using the espp Python library.

Announces a writer and periodically publishes std_msgs/msg/UInt32 samples. Pair it with
rtps_subscriber.py, the C++ rtps_subscriber, or rtps_host.py.

Usage: python rtps_publisher.py [topic] [advertised_ipv4] [period_seconds]
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


def serialize_uint32(value: int) -> bytes:
    # Little-endian CDR (XCDR1) with a 4-byte encapsulation header — the wire format of
    # std_msgs/msg/UInt32. Plain struct.pack; no native bindings needed for CDR payloads.
    return b"\x00\x01\x00\x00" + struct.pack("<I", value)


def main() -> int:
    R = espp.RtpsParticipant
    topic = sys.argv[1] if len(sys.argv) > 1 else "espp/test/counter"
    address = sys.argv[2] if len(sys.argv) > 2 else guess_local_ipv4()
    period = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    cfg = R.Config()
    cfg.node_name = "py_publisher"
    cfg.participant_id = 20
    cfg.advertised_address = address
    cfg.announce_period = datetime.timedelta(milliseconds=500)
    cfg.on_endpoint_discovered = lambda e: print(
        f"discovered {'reader' if e.is_reader else 'writer'} '{e.topic_name}'")
    participant = R(cfg)
    wc = R.WriterConfig()
    wc.topic_name = topic
    participant.add_writer(wc)

    if not participant.start():
        print("Failed to start participant (is multicast networking available?)")
        return 1
    print(f"publishing on '{topic}' from {address} every {period}s (Ctrl-C to stop)")

    value = 0
    try:
        while True:
            value += 1
            sent = participant.publish(topic, serialize_uint32(value))
            print(f"publish {value} -> {'sent' if sent else 'no destinations yet'}")
            time.sleep(period)
    except KeyboardInterrupt:
        participant.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
