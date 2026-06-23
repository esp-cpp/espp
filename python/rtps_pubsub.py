#!/usr/bin/env python3
"""Self-contained RTPS test using the espp Python library.

Creates two participants (a publisher and a subscriber) in one process and exchanges
std_msgs/msg/UInt32 samples over best-effort CDR-over-RTPS, exercising SPDP/SEDP discovery and the
user-data path end to end. Exits 0 if the subscriber received samples, 1 otherwise.

Usage: python rtps_pubsub.py [advertised_ipv4] [run_seconds]
"""

import datetime
import socket
import sys
import time

from support_loader import espp


def guess_local_ipv4() -> str:
    # RTPS discovery is multicast, so a real interface address (not 127.0.0.1) is needed.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 53))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def serialize_uint32(value: int) -> bytes:
    writer = espp.CdrWriter()  # default little-endian CDR with encapsulation header
    writer.write_uint32(value)
    return bytes(writer.take_buffer())


def deserialize_uint32(data: bytes):
    return espp.CdrReader(data).read_uint32()  # int, or None on failure


def main() -> int:
    R = espp.RtpsParticipant
    address = sys.argv[1] if len(sys.argv) > 1 else guess_local_ipv4()
    run_seconds = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    topic = "espp/test/counter"
    print(f"advertising on {address} for {run_seconds}s, topic '{topic}'")

    stats = {"received": 0, "last": 0}

    # --- Subscriber ---
    sub_cfg = R.Config()
    sub_cfg.node_name = "py_pubsub_subscriber"
    sub_cfg.participant_id = 21
    sub_cfg.advertised_address = address
    sub_cfg.announce_period = datetime.timedelta(milliseconds=200)
    sub_cfg.log_level = espp.Logger.Verbosity.warn
    subscriber = R(sub_cfg)

    def on_sample(cdr: bytes):
        value = deserialize_uint32(cdr)
        if value is not None:
            stats["received"] += 1
            stats["last"] = value

    rc = R.ReaderConfig()
    rc.topic_name = topic
    rc.on_sample = on_sample
    subscriber.add_reader(rc)

    # --- Publisher ---
    pub_cfg = R.Config()
    pub_cfg.node_name = "py_pubsub_publisher"
    pub_cfg.participant_id = 20
    pub_cfg.advertised_address = address
    pub_cfg.announce_period = datetime.timedelta(milliseconds=200)
    pub_cfg.log_level = espp.Logger.Verbosity.warn
    publisher = R(pub_cfg)
    wc = R.WriterConfig()
    wc.topic_name = topic
    publisher.add_writer(wc)

    if not subscriber.start() or not publisher.start():
        print("Failed to start participants (is multicast networking available?)")
        return 1

    print("waiting for discovery...")
    for _ in range(50):
        if publisher.discovered_readers() and subscriber.discovered_writers():
            break
        time.sleep(0.1)
    print(f"discovered {len(publisher.discovered_readers())} remote reader(s), "
          f"{len(subscriber.discovered_writers())} remote writer(s)")

    sent = 0
    deadline = time.monotonic() + run_seconds
    while time.monotonic() < deadline:
        sent += 1
        if publisher.publish(topic, serialize_uint32(sent)):
            print(f"published {sent} -> received so far {stats['received']} (last={stats['last']})")
        time.sleep(0.5)

    publisher.stop()
    subscriber.stop()
    print(f"done: sent {sent}, received {stats['received']}, last value {stats['last']}")
    return 0 if stats["received"] else 1


if __name__ == "__main__":
    sys.exit(main())
