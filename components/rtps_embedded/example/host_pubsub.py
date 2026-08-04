"""
PC-side pub/sub counterpart for the rtps_embedded example firmware.

The firmware runs on an ESP32-Ethernet-Kit acting as a DHCP server at
192.168.4.1.  Connect the host PC to the same Ethernet segment so it
obtains an address in the 192.168.4.0/24 subnet.

Roles mirror the firmware:
  initiator  -- publishes <prefix>/request, subscribes <prefix>/response
  responder  -- publishes <prefix>/response, subscribes <prefix>/request

Usage:
  python host_pubsub.py                          # initiator (default)
  python host_pubsub.py --role responder
  python host_pubsub.py --prefix espp/rtps_example --domain 0

Requirements:
  pip install fastdds          # eProsima Fast DDS Python bindings
"""

from __future__ import annotations

import argparse
import signal
import time

try:
    import fastdds
except ImportError as exc:
    raise SystemExit(
        "Fast DDS Python bindings not found.\n"
        "Install the package that provides the fastdds module and retry."
    ) from exc

# ---------------------------------------------------------------------------
# Custom DDS type: raw null-terminated string, name matches the firmware.
# embeddedRTPS puts the user-supplied bytes directly into the DATA submessage
# payload with no CDR encapsulation prefix, so we do the same here.
# ---------------------------------------------------------------------------

_TYPE_NAME = "std_msgs::msg::String"


class RawStringType(fastdds.TopicDataType):
    def __init__(self) -> None:
        super().__init__()
        self.set_name(_TYPE_NAME)

    def get_name(self) -> str:
        return _TYPE_NAME

    def create_data(self) -> bytearray:
        return bytearray()

    def delete_data(self, data) -> None:
        return None

    def is_bounded(self) -> bool:
        return False

    def is_plain(self, data_representation) -> bool:
        return False

    def get_max_serialized_size_ctx(self, context) -> int:
        return 0

    def calculate_serialized_size(self, data, data_representation) -> int:
        return len(data) + 1  # +1 for null terminator

    def serialize(self, data: bytearray, payload, data_representation) -> bool:
        """Write raw bytes followed by a null terminator — matches embedded side."""
        if isinstance(data, str):
            data = data.encode()
        raw = bytes(data).rstrip(b"\x00") + b"\x00"
        return self._write_payload(payload, raw)

    def deserialize(self, payload, data: bytearray) -> bool:
        """Read raw bytes and strip the null terminator."""
        raw = self._read_payload(payload)
        text = raw.rstrip(b"\x00")
        data.clear()
        data.extend(text)
        return True

    # --- helpers -----------------------------------------------------------

    @staticmethod
    def _write_payload(payload, raw: bytes) -> bool:
        if hasattr(payload, "data"):
            try:
                payload.data[: len(raw)] = raw
            except Exception:
                payload.data = bytearray(raw)
        else:
            payload.data = bytearray(raw)
        if hasattr(payload, "length"):
            payload.length = len(raw)
        return True

    @staticmethod
    def _read_payload(payload) -> bytes:
        length = getattr(payload, "length", 0)
        buf = getattr(payload, "data", None)
        if buf is None:
            return b""
        try:
            return bytes(buf[:length])
        except Exception:
            return bytes(buf)


# ---------------------------------------------------------------------------
# Listener that prints received messages
# ---------------------------------------------------------------------------

class StringListener(fastdds.DataReaderListener):
    def __init__(self, label: str) -> None:
        super().__init__()
        self._label = label
        self._type = RawStringType()

    def on_data_available(self, reader) -> None:
        data = self._type.create_data()
        info = fastdds.SampleInfo()
        ret = reader.take_next_sample(data, info)
        ok = getattr(fastdds, "RETCODE_OK", 0)
        if ret == ok and info.valid_data:
            text = bytes(data).decode(errors="replace")
            print(f"[rx {self._label}] {text}")


# ---------------------------------------------------------------------------
# DDS participant / topic / reader / writer helpers
# ---------------------------------------------------------------------------

RETCODE_OK = getattr(fastdds, "RETCODE_OK", 0)


def _create_participant(domain_id: int, interface_ip: str | None = None) -> fastdds.DomainParticipant:
    factory = fastdds.DomainParticipantFactory.get_instance()
    qos = factory.get_default_participant_qos()

    if interface_ip:
        transport = fastdds.UDPv4TransportDescriptor()
        transport.interfaceWhiteList = [interface_ip]
        transport.sendBufferSize = 65536
        transport.receiveBufferSize = 65536
        qos.transport().use_builtin_transports = False
        qos.transport().user_transports = [transport]

    # Explicitly advertise SPDP discovery on the RTPS multicast group.
    peer = fastdds.Locator()
    peer.kind = fastdds.LOCATOR_KIND_UDPv4
    peer.address = "239.255.0.1"
    peer.port = 7400
    qos.wire_protocol().builtin.initialPeersList = [peer]

    participant = factory.create_participant(domain_id, qos)
    if participant is None:
        raise RuntimeError("Failed to create DDS participant")
    return participant


def _register_type(participant, data_type: RawStringType) -> None:
    ret = participant.register_type(data_type)
    if ret != RETCODE_OK:
        raise RuntimeError(f"Failed to register type: {ret}")


def _create_topic(participant, topic_name: str, type_name: str):
    qos = participant.get_default_topic_qos()
    topic = participant.create_topic(topic_name, type_name, qos)
    if topic is None:
        raise RuntimeError(f"Failed to create topic '{topic_name}'")
    return topic


def _create_writer(participant, topic):
    pub_qos = participant.get_default_publisher_qos()
    publisher = participant.create_publisher(pub_qos)
    if publisher is None:
        raise RuntimeError("Failed to create publisher")

    dw_qos = publisher.get_default_datawriter_qos()
    writer = publisher.create_datawriter(topic, dw_qos)
    if writer is None:
        raise RuntimeError("Failed to create data writer")
    return publisher, writer


def _create_reader(participant, topic, listener):
    sub_qos = participant.get_default_subscriber_qos()
    subscriber = participant.create_subscriber(sub_qos)
    if subscriber is None:
        raise RuntimeError("Failed to create subscriber")

    dr_qos = subscriber.get_default_datareader_qos()
    reader = subscriber.create_datareader(topic, dr_qos, listener)
    if reader is None:
        raise RuntimeError("Failed to create data reader")
    return subscriber, reader


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(role: str, prefix: str, domain_id: int, period_ms: int, interface_ip: str | None = None) -> None:
    if role == "initiator":
        pub_topic_name = f"{prefix}/request"
        sub_topic_name = f"{prefix}/response"
    else:
        pub_topic_name = f"{prefix}/response"
        sub_topic_name = f"{prefix}/request"

    print(f"Role      : {role}")
    print(f"Domain    : {domain_id}")
    print(f"Publish   : {pub_topic_name}")
    print(f"Subscribe : {sub_topic_name}")
    print()

    participant = _create_participant(domain_id, interface_ip)
    data_type = RawStringType()
    _register_type(participant, data_type)

    pub_topic = _create_topic(participant, pub_topic_name, _TYPE_NAME)
    sub_topic = _create_topic(participant, sub_topic_name, _TYPE_NAME)

    listener = StringListener(sub_topic_name)
    publisher, writer = _create_writer(participant, pub_topic)
    subscriber, reader = _create_reader(participant, sub_topic, listener)

    # Allow discovery to complete before the first write.
    print("Waiting for discovery (2 s)...")
    time.sleep(2.0)

    stop = False

    def _sigint(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)

    counter = 0
    print("Running — Ctrl-C to stop\n")
    try:
        while not stop:
            if role == "initiator":
                text = f"request {counter}"
            else:
                text = f"response {counter}"

            sample = bytearray(text.encode())
            ret = writer.write(sample)
            if ret == RETCODE_OK:
                print(f"[tx {pub_topic_name}] {text}")
            else:
                print(f"[tx ERROR {ret}] {text}")

            counter += 1
            time.sleep(period_ms / 1000.0)
    finally:
        participant.delete_contained_entities()
        fastdds.DomainParticipantFactory.get_instance().delete_participant(participant)
        print("\nShutdown complete.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PC pub/sub peer for the rtps_embedded ESP32 example"
    )
    parser.add_argument(
        "--role",
        choices=["initiator", "responder"],
        default="initiator",
        help="Mirror the firmware role (default: initiator)",
    )
    parser.add_argument(
        "--prefix",
        default="espp/rtps_example",
        help="Topic prefix used in the firmware (default: espp/rtps_example)",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="RTPS domain ID, must match CONFIG_RTPS_EXAMPLE_DOMAIN_ID (default: 0)",
    )
    parser.add_argument(
        "--period-ms",
        type=int,
        default=2000,
        help="Publish interval in milliseconds (default: 2000)",
    )
    parser.add_argument(
        "--interface-ip",
        default=None,
        help="Optional local interface IP to constrain Fast DDS to (for example 192.168.4.2)",
    )
    args = parser.parse_args()
    run(args.role, args.prefix, args.domain, args.period_ms, args.interface_ip)


if __name__ == "__main__":
    main()
