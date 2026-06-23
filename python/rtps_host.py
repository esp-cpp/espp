#!/usr/bin/env python3
"""Simple host-side RTPS test harness for the ESPP RTPS component.

This script speaks the ESPP RTPS discovery wire format plus the standard
CDR-over-RTPS user-data path used by ``RtpsParticipant``. It is useful for:

1. discovering an embedded ESPP RTPS participant from a PC/host,
2. inspecting SPDP/SEDP announcements, and
3. sending or receiving ``std_msgs/msg/UInt32``-style test samples. User samples
   are standard RTPS ``DATA`` submessages whose serializedPayload is the raw
   CDR-encapsulated sample; the topic is identified by the writer GUID resolved
   through SEDP discovery (no ESPP-specific payload framing).

Run ``python rtps_host.py --self-test`` to validate the wire-format encoders and
decoders against the firmware's expectations without any network I/O.

It uses only the Python standard library, so it does not require Python
bindings or a rebuilt host ``lib/`` tree.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple


RTPS_MAGIC = b"RTPS"
PL_CDR_LE = b"\x00\x03\x00\x00"

PORT_BASE = 7400
DOMAIN_GAIN = 250
PARTICIPANT_GAIN = 2
METATRAFFIC_MULTICAST_OFFSET = 0
METATRAFFIC_UNICAST_OFFSET = 10
USER_MULTICAST_OFFSET = 1
USER_UNICAST_OFFSET = 11

DATA_SUBMESSAGE_KIND = 0x15
DATA_SUBMESSAGE_FLAGS = 0x01 | 0x04
DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS = 16

RTPS_QOS_RELIABILITY_BEST_EFFORT = 1
RTPS_QOS_RELIABILITY_RELIABLE = 2

KIND_UDP_V4 = 1
VENDOR_ID = b"\xca\xfe"

ENTITY_ID_UNKNOWN = b"\x00\x00\x00\x00"
PARTICIPANT_ENTITY_ID = b"\x00\x00\x01\xc1"
SPDP_WRITER_ENTITY_ID = b"\x00\x01\x00\xc2"
SPDP_READER_ENTITY_ID = b"\x00\x01\x00\xc7"
SEDP_PUBLICATIONS_WRITER_ENTITY_ID = b"\x00\x00\x03\xc2"
SEDP_PUBLICATIONS_READER_ENTITY_ID = b"\x00\x00\x03\xc7"
SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID = b"\x00\x00\x04\xc2"
SEDP_SUBSCRIPTIONS_READER_ENTITY_ID = b"\x00\x00\x04\xc7"
USER_WRITER_NO_KEY_KIND = 0x03
USER_READER_NO_KEY_KIND = 0x04

BUILTIN_ENDPOINT_SET = (
    (1 << 0)
    | (1 << 1)
    | (1 << 2)
    | (1 << 3)
    | (1 << 4)
    | (1 << 5)
    | (1 << 10)
    | (1 << 11)
)

PID_SENTINEL = 0x0001
PID_PARTICIPANT_LEASE_DURATION = 0x0002
PID_TOPIC_NAME = 0x0005
PID_TYPE_NAME = 0x0007
PID_DOMAIN_ID = 0x000F
PID_PROTOCOL_VERSION = 0x0015
PID_VENDORID = 0x0016
PID_RELIABILITY = 0x001A
PID_LIVELINESS = 0x001B
PID_DURABILITY = 0x001D
PID_USER_DATA = 0x002C
PID_MULTICAST_LOCATOR = 0x0030
PID_UNICAST_LOCATOR = 0x002F
PID_DEFAULT_UNICAST_LOCATOR = 0x0031
PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032
PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033
PID_HISTORY = 0x0040
PID_EXPECTS_INLINE_QOS = 0x0043
PID_DEFAULT_MULTICAST_LOCATOR = 0x0048
PID_PARTICIPANT_GUID = 0x0050
PID_BUILTIN_ENDPOINT_SET = 0x0058
PID_ENDPOINT_GUID = 0x005A
PID_TYPE_MAX_SIZE_SERIALIZED = 0x0060
PID_ENTITY_NAME = 0x0062
PID_KEY_HASH = 0x0070

DEFAULT_LEASE_DURATION_SECONDS = 20
DEFAULT_LEASE_DURATION_NANOSECONDS = 0
DEFAULT_MAX_BLOCKING_SECONDS = 0
DEFAULT_MAX_BLOCKING_NANOSECONDS = 100_000_000
DEFAULT_TOPIC_PREFIX = "espp/rtps_example"
DEFAULT_REQUEST_TOPIC = f"{DEFAULT_TOPIC_PREFIX}/request"
DEFAULT_RESPONSE_TOPIC = f"{DEFAULT_TOPIC_PREFIX}/response"


@dataclass
class PortMapping:
    metatraffic_multicast: int
    metatraffic_unicast: int
    user_multicast: int
    user_unicast: int


@dataclass
class ParticipantProxy:
    participant_guid: bytes
    guid_prefix: bytes
    name: str
    enclave: str
    address: str
    ports: PortMapping
    builtin_endpoints: int


@dataclass
class EndpointProxy:
    guid: bytes
    participant_guid: bytes
    topic_name: str
    type_name: str
    reliability: str
    is_reader: bool
    expects_inline_qos: bool
    unicast_address: str
    unicast_port: int
    multicast_locators: List[Tuple[str, int]]


@dataclass
class WriterConfig:
    topic_name: str
    type_name: str
    reliable: bool
    entity_index: int


@dataclass
class ReaderConfig:
    topic_name: str
    type_name: str
    reliable: bool
    entity_index: int


def log(message: str) -> None:
    print(message, flush=True)


def hex_string(value: bytes) -> str:
    return value.hex()


def guid_to_string(guid: bytes) -> str:
    return hex_string(guid[:12]) + ":" + hex_string(guid[12:])


def entity_id_for_index(entity_index: int, kind: int) -> bytes:
    return bytes((0x00, 0x00, 0x10 + entity_index, kind))


def reliability_to_name(reliable: bool) -> str:
    return "reliable" if reliable else "best-effort"


def ntp_fraction_from_nanoseconds(nanoseconds: int) -> int:
    # RTPS Duration_t/Time_t use NTP fraction units of 1/2^32 s, not nanoseconds.
    return (nanoseconds << 32) // 1_000_000_000


def padded_parameter_length(length: int) -> int:
    # RTPS PL_CDR requires each parameterLength to be a multiple of 4.
    return (length + 3) & ~3


def compute_port_mapping(domain_id: int, participant_id: int) -> PortMapping:
    base = PORT_BASE + DOMAIN_GAIN * domain_id
    participant_offset = PARTICIPANT_GAIN * participant_id
    return PortMapping(
        metatraffic_multicast=base + METATRAFFIC_MULTICAST_OFFSET,
        metatraffic_unicast=base + METATRAFFIC_UNICAST_OFFSET + participant_offset,
        user_multicast=base + USER_MULTICAST_OFFSET,
        user_unicast=base + USER_UNICAST_OFFSET + participant_offset,
    )


def guess_local_ipv4() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def make_guid_prefix(node_name: str, domain_id: int, participant_id: int) -> bytes:
    digest = hashlib.sha256(node_name.encode("utf-8")).digest()
    return bytes(
        (
            participant_id & 0xFF,
            (participant_id >> 8) & 0xFF,
            domain_id & 0xFF,
            (domain_id >> 8) & 0xFF,
        )
    ) + digest[:8]


def make_guid(prefix: bytes, entity_id: bytes) -> bytes:
    return prefix + entity_id


def align4(buffer: bytearray) -> None:
    while len(buffer) % 4 != 0:
        buffer.append(0)


def append_parameter_header(buffer: bytearray, pid: int, length: int) -> None:
    buffer.extend(struct.pack("<HH", pid, length))


def append_parameter_guid(buffer: bytearray, pid: int, guid: bytes) -> None:
    append_parameter_header(buffer, pid, 16)
    buffer.extend(guid)


def append_parameter_protocol_version(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_PROTOCOL_VERSION, 4)
    buffer.extend((2, 3, 0, 0))


def append_parameter_vendor_id(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_VENDORID, 4)
    buffer.extend(VENDOR_ID)
    buffer.extend((0, 0))


def append_parameter_u32(buffer: bytearray, pid: int, value: int) -> None:
    append_parameter_header(buffer, pid, 4)
    buffer.extend(struct.pack("<I", value))


def append_parameter_bool(buffer: bytearray, pid: int, value: bool) -> None:
    append_parameter_header(buffer, pid, 4)
    buffer.extend((1 if value else 0, 0, 0, 0))


def append_parameter_duration(buffer: bytearray, pid: int, seconds: int, nanoseconds: int) -> None:
    append_parameter_header(buffer, pid, 8)
    buffer.extend(struct.pack("<iI", seconds, ntp_fraction_from_nanoseconds(nanoseconds)))


def locator_bytes(ip_address: str, port: int) -> bytes:
    # Locator_t.kind/.port are little-endian in PL_CDR_LE; only the 16-byte address is raw bytes.
    locator = bytearray(24)
    struct.pack_into("<I", locator, 0, KIND_UDP_V4)
    struct.pack_into("<I", locator, 4, port)
    locator[20:24] = socket.inet_aton(ip_address)
    return bytes(locator)


def append_parameter_locator(buffer: bytearray, pid: int, ip_address: str, port: int) -> None:
    append_parameter_header(buffer, pid, 24)
    buffer.extend(locator_bytes(ip_address, port))


def append_parameter_string_cdr(buffer: bytearray, pid: int, text: str) -> None:
    encoded = text.encode("utf-8")
    # parameterLength must be a multiple of 4 and includes the trailing CDR padding.
    append_parameter_header(buffer, pid, padded_parameter_length(4 + len(encoded) + 1))
    buffer.extend(struct.pack("<I", len(encoded) + 1))
    buffer.extend(encoded)
    buffer.append(0)
    align4(buffer)


def append_parameter_octet_sequence(buffer: bytearray, pid: int, payload: bytes) -> None:
    append_parameter_header(buffer, pid, padded_parameter_length(4 + len(payload)))
    buffer.extend(struct.pack("<I", len(payload)))
    buffer.extend(payload)
    align4(buffer)


def append_parameter_reliability(buffer: bytearray, reliable: bool) -> None:
    append_parameter_header(buffer, PID_RELIABILITY, 12)
    kind = RTPS_QOS_RELIABILITY_RELIABLE if reliable else RTPS_QOS_RELIABILITY_BEST_EFFORT
    buffer.extend(struct.pack("<I", kind))
    buffer.extend(
        struct.pack(
            "<iI",
            DEFAULT_MAX_BLOCKING_SECONDS,
            ntp_fraction_from_nanoseconds(DEFAULT_MAX_BLOCKING_NANOSECONDS),
        )
    )


def append_parameter_durability(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_DURABILITY, 4)
    buffer.extend(struct.pack("<I", 0))


def append_parameter_liveliness(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_LIVELINESS, 12)
    buffer.extend(struct.pack("<I", 0))
    buffer.extend(
        struct.pack(
            "<iI",
            DEFAULT_LEASE_DURATION_SECONDS,
            ntp_fraction_from_nanoseconds(DEFAULT_LEASE_DURATION_NANOSECONDS),
        )
    )


def append_parameter_history(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_HISTORY, 8)
    buffer.extend(struct.pack("<II", 0, 1))


def append_parameter_key_hash(buffer: bytearray, guid: bytes) -> None:
    append_parameter_header(buffer, PID_KEY_HASH, 16)
    buffer.extend(guid)


def append_parameter_sentinel(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_SENTINEL, 0)


def build_parameter_list_payload(parameter_buffer: bytearray) -> bytes:
    return PL_CDR_LE + bytes(parameter_buffer)


def build_data_submessage(reader_id: bytes, writer_id: bytes, sequence_number: int, payload: bytes) -> bytes:
    high = sequence_number >> 32
    low = sequence_number & 0xFFFFFFFF
    submessage_payload = bytearray()
    submessage_payload.extend(struct.pack("<HH", 0, DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS))
    submessage_payload.extend(reader_id)
    submessage_payload.extend(writer_id)
    submessage_payload.extend(struct.pack("<iI", high, low))
    submessage_payload.extend(payload)
    align4(submessage_payload)
    return (
        struct.pack("<BBH", DATA_SUBMESSAGE_KIND, DATA_SUBMESSAGE_FLAGS, len(submessage_payload))
        + submessage_payload
    )


def build_rtps_message(guid_prefix: bytes, reader_id: bytes, writer_id: bytes, sequence_number: int, payload: bytes) -> bytes:
    header = RTPS_MAGIC + bytes((2, 3)) + VENDOR_ID + guid_prefix
    return header + build_data_submessage(reader_id, writer_id, sequence_number, payload)


def parse_parameter_list(payload: bytes) -> List[tuple[int, bytes]]:
    if len(payload) < 4 or payload[:4] != PL_CDR_LE:
        return []
    parameters: List[tuple[int, bytes]] = []
    offset = 4
    while offset + 4 <= len(payload):
        pid, length = struct.unpack_from("<HH", payload, offset)
        offset += 4
        if pid == PID_SENTINEL:
            break
        if offset + length > len(payload):
            return []
        value = payload[offset : offset + length]
        parameters.append((pid, value))
        offset += length
        offset += (4 - (length % 4)) & 0x3
    return parameters


def find_parameter(parameters: Iterable[tuple[int, bytes]], pid: int) -> Optional[bytes]:
    for candidate_pid, candidate_value in parameters:
        if candidate_pid == pid:
            return candidate_value
    return None


def find_parameters(parameters: Iterable[tuple[int, bytes]], pid: int) -> List[bytes]:
    return [candidate_value for candidate_pid, candidate_value in parameters if candidate_pid == pid]


def parse_guid(value: Optional[bytes]) -> Optional[bytes]:
    if value is None or len(value) != 16:
        return None
    return value


def parse_u32_le(value: Optional[bytes]) -> Optional[int]:
    if value is None or len(value) < 4:
        return None
    return struct.unpack_from("<I", value, 0)[0]


def parse_bool(value: Optional[bytes]) -> Optional[bool]:
    if value is None or not value:
        return None
    return value[0] != 0


def parse_cdr_string(value: Optional[bytes]) -> Optional[str]:
    if value is None or len(value) < 4:
        return None
    length = struct.unpack_from("<I", value, 0)[0]
    if length == 0 or 4 + length > len(value):
        return None
    raw = value[4 : 4 + length]
    if raw.endswith(b"\x00"):
        raw = raw[:-1]
    return raw.decode("utf-8", errors="replace")


def parse_octet_sequence(value: Optional[bytes]) -> Optional[bytes]:
    if value is None or len(value) < 4:
        return None
    length = struct.unpack_from("<I", value, 0)[0]
    if 4 + length > len(value):
        return None
    return value[4 : 4 + length]


def parse_locator(value: Optional[bytes]) -> tuple[str, int]:
    if value is None or len(value) != 24:
        return ("0.0.0.0", 0)
    kind = struct.unpack_from("<I", value, 0)[0]
    if kind != KIND_UDP_V4:
        return ("0.0.0.0", 0)
    port = struct.unpack_from("<I", value, 4)[0]
    ip_address = socket.inet_ntoa(value[20:24])
    return (ip_address, port)


def parse_reliability(value: Optional[bytes]) -> str:
    kind = parse_u32_le(value)
    return "reliable" if kind == RTPS_QOS_RELIABILITY_RELIABLE else "best-effort"


def extract_enclave(value: Optional[bytes]) -> str:
    if not value:
        return "/"
    text = value.decode("utf-8", errors="replace")
    marker = "enclave="
    start = text.find(marker)
    if start < 0:
        return "/"
    start += len(marker)
    end = text.find(";", start)
    if end < 0:
        end = len(text)
    return text[start:end] or "/"


def serialize_uint32_cdr(value: int) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<I", value & 0xFFFFFFFF)


def deserialize_uint32_cdr(payload: bytes) -> Optional[int]:
    if len(payload) < 8 or payload[:2] != b"\x00\x01":
        return None
    return struct.unpack_from("<I", payload, 4)[0]


class RtpsHostHarness:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ports = compute_port_mapping(args.domain_id, args.participant_id)
        self.guid_prefix = make_guid_prefix(args.node_name, args.domain_id, args.participant_id)
        self.participant_guid = make_guid(self.guid_prefix, PARTICIPANT_ENTITY_ID)
        self.sequence_numbers: Dict[bytes, int] = {}
        self.discovered_participants: Dict[bytes, ParticipantProxy] = {}
        self.discovered_writers: Dict[bytes, EndpointProxy] = {}
        self.discovered_readers: Dict[bytes, EndpointProxy] = {}
        self.joined_user_multicast_groups: Set[str] = set()

        self.local_writers = [
            WriterConfig(
                topic_name=args.publish_topic,
                type_name=args.type_name,
                reliable=args.reliable,
                entity_index=0,
            )
        ] if args.publish_topic else []
        self.local_readers = [
            ReaderConfig(
                topic_name=topic_name,
                type_name=args.type_name,
                reliable=False,
                entity_index=index,
            )
            for index, topic_name in enumerate(args.subscribe_topic)
        ]

        self.metatraffic_multicast_sock = self._create_metatraffic_multicast_socket()
        self.metatraffic_unicast_sock = self._create_bound_udp_socket(self.ports.metatraffic_unicast)
        self.user_unicast_sock = self._create_bound_udp_socket(self.ports.user_unicast)
        self.user_multicast_sock = self._create_user_multicast_socket()
        self._configure_multicast_sender(self.metatraffic_unicast_sock)
        self._configure_multicast_sender(self.user_unicast_sock)

        self.next_discovery_send = 0.0
        self.next_publish_send = 0.0
        self.last_no_participant_log = 0.0
        self.last_unknown_writer_log = 0.0

    def _create_bound_udp_socket(self, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        sock.bind((self.args.bind_address, port))
        sock.setblocking(False)
        return sock

    def _create_metatraffic_multicast_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        try:
            sock.bind((self.args.multicast_group, self.ports.metatraffic_multicast))
        except OSError:
            # Not all platforms allow binding directly to the multicast group
            # address, so fall back to the selected local interface address.
            sock.bind((self.args.bind_address, self.ports.metatraffic_multicast))
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        membership = socket.inet_aton(self.args.multicast_group) + socket.inet_aton(interface_ip)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sock.setblocking(False)
        return sock

    def _create_user_multicast_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        # Bind to INADDR_ANY, not a unicast address: multicast datagrams are addressed to the group
        # (e.g. 239.255.0.11), so a socket bound to a specific unicast interface address will not
        # receive them on Linux (and unreliably on macOS). Delivery is decided by the joined
        # group(s) + port. This socket may join several user-data groups, so binding to a single
        # group address is not an option.
        sock.bind(("", self.ports.user_multicast))
        sock.setblocking(False)
        return sock

    def _join_user_multicast_group(self, group: str) -> None:
        if group in self.joined_user_multicast_groups:
            return
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        membership = socket.inet_aton(group) + socket.inet_aton(interface_ip)
        self.user_multicast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        self.joined_user_multicast_groups.add(group)
        log(f"[multicast] joined user-data group {group}:{self.ports.user_multicast}")

    def _configure_multicast_sender(self, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface_ip))

    def _next_sequence(self, writer_entity_id: bytes) -> int:
        value = self.sequence_numbers.get(writer_entity_id, 1)
        self.sequence_numbers[writer_entity_id] = value + 1
        return value

    def _local_writer_guid(self, entity_index: int) -> bytes:
        return make_guid(self.guid_prefix, entity_id_for_index(entity_index, USER_WRITER_NO_KEY_KIND))

    def _local_reader_guid(self, entity_index: int) -> bytes:
        return make_guid(self.guid_prefix, entity_id_for_index(entity_index, USER_READER_NO_KEY_KIND))

    def build_spdp_announce_message(self) -> bytes:
        parameters = bytearray()
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_u32(parameters, PID_DOMAIN_ID, self.args.domain_id)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_locator(
            parameters,
            PID_METATRAFFIC_MULTICAST_LOCATOR,
            self.args.multicast_group,
            self.ports.metatraffic_multicast,
        )
        append_parameter_locator(
            parameters,
            PID_METATRAFFIC_UNICAST_LOCATOR,
            self.args.advertised_address,
            self.ports.metatraffic_unicast,
        )
        append_parameter_locator(
            parameters,
            PID_DEFAULT_UNICAST_LOCATOR,
            self.args.advertised_address,
            self.ports.user_unicast,
        )
        append_parameter_locator(
            parameters,
            PID_DEFAULT_MULTICAST_LOCATOR,
            self.args.multicast_group,
            self.ports.user_multicast,
        )
        append_parameter_duration(
            parameters,
            PID_PARTICIPANT_LEASE_DURATION,
            DEFAULT_LEASE_DURATION_SECONDS,
            DEFAULT_LEASE_DURATION_NANOSECONDS,
        )
        append_parameter_u32(parameters, PID_BUILTIN_ENDPOINT_SET, BUILTIN_ENDPOINT_SET)
        append_parameter_octet_sequence(
            parameters,
            PID_USER_DATA,
            f"enclave={self.args.enclave};".encode("utf-8"),
        )
        append_parameter_string_cdr(parameters, PID_ENTITY_NAME, self.args.node_name)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        return build_rtps_message(
            self.guid_prefix,
            ENTITY_ID_UNKNOWN,
            SPDP_WRITER_ENTITY_ID,
            self._next_sequence(SPDP_WRITER_ENTITY_ID),
            payload,
        )

    def build_sedp_publication_message(self, writer: WriterConfig) -> bytes:
        guid = self._local_writer_guid(writer.entity_index)
        parameters = bytearray()
        append_parameter_guid(parameters, PID_ENDPOINT_GUID, guid)
        append_parameter_locator(parameters, PID_UNICAST_LOCATOR, self.args.advertised_address, self.ports.user_unicast)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_string_cdr(parameters, PID_TOPIC_NAME, writer.topic_name)
        append_parameter_string_cdr(parameters, PID_TYPE_NAME, writer.type_name)
        append_parameter_key_hash(parameters, guid)
        append_parameter_u32(parameters, PID_TYPE_MAX_SIZE_SERIALIZED, 8)
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_durability(parameters)
        append_parameter_liveliness(parameters)
        append_parameter_reliability(parameters, writer.reliable)
        append_parameter_history(parameters)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        return build_rtps_message(
            self.guid_prefix,
            SEDP_PUBLICATIONS_READER_ENTITY_ID,
            SEDP_PUBLICATIONS_WRITER_ENTITY_ID,
            self._next_sequence(SEDP_PUBLICATIONS_WRITER_ENTITY_ID),
            payload,
        )

    def build_sedp_subscription_message(self, reader: ReaderConfig) -> bytes:
        guid = self._local_reader_guid(reader.entity_index)
        parameters = bytearray()
        append_parameter_guid(parameters, PID_ENDPOINT_GUID, guid)
        append_parameter_locator(parameters, PID_UNICAST_LOCATOR, self.args.advertised_address, self.ports.user_unicast)
        append_parameter_bool(parameters, PID_EXPECTS_INLINE_QOS, False)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_string_cdr(parameters, PID_TOPIC_NAME, reader.topic_name)
        append_parameter_string_cdr(parameters, PID_TYPE_NAME, reader.type_name)
        append_parameter_key_hash(parameters, guid)
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_durability(parameters)
        append_parameter_liveliness(parameters)
        append_parameter_reliability(parameters, reader.reliable)
        append_parameter_history(parameters)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        return build_rtps_message(
            self.guid_prefix,
            SEDP_SUBSCRIPTIONS_READER_ENTITY_ID,
            SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID,
            self._next_sequence(SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID),
            payload,
        )

    def build_data_message(self, writer: WriterConfig, cdr_payload: bytes) -> bytes:
        # Standard RTPS: the DATA serializedPayload is exactly the CDR-encapsulated sample.
        writer_entity_id = entity_id_for_index(writer.entity_index, USER_WRITER_NO_KEY_KIND)
        return build_rtps_message(
            self.guid_prefix,
            ENTITY_ID_UNKNOWN,
            writer_entity_id,
            self._next_sequence(writer_entity_id),
            cdr_payload,
        )

    def send_spdp_announce_now(self) -> None:
        payload = self.build_spdp_announce_message()
        self.metatraffic_unicast_sock.sendto(
            payload,
            (self.args.multicast_group, self.ports.metatraffic_multicast),
        )

    def send_sedp_announcements_to(self, participant: ParticipantProxy) -> None:
        target = (participant.address, participant.ports.metatraffic_unicast)
        if participant.ports.metatraffic_unicast == 0 or not participant.address:
            return
        for writer in self.local_writers:
            self.metatraffic_unicast_sock.sendto(self.build_sedp_publication_message(writer), target)
        for reader in self.local_readers:
            self.metatraffic_unicast_sock.sendto(self.build_sedp_subscription_message(reader), target)

    def send_discovery_now(self) -> None:
        self.send_spdp_announce_now()
        for participant in list(self.discovered_participants.values()):
            self.send_sedp_announcements_to(participant)

    def publish_now(self) -> None:
        if not self.local_writers:
            return
        if not self._publish_value(self.local_writers[0], self.args.publish_value):
            now = time.monotonic()
            if now - self.last_no_participant_log > 2.0:
                log(
                    f"[publish] no discovered participants yet for topic '{self.local_writers[0].topic_name}', "
                    "waiting for SPDP"
                )
                self.last_no_participant_log = now
        else:
            log(
                f"[publish] sent {self.args.publish_value} on '{self.local_writers[0].topic_name}' "
                f"using {len(self._build_user_targets(self.local_writers[0]))} discovered target(s)"
            )

    def _build_user_targets(self, writer: WriterConfig) -> List[Tuple[str, int]]:
        targets: List[Tuple[str, int]] = []
        for reader in self.discovered_readers.values():
            if reader.topic_name != writer.topic_name:
                continue
            if reader.multicast_locators:
                for multicast_address, multicast_port in reader.multicast_locators:
                    target = (multicast_address, multicast_port)
                    if target not in targets:
                        targets.append(target)
                continue
            if reader.unicast_port > 0 and reader.unicast_address:
                target = (reader.unicast_address, reader.unicast_port)
                if target not in targets:
                    targets.append(target)
        if targets:
            return targets
        for participant in self.discovered_participants.values():
            target = (participant.address, participant.ports.user_unicast)
            if participant.address and participant.ports.user_unicast > 0 and target not in targets:
                targets.append(target)
        return targets

    def _publish_value(self, writer: WriterConfig, value: int, target: Optional[tuple[str, int]] = None) -> bool:
        payload = self.build_data_message(writer, serialize_uint32_cdr(value))
        if target is not None:
            self.user_unicast_sock.sendto(payload, target)
            return True
        targets = self._build_user_targets(writer)
        if not targets:
            return False
        for destination in targets:
            self.user_unicast_sock.sendto(payload, destination)
        return True

    def handle_metatraffic_packet(self, packet: bytes, sender_ip: str) -> None:
        for _guid_prefix, writer_id, serialized_payload in parse_rtps_data_messages(packet):
            parameters = parse_parameter_list(serialized_payload)
            if not parameters:
                continue

            if writer_id == SPDP_WRITER_ENTITY_ID:
                self._handle_spdp(parameters, sender_ip)
            elif writer_id == SEDP_PUBLICATIONS_WRITER_ENTITY_ID:
                self._handle_sedp(parameters, sender_ip, is_reader=False)
            elif writer_id == SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID:
                self._handle_sedp(parameters, sender_ip, is_reader=True)

    def _handle_spdp(self, parameters: List[tuple[int, bytes]], sender_ip: str) -> None:
        participant_guid = parse_guid(find_parameter(parameters, PID_PARTICIPANT_GUID))
        if participant_guid is None or participant_guid[:12] == self.guid_prefix:
            return

        meta_ip, meta_uc_port = parse_locator(find_parameter(parameters, PID_METATRAFFIC_UNICAST_LOCATOR))
        _, meta_mc_port = parse_locator(find_parameter(parameters, PID_METATRAFFIC_MULTICAST_LOCATOR))
        user_ip, user_uc_port = parse_locator(find_parameter(parameters, PID_DEFAULT_UNICAST_LOCATOR))
        _, user_mc_port = parse_locator(find_parameter(parameters, PID_DEFAULT_MULTICAST_LOCATOR))

        participant = ParticipantProxy(
            participant_guid=participant_guid,
            guid_prefix=participant_guid[:12],
            name=parse_cdr_string(find_parameter(parameters, PID_ENTITY_NAME)) or "",
            enclave=extract_enclave(parse_octet_sequence(find_parameter(parameters, PID_USER_DATA))),
            address=user_ip if user_ip != "0.0.0.0" else sender_ip,
            ports=PortMapping(
                metatraffic_multicast=meta_mc_port,
                metatraffic_unicast=meta_uc_port,
                user_multicast=user_mc_port,
                user_unicast=user_uc_port,
            ),
            builtin_endpoints=parse_u32_le(find_parameter(parameters, PID_BUILTIN_ENDPOINT_SET)) or 0,
        )

        is_new = participant_guid not in self.discovered_participants
        self.discovered_participants[participant_guid] = participant
        label = participant.name or hex_string(participant.guid_prefix)
        log(
            f"[spdp] participant '{label}' at {participant.address} "
            f"(meta={participant.ports.metatraffic_unicast}, user={participant.ports.user_unicast}, "
            f"enclave={participant.enclave})"
        )
        if is_new:
            self.send_sedp_announcements_to(participant)

    def _handle_sedp(self, parameters: List[tuple[int, bytes]], sender_ip: str, is_reader: bool) -> None:
        endpoint_guid = parse_guid(find_parameter(parameters, PID_ENDPOINT_GUID))
        if endpoint_guid is None or endpoint_guid[:12] == self.guid_prefix:
            return

        participant_guid = parse_guid(find_parameter(parameters, PID_PARTICIPANT_GUID))
        if participant_guid is None:
            participant_guid = endpoint_guid[:12] + PARTICIPANT_ENTITY_ID

        endpoint_ip, endpoint_port = parse_locator(find_parameter(parameters, PID_UNICAST_LOCATOR))
        multicast_locators = [
            parse_locator(value)
            for value in find_parameters(parameters, PID_MULTICAST_LOCATOR)
        ]
        multicast_locators = [
            (multicast_address, multicast_port)
            for multicast_address, multicast_port in multicast_locators
            if multicast_address != "0.0.0.0" and multicast_port > 0
        ]
        endpoint = EndpointProxy(
            guid=endpoint_guid,
            participant_guid=participant_guid,
            topic_name=parse_cdr_string(find_parameter(parameters, PID_TOPIC_NAME)) or "",
            type_name=parse_cdr_string(find_parameter(parameters, PID_TYPE_NAME)) or "",
            reliability=parse_reliability(find_parameter(parameters, PID_RELIABILITY)),
            is_reader=is_reader,
            expects_inline_qos=parse_bool(find_parameter(parameters, PID_EXPECTS_INLINE_QOS)) or False,
            unicast_address=endpoint_ip if endpoint_ip != "0.0.0.0" else sender_ip,
            unicast_port=endpoint_port,
            multicast_locators=multicast_locators,
        )
        endpoint_map = self.discovered_readers if is_reader else self.discovered_writers
        is_new = endpoint_guid not in endpoint_map
        endpoint_map[endpoint_guid] = endpoint
        if not is_reader:
            subscribed_topics = {reader.topic_name for reader in self.local_readers}
            if endpoint.topic_name in subscribed_topics:
                for multicast_address, multicast_port in endpoint.multicast_locators:
                    if multicast_port == self.ports.user_multicast:
                        self._join_user_multicast_group(multicast_address)
        if is_new:
            kind = "reader" if is_reader else "writer"
            log(
                f"[sedp] {kind} topic='{endpoint.topic_name}' type='{endpoint.type_name}' "
                f"reliability={endpoint.reliability} participant={guid_to_string(endpoint.participant_guid)}"
            )

    def handle_user_packet(self, packet: bytes, sender_ip: str, sender_port: int) -> None:
        subscribed_topics = {reader.topic_name for reader in self.local_readers}
        for guid_prefix, writer_id, serialized_payload in parse_rtps_data_messages(packet):
            # Standard RTPS: resolve the topic from the writer GUID via SEDP discovery state.
            writer_guid = guid_prefix + writer_id
            writer = self.discovered_writers.get(writer_guid)
            if writer is None:
                # Sample arrived before its writer was discovered via SEDP; drop it (best-effort).
                # Surface it (rate-limited) so a missing SEDP exchange is visible rather than silent.
                now = time.monotonic()
                if now - self.last_unknown_writer_log > 2.0:
                    log(
                        f"[data] received {len(serialized_payload)}-byte sample from UNDISCOVERED "
                        f"writer {guid_to_string(writer_guid)} at {sender_ip}:{sender_port}; cannot "
                        f"route without SEDP (discovered_writers={len(self.discovered_writers)})"
                    )
                    self.last_unknown_writer_log = now
                continue
            topic_name = writer.topic_name
            if topic_name not in subscribed_topics:
                continue
            maybe_value = deserialize_uint32_cdr(serialized_payload)
            if maybe_value is None:
                continue
            log(
                f"[data] topic='{topic_name}' value={maybe_value} reliability={writer.reliability} "
                f"from {sender_ip}:{sender_port} writer={hex_string(writer_id)}"
            )
            if self.args.echo_received and self.local_writers:
                out_writer = self.local_writers[0]
                if self._publish_value(out_writer, maybe_value):
                    log(f"[echo] responded with value={maybe_value} on '{out_writer.topic_name}'")
                else:
                    self._publish_value(out_writer, maybe_value, (sender_ip, sender_port))
                    log(
                        f"[echo] responded with value={maybe_value} on '{out_writer.topic_name}' "
                        f"to {sender_ip}:{sender_port}"
                    )

    def run(self) -> None:
        start_time = time.monotonic()
        self.next_discovery_send = start_time
        self.next_publish_send = start_time + self.args.publish_interval

        log(
            "Starting RTPS host harness\n"
            f"  node: {self.args.node_name}\n"
            f"  advertised address: {self.args.advertised_address}\n"
            f"  domain/participant: {self.args.domain_id}/{self.args.participant_id}\n"
            f"  ports: meta_mc={self.ports.metatraffic_multicast}, meta_uc={self.ports.metatraffic_unicast}, "
            f"user_mc={self.ports.user_multicast}, user_uc={self.ports.user_unicast}"
        )
        if self.local_readers:
            log("  readers: " + ", ".join(reader.topic_name for reader in self.local_readers))
        if self.local_writers:
            writer = self.local_writers[0]
            writer_mode = "echo responder" if self.args.echo_received else "periodic publisher"
            interval_text = (
                f", publish value={self.args.publish_value} every {self.args.publish_interval:.2f}s"
                if self.args.publish_interval > 0
                else ""
            )
            log(f"  writer: {writer.topic_name} ({reliability_to_name(writer.reliable)}, {writer_mode}{interval_text})")

        try:
            while True:
                now = time.monotonic()
                if now >= self.next_discovery_send:
                    self.send_discovery_now()
                    self.next_discovery_send = now + self.args.announce_period
                if self.local_writers and self.args.publish_interval > 0 and now >= self.next_publish_send:
                    self.publish_now()
                    self.next_publish_send = now + self.args.publish_interval
                if self.args.duration > 0 and now - start_time >= self.args.duration:
                    break

                readable, _, _ = select.select(
                    [
                        self.metatraffic_multicast_sock,
                        self.metatraffic_unicast_sock,
                        self.user_unicast_sock,
                        self.user_multicast_sock,
                    ],
                    [],
                    [],
                    0.2,
                )
                for sock in readable:
                    packet, sender = sock.recvfrom(4096)
                    sender_ip, sender_port = sender[0], sender[1]
                    if sock is self.user_unicast_sock or sock is self.user_multicast_sock:
                        self.handle_user_packet(packet, sender_ip, sender_port)
                    else:
                        self.handle_metatraffic_packet(packet, sender_ip)
        except KeyboardInterrupt:
            log("Stopping RTPS host harness")
        finally:
            self.close()

    def close(self) -> None:
        for sock in (
            self.metatraffic_multicast_sock,
            self.metatraffic_unicast_sock,
            self.user_unicast_sock,
            self.user_multicast_sock,
        ):
            try:
                sock.close()
            except OSError as exc:
                log(f"[close] ignoring socket close failure for {sock!r}: {exc}")


def parse_rtps_data_messages(packet: bytes) -> List[tuple[bytes, bytes, bytes]]:
    """Return (guid_prefix, writer_id, serialized_payload) for each DATA submessage."""
    if len(packet) < 20 or not packet.startswith(RTPS_MAGIC):
        return []
    guid_prefix = packet[8:20]
    offset = 20
    messages: List[tuple[bytes, bytes, bytes]] = []
    while offset + 4 <= len(packet):
        kind = packet[offset]
        flags = packet[offset + 1]
        length = struct.unpack_from("<H", packet, offset + 2)[0]
        offset += 4
        if offset + length > len(packet):
            break
        payload = packet[offset : offset + length]
        offset += length
        if kind != DATA_SUBMESSAGE_KIND or (flags & 0x04) == 0:
            continue
        if len(payload) < 20:
            continue
        extra_flags, octets_to_inline_qos = struct.unpack_from("<HH", payload, 0)
        del extra_flags
        if (flags & 0x02) != 0 or octets_to_inline_qos != DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS:
            continue
        writer_id = payload[8:12]
        serialized_payload = payload[20:]
        messages.append((guid_prefix, writer_id, serialized_payload))
    return messages


def run_self_test() -> int:
    """Validate the wire-format encoders/decoders against firmware expectations (no network I/O)."""
    failures: List[str] = []

    def check(name: str, condition: bool) -> None:
        log(f"  [{'PASS' if condition else 'FAIL'}] {name}")
        if not condition:
            failures.append(name)

    # Locators: kind/port are little-endian, address is raw network-order bytes; round-trips.
    loc = locator_bytes("192.168.1.5", 7411)
    check("locator kind is little-endian", loc[:4] == struct.pack("<I", KIND_UDP_V4))
    check("locator port is little-endian", loc[4:8] == struct.pack("<I", 7411))
    check("locator round-trips", parse_locator(loc) == ("192.168.1.5", 7411))

    # Durations are encoded as NTP fraction (1/2^32 s), not nanoseconds.
    expected_fraction = (DEFAULT_MAX_BLOCKING_NANOSECONDS << 32) // 1_000_000_000
    rel = bytearray()
    append_parameter_reliability(rel, reliable=False)
    fraction = struct.unpack_from("<I", rel, 4 + 4 + 4)[0]  # header(4) + kind(4) + max_blocking_sec(4)
    check("reliability max_blocking is NTP fraction", fraction == expected_fraction)
    check("100ms fraction ~= 0.1 * 2^32", abs(fraction - (1 << 32) // 10) <= 1)

    # Every emitted parameterLength is a multiple of 4 (PL_CDR requirement).
    params = bytearray()
    append_parameter_string_cdr(params, PID_TOPIC_NAME, "rt/chatter")  # body 4+10+1=15 -> 16
    append_parameter_string_cdr(params, PID_TYPE_NAME, "std_msgs::msg::dds_::UInt32_")
    append_parameter_octet_sequence(params, PID_USER_DATA, b"enclave=/;")  # body 4+10=14 -> 16
    offset = 0
    aligned = True
    while offset + 4 <= len(params):
        pid, length = struct.unpack_from("<HH", params, offset)
        offset += 4
        if pid == PID_SENTINEL:
            break
        if length % 4 != 0:
            aligned = False
        offset += length
    check("all parameterLengths are multiples of 4", aligned)

    # PL_CDR round-trips the strings through the standard parser.
    parsed_params = parse_parameter_list(build_parameter_list_payload(params))
    check("topic name round-trips", parse_cdr_string(find_parameter(parsed_params, PID_TOPIC_NAME)) == "rt/chatter")

    # Standard CDR-over-RTPS DATA: payload is exactly the CDR sample, routed by writer GUID.
    prefix = make_guid_prefix("selftest", 0, 10)
    writer_id = entity_id_for_index(0, USER_WRITER_NO_KEY_KIND)
    cdr = serialize_uint32_cdr(0xDEADBEEF)
    message = build_rtps_message(prefix, ENTITY_ID_UNKNOWN, writer_id, 1, cdr)
    parsed = parse_rtps_data_messages(message)
    check("one DATA submessage parsed", len(parsed) == 1)
    if parsed:
        got_prefix, got_writer_id, payload = parsed[0]
        check("guid_prefix recovered", got_prefix == prefix)
        check("writer_id recovered", got_writer_id == writer_id)
        check("serializedPayload is raw CDR (no framing)", payload == cdr)
        check("uint32 round-trips", deserialize_uint32_cdr(payload) == 0xDEADBEEF)

    if failures:
        log(f"SELF-TEST FAILED ({len(failures)} check(s)): {', '.join(failures)}")
        return 1
    log("SELF-TEST PASSED")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Discover an ESPP RTPS participant from a PC/host and optionally "
            "exchange temporary UInt32 user-data samples."
        )
    )
    parser.add_argument("--node-name", default="python_rtps_host", help="Local participant name")
    parser.add_argument("--domain-id", type=int, default=0, help="RTPS domain id")
    parser.add_argument("--participant-id", type=int, default=10, help="Local participant id")
    parser.add_argument(
        "--bind-address",
        default=None,
        help="Local bind address (defaults to the advertised address rather than all interfaces)",
    )
    parser.add_argument(
        "--advertised-address",
        default=None,
        help="IPv4 address to advertise to peers (defaults to best-effort local IPv4)",
    )
    parser.add_argument(
        "--multicast-interface",
        default=None,
        help="IPv4 interface to use for multicast join/send (defaults to advertised address)",
    )
    parser.add_argument("--multicast-group", default="239.255.0.1", help="RTPS metatraffic multicast group")
    parser.add_argument("--enclave", default="/", help="Enclave string advertised in SPDP user data")
    parser.add_argument(
        "--subscribe-topic",
        action="append",
        default=None,
        help=f"Topic name to advertise as a local reader (repeatable). Defaults to {DEFAULT_REQUEST_TOPIC}.",
    )
    parser.add_argument(
        "--publish-topic",
        default=None,
        help=f"Topic name to publish as a local writer. Defaults to {DEFAULT_RESPONSE_TOPIC}.",
    )
    parser.add_argument("--publish-value", type=int, default=42, help="UInt32 value to publish")
    parser.add_argument(
        "--publish-interval",
        type=float,
        default=0.0,
        help="Seconds between periodic publish attempts when --publish-topic is set (0 disables periodic publishing)",
    )
    parser.set_defaults(echo_received=True)
    parser.add_argument(
        "--echo-received",
        dest="echo_received",
        action="store_true",
        help="Echo received subscribed-topic values back on the publish topic (enabled by default)",
    )
    parser.add_argument(
        "--no-echo-received",
        dest="echo_received",
        action="store_false",
        help="Disable request/response echo behavior and only use periodic publishing",
    )
    parser.add_argument("--reliable", action="store_true", help="Mark the local writer as reliable")
    parser.add_argument("--type-name", default="std_msgs/msg/UInt32", help="Advertised type name")
    parser.add_argument(
        "--announce-period",
        type=float,
        default=1.0,
        help="Seconds between periodic SPDP/SEDP discovery announcements",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds (0 = run until Ctrl+C)",
    )
    args = parser.parse_args()

    if args.subscribe_topic is None:
        args.subscribe_topic = [DEFAULT_REQUEST_TOPIC]
    if args.publish_topic is None:
        args.publish_topic = DEFAULT_RESPONSE_TOPIC
    if args.advertised_address is None:
        args.advertised_address = guess_local_ipv4()
    if args.bind_address is None:
        args.bind_address = args.advertised_address
    try:
        ipaddress.IPv4Address(args.bind_address)
        ipaddress.IPv4Address(args.advertised_address)
        ipaddress.IPv4Address(args.multicast_interface or args.advertised_address)
        ipaddress.IPv4Address(args.multicast_group)
    except ipaddress.AddressValueError as exc:
        parser.error(str(exc))
    if args.publish_interval < 0:
        parser.error("--publish-interval must be >= 0")
    if args.announce_period <= 0:
        parser.error("--announce-period must be > 0")
    return args


def main() -> int:
    # Handle --self-test before full argument parsing so it needs no network/address configuration.
    if "--self-test" in sys.argv:
        return run_self_test()
    args = parse_args()
    harness = RtpsHostHarness(args)
    harness.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
