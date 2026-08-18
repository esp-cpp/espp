"""ODrive legacy (Fibre) endpoint protocol -- packet codec + type codecs.

A clean re-implementation of the documented wire format (``PROTOCOL.md``),
with no dependency on the ``odrive``/``fibre`` pip package.

Packet (little-endian throughout)::

    request : [seq u16][endpoint u16 (bit15 => expect response)][output_len u16][payload][trailer u16]
    response: [seq | 0x8000 u16][data ...]

The ``trailer`` is a canary the server checks: ``PROTOCOL_VERSION`` for
endpoint 0, else the ``json_crc``.
"""

import struct

from .crc import PROTOCOL_VERSION

__all__ = [
    "PROTOCOL_VERSION",
    "TYPE_CODECS",
    "TypeCodec",
    "build_packet",
    "parse_response",
]


class TypeCodec:
    """(de)serializer for one primitive wire type, backed by ``struct``."""

    __slots__ = ("name", "fmt", "size", "py_type")

    def __init__(self, name: str, fmt: str, py_type):
        self.name = name
        self.fmt = "<" + fmt
        self.size = struct.calcsize(self.fmt)
        self.py_type = py_type

    def encode(self, value) -> bytes:
        return struct.pack(self.fmt, self.py_type(value))

    def decode(self, data: bytes):
        # Accept a short/long buffer defensively; only the leading bytes matter.
        return struct.unpack(self.fmt, data[: self.size])[0]


# All little-endian; sizes 1/1/1/2/2/4/4/8/8/4 per PROTOCOL.md.
TYPE_CODECS = {
    "bool": TypeCodec("bool", "?", bool),
    "int8": TypeCodec("int8", "b", int),
    "uint8": TypeCodec("uint8", "B", int),
    "int16": TypeCodec("int16", "h", int),
    "uint16": TypeCodec("uint16", "H", int),
    "int32": TypeCodec("int32", "i", int),
    "uint32": TypeCodec("uint32", "I", int),
    "int64": TypeCodec("int64", "q", int),
    "uint64": TypeCodec("uint64", "Q", int),
    "float": TypeCodec("float", "f", float),
}


def build_packet(seq: int, endpoint_id: int, output_len: int, payload: bytes,
                 expect_response: bool, json_crc: int) -> bytes:
    """Assemble one request packet (without stream framing).

    ``seq`` should be the 15-bit outbound sequence number; the caller keeps it
    unique. ``endpoint_id`` is the low 15-bit endpoint number.
    """
    ep_field = (endpoint_id & 0x7FFF) | (0x8000 if expect_response else 0)
    packet = struct.pack("<HHH", seq & 0xFFFF, ep_field, output_len & 0xFFFF)
    packet += payload
    trailer = PROTOCOL_VERSION if (endpoint_id & 0x7FFF) == 0 else json_crc
    packet += struct.pack("<H", trailer & 0xFFFF)
    return packet


def parse_response(packet: bytes):
    """Split a response packet into ``(seq_no_with_ack_bit, data_bytes)``.

    Returns ``None`` if the packet is too short to contain a sequence number.
    """
    if len(packet) < 2:
        return None
    seq_no = struct.unpack("<H", packet[:2])[0]
    return seq_no, packet[2:]
