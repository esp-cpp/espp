"""espp ``stream_frame`` v2 codec — pure Python, standard library only.

This mirrors ``components/stream_frame/include/stream_frame.hpp`` so a host tool
can speak the exact same wire protocol the device does, without building the
espp Python bindings.

Wire format (all multi-byte fields little-endian)::

    [magic u16 = 0x4F54 "OT"][flags u8][module u8][type u8]
        {[correlation u16] iff flags bit1}[len u32][payload][crc32 u32]

``crc32`` is the standard zlib CRC-32 over every byte from the magic through the
payload (i.e. the whole frame except the trailing CRC field). Python's
``zlib.crc32`` is that exact algorithm, so ``crc32(b"123456789") == 0xCBF43926``
matches the C++ golden value.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---- constants (kept in step with stream_frame.hpp) -------------------------
MAGIC = 0x4F54
MAGIC_BYTES = struct.pack("<H", MAGIC)  # b"TO" on the wire (0x54, 0x4F)
VERSION = 1
FLAG_REPLY = 0x01
FLAG_CORRELATION = 0x02
HEADER_SIZE = 9          # magic(2) + flags(1) + module(1) + type(1) + len(4)
CORRELATION_SIZE = 2
CRC_SIZE = 4
MAX_PAYLOAD_SIZE = 4096
MAX_FRAME_SIZE = HEADER_SIZE + CORRELATION_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE  # 4111


def crc32(data: bytes) -> int:
    """Standard zlib/IEEE CRC-32 (matches ``espp::stream_frame::crc32``)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def make_flags(reply: bool, version: int = VERSION) -> int:
    return ((version & 0x0F) << 4) | (FLAG_REPLY if reply else 0)


def flags_is_reply(flags: int) -> bool:
    return bool(flags & FLAG_REPLY)


def flags_version(flags: int) -> int:
    return (flags >> 4) & 0x0F


def flags_has_correlation(flags: int) -> bool:
    return bool(flags & FLAG_CORRELATION)


def build_frame(
    module: int,
    type_: int,
    payload: bytes = b"",
    reply: bool = False,
    correlation: Optional[int] = None,
) -> bytes:
    """Encode one frame. Raises ``ValueError`` if the payload is too large."""
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"payload {len(payload)} bytes exceeds MAX_PAYLOAD_SIZE ({MAX_PAYLOAD_SIZE})"
        )
    flags = make_flags(reply)
    if correlation is not None:
        flags |= FLAG_CORRELATION
    out = bytearray(MAGIC_BYTES)
    out += bytes((flags & 0xFF, module & 0xFF, type_ & 0xFF))
    if correlation is not None:
        out += struct.pack("<H", correlation & 0xFFFF)
    out += struct.pack("<I", len(payload))
    out += payload
    out += struct.pack("<I", crc32(bytes(out)))
    return bytes(out)


@dataclass
class Frame:
    """A complete, CRC-verified frame."""

    flags: int
    module: int
    type: int
    payload: bytes = b""
    correlation: Optional[int] = None

    @property
    def is_reply(self) -> bool:
        return flags_is_reply(self.flags)

    @property
    def version(self) -> int:
        return flags_version(self.flags)


class StreamParser:
    """Incremental, resynchronizing frame parser.

    Mirrors ``espp::stream_frame::StreamParser``: feed arbitrary chunks (USB bulk
    transfers may split or coalesce frames) and it yields the complete, CRC-valid
    frames they contain, resynchronizing past a bad magic, an oversized length,
    or a CRC mismatch by dropping one byte and retrying.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.dropped_bytes = 0

    def reset(self) -> None:
        self._buf.clear()

    def buffered(self) -> int:
        return len(self._buf)

    def feed(self, data: bytes) -> List[Frame]:
        self._buf += data
        frames: List[Frame] = []
        while True:
            frame, consumed = self._try_one()
            if consumed == 0:
                break  # need more bytes
            del self._buf[:consumed]
            if frame is not None:
                frames.append(frame)
        return frames

    def _try_one(self) -> Tuple[Optional[Frame], int]:
        """Return (frame|None, bytes_to_consume). consumed==0 means "need more".

        A non-None frame with consumed>0 is a good frame; a None frame with
        consumed==1 is a resync (drop one byte and keep scanning)."""
        buf = self._buf
        n = len(buf)
        # Find the magic. Need at least 2 bytes to check it.
        if n < 2:
            # Could a single trailing byte be the start of the magic? Keep it.
            if n == 1 and buf[0] == MAGIC_BYTES[0]:
                return None, 0
            if n == 1:
                self.dropped_bytes += 1
                return None, 1
            return None, 0
        if buf[0] != MAGIC_BYTES[0] or buf[1] != MAGIC_BYTES[1]:
            self.dropped_bytes += 1
            return None, 1  # resync: drop one byte
        # Enough for the fixed header?
        if n < HEADER_SIZE:
            return None, 0
        flags = buf[2]
        module = buf[3]
        type_ = buf[4]
        offset = 5
        correlation: Optional[int] = None
        if flags_has_correlation(flags):
            if n < HEADER_SIZE + CORRELATION_SIZE:
                return None, 0
            correlation = struct.unpack_from("<H", buf, offset)[0]
            offset += CORRELATION_SIZE
        length = struct.unpack_from("<I", buf, offset)[0]
        offset += 4
        if length > MAX_PAYLOAD_SIZE:
            self.dropped_bytes += 1
            return None, 1  # bogus length -> resync
        total = offset + length + CRC_SIZE
        if n < total:
            return None, 0  # wait for the rest of the frame
        want_crc = struct.unpack_from("<I", buf, offset + length)[0]
        got_crc = crc32(bytes(buf[: offset + length]))
        if want_crc != got_crc:
            self.dropped_bytes += 1
            return None, 1  # CRC mismatch -> resync
        payload = bytes(buf[offset : offset + length])
        return Frame(flags, module, type_, payload, correlation), total
