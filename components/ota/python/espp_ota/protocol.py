"""espp OTA stream protocol (dispatcher module 0).

Mirrors ``components/ota/include/detail/ota_stream_protocol.hpp``: the OTA
message-type enum, frame builders (``make_*``) and reply parsers (``parse_*``)
layered on the :mod:`espp_ota.frame` codec.

Requests are host->device (reply flag = 0); replies are device->host
(reply flag = 1). Flow control is one-frame-in-flight: the host sends a request
and waits for the matching OK/ERROR reply before sending the next.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from . import frame as _f

#: OTA occupies dispatcher module id 0.
MODULE = 0

#: Discovery meta-module (see components/dispatcher). ListModules == 0x00.
DISCOVERY_MODULE = 0xFF
DISCOVERY_LIST_MODULES = 0x00


class MessageType(IntEnum):
    BEGIN = 0x01     # host->device: u32 image_size (0 = unknown / streaming)
    DATA = 0x02      # host->device: raw image bytes (1..MAX_PAYLOAD_SIZE)
    FINISH = 0x03    # host->device: validate + activate (no payload)
    ABORT = 0x04     # host->device: discard the session (no payload)
    OK = 0x05        # device->host: u32 bytes_received so far
    ERROR = 0x06     # device->host: u32 code + utf-8 message
    PROGRESS = 0x07  # device->host: u32 written, u32 total (0 if unknown)


_REPLY_TYPES = {MessageType.OK, MessageType.ERROR, MessageType.PROGRESS}


def _build(type_: MessageType, payload: bytes = b"") -> bytes:
    return _f.build_frame(MODULE, int(type_), payload, reply=type_ in _REPLY_TYPES)


# ---- request builders (host -> device) --------------------------------------
def make_begin(image_size: int) -> bytes:
    return _build(MessageType.BEGIN, struct.pack("<I", image_size & 0xFFFFFFFF))


def make_data(chunk: bytes) -> bytes:
    return _build(MessageType.DATA, chunk)


def make_finish() -> bytes:
    return _build(MessageType.FINISH)


def make_abort() -> bytes:
    return _build(MessageType.ABORT)


def make_discovery_request() -> bytes:
    """A dispatcher discovery (ListModules) request on module 0xFF."""
    return _f.build_frame(DISCOVERY_MODULE, DISCOVERY_LIST_MODULES, b"", reply=False)


# ---- reply parsers (device -> host) -----------------------------------------
@dataclass
class ErrorInfo:
    code: int
    message: str


@dataclass
class ProgressInfo:
    written: int
    total: int  # 0 if unknown


def parse_u32(fr: _f.Frame) -> Optional[int]:
    """The single-u32 payload of a BEGIN echo or an OK (bytes_received)."""
    if len(fr.payload) != 4:
        return None
    return struct.unpack("<I", fr.payload)[0]


def parse_error(fr: _f.Frame) -> Optional[ErrorInfo]:
    if len(fr.payload) < 4:
        return None
    code = struct.unpack_from("<I", fr.payload, 0)[0]
    message = fr.payload[4:].decode("utf-8", errors="replace")
    return ErrorInfo(code, message)


def parse_progress(fr: _f.Frame) -> Optional[ProgressInfo]:
    if len(fr.payload) != 8:
        return None
    written, total = struct.unpack("<II", fr.payload)
    return ProgressInfo(written, total)


class OtaError(RuntimeError):
    """An ERROR reply, a protocol violation, or a transport failure."""

    def __init__(self, message: str, code: Optional[int] = None) -> None:
        super().__init__(message if code is None else f"{message} (code {code})")
        self.code = code
