"""espp core-dump stream protocol (dispatcher module 4).

Mirrors ``components/coredump/include/coredump_service.hpp``: the message enum,
frame builders (``make_*``) and reply parsers (``parse_*``) layered on the
:mod:`espp_coredump.frame` codec, plus the dispatcher discovery (ListModules)
request and its TLV reply parser.

Requests are host->device (reply flag = 0); replies are device->host (reply
flag = 1). ``CoreDumpService`` derives the reply flag from the high bit of the
type value, which this module mirrors. Flow control is one request in flight:
the host sends a request and waits for its reply before sending the next.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional

from . import frame as _f

#: CoreDumpService's default dispatcher module id.
MODULE = 4

#: Discovery meta-module (see components/dispatcher). ListModules == 0x00.
DISCOVERY_MODULE = 0xFF
DISCOVERY_LIST_MODULES = 0x00

#: READ length cap: the DATA reply carries u32 offset + bytes in one frame.
MAX_READ_LENGTH = _f.MAX_PAYLOAD_SIZE - 4  # 4092


class MessageType(IntEnum):
    # host -> device
    GET_SUMMARY = 0x40  # request the crash report text (no payload)
    GET_SIZE = 0x41     # request the core-dump image size (no payload)
    READ = 0x42         # u32 offset + u16 length -> DATA
    ERASE = 0x43        # erase the stored core dump (no payload) -> OK
    # device -> host
    SUMMARY = 0xC0      # utf-8 crash report (empty = clean boot history)
    SIZE = 0xC1         # u32 image size (0 = no core dump)
    DATA = 0xC2         # u32 offset + image bytes
    OK = 0xC3           # u32 context-dependent success value
    ERROR = 0xC4        # u32 informational code + utf-8 message


def is_reply_type(type_: int) -> bool:
    """The service marks replies by the high bit of the type value."""
    return bool(type_ & 0x80)


def _build(type_: MessageType, payload: bytes = b"") -> bytes:
    return _f.build_frame(MODULE, int(type_), payload, reply=is_reply_type(type_))


# ---- request builders (host -> device) --------------------------------------
def make_get_summary() -> bytes:
    return _build(MessageType.GET_SUMMARY)


def make_get_size() -> bytes:
    return _build(MessageType.GET_SIZE)


def make_read(offset: int, length: int) -> bytes:
    if not (0 <= offset <= 0xFFFFFFFF):
        raise ValueError("offset must fit in u32")
    if not (1 <= length <= MAX_READ_LENGTH):
        raise ValueError(f"length must be 1..{MAX_READ_LENGTH}")
    return _build(MessageType.READ, struct.pack("<IH", offset, length))


def make_erase() -> bytes:
    return _build(MessageType.ERASE)


def make_discovery_request() -> bytes:
    """A dispatcher discovery (ListModules) request on module 0xFF."""
    return _f.build_frame(DISCOVERY_MODULE, DISCOVERY_LIST_MODULES, b"", reply=False)


# ---- reply parsers (device -> host) -----------------------------------------
@dataclass
class ErrorInfo:
    code: int      # a std::errc value; informational only (the message is authoritative)
    message: str


@dataclass
class DataInfo:
    offset: int
    data: bytes


def parse_u32(fr: _f.Frame) -> Optional[int]:
    """The single-u32 payload of a SIZE or OK reply."""
    if len(fr.payload) != 4:
        return None
    return struct.unpack("<I", fr.payload)[0]


def parse_summary(fr: _f.Frame) -> str:
    return fr.payload.decode("utf-8", errors="replace")


def parse_data(fr: _f.Frame) -> Optional[DataInfo]:
    if len(fr.payload) < 4:
        return None
    offset = struct.unpack_from("<I", fr.payload, 0)[0]
    return DataInfo(offset, bytes(fr.payload[4:]))


def parse_error(fr: _f.Frame) -> Optional[ErrorInfo]:
    if len(fr.payload) < 4:
        return None
    code = struct.unpack_from("<I", fr.payload, 0)[0]
    message = fr.payload[4:].decode("utf-8", errors="replace")
    return ErrorInfo(code, message)


# ---- discovery TLV ----------------------------------------------------------
@dataclass
class ModuleInfo:
    id: int
    name: str
    app: str
    description: str


@dataclass
class DiscoveryInfo:
    version: int
    device_name: str
    firmware: str
    modules: List[ModuleInfo] = field(default_factory=list)

    def has_module(self, module_id: int) -> bool:
        return any(m.id == module_id for m in self.modules)


def parse_discovery(fr: _f.Frame) -> Optional[DiscoveryInfo]:
    """Decode a dispatcher ListModules reply::

        [version u8][reserved u8][device_name str][device_fw str][module_count u8]
        then per module: [id u8][name str][app str][desc str]

    where ``str`` = ``[len u8][bytes]``. Returns None on a malformed payload
    (a record truncated mid-way is dropped, the ones before it are kept: the
    device itself trims records that would not fit the frame)."""
    p = fr.payload
    if len(p) < 3:
        return None
    pos = 0
    version = p[pos]
    pos += 2  # version + reserved

    def read_str() -> Optional[str]:
        nonlocal pos
        if pos >= len(p):
            return None
        n = p[pos]
        pos += 1
        if pos + n > len(p):
            return None
        s = p[pos:pos + n].decode("utf-8", errors="replace")
        pos += n
        return s

    device_name = read_str()
    firmware = read_str()
    if device_name is None or firmware is None or pos >= len(p):
        return None
    count = p[pos]
    pos += 1
    info = DiscoveryInfo(version, device_name, firmware)
    for _ in range(count):
        if pos >= len(p):
            break
        mid = p[pos]
        pos += 1
        name = read_str()
        app = read_str()
        desc = read_str()
        if name is None or app is None or desc is None:
            break
        info.modules.append(ModuleInfo(mid, name, app, desc))
    return info


class CoreDumpError(RuntimeError):
    """An ERROR reply, a protocol violation, or a transport failure."""

    def __init__(self, message: str, code: Optional[int] = None) -> None:
        super().__init__(message if code is None else f"{message} (code {code})")
        self.code = code


class CoreDumpTimeout(CoreDumpError):
    """No reply from the device within the timeout (the only failure the client
    retries: a device ERROR or a protocol violation is final)."""
