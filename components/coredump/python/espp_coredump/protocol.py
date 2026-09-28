"""espp core-dump stream protocol (dispatcher module 4 by default).

Mirrors ``components/coredump/include/coredump_service.hpp``: the message enum,
frame builders (``make_*``) and reply parsers (``parse_*``) layered on the
:mod:`espp_coredump.frame` codec, plus (re-exported from
:mod:`espp_coredump.discovery`) the dispatcher discovery (ListModules) request,
its TLV reply parser and the module-id resolution rule.

The module id is only a routing key: ``MODULE`` is the service's published
default, every builder takes a ``module`` argument, and a client finds the id a
device actually serves the protocol on through discovery (``PROTOCOL`` is the
stable identity the device advertises -- ``CoreDumpService::kProtocol``).

Requests are host->device (reply flag = 0); replies are device->host (reply
flag = 1). ``CoreDumpService`` derives the reply flag from the high bit of the
type value, which this module mirrors. Flow control is one request in flight:
the host sends a request and waits for its reply before sending the next.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from . import frame as _f
from .discovery import (DISCOVERY_LIST_MODULES, DISCOVERY_MODULE, DISCOVERY_VERSION_KNOWN,
                        DiscoveryInfo, ModuleInfo, Resolution, describe_resolution,
                        make_discovery_request, parse_discovery, resolve_module_id)

#: The public wire API: this module's own builders / parsers plus the
#: discovery API re-exported from :mod:`.discovery` (so ``protocol`` is the one
#: import a host tool needs for everything on the wire).
__all__ = [
    "MODULE", "PROTOCOL", "PROTOCOL_VERSION", "MODULE_NAME", "MODULE_APP",
    "MAX_READ_LENGTH",
    "MessageType",
    "is_reply_type",
    "make_get_summary",
    "make_get_size",
    "make_read",
    "make_erase",
    "ErrorInfo",
    "DataInfo",
    "parse_u32",
    "parse_summary",
    "parse_data",
    "parse_error",
    "CoreDumpError",
    "CoreDumpTimeout",
    # re-exported discovery API
    "DISCOVERY_LIST_MODULES", "DISCOVERY_MODULE", "DISCOVERY_VERSION_KNOWN", "DiscoveryInfo",
    "ModuleInfo", "Resolution", "describe_resolution", "make_discovery_request",
    "parse_discovery", "resolve_module_id",
]

#: CoreDumpService's default dispatcher module id (a routing key only).
MODULE = 4

#: The protocol identity the service advertises through discovery
#: (``CoreDumpService::kProtocol`` / ``kProtocolVersion``), plus the name and
#: hosted app it advertises: what a client matches on to find its module id.
PROTOCOL = "espp.coredump"
PROTOCOL_VERSION = 1
MODULE_NAME = "Core Dump"
MODULE_APP = "coredump_console.html"

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


def _build(type_: MessageType, payload: bytes = b"",
           correlation: Optional[int] = None, module: int = MODULE) -> bytes:
    return _f.build_frame(module, int(type_), payload, reply=is_reply_type(type_),
                          correlation=correlation)


# ---- request builders (host -> device) --------------------------------------
# ``correlation`` (optional u16) is echoed by the device's reply, which is how
# the client tells a late reply of a timed-out request from the retry's.
# ``module`` is the dispatcher module id to stamp (the published default, or
# the id discovery found the device serving the protocol on).
def make_get_summary(correlation: Optional[int] = None, module: int = MODULE) -> bytes:
    return _build(MessageType.GET_SUMMARY, correlation=correlation, module=module)


def make_get_size(correlation: Optional[int] = None, module: int = MODULE) -> bytes:
    return _build(MessageType.GET_SIZE, correlation=correlation, module=module)


def make_read(offset: int, length: int, correlation: Optional[int] = None,
              module: int = MODULE) -> bytes:
    if not (0 <= offset <= 0xFFFFFFFF):
        raise ValueError("offset must fit in u32")
    if not (1 <= length <= MAX_READ_LENGTH):
        raise ValueError(f"length must be 1..{MAX_READ_LENGTH}")
    return _build(MessageType.READ, struct.pack("<IH", offset, length), correlation=correlation,
                  module=module)


def make_erase(correlation: Optional[int] = None, module: int = MODULE) -> bytes:
    return _build(MessageType.ERASE, correlation=correlation, module=module)


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


class CoreDumpError(RuntimeError):
    """An ERROR reply, a protocol violation, or a transport failure."""

    def __init__(self, message: str, code: Optional[int] = None) -> None:
        super().__init__(message if code is None else f"{message} (code {code})")
        self.code = code


class CoreDumpTimeout(CoreDumpError):
    """No reply from the device within the timeout (the only failure the client
    retries: a device ERROR or a protocol violation is final)."""
