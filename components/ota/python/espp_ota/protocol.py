"""espp OTA stream protocol (dispatcher module 0 by default).

Mirrors ``components/ota/include/detail/ota_stream_protocol.hpp``: the OTA
message-type enum, frame builders (``make_*``) and reply parsers (``parse_*``)
layered on the :mod:`espp_ota.frame` codec, plus (re-exported from
:mod:`espp_ota.discovery`) the dispatcher discovery (ListModules) request, its
TLV reply parser and the module-id resolution rule.

The module id is only a routing key: ``MODULE`` is the service's published
default, every builder takes a ``module`` argument, and a client finds the id a
device actually serves the protocol on through discovery (``PROTOCOL`` is the
stable identity the device advertises -- ``OtaService::kProtocol``).

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
from .discovery import (DISCOVERY_LIST_MODULES, DISCOVERY_MODULE, DISCOVERY_VERSION_KNOWN,
                        DiscoveryInfo, ModuleInfo, Resolution, describe_resolution,
                        make_discovery_request, parse_discovery, resolve_module_id)

#: The public wire API: this module's own builders / parsers plus the
#: discovery API re-exported from :mod:`.discovery` (so ``protocol`` is the one
#: import a host tool needs for everything on the wire).
__all__ = [
    "MODULE", "PROTOCOL", "PROTOCOL_VERSION", "MODULE_NAME", "MODULE_APP",
    "MessageType",
    "StatusFlags",
    "make_begin",
    "make_data",
    "make_finish",
    "make_abort",
    "make_get_status",
    "make_mark_valid",
    "make_mark_invalid",
    "ErrorInfo",
    "ProgressInfo",
    "StatusInfo",
    "parse_u32",
    "parse_error",
    "parse_progress",
    "parse_status",
    "OtaError",
    # re-exported discovery API
    "DISCOVERY_LIST_MODULES", "DISCOVERY_MODULE", "DISCOVERY_VERSION_KNOWN", "DiscoveryInfo",
    "ModuleInfo", "Resolution", "describe_resolution", "make_discovery_request",
    "parse_discovery", "resolve_module_id",
]

#: OtaService's default dispatcher module id (a routing key only).
MODULE = 0

#: The protocol identity the service advertises through discovery
#: (``OtaService::kProtocol`` / ``kProtocolVersion``), plus the name and hosted
#: app it advertises: what a client matches on to find its module id.
PROTOCOL = "espp.ota"
PROTOCOL_VERSION = 1
MODULE_NAME = "OTA"
MODULE_APP = "ota_console.html"


class MessageType(IntEnum):
    BEGIN = 0x01     # host->device: u32 image_size (0 = unknown / streaming)
    DATA = 0x02      # host->device: raw image bytes (1..MAX_PAYLOAD_SIZE)
    FINISH = 0x03    # host->device: validate + activate (no payload)
    ABORT = 0x04     # host->device: discard the session (no payload)
    OK = 0x05        # device->host: u32 bytes_received so far
    ERROR = 0x06     # device->host: u32 code + utf-8 message
    PROGRESS = 0x07  # device->host: u32 written, u32 total (0 if unknown)
    GET_STATUS = 0x08    # host->device: query rollback status (no payload) -> STATUS
    MARK_VALID = 0x09    # host->device: confirm the running image (cancel rollback)
    MARK_INVALID = 0x0A  # host->device: reject the running image (roll back + reboot)
    STATUS = 0x0B        # device->host: u8 flags (see StatusFlags)


class StatusFlags(IntEnum):
    PENDING_VERIFY = 0x01     # running image awaits confirmation (rolls back if not)
    ROLLBACK_SUPPORTED = 0x02  # bootloader rollback support is compiled in


_REPLY_TYPES = {MessageType.OK, MessageType.ERROR, MessageType.PROGRESS, MessageType.STATUS}


def _build(type_: MessageType, payload: bytes = b"", module: int = MODULE) -> bytes:
    return _f.build_frame(module, int(type_), payload, reply=type_ in _REPLY_TYPES)


# ---- request builders (host -> device) --------------------------------------
# ``module`` is the dispatcher module id to stamp (the published default, or
# the id discovery found the device serving the protocol on).
def make_begin(image_size: int, module: int = MODULE) -> bytes:
    return _build(MessageType.BEGIN, struct.pack("<I", image_size & 0xFFFFFFFF), module)


def make_data(chunk: bytes, module: int = MODULE) -> bytes:
    return _build(MessageType.DATA, chunk, module)


def make_finish(module: int = MODULE) -> bytes:
    return _build(MessageType.FINISH, module=module)


def make_abort(module: int = MODULE) -> bytes:
    return _build(MessageType.ABORT, module=module)


def make_get_status(module: int = MODULE) -> bytes:
    return _build(MessageType.GET_STATUS, module=module)


def make_mark_valid(module: int = MODULE) -> bytes:
    return _build(MessageType.MARK_VALID, module=module)


def make_mark_invalid(module: int = MODULE) -> bytes:
    return _build(MessageType.MARK_INVALID, module=module)


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


@dataclass
class StatusInfo:
    pending_verify: bool      # running image awaits confirmation (rolls back if not)
    rollback_supported: bool  # bootloader rollback support is compiled in
    version: str = ""         # running app version (may be empty)
    project_name: str = ""    # running app project name (may be empty)

    def firmware_str(self) -> str:
        """A short 'project vX.Y' label for the running firmware."""
        if self.project_name and self.version:
            return f"{self.project_name} {self.version}"
        return self.project_name or self.version or "(unknown)"


def parse_status(fr: _f.Frame) -> Optional[StatusInfo]:
    # payload: [flags u8][version u8-len+bytes][project u8-len+bytes]; the strings
    # are optional (older devices sent flags only).
    if not fr.payload:
        return None
    flags = fr.payload[0]
    i = 1

    def read_str() -> str:
        nonlocal i
        if i >= len(fr.payload):
            return ""
        n = fr.payload[i]
        i += 1
        s = fr.payload[i:i + n].decode("utf-8", errors="replace")
        i += n
        return s

    version = read_str()
    project = read_str()
    return StatusInfo(
        pending_verify=bool(flags & StatusFlags.PENDING_VERIFY),
        rollback_supported=bool(flags & StatusFlags.ROLLBACK_SUPPORTED),
        version=version,
        project_name=project,
    )


class OtaError(RuntimeError):
    """An ERROR reply, a protocol violation, or a transport failure."""

    def __init__(self, message: str, code: Optional[int] = None) -> None:
        super().__init__(message if code is None else f"{message} (code {code})")
        self.code = code
