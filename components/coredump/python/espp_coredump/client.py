"""The core-dump session driver over a byte transport.

Transport-agnostic: it needs an object with ``write(bytes, timeout_ms)`` and
``read(max_len, timeout_ms) -> bytes`` (``b""`` on timeout), e.g.
:class:`espp_coredump.transport.UsbVendorTransport`. Flow control is one request
in flight - each request waits for its reply before the next is sent - matching
the device and ``coredump_console.html``.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Deque, Optional

from . import frame as _f
from . import protocol as _p
from .protocol import CoreDumpError, CoreDumpTimeout, DiscoveryInfo, MessageType, Resolution

ProgressFn = Callable[[int, int], None]  # (read, total) -> None
RequestFn = Callable[[int, int], bytes]  # (correlation id, module id) -> encoded request frame

#: READ chunk the browser console uses; well inside the 4092-byte cap and the
#: example's 4096-byte USB buffers.
DEFAULT_CHUNK = 2048


class CoreDumpClient:
    def __init__(
        self,
        transport,
        chunk_size: int = DEFAULT_CHUNK,
        progress: Optional[ProgressFn] = None,
        timeout_ms: int = 5000,
        retries: int = 2,
        module: Optional[int] = None,
        discover_timeout_ms: int = 2000,
    ) -> None:
        """``module`` is the dispatcher module id to talk to. The default
        (None) resolves it from the device's discovery reply before the first
        request -- see resolve_module() -- since the id is only a routing key
        and a device may serve the core-dump protocol on any id; an explicit
        id is used as given (it still shows up in resolve_module()'s warnings
        if the device advertises something else there)."""
        if not (1 <= chunk_size <= _p.MAX_READ_LENGTH):
            raise ValueError(f"chunk_size must be 1..{_p.MAX_READ_LENGTH}")
        if module is not None and not (0 <= module <= 0xFE):
            raise ValueError("module must be 0..254")
        self._t = transport
        self._chunk = chunk_size
        #: The module id requests are stamped with / replies matched on. Starts
        #: at the explicit id or the published default; resolve_module() (run
        #: automatically before the first request unless an id was given)
        #: replaces it with what the device advertises.
        self.module: int = _p.MODULE if module is None else module
        self._module_override = module
        #: How the module id was chosen (None until resolve_module() ran).
        self.resolution: Optional[Resolution] = None
        self._discover_to = discover_timeout_ms
        #: Progress callback ``(bytes_read, total)`` for read_image() /
        #: read_image_to(); may be (re)assigned between operations.
        self.progress = progress
        self._timeout = timeout_ms
        self._retries = max(0, retries)
        self._parser = _f.StreamParser()
        self._pending: Deque[_f.Frame] = deque()
        self._correlation = 0  # last correlation id sent (u16, wraps)
        #: The device's decoded discovery reply (None until resolve_module()
        #: ran, or when the device gave none).
        self.discovered: Optional[DiscoveryInfo] = None
        #: Whether the device echoes correlation ids: None until its first reply
        #: has been seen, then True, or False for a CoreDumpService predating
        #: correlation support. A request is retried after a timeout ONLY once
        #: support is established (True): without ids a late reply cannot be
        #: told from the retry's, and before the first reply nothing is known.
        self.correlation_supported: Optional[bool] = None

    # -- reply plumbing -------------------------------------------------------
    def _next_frame(self, deadline: float) -> _f.Frame:
        while True:
            if self._pending:
                return self._pending.popleft()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CoreDumpTimeout("timed out waiting for a device reply")
            data = self._t.read(_f.MAX_FRAME_SIZE, timeout_ms=max(1, int(remaining * 1000)))
            if data:
                self._pending.extend(self._parser.feed(data))

    def _transact(self, make_request: RequestFn, want: MessageType,
                  timeout_ms: Optional[int] = None) -> _f.Frame:
        """Send one request and return the matching reply (on self.module).

        Every request carries a fresh correlation id, which the device echoes:
        a reply with a different id (the late reply of an earlier, timed-out
        request) is discarded. Only device->host replies on our module count:
        other modules' traffic and device-originated requests are skipped (as
        the browser console also enforces). An ERROR reply raises
        :class:`CoreDumpError`."""
        self._ensure_module()
        to = self._timeout if timeout_ms is None else timeout_ms
        self._correlation = (self._correlation + 1) & 0xFFFF
        corr = self._correlation
        self._t.write(make_request(corr, self.module), timeout_ms=to)
        deadline = time.monotonic() + to / 1000.0
        while True:
            fr = self._next_frame(deadline)
            if fr.module != self.module or not fr.is_reply:
                continue
            if fr.correlation is None:
                self.correlation_supported = False
            else:
                self.correlation_supported = True
                if fr.correlation != corr:
                    continue  # a late reply to an earlier request
            if fr.type == MessageType.ERROR:
                info = _p.parse_error(fr)
                if info:
                    raise CoreDumpError(f"device error: {info.message}", info.code)
                raise CoreDumpError("device error (unparseable ERROR reply)")
            if fr.type == want:
                return fr
            raise CoreDumpError(f"unexpected reply type 0x{fr.type:02x}")

    def _transact_retry(self, make_request: RequestFn, want: MessageType) -> _f.Frame:
        """_transact() with retries on a reply timeout only (a device ERROR or a
        protocol violation is final). A retry re-sends the same idempotent
        request under a new correlation id, so the timed-out attempt's reply,
        should it still arrive, is recognised and dropped. That only holds once
        the device is known to echo ids (correlation_supported is True): a
        device that does not, and a device that has not answered yet, are not
        retried, since there a late reply would be taken for the retry's and
        the reply after that for the next request's."""
        attempt = 0
        while True:
            try:
                return self._transact(make_request, want)
            except CoreDumpTimeout:
                if attempt >= self._retries or self.correlation_supported is not True:
                    raise
                attempt += 1

    # -- module id ------------------------------------------------------------
    def resolve_module(self, timeout_ms: Optional[int] = None) -> Resolution:
        """Ask the device which modules it serves (discover()) and adopt the
        one speaking the core-dump protocol: an explicit ``module`` given to the
        constructor wins, else the module advertising ``PROTOCOL`` (then the
        console's app filename, then the module name), else the published
        default. Runs automatically before the first request when no explicit
        id was given; call it yourself to report the outcome (the Resolution
        carries the source and any warnings). A device that does not answer
        discovery keeps the default id."""
        info = self.discover(timeout_ms=self._discover_to if timeout_ms is None else timeout_ms)
        r = _p.resolve_module_id(info, protocol=_p.PROTOCOL, protocol_version=_p.PROTOCOL_VERSION,
                                 app=_p.MODULE_APP, name=_p.MODULE_NAME, fallback=_p.MODULE,
                                 override=self._module_override)
        self.module = r.id
        self.resolution = r
        self.discovered = info
        return r

    def _ensure_module(self) -> None:
        if self.resolution is None and self._module_override is None:
            self.resolve_module()

    # -- public API -----------------------------------------------------------
    def summary(self) -> str:
        """The device's crash report text; empty when the boot history is clean."""
        return _p.parse_summary(self._transact_retry(
            lambda corr, mod: _p.make_get_summary(corr, module=mod), MessageType.SUMMARY))

    def size(self) -> int:
        """The stored core-dump image size in bytes; 0 when there is none."""
        n = _p.parse_u32(self._transact_retry(
            lambda corr, mod: _p.make_get_size(corr, module=mod), MessageType.SIZE))
        if n is None:
            raise CoreDumpError("unparseable SIZE reply")
        return n

    def read_image_to(self, sink: Callable[[bytes], object], size: Optional[int] = None) -> int:
        """Stream the stored image (the raw partition contents: flash header +
        ELF + checksum; see :mod:`espp_coredump.elf`) in READ chunks to
        ``sink`` -- typically a binary file's ``write`` -- verifying that every
        DATA reply echoes the requested offset and length, so nothing beyond one
        chunk is held in memory.

        ``size`` defaults to a GET_SIZE query. Returns the number of bytes
        delivered (0 when there is no core dump; ``sink`` is then never called)."""
        total = self.size() if size is None else size
        read = 0
        while read < total:
            length = min(self._chunk, total - read)
            fr = self._transact_retry(
                lambda corr, mod, off=read, n=length: _p.make_read(off, n, corr, module=mod),
                MessageType.DATA)
            info = _p.parse_data(fr)
            if info is None:
                raise CoreDumpError("malformed DATA reply")
            if info.offset != read or len(info.data) != length:
                raise CoreDumpError(
                    f"DATA reply mismatch (expected {length} B @ {read}, "
                    f"got {len(info.data)} B @ {info.offset})")
            sink(info.data)
            read += length
            if self.progress:
                self.progress(read, total)
        return read

    def read_image(self, size: Optional[int] = None) -> bytes:
        """Download the whole stored image into memory (see read_image_to() for
        the streaming form; images are at most the core-dump partition, tens to
        a few hundred KiB, so this is the convenient default). An empty image
        (no core dump) returns ``b""``."""
        out = bytearray()
        self.read_image_to(out.extend, size)
        return bytes(out)

    def erase(self) -> None:
        """Erase the stored core dump (the device answers OK)."""
        self._transact_retry(lambda corr, mod: _p.make_erase(corr, module=mod), MessageType.OK)

    def discover(self, timeout_ms: int = 2000) -> Optional[DiscoveryInfo]:
        """Send a dispatcher ListModules request and decode the reply; None on
        timeout (no Dispatcher serving discovery on this interface). Does not
        change self.module -- resolve_module() does."""
        # the caller's timeout bounds the whole probe, the request write included
        self._t.write(_p.make_discovery_request(), timeout_ms=timeout_ms)
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            try:
                fr = self._next_frame(deadline)
            except CoreDumpError:
                return None
            if (fr.module == _p.DISCOVERY_MODULE and fr.is_reply
                    and fr.type == _p.DISCOVERY_LIST_MODULES):
                info = _p.parse_discovery(fr)
                if info is None:
                    raise CoreDumpError("unparseable discovery reply")
                return info
