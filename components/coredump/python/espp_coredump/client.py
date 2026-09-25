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
from .protocol import CoreDumpError, CoreDumpTimeout, DiscoveryInfo, MessageType

ProgressFn = Callable[[int, int], None]  # (read, total) -> None

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
    ) -> None:
        if not (1 <= chunk_size <= _p.MAX_READ_LENGTH):
            raise ValueError(f"chunk_size must be 1..{_p.MAX_READ_LENGTH}")
        self._t = transport
        self._chunk = chunk_size
        self._progress = progress
        self._timeout = timeout_ms
        self._retries = max(0, retries)
        self._parser = _f.StreamParser()
        self._pending: Deque[_f.Frame] = deque()

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

    def _transact(self, request: bytes, want: MessageType,
                  timeout_ms: Optional[int] = None) -> _f.Frame:
        """Send one request and return the matching reply (module 4).

        Only device->host replies on our module count: other modules' traffic
        and device-originated requests are skipped (as the browser console also
        enforces). An ERROR reply raises :class:`CoreDumpError`."""
        to = self._timeout if timeout_ms is None else timeout_ms
        self._t.write(request, timeout_ms=to)
        deadline = time.monotonic() + to / 1000.0
        while True:
            fr = self._next_frame(deadline)
            if fr.module != _p.MODULE or not fr.is_reply:
                continue
            if fr.type == MessageType.ERROR:
                info = _p.parse_error(fr)
                if info:
                    raise CoreDumpError(f"device error: {info.message}", info.code)
                raise CoreDumpError("device error (unparseable ERROR reply)")
            if fr.type == want:
                return fr
            raise CoreDumpError(f"unexpected reply type 0x{fr.type:02x}")

    def _transact_retry(self, request: bytes, want: MessageType) -> _f.Frame:
        """_transact() with retries on a reply timeout only (a device ERROR or a
        protocol violation is final). A retry re-sends the same idempotent
        request; a reply to the timed-out attempt that arrives late is
        indistinguishable from the retry's, which is fine for GET_* and for READ
        (whose DATA echoes the offset the caller verifies)."""
        attempt = 0
        while True:
            try:
                return self._transact(request, want)
            except CoreDumpTimeout:
                if attempt >= self._retries:
                    raise
                attempt += 1

    # -- public API -----------------------------------------------------------
    def summary(self) -> str:
        """The device's crash report text; empty when the boot history is clean."""
        return _p.parse_summary(self._transact_retry(_p.make_get_summary(), MessageType.SUMMARY))

    def size(self) -> int:
        """The stored core-dump image size in bytes; 0 when there is none."""
        n = _p.parse_u32(self._transact_retry(_p.make_get_size(), MessageType.SIZE))
        if n is None:
            raise CoreDumpError("unparseable SIZE reply")
        return n

    def read_image(self, size: Optional[int] = None) -> bytes:
        """Download the whole stored image (the raw partition contents: flash
        header + ELF + checksum; see :mod:`espp_coredump.elf`) in READ chunks,
        verifying that every DATA reply echoes the requested offset and length.

        ``size`` defaults to a GET_SIZE query; an empty image (no core dump)
        returns ``b""``."""
        total = self.size() if size is None else size
        if total == 0:
            return b""
        out = bytearray(total)
        read = 0
        while read < total:
            length = min(self._chunk, total - read)
            fr = self._transact_retry(_p.make_read(read, length), MessageType.DATA)
            info = _p.parse_data(fr)
            if info is None:
                raise CoreDumpError("malformed DATA reply")
            if info.offset != read or len(info.data) != length:
                raise CoreDumpError(
                    f"DATA reply mismatch (expected {length} B @ {read}, "
                    f"got {len(info.data)} B @ {info.offset})")
            out[read:read + length] = info.data
            read += length
            if self._progress:
                self._progress(read, total)
        return bytes(out)

    def erase(self) -> None:
        """Erase the stored core dump (the device answers OK)."""
        self._transact_retry(_p.make_erase(), MessageType.OK)

    def discover(self, timeout_ms: int = 2000) -> Optional[DiscoveryInfo]:
        """Send a dispatcher ListModules request and decode the reply; None on
        timeout (no Dispatcher serving discovery on this interface)."""
        self._t.write(_p.make_discovery_request(), timeout_ms=self._timeout)
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
