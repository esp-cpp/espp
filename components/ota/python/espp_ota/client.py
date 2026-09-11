"""The OTA session driver: BEGIN -> DATA* -> FINISH over a byte transport.

Transport-agnostic: it needs an object with ``write(bytes, timeout_ms)`` and
``read(max_len, timeout_ms) -> bytes`` (``b""`` on timeout), e.g.
:class:`espp_ota.transport.UsbVendorTransport`. Flow control is one request in
flight — each request waits for its OK/ERROR reply before the next is sent —
matching the device and ``ota_console.html``.
"""

from __future__ import annotations

import errno
import time
from collections import deque
from typing import Callable, Deque, List, Optional

from . import frame as _f
from . import protocol as _p
from .protocol import MessageType, OtaError

ProgressFn = Callable[[int, int], None]  # (written, total) -> None


class OtaClient:
    def __init__(
        self,
        transport,
        chunk_size: int = _f.MAX_PAYLOAD_SIZE,
        progress: Optional[ProgressFn] = None,
        begin_timeout_ms: int = 60000,
        data_timeout_ms: int = 5000,
        finish_timeout_ms: int = 60000,
    ) -> None:
        if not (1 <= chunk_size <= _f.MAX_PAYLOAD_SIZE):
            raise ValueError(f"chunk_size must be 1..{_f.MAX_PAYLOAD_SIZE}")
        self._t = transport
        self._chunk = chunk_size
        self._progress = progress
        self._begin_to = begin_timeout_ms
        self._data_to = data_timeout_ms
        self._finish_to = finish_timeout_ms
        self._parser = _f.StreamParser()
        self._pending: Deque[_f.Frame] = deque()

    # -- reply plumbing -------------------------------------------------------
    def _next_frame(self, deadline: float) -> _f.Frame:
        while True:
            if self._pending:
                return self._pending.popleft()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise OtaError("timed out waiting for a device reply")
            data = self._t.read(_f.MAX_FRAME_SIZE, timeout_ms=max(1, int(remaining * 1000)))
            if data:
                self._pending.extend(self._parser.feed(data))

    def _transact(self, request: bytes, timeout_ms: int,
                  want: MessageType = MessageType.OK) -> _f.Frame:
        """Send one request and return the matching reply (module 0).

        ``want`` is the success reply type expected (OK by default; STATUS for a
        status query). PROGRESS frames are surfaced to the callback and skipped;
        an ERROR reply raises :class:`OtaError`; frames for other modules are
        ignored."""
        self._t.write(request, timeout_ms=timeout_ms)
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            fr = self._next_frame(deadline)
            # Only device->host replies on module 0 are ours. Skip other modules'
            # traffic and any device-originated *request* on module 0 (reply flag
            # clear) so a request whose type collides with a reply type can never
            # be mistaken for a reply (as the browser probe also enforces).
            if fr.module != _p.MODULE or not fr.is_reply:
                continue
            if fr.type == MessageType.PROGRESS:
                info = _p.parse_progress(fr)
                if info and self._progress:
                    self._progress(info.written, info.total)
                continue
            if fr.type == MessageType.ERROR:
                info = _p.parse_error(fr)
                if info:
                    raise OtaError(f"device error: {info.message}", info.code)
                raise OtaError("device error (unparseable ERROR reply)")
            if fr.type == want:
                return fr
            raise OtaError(f"unexpected reply type 0x{fr.type:02x}")

    # -- public API -----------------------------------------------------------
    def flash(self, image: bytes, image_size: Optional[int] = None) -> None:
        """Run a full OTA: BEGIN(size) -> DATA chunks -> FINISH.

        ``image_size`` defaults to ``len(image)``; pass 0 for an unknown-size
        (streaming) session (the device erases the whole partition)."""
        if not image:
            raise OtaError("empty image")
        size = len(image) if image_size is None else image_size

        # The device keeps its OTA session across a host disconnect, so a
        # previously interrupted flash can leave it "busy" and reject this run's
        # BEGIN. Recover from THAT case only: if BEGIN is rejected specifically
        # with device_or_resource_busy (EBUSY), send an ABORT to release the stale
        # session and retry BEGIN once. Any other failure (a timeout, a transport
        # error) is NOT retried — retrying on the same uncorrelated stream could
        # pair a delayed reply with the wrong request and desync the protocol.
        try:
            self._transact(_p.make_begin(size), self._begin_to)
        except OtaError as exc:
            if exc.code != errno.EBUSY:
                raise
            self.abort()  # clear a stale session left by a prior interrupted run
            self._transact(_p.make_begin(size), self._begin_to)

        # From here on, on any failure (device ERROR, timeout, Ctrl-C, transport
        # error) send a best-effort ABORT to release the session before propagating.
        try:
            total = len(image)
            sent = 0
            for off in range(0, total, self._chunk):
                chunk = image[off : off + self._chunk]
                ok = self._transact(_p.make_data(chunk), self._data_to)
                sent += len(chunk)
                # OK carries bytes_received; prefer it, fall back to our own count.
                received = _p.parse_u32(ok)
                if self._progress:
                    self._progress(received if received is not None else sent, total)

            self._transact(_p.make_finish(), self._finish_to)
        except BaseException:
            self.abort()  # best-effort; swallows its own errors
            raise

    def abort(self) -> None:
        # Best-effort: called from flash()'s failure path where the transport may
        # already be gone, so swallow every error (OtaError, transport, etc.).
        try:
            self._transact(_p.make_abort(), self._data_to)
        except Exception:
            pass  # best-effort cleanup; the link may already be gone

    # -- rollback control -----------------------------------------------------
    def get_status(self) -> "_p.StatusInfo":
        """Query the device's rollback status (a STATUS reply)."""
        fr = self._transact(_p.make_get_status(), self._data_to, want=MessageType.STATUS)
        info = _p.parse_status(fr)
        if info is None:
            raise OtaError("unparseable STATUS reply")
        return info

    def mark_valid(self) -> None:
        """Confirm the running image (cancel the pending rollback). The host does
        this after verifying the device is healthy — the app must not confirm
        itself."""
        self._transact(_p.make_mark_valid(), self._data_to)

    def mark_invalid(self) -> None:
        """Reject the running image: the device rolls back to the previous app and
        reboots. On success it reboots *without* replying, so a missing reply is the
        expected outcome. This must NOT use _transact(), which cannot tell a write
        failure from link loss while awaiting the reply. Instead: send first (a
        failure there means the command never reached the device -> a real error
        that propagates), then wait -- a read timeout OR a USB disconnect (the
        device dropping off the bus as it re-enumerates) both mean it rebooted, so
        both are success. Only an explicit device ERROR (rollback refused, e.g. no
        previous app to roll back to) is a genuine failure."""
        self._t.write(_p.make_mark_invalid(), timeout_ms=self._data_to)
        deadline = time.monotonic() + self._data_to / 1000.0
        while True:
            try:
                fr = self._next_frame(deadline)
            except OtaError:
                return  # read timed out: no reply -> the device rebooted (success)
            except OSError:
                return  # USB disconnect while awaiting -> the device rebooted (success)
            if fr.module != _p.MODULE or not fr.is_reply:
                continue  # not our reply; keep waiting
            if fr.type == MessageType.ERROR:
                info = _p.parse_error(fr)
                raise OtaError(
                    f"device error: {info.message}" if info else "rollback refused",
                    info.code if info else None)
            return  # OK / any other reply: the device acknowledged -> done

    def discover(self, timeout_ms: int = 2000) -> List[_f.Frame]:
        """Send a dispatcher ListModules request; return the matching reply.

        Useful as a connectivity probe before flashing. Reads until the actual
        discovery reply arrives (module 0xFF, reply flag set, ListModules type),
        ignoring unrelated / device-initiated frames; returns [] on timeout. The
        discovery TLV payload is not decoded here."""
        self._t.write(_p.make_discovery_request(), timeout_ms=self._data_to)
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            try:
                fr = self._next_frame(deadline)
            except OtaError:
                return []  # timed out
            if (fr.module == _p.DISCOVERY_MODULE and fr.is_reply
                    and fr.type == _p.DISCOVERY_LIST_MODULES):
                return [fr]
