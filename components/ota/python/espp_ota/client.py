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
from typing import Callable, Deque, Optional

from . import frame as _f
from . import protocol as _p
from .protocol import DiscoveryInfo, MessageType, OtaError, Resolution

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
        module: Optional[int] = None,
        discover_timeout_ms: int = 2000,
    ) -> None:
        """``module`` is the dispatcher module id to talk to. The default
        (None) resolves it from the device's discovery reply before the first
        request -- see resolve_module() -- since the id is only a routing key
        and a device may serve OTA on any id; an explicit id is used as given
        (it still shows up in resolve_module()'s warnings if the device
        advertises something else there)."""
        if not (1 <= chunk_size <= _f.MAX_PAYLOAD_SIZE):
            raise ValueError(f"chunk_size must be 1..{_f.MAX_PAYLOAD_SIZE}")
        if module is not None and not (0 <= module <= 0xFE):
            raise ValueError("module must be 0..254")
        self._t = transport
        self._chunk = chunk_size
        self._progress = progress
        self._begin_to = begin_timeout_ms
        self._data_to = data_timeout_ms
        self._finish_to = finish_timeout_ms
        self._parser = _f.StreamParser()
        self._pending: Deque[_f.Frame] = deque()
        #: The module id requests are stamped with / replies matched on. Starts
        #: at the explicit id or the published default; resolve_module() (run
        #: automatically before the first request unless an id was given)
        #: replaces it with what the device advertises.
        self.module: int = _p.MODULE if module is None else module
        self._module_override = module
        #: How the module id was chosen (None until resolve_module() ran).
        self.resolution: Optional[Resolution] = None
        #: The device's decoded discovery reply (None until resolve_module()
        #: ran, or when the device gave none).
        self.discovered: Optional[DiscoveryInfo] = None
        self._discover_to = discover_timeout_ms

    # -- module id ------------------------------------------------------------
    def resolve_module(self, timeout_ms: Optional[int] = None) -> Resolution:
        """Ask the device which modules it serves (discover()) and adopt the
        one speaking the OTA protocol: an explicit ``module`` given to the
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
        """Send one (already module-stamped) request and return the matching
        reply on self.module.

        ``want`` is the success reply type expected (OK by default; STATUS for a
        status query). PROGRESS frames are surfaced to the callback and skipped;
        an ERROR reply raises :class:`OtaError`; frames for other modules are
        ignored."""
        self._t.write(request, timeout_ms=timeout_ms)
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            fr = self._next_frame(deadline)
            # Only device->host replies on our module are ours. Skip other
            # modules' traffic and any device-originated *request* on our module
            # (reply flag clear) so a request whose type collides with a reply
            # type can never be mistaken for a reply (as the browser also enforces).
            if fr.module != self.module or not fr.is_reply:
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
        self._ensure_module()
        mod = self.module

        # The device keeps its OTA session across a host disconnect, so a
        # previously interrupted flash can leave it "busy" and reject this run's
        # BEGIN. Recover from THAT case only: if BEGIN is rejected specifically
        # with device_or_resource_busy (EBUSY), send an ABORT to release the stale
        # session and retry BEGIN once. Any other failure (a timeout, a transport
        # error) is NOT retried — retrying on the same uncorrelated stream could
        # pair a delayed reply with the wrong request and desync the protocol.
        try:
            self._transact(_p.make_begin(size, mod), self._begin_to)
        except OtaError as exc:
            if exc.code != errno.EBUSY:
                raise
            self.abort()  # clear a stale session left by a prior interrupted run
            self._transact(_p.make_begin(size, mod), self._begin_to)

        # From here on, on any failure (device ERROR, timeout, Ctrl-C, transport
        # error) send a best-effort ABORT to release the session before propagating.
        try:
            total = len(image)
            sent = 0
            for off in range(0, total, self._chunk):
                chunk = image[off : off + self._chunk]
                ok = self._transact(_p.make_data(chunk, mod), self._data_to)
                sent += len(chunk)
                # OK carries bytes_received; prefer it, fall back to our own count.
                received = _p.parse_u32(ok)
                if self._progress:
                    self._progress(received if received is not None else sent, total)

            self._transact(_p.make_finish(mod), self._finish_to)
        except BaseException:
            self.abort()  # best-effort; swallows its own errors
            raise

    def abort(self) -> None:
        # Best-effort: called from flash()'s failure path where the transport may
        # already be gone, so swallow every error (OtaError, transport, etc.).
        try:
            self._ensure_module()
            self._transact(_p.make_abort(self.module), self._data_to)
        except Exception:
            pass  # best-effort cleanup; the link may already be gone

    # -- rollback control -----------------------------------------------------
    def get_status(self) -> "_p.StatusInfo":
        """Query the device's rollback status (a STATUS reply)."""
        self._ensure_module()
        fr = self._transact(_p.make_get_status(self.module), self._data_to,
                            want=MessageType.STATUS)
        info = _p.parse_status(fr)
        if info is None:
            raise OtaError("unparseable STATUS reply")
        return info

    def mark_valid(self) -> None:
        """Confirm the running image (cancel the pending rollback). The host does
        this after verifying the device is healthy — the app must not confirm
        itself."""
        self._ensure_module()
        self._transact(_p.make_mark_valid(self.module), self._data_to)

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
        self._ensure_module()
        self._t.write(_p.make_mark_invalid(self.module), timeout_ms=self._data_to)
        deadline = time.monotonic() + self._data_to / 1000.0
        while True:
            try:
                fr = self._next_frame(deadline)
            except OtaError:
                return  # read timed out: no reply -> the device rebooted (success)
            except OSError:
                return  # USB disconnect while awaiting -> the device rebooted (success)
            if fr.module != self.module or not fr.is_reply:
                continue  # not our reply; keep waiting
            if fr.type == MessageType.ERROR:
                info = _p.parse_error(fr)
                raise OtaError(
                    f"device error: {info.message}" if info else "rollback refused",
                    info.code if info else None)
            return  # OK / any other reply: the device acknowledged -> done

    def discover(self, timeout_ms: int = 2000) -> Optional[DiscoveryInfo]:
        """Send a dispatcher ListModules request and decode the reply; None on
        timeout (no Dispatcher serving discovery on this interface).

        Reads until the actual discovery reply arrives (module 0xFF, reply flag
        set, ListModules type), ignoring unrelated / device-initiated frames.
        Does not change self.module -- resolve_module() does."""
        # one deadline for the whole probe, the request write included: a slow
        # write eats into the reply wait, it never adds a second timeout_ms
        deadline = time.monotonic() + timeout_ms / 1000.0
        self._t.write(_p.make_discovery_request(), timeout_ms=timeout_ms)
        while True:
            try:
                fr = self._next_frame(deadline)
            except OtaError:
                return None  # timed out
            if (fr.module == _p.DISCOVERY_MODULE and fr.is_reply
                    and fr.type == _p.DISCOVERY_LIST_MODULES):
                info = _p.parse_discovery(fr)
                if info is None:
                    raise OtaError("unparseable discovery reply")
                return info
