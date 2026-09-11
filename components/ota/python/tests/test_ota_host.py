"""Host tests for espp_ota: codec round-trips + a full OTA against a mock device.

Runs with plain ``python3`` (no hardware, no pyusb). Also importable by pytest.
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espp_ota import frame as F  # noqa: E402
from espp_ota import protocol as P  # noqa: E402
from espp_ota.client import OtaClient  # noqa: E402
from espp_ota.protocol import MessageType, OtaError  # noqa: E402


class MockDevice:
    """Loopback transport implementing the OTA device side in memory.

    The host writes request frames; ``read()`` returns the device's replies.
    Set ``fail_after`` to make the device answer a DATA frame with ERROR."""

    def __init__(self, fail_after=None, emit_progress=False):
        self._parser = F.StreamParser()
        self._out = bytearray()
        self.image = bytearray()
        self.received = 0
        self.data_frames = 0
        self.finished = False
        self._fail_after = fail_after
        self._emit_progress = emit_progress

    def write(self, data, timeout_ms=0):
        for fr in self._parser.feed(data):
            self._handle(fr)

    def read(self, max_len, timeout_ms=0):
        # Simulate the device dropping off the bus (a reboot mid-transaction):
        # UsbVendorTransport.read() re-raises a non-timeout USBError, which is an
        # OSError subclass. Only trigger once there is nothing buffered to hand back.
        if getattr(self, "_disconnect", False) and not self._out:
            raise OSError("device disconnected")
        if not self._out:
            return b""
        chunk = bytes(self._out[:max_len])
        del self._out[: len(chunk)]
        return chunk

    def _reply(self, b):
        self._out += b

    def _handle(self, fr):
        if fr.module != P.MODULE:
            return
        t = fr.type
        if t == MessageType.BEGIN:
            # Emulate a stale session left by a prior interrupted flash: reject the
            # first BEGIN as busy until an ABORT clears it.
            if getattr(self, "_busy", False):
                self._reply(P._build(MessageType.ERROR, struct.pack("<I", 16) + b"busy"))
                return
            self.received = 0
            self.image = bytearray()
            self._reply(P._build(MessageType.OK, struct.pack("<I", 0)))
        elif t == MessageType.DATA:
            self.data_frames += 1
            if self._fail_after is not None and self.data_frames > self._fail_after:
                self._reply(P._build(MessageType.ERROR, struct.pack("<I", 22) + b"boom"))
                return
            self.image += fr.payload
            self.received += len(fr.payload)
            if self._emit_progress:
                self._reply(P._build(MessageType.PROGRESS, struct.pack("<II", self.received, 0)))
            self._reply(P._build(MessageType.OK, struct.pack("<I", self.received)))
        elif t == MessageType.FINISH:
            self.finished = True
            self._reply(P._build(MessageType.OK, struct.pack("<I", self.received)))
        elif t == MessageType.ABORT:
            self._busy = False  # ABORT clears a stale session
            self._reply(P._build(MessageType.OK, struct.pack("<I", self.received)))
        elif t == MessageType.GET_STATUS:
            flags = P.StatusFlags.ROLLBACK_SUPPORTED
            if getattr(self, "_pending", False):
                flags |= P.StatusFlags.PENDING_VERIFY
            ver = getattr(self, "_version", "1.0.0").encode()
            proj = b"ota_example"
            payload = bytes([flags, len(ver)]) + ver + bytes([len(proj)]) + proj
            self._reply(P._build(MessageType.STATUS, payload))
        elif t == MessageType.MARK_VALID:
            self.marked_valid = True
            self._pending = False
            self._reply(P._build(MessageType.OK, struct.pack("<I", 0)))
        elif t == MessageType.MARK_INVALID:
            self.rolled_back = True
            if getattr(self, "_rollback_refused", False):
                # e.g. no valid previous image: the device refuses and stays put.
                self._reply(P._build(MessageType.ERROR, struct.pack("<I", 2) + b"no valid app"))
            elif getattr(self, "_rollback_disconnect", False):
                self._disconnect = True  # the device reboots -> read() raises OSError
            # else: reboot with NO reply -> the host's read times out (success)


def _ok(name, cond):
    print(("PASS" if cond else "FAIL"), name)
    if not cond:
        raise SystemExit(1)


def test_frame_golden():
    """Byte-level fixtures independent of the mock loopback: any change to the
    wire encoding (or a divergence from the C++ codec) breaks these."""
    # zlib/IEEE CRC-32 golden vector, same as espp::stream_frame::crc32.
    _ok("golden crc vector", F.crc32(b"123456789") == 0xCBF43926)
    # A request's flags byte is version 1 << 4, reply bit clear.
    _ok("request flags 0x10", F.make_flags(False) == 0x10)
    # Whole-frame goldens (magic "TO", flags, module, type, len LE, payload, crc LE).
    _ok("BEGIN(0) bytes",
        P.make_begin(0) == bytes.fromhex("544f100001040000000000000096ed77b9"))
    _ok("discovery request bytes",
        P.make_discovery_request() == bytes.fromhex("544f10ff000000000097e310ba"))


def test_parser_resync():
    """The StreamParser must skip leading garbage and a bad-CRC frame and still
    yield the valid frame that follows (the interoperability-critical behavior)."""
    good = P.make_begin(12345)
    # leading garbage (no 0x54 magic byte) before a valid frame
    p = F.StreamParser()
    _ok("garbage holds", p.feed(b"\x00\xffJUNK") == [])
    frames = p.feed(good)
    _ok("resync past garbage", len(frames) == 1 and frames[0].type == 1 and p.dropped_bytes == 6)
    # a CRC-corrupted frame followed by a good one -> only the good one survives
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    p2 = F.StreamParser()
    frames2 = p2.feed(bytes(bad) + good)
    _ok("skip bad CRC, keep good",
        len(frames2) == 1 and frames2[0].type == 1 and p2.dropped_bytes >= 1)
    # a frame split across two feeds
    p3 = F.StreamParser()
    half = len(good) // 2
    _ok("partial holds", p3.feed(good[:half]) == [])
    _ok("completes on rest", len(p3.feed(good[half:])) == 1)


def test_full_flash():
    image = bytes(bytearray((i * 7) & 0xFF for i in range(4096 * 2 + 123)))  # 2+ chunks
    dev = MockDevice(emit_progress=True)
    seen = []
    OtaClient(dev, progress=lambda w, t: seen.append((w, t))).flash(image)
    _ok("device received full image", bytes(dev.image) == image)
    _ok("device saw FINISH", dev.finished)
    _ok("chunking (3 DATA frames)", dev.data_frames == 3)
    _ok("progress reported", len(seen) > 0 and seen[-1][0] == len(image))


def test_error_reply():
    dev = MockDevice(fail_after=1)
    raised = False
    try:
        OtaClient(dev).flash(bytes(4096 * 3))
    except OtaError as exc:
        raised = True
        _ok("error carries device code", exc.code == 22)
    _ok("error path raises OtaError", raised)


def test_small_image_one_chunk():
    dev = MockDevice()
    OtaClient(dev).flash(b"\xe9tiny firmware")
    _ok("single-chunk image", bytes(dev.image) == b"\xe9tiny firmware" and dev.data_frames == 1)


def test_rollback_control():
    """get_status / mark_valid over the loopback mock."""
    dev = MockDevice()
    dev._pending = True
    st = OtaClient(dev).get_status()
    _ok("status pending+supported", st.pending_verify and st.rollback_supported)
    _ok("status reports firmware", st.version == "1.0.0" and st.project_name == "ota_example")
    OtaClient(dev).mark_valid()
    _ok("mark_valid confirmed", getattr(dev, "marked_valid", False) and not dev._pending)
    st2 = OtaClient(dev).get_status()
    _ok("status confirmed after mark_valid", not st2.pending_verify)


def test_rollback_reboots_no_reply():
    """Success = the device reboots WITHOUT replying, so the host's read times out.
    A short data timeout keeps the test fast."""
    dev = MockDevice()
    OtaClient(dev, data_timeout_ms=50).mark_invalid()  # no reply -> timeout -> success
    _ok("rollback (no reply) requested", getattr(dev, "rolled_back", False))


def test_rollback_disconnect_is_success():
    """A USB disconnect while awaiting the reply (read() raises OSError) also means
    the device rebooted -> success, not a raised error."""
    dev = MockDevice()
    dev._rollback_disconnect = True
    OtaClient(dev, data_timeout_ms=50).mark_invalid()  # OSError on read -> success
    _ok("rollback (disconnect) requested", getattr(dev, "rolled_back", False))


def test_rollback_refused_raises():
    """A device ERROR reply (e.g. no valid previous image) is a genuine failure."""
    dev = MockDevice()
    dev._rollback_refused = True
    try:
        OtaClient(dev, data_timeout_ms=50).mark_invalid()
        _ok("rollback refused raises", False)
    except OtaError:
        _ok("rollback refused raises", True)


def test_begin_busy_recovers():
    """A stale session (BEGIN rejected as busy) is cleared by an ABORT + retry."""
    dev = MockDevice()
    dev._busy = True  # device thinks a prior session is still open
    OtaClient(dev).flash(b"\xe9hello world payload")
    _ok("recovered from busy BEGIN", bytes(dev.image) == b"\xe9hello world payload"
        and dev.finished)


if __name__ == "__main__":
    test_frame_golden()
    test_parser_resync()
    test_full_flash()
    test_error_reply()
    test_small_image_one_chunk()
    test_begin_busy_recovers()
    test_rollback_control()
    test_rollback_reboots_no_reply()
    test_rollback_disconnect_is_success()
    test_rollback_refused_raises()
    print("all host tests passed")
