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
            self._reply(P._build(MessageType.OK, struct.pack("<I", self.received)))


def _ok(name, cond):
    print(("PASS" if cond else "FAIL"), name)
    if not cond:
        raise SystemExit(1)


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


if __name__ == "__main__":
    test_full_flash()
    test_error_reply()
    test_small_image_one_chunk()
    print("all host tests passed")
