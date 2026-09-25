"""Host tests for espp_coredump: codec round-trips + a full download against a
mock device.

Runs with plain ``python3`` (no hardware, no pyusb). Also importable by pytest.
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espp_coredump import frame as F  # noqa: E402
from espp_coredump import protocol as P  # noqa: E402
from espp_coredump.client import CoreDumpClient  # noqa: E402
from espp_coredump.elf import extract_elf, find_elf_offset  # noqa: E402
from espp_coredump.protocol import CoreDumpError, CoreDumpTimeout, MessageType  # noqa: E402

FLASH_HEADER = struct.pack("<III", 0, 2, 0)  # core_dump_header_t: data_len, version, chip_rev
ELF_BODY = b"\x7fELF" + bytes(bytearray((i * 13) & 0xFF for i in range(2048 * 2 + 77)))
CHECKSUM = b"\xaa\xbb\xcc\xdd"
IMAGE = FLASH_HEADER + ELF_BODY + CHECKSUM


class MockDevice:
    """Loopback transport implementing the core-dump device side in memory.

    The host writes request frames; ``read()`` returns the device's replies."""

    def __init__(self, image=IMAGE, summary="Guru Meditation Error: Core 0 panic'ed\n"):
        self._parser = F.StreamParser()
        self._out = bytearray()
        self.image = bytes(image)
        self.summary = summary
        self.reads = 0
        self.erased = False
        self.corrupt_offset_on_read = None  # the Nth READ answers with a wrong offset
        self.drop_first_reply = False       # the first request gets no reply (host retries)
        self.delay_first_reply = False      # the first reply arrives only after the host's retry
        self._held = b""
        self.error_on_read = False          # every READ answers ERROR
        self.legacy_no_correlation = False  # pre-correlation firmware: never echoes the id

    def write(self, data, timeout_ms=0):
        for fr in self._parser.feed(data):
            self._handle(fr)

    def read(self, max_len, timeout_ms=0):
        if not self._out:
            return b""
        chunk = bytes(self._out[:max_len])
        del self._out[: len(chunk)]
        return chunk

    def _reply(self, b, fr=None):
        if fr is not None and not self.legacy_no_correlation and fr.correlation is not None:
            # echo the request's correlation id, as CoreDumpService does
            parsed = F.StreamParser().feed(b)[0]
            b = F.build_frame(parsed.module, parsed.type, parsed.payload, reply=True,
                              correlation=fr.correlation)
        if self.drop_first_reply:
            self.drop_first_reply = False
            return
        if self.delay_first_reply:
            # held back: it lands on the wire just ahead of the NEXT reply, i.e.
            # after the host gave up on it and re-sent the request
            self.delay_first_reply = False
            self._held = b
            return
        self._out += self._held + b
        self._held = b""

    def _handle(self, fr):
        if fr.module == P.DISCOVERY_MODULE and fr.type == P.DISCOVERY_LIST_MODULES:
            def s(text):
                b = text.encode()
                return bytes([len(b)]) + b
            payload = (bytes([1, 0]) + s("espp CoreDump") + s("1.2.3") + bytes([2])
                       + bytes([1]) + s("Crash") + s("") + s("trigger a crash")
                       + bytes([4]) + s("Core Dump") + s("coredump_console.html")
                       + s("Inspect the last crash core dump"))
            self._out += F.build_frame(P.DISCOVERY_MODULE, P.DISCOVERY_LIST_MODULES, payload,
                                       reply=True)
            return
        if fr.module != P.MODULE or fr.is_reply:
            return
        t = fr.type
        if t == MessageType.GET_SUMMARY:
            self._reply(P._build(MessageType.SUMMARY, self.summary.encode()), fr)
        elif t == MessageType.GET_SIZE:
            self._reply(P._build(MessageType.SIZE, struct.pack("<I", len(self.image))), fr)
        elif t == MessageType.READ:
            self.reads += 1
            if self.error_on_read:
                self._reply(P._build(MessageType.ERROR, struct.pack("<I", 5) + b"READ failed"), fr)
                return
            offset, length = struct.unpack("<IH", fr.payload)
            data = self.image[offset:offset + length]
            echoed = offset
            if self.corrupt_offset_on_read == self.reads:
                echoed = offset + 1
            self._reply(P._build(MessageType.DATA, struct.pack("<I", echoed) + data), fr)
        elif t == MessageType.ERASE:
            self.erased = True
            self.image = b""
            self._reply(P._build(MessageType.OK, struct.pack("<I", 0)), fr)


def _ok(name, cond):
    print(("PASS" if cond else "FAIL"), name)
    if not cond:
        raise SystemExit(1)


def test_frame_golden():
    _ok("golden crc vector", F.crc32(b"123456789") == 0xCBF43926)
    _ok("request flags 0x10", F.make_flags(False) == 0x10)
    # READ(0x1000, 2048): module 4, type 0x42, len 6, payload u32 offset + u16 length
    req = P.make_read(0x1000, 2048)
    frames = F.StreamParser().feed(req)
    _ok("READ round trip", len(frames) == 1 and frames[0].module == 4
        and frames[0].type == 0x42 and not frames[0].is_reply
        and frames[0].payload == struct.pack("<IH", 0x1000, 2048))
    # replies carry the reply flag, derived from the type's high bit
    rep = F.StreamParser().feed(P._build(MessageType.SIZE, struct.pack("<I", 7)))[0]
    _ok("reply flag from type high bit", rep.is_reply and P.is_reply_type(rep.type))
    _ok("discovery request bytes",
        P.make_discovery_request() == bytes.fromhex("544f10ff000000000097e310ba"))


def test_size_and_summary():
    dev = MockDevice()
    c = CoreDumpClient(dev)
    _ok("size", c.size() == len(IMAGE))
    _ok("summary", c.summary().startswith("Guru Meditation"))
    empty = MockDevice(image=b"", summary="")
    _ok("no dump: size 0", CoreDumpClient(empty).size() == 0)
    _ok("no dump: empty summary", CoreDumpClient(empty).summary() == "")
    _ok("no dump: read_image is empty", CoreDumpClient(empty).read_image() == b"")


def test_chunked_download():
    dev = MockDevice()
    seen = []
    image = CoreDumpClient(dev, progress=lambda r, t: seen.append((r, t))).read_image()
    _ok("image matches", image == IMAGE)
    _ok("chunked at 2048 (3 READs)", dev.reads == 3)
    _ok("progress reaches total", seen and seen[-1] == (len(IMAGE), len(IMAGE)))


def test_offset_mismatch_fails():
    dev = MockDevice()
    dev.corrupt_offset_on_read = 2
    try:
        CoreDumpClient(dev).read_image()
        _ok("offset mismatch raises", False)
    except CoreDumpError as exc:
        _ok("offset mismatch raises", "mismatch" in str(exc))


def test_retry_on_timeout():
    dev = MockDevice()
    c = CoreDumpClient(dev, timeout_ms=30, retries=1)
    # before any reply, correlation support is unknown: a timeout is NOT retried
    dev.drop_first_reply = True
    try:
        c.size()
        _ok("unknown correlation support -> no retry", False)
    except CoreDumpTimeout:
        _ok("unknown correlation support -> no retry", c.correlation_supported is None)
    _ok("first reply establishes support", c.size() == len(IMAGE) and c.correlation_supported)
    dev.drop_first_reply = True  # this GET_SIZE gets no reply; the retry does
    n = c.size()
    _ok("retried after a timeout", n == len(IMAGE))
    dev2 = MockDevice()
    c2 = CoreDumpClient(dev2, timeout_ms=30, retries=0)
    c2.size()  # establish correlation support
    dev2.drop_first_reply = True
    try:
        c2.size()
        _ok("no retries -> timeout raises", False)
    except CoreDumpError as exc:
        _ok("no retries -> timeout raises", "timed out" in str(exc))
        _ok("timeout is the CoreDumpTimeout subclass", isinstance(exc, CoreDumpTimeout))
    # an ERROR reply is final: it must NOT be retried even with retries left
    dev3 = MockDevice()
    dev3.error_on_read = True
    try:
        CoreDumpClient(dev3, timeout_ms=30, retries=3).read_image()
        _ok("ERROR reply is not retried", False)
    except CoreDumpError as exc:
        _ok("ERROR reply is not retried", not isinstance(exc, CoreDumpTimeout) and dev3.reads == 1)


def test_late_reply_after_retry():
    # The reviewer's scenario: the first READ's DATA arrives just after the host
    # timed out and re-sent it, so two identical DATA replies come back; the
    # second must not be taken for the next chunk's reply.
    dev = MockDevice()
    c = CoreDumpClient(dev, timeout_ms=30, retries=1)
    size = c.size()  # establishes correlation support (as the CLI's GET_SIZE does)
    dev.delay_first_reply = True  # the first READ's DATA arrives after the retry
    image = c.read_image(size=size)
    _ok("late DATA duplicate is discarded, image intact", image == IMAGE and dev.reads == 4)
    # same for a non-READ request: a late SIZE must not answer the next GET_SUMMARY
    dev2 = MockDevice()
    c = CoreDumpClient(dev2, timeout_ms=30, retries=1)
    c.size()
    dev2.delay_first_reply = True
    _ok("late SIZE duplicate is discarded", c.size() == len(IMAGE)
        and c.summary().startswith("Guru Meditation"))
    _ok("device echoed correlation ids", c.correlation_supported is True)
    # a device that never echoes the id still works, but is not retried once
    # that is known (a retry could not be told from a late reply)
    dev3 = MockDevice()
    dev3.legacy_no_correlation = True
    c3 = CoreDumpClient(dev3, timeout_ms=30, retries=2)
    _ok("legacy device: plain transactions work",
        c3.read_image() == IMAGE and c3.correlation_supported is False)
    dev3.drop_first_reply = True
    try:
        c3.size()
        _ok("legacy device: no retry after a timeout", False)
    except CoreDumpTimeout:
        _ok("legacy device: no retry after a timeout", True)


def test_error_reply():
    dev = MockDevice()
    dev.error_on_read = True
    try:
        CoreDumpClient(dev).read_image()
        _ok("ERROR reply raises", False)
    except CoreDumpError as exc:
        _ok("ERROR reply raises with code + message", exc.code == 5 and "READ failed" in str(exc))


def test_erase():
    dev = MockDevice()
    CoreDumpClient(dev).erase()
    _ok("erased", dev.erased and CoreDumpClient(dev).size() == 0)


def test_suggested_command_quoting():
    from espp_coredump import decoder
    cmd = decoder.suggested_command("/tmp/my dumps/core.elf", "build/app.elf")
    _ok("paths with spaces are quoted", "'/tmp/my dumps/core.elf'" in cmd or '"/tmp/my dumps/core.elf"' in cmd)
    _ok("sub-command and format present", "info_corefile" in cmd and "--core-format elf" in cmd)
    _ok("gdb variant", "dbg_corefile" in decoder.suggested_command("c", "a", gdb=True))


def test_extract_elf():
    _ok("ELF found behind the 12-byte header", find_elf_offset(IMAGE) == 12)
    _ok("ELF slice runs to the end (checksum kept)", extract_elf(IMAGE) == ELF_BODY + CHECKSUM)
    _ok("no ELF -> None", extract_elf(FLASH_HEADER + b"\x00" * 100) is None)
    _ok("ELF beyond the first KiB is not found",
        extract_elf(b"\x00" * 1100 + b"\x7fELF" + b"\x00" * 10) is None)


def test_discovery():
    dev = MockDevice()
    info = CoreDumpClient(dev).discover(timeout_ms=100)
    _ok("discovery decoded", info is not None and info.device_name == "espp CoreDump"
        and info.firmware == "1.2.3" and len(info.modules) == 2)
    _ok("core dump module advertised", info.has_module(4)
        and info.modules[1].app == "coredump_console.html")
    _ok("truncated TLV is tolerated", P.parse_discovery(F.Frame(0x11, 0xFF, 0, b"\x01\x00", None)) is None)


if __name__ == "__main__":
    test_frame_golden()
    test_size_and_summary()
    test_chunked_download()
    test_offset_mismatch_fails()
    test_retry_on_timeout()
    test_late_reply_after_retry()
    test_error_reply()
    test_erase()
    test_suggested_command_quoting()
    test_extract_elf()
    test_discovery()
    print("all host tests passed")
