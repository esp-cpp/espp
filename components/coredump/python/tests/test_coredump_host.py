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
    """Check + report one condition. A failure is an AssertionError (so pytest
    reports it normally); the standalone runner below turns it into exit 1."""
    print(("PASS" if cond else "FAIL"), name)
    assert cond, name


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
    _ok("ELF starting at the last offset of the first KiB is found (magic may end past it)",
        find_elf_offset(b"\x00" * 1023 + b"\x7fELF" + b"\x00" * 10) == 1023)
    _ok("ELF starting just past the first KiB is not found",
        find_elf_offset(b"\x00" * 1024 + b"\x7fELF" + b"\x00" * 10) is None)


def test_discovery():
    dev = MockDevice()
    write_timeouts = []
    real_write = dev.write
    dev.write = lambda data, timeout_ms=0: (write_timeouts.append(timeout_ms), real_write(data, timeout_ms))[1]
    info = CoreDumpClient(dev).discover(timeout_ms=100)
    _ok("discovery request is written with the caller's timeout", write_timeouts == [100])
    _ok("discovery decoded", info is not None and info.device_name == "espp CoreDump"
        and info.firmware == "1.2.3" and len(info.modules) == 2)
    _ok("core dump module advertised", info.has_module(4)
        and info.modules[1].app == "coredump_console.html")
    _ok("truncated TLV is tolerated", P.parse_discovery(F.Frame(0x11, 0xFF, 0, b"\x01\x00", None)) is None)


def test_streaming_download():
    dev = MockDevice()
    chunks = []
    n = CoreDumpClient(dev).read_image_to(chunks.append)
    _ok("streamed in READ-sized chunks", n == len(IMAGE) and len(chunks) == dev.reads == 3
        and b"".join(chunks) == IMAGE)
    _ok("no dump -> sink never called",
        CoreDumpClient(MockDevice(image=b"")).read_image_to(chunks.append) == 0 and len(chunks) == 3)


def test_summary_panel():
    from espp_coredump import ui

    report = ("last reset: POWERON (1)\n"
              "crashed task: 'main' PC=0x4202068a\n"
              "backtrace: 0x4202068a 0x42024d0a\n"
              "decode with: xtensa-esp32s3-elf-addr2line -pfiaC -e build/<app>.elf <addrs>\n")
    lines = report.rstrip("\n").split("\n")
    _ok("panel: crash-report lines get distinct styles",
        ui.report_line_style(lines[1]) == ("bold red", "1;31")
        and ui.report_line_style(lines[2]) == ("bold yellow", "1;33")
        and ui.report_line_style("something else") == (None, None))
    plain = ui.render_panel_plain("Core dump summary", lines)
    rows = plain.split("\n")
    _ok("panel: framed with the title, one row per line, equal width",
        rows[0].startswith("╭─ Core dump summary ") and rows[0].endswith("╮")
        and rows[-1].startswith("╰") and rows[-1].endswith("╯")
        and len(rows) == len(lines) + 2 and len(set(len(r) for r in rows)) == 1
        and all(r.startswith("│ ") and r.endswith(" │") for r in rows[1:-1]))
    colored = ui.render_panel_plain("t", lines, color=True)
    _ok("panel: ANSI styling only when asked", "\033[1;31m" in colored and "\033[" not in plain)
    narrow_rows = ui.render_panel_plain("t", lines, width=30).split("\n")
    _ok("panel: a narrow frame wraps long lines and keeps every row the same width",
        len(set(len(r) for r in narrow_rows)) == 1 and len(narrow_rows[0]) == 30
        and len(narrow_rows) > len(lines) + 2
        and all(r.startswith("│ ") and r.endswith(" │") for r in narrow_rows[1:-1])
        and "".join(narrow_rows[1:-1]).count("PC=0x4202068a") == 1)
    tiny_rows = ui.render_panel_plain("a title", lines, width=5).split("\n")
    _ok("panel: width cap never shrinks below the title",
        len(set(len(r) for r in tiny_rows)) == 1 and "a title" in tiny_rows[0])
    ascii_rows = ui.render_panel_plain("t", lines, ascii_only=True).split("\n")
    _ok("panel: ASCII frame for streams that cannot encode box characters",
        all(ord(c) < 128 for c in "".join(ascii_rows)) and ascii_rows[0].startswith("+- t ")
        and len(set(len(r) for r in ascii_rows)) == 1)

    class Stream:
        def __init__(self, encoding):
            self.encoding = encoding

    _ok("panel: encoding probe", ui._stream_can_encode(Stream("utf-8"), "╭")
        and not ui._stream_can_encode(Stream("ascii"), "╭")
        and not ui._stream_can_encode(Stream(None), "╭")
        and not ui._stream_can_encode(Stream("no-such-codec"), "╭"))


class FakeTransport(MockDevice):
    """MockDevice as the CLI's transport: context manager + description."""

    description = "mock 0x1209:0x0d36"

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _cli_args(**kw):
    import argparse
    base = dict(vid=0x1209, pid=0x0d36, serial=None, interface=None, quiet=True,
                chunk_size=2048, timeout=5000, out=None, gdb=False, erase=False)
    base.update(kw)
    return argparse.Namespace(**base)


def test_transport_timeout_classification():
    from espp_coredump.transport import UsbVendorTransport

    class USBError(Exception):
        def __init__(self, errno=None, backend_error_code=None):
            super().__init__("usb")
            self.errno = errno
            self.backend_error_code = backend_error_code

    class USBTimeoutError(USBError):
        pass

    class Core:  # a pyusb `usb.core` stand-in
        pass

    Core.USBError = USBError
    Core.USBTimeoutError = USBTimeoutError
    is_timeout = UsbVendorTransport.is_usb_timeout
    _ok("transport: pyusb's USBTimeoutError is a timeout", is_timeout(Core, USBTimeoutError()))
    _ok("transport: ETIMEDOUT on Linux / macOS / Windows is a timeout",
        all(is_timeout(Core, USBError(errno=e)) for e in (110, 60, 10060)))
    _ok("transport: libusb LIBUSB_ERROR_TIMEOUT is a timeout",
        is_timeout(Core, USBError(backend_error_code=-7)))
    _ok("transport: other errors are not timeouts",
        not is_timeout(Core, USBError(errno=5)) and not is_timeout(Core, USBError()))

    OldCore = type("OldCore", (), {"USBError": USBError})  # pyusb < 1.1: no USBTimeoutError
    _ok("transport: old pyusb without USBTimeoutError still classifies by errno",
        is_timeout(OldCore, USBError(errno=60)) and not is_timeout(OldCore, USBError(errno=19)))


def test_cli_erase_same_connection():
    import tempfile

    from espp_coredump import cli, decoder

    real_transport, real_decoder = cli._make_transport, decoder.run_decoder
    try:
        # summary --erase: erased on the connection the report came from
        dev = FakeTransport()
        cli._make_transport = lambda args: dev
        rc = cli._cmd_summary(_cli_args(erase=True))
        _ok("cli: summary --erase erases after the report", rc == 0 and dev.erased)
        # nothing recorded -> nothing to erase, even with --erase
        dev = FakeTransport(summary="")
        cli._make_transport = lambda args: dev
        rc = cli._cmd_summary(_cli_args(erase=True))
        _ok("cli: summary --erase on a clean device erases nothing", rc == 0 and not dev.erased)
        # debug --erase: the core file is on disk before the erase, on the same connection
        with tempfile.TemporaryDirectory() as d:
            app_elf = os.path.join(d, "app.elf")
            with open(app_elf, "wb") as f:
                f.write(b"\x7fELF")
            core = os.path.join(d, "core.elf")
            decoder.run_decoder = lambda *a, **k: 0
            dev = FakeTransport()
            cli._make_transport = lambda args: dev
            rc = cli._cmd_debug(_cli_args(app_elf=app_elf, out=core, erase=True))
            with open(core, "rb") as f:
                saved = f.read()
            _ok("cli: debug --erase saves the core file, decodes, then erases",
                rc == 0 and dev.erased and saved == ELF_BODY + CHECKSUM)
            # a failed decode, or no decoder at all, leaves the dump on the device
            decoder.run_decoder = lambda *a, **k: 1
            dev = FakeTransport()
            cli._make_transport = lambda args: dev
            rc = cli._cmd_debug(_cli_args(app_elf=app_elf, out=core, erase=True))
            _ok("cli: debug --erase does not erase after a failed decode", rc == 1 and not dev.erased)
            decoder.run_decoder = lambda *a, **k: -1
            dev = FakeTransport()
            cli._make_transport = lambda args: dev
            rc = cli._cmd_debug(_cli_args(app_elf=app_elf, out=core, erase=True))
            _ok("cli: debug --erase does not erase when no decoder is available",
                rc == 1 and not dev.erased)
            # no dump stored: nothing saved, nothing erased, exit 1
            dev = FakeTransport(image=b"")
            cli._make_transport = lambda args: dev
            rc = cli._cmd_debug(_cli_args(app_elf=app_elf, out=os.path.join(d, "x.elf"),
                                          erase=True))
            _ok("cli: debug --erase with no dump erases nothing", rc == 1 and not dev.erased)
    finally:
        cli._make_transport, decoder.run_decoder = real_transport, real_decoder


def test_idf_extension():
    import json
    import tempfile

    from espp_coredump import idf_ext as X

    ext = X.action_extensions({}, "/proj")
    action = ext["actions"][X.ACTION_NAME]
    names = {n for opt in action["options"] for n in opt["names"]}
    _ok("idf_ext registers coredump-usb with its options",
        action["callback"] is not None and action["dependencies"] == ["all"]
        and {"--gdb", "--summary", "--erase", "--out", "--vid", "--pid", "--serial",
             "--interface"} <= names)
    _ok("idf_ext declares the extension version", "version" in ext)
    _ok("idf_ext skips a second registration",
        X.action_extensions({"actions": {X.ACTION_NAME: {}}}, "/proj") == {})

    with tempfile.TemporaryDirectory() as build_dir:
        try:
            X.project_elf(build_dir)
            _ok("idf_ext: unconfigured build dir is a FatalError", False)
        except X.FatalError:
            _ok("idf_ext: unconfigured build dir is a FatalError", True)
        desc_path = os.path.join(build_dir, "project_description.json")
        with open(desc_path, "w", encoding="utf-8") as f:
            f.write("{not json")
        try:
            X.project_elf(build_dir)
            _ok("idf_ext: a corrupt project description is a FatalError", False)
        except X.FatalError as exc:
            _ok("idf_ext: a corrupt project description is a FatalError",
                "project_description.json" in str(exc))
        with open(desc_path, "w", encoding="utf-8") as f:
            json.dump({"build_dir": build_dir, "app_elf": "my_app.elf"}, f)
        elf = os.path.join(build_dir, "my_app.elf")
        _ok("idf_ext: ELF from project_description.json", X.project_elf(build_dir) == elf)
        core = os.path.join(build_dir, "core.elf")
        _ok("idf_ext: argv for decode saves the core file in the build dir",
            X.build_tool_argv(build_dir) == ["debug", elf, "--out", core])
        _ok("idf_ext: argv for gdb + out + device ids",
            X.build_tool_argv(build_dir, gdb=True, out="c.elf", vid="0x1209", pid="0x1234",
                              serial="S1", interface="2")
            == ["--vid", "0x1209", "--pid", "0x1234", "--serial", "S1", "--interface", "2",
                "debug", elf, "--out", "c.elf", "--gdb"])
        _ok("idf_ext: argv for summary needs no ELF",
            X.build_tool_argv(build_dir, summary=True, pid="-1") == ["--pid", "-1", "summary"])
        try:
            X.build_tool_argv(build_dir, summary=True, gdb=True)
            _ok("idf_ext: --summary with --gdb is refused", False)
        except X.FatalError as exc:
            _ok("idf_ext: --summary with --gdb is refused", "mutually exclusive" in str(exc))

        # the action callback drives the CLI in-process and maps a non-zero exit to FatalError
        calls = []
        import espp_coredump.cli as cli

        real_main = cli.main
        cli.main = lambda argv=None: (calls.append(list(argv)), 0)[1]
        try:
            class Args:
                pass

            args = Args()
            args.build_dir = build_dir
            action["callback"]("coredump-usb", None, args, gdb=True)
            _ok("idf_ext: callback runs the tool with the project ELF",
                calls == [["debug", elf, "--out", core, "--gdb"]])
            calls.clear()
            action["callback"]("coredump-usb", None, args, summary=True, erase=True, pid="0x1234")
            _ok("idf_ext: --erase is one tool run (same connection), not a second device pick",
                calls == [["--pid", "0x1234", "summary", "--erase"]])
            calls.clear()
            action["callback"]("coredump-usb", None, args, erase=True)
            _ok("idf_ext: --erase with a decode is passed to `debug`",
                calls == [["debug", elf, "--out", core, "--erase"]])
            cli.main = lambda argv=None: 1
            try:
                action["callback"]("coredump-usb", None, args)
                _ok("idf_ext: non-zero tool exit is a FatalError", False)
            except X.FatalError:
                _ok("idf_ext: non-zero tool exit is a FatalError", True)

            def _exit(code):
                raise SystemExit(code)

            cli.main = lambda argv=None: _exit(0)
            action["callback"]("coredump-usb", None, args)
            _ok("idf_ext: SystemExit(0) from the CLI is a clean run", True)
            cli.main = lambda argv=None: _exit(2)
            try:
                action["callback"]("coredump-usb", None, args)
                _ok("idf_ext: SystemExit(2) from the CLI is a FatalError", False)
            except X.FatalError as exc:
                _ok("idf_ext: SystemExit(2) from the CLI is a FatalError", "status 2" in str(exc))
            cli.main = lambda argv=None: None
            action["callback"]("coredump-usb", None, args)
            _ok("idf_ext: a None return counts as success", True)
        finally:
            cli.main = real_main


if __name__ == "__main__":
    tests = [
        test_frame_golden,
        test_size_and_summary,
        test_chunked_download,
        test_streaming_download,
        test_offset_mismatch_fails,
        test_retry_on_timeout,
        test_late_reply_after_retry,
        test_error_reply,
        test_erase,
        test_suggested_command_quoting,
        test_extract_elf,
        test_discovery,
        test_summary_panel,
        test_transport_timeout_classification,
        test_cli_erase_same_connection,
        test_idf_extension,
    ]
    try:
        for t in tests:
            t()
    except AssertionError as exc:
        print("FAILED:", exc)
        raise SystemExit(1)
    print("all host tests passed")
