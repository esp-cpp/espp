"""Host tests for espp_ota: codec round-trips + a full OTA against a mock device.

Runs with plain ``python3`` (no hardware, no pyusb). Also importable by pytest.
"""

import contextlib
import io
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espp_ota import frame as F  # noqa: E402
from espp_ota import protocol as P  # noqa: E402
from espp_ota.client import OtaClient  # noqa: E402
from espp_ota.protocol import MessageType, OtaError  # noqa: E402


class MockDevice:
    """Loopback transport implementing the OTA device side in memory.

    The host writes request frames; ``read()`` returns the device's replies.
    Set ``fail_after`` to make the device answer a DATA frame with ERROR."""

    def __init__(self, fail_after=None, emit_progress=False, module=0, discovery_version=2,
                 protocol=P.PROTOCOL, protocol_version=1, answer_discovery=True):
        self._parser = F.StreamParser()
        self._out = bytearray()
        self.image = bytearray()
        self.received = 0
        self.data_frames = 0
        self.finished = False
        self._fail_after = fail_after
        self._emit_progress = emit_progress
        # the dispatcher module OTA is served on (a routing key: the client is
        # expected to find it through discovery, not assume 0)
        self.module = module
        self.discovery_version = discovery_version  # 1 = pre-protocol-id records
        self.protocol = protocol
        self.protocol_version = protocol_version
        self.answer_discovery = answer_discovery    # False = no Dispatcher discovery at all
        self.discovery_requests = 0
        self.request_modules = set()                # every module id a request arrived on

    def discovery_payload(self):
        """The describe() TLV: a v1 payload (records end after the description)
        or v2 (records add [protocol str][protocol_version u16 LE])."""
        def s(text):
            b = text.encode()
            return bytes([len(b)]) + b

        def record(mid, name, app, desc, proto, pver):
            r = bytes([mid]) + s(name) + s(app) + s(desc)
            if self.discovery_version >= 2:
                r += s(proto) + struct.pack("<H", pver)
            return r
        return (bytes([self.discovery_version, 0]) + s("espp OTA") + s("1.0.0") + bytes([2])
                + record(4, "Core Dump", "coredump_console.html", "Inspect the last crash",
                         "espp.coredump", 1)
                + record(self.module, "OTA", "ota_console.html", "Firmware update",
                         self.protocol, self.protocol_version))

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
        # replies go out on the module OTA is served on (OtaService stamps its
        # Config::module on every reply)
        parsed = F.StreamParser().feed(b)[0]
        self._out += F.build_frame(self.module, parsed.type, parsed.payload, reply=True)

    def _handle(self, fr):
        if fr.module == P.DISCOVERY_MODULE and fr.type == P.DISCOVERY_LIST_MODULES:
            self.discovery_requests += 1
            if self.answer_discovery:
                self._out += F.build_frame(P.DISCOVERY_MODULE, P.DISCOVERY_LIST_MODULES,
                                           self.discovery_payload(), reply=True)
            return
        if fr.module != self.module:
            return
        self.request_modules.add(fr.module)
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


def test_discovery_and_module_resolution():
    dev = MockDevice(module=9)
    info = OtaClient(dev).discover(timeout_ms=100)
    _ok("discover decodes the v2 TLV", info is not None and info.version == 2
        and info.device_name == "espp OTA" and len(info.modules) == 2
        and info.modules[1].protocol == "espp.ota" and info.modules[1].protocol_version == 1
        and info.modules[0].protocol == "espp.coredump")
    v1 = OtaClient(MockDevice(discovery_version=1)).discover(timeout_ms=100)
    _ok("discover decodes a v1 TLV (no protocol fields)", v1 is not None and v1.version == 1
        and v1.modules[1].protocol == "" and v1.modules[1].protocol_version == 0)
    _ok("discover: silent device -> None",
        OtaClient(MockDevice(answer_discovery=False)).discover(timeout_ms=50) is None)
    # forward compatibility: a newer payload version keeps the v2 record
    # layout (records carry no length, so per-record extensions are not
    # possible); only bytes AFTER the whole record list may follow, and they
    # are ignored. Two records + a trailer prove the second record is not
    # desynchronised.
    dev_v3 = MockDevice(module=9, discovery_version=3)
    dev_v3.discovery_payload = (lambda f=dev_v3.discovery_payload:
                                f() + b"\x07future!" + bytes([0xAA, 0xBB, 0xCC]))
    v3 = OtaClient(dev_v3).discover(timeout_ms=100)
    _ok("v3 payload: v2 record layout, trailing bytes after the records ignored",
        v3 is not None and v3.version == 3 and len(v3.modules) == 2
        and v3.modules[0].name == "Core Dump" and v3.modules[1].name == "OTA"
        and v3.modules[1].id == 9 and v3.modules[1].protocol == "espp.ota"
        and v3.modules[1].protocol_version == 1)
    # the probe's timeout bounds the whole thing, the request write included
    slow = MockDevice(answer_discovery=False)
    real_write = slow.write

    def slow_write(data, timeout_ms=0):
        time.sleep(0.08)  # the transport takes most of the budget to write
        real_write(data, timeout_ms)
    slow.write = slow_write
    t0 = time.monotonic()
    silent = OtaClient(slow).discover(timeout_ms=100)
    elapsed = time.monotonic() - t0
    _ok("discover: one timeout bounds write + reply wait",
        silent is None and elapsed < 0.18)
    # the resolution rule (shared with the coredump tool and the consoles)
    ident = dict(protocol="espp.ota", protocol_version=1, app="ota_console.html", name="OTA",
                 fallback=0)
    R = P.resolve_module_id
    r = R(info, **ident)
    _ok("resolve: protocol id -> the served module", r.id == 9 and r.source == "protocol"
        and not r.warnings)
    r = R(v1, **ident)
    _ok("resolve: v1 device found by app filename", r.id == 0 and r.source == "app")
    r = R(None, **ident)
    _ok("resolve: no reply -> default silently", r.id == 0 and r.source == "default"
        and not r.warnings)
    other = P.DiscoveryInfo(2, "d", "1", [P.ModuleInfo(4, "Core Dump", "coredump_console.html",
                                                       "", "espp.coredump", 1)])
    r = R(other, **ident)
    _ok("resolve: nothing matches -> default + warning", r.id == 0 and r.source == "default"
        and len(r.warnings) == 1 and "Core Dump (#4)" in r.warnings[0])
    newer = P.DiscoveryInfo(2, "d", "1", [P.ModuleInfo(0, "OTA", "", "", "espp.ota", 2)])
    r = R(newer, **ident)
    _ok("resolve: protocol version mismatch warns", r.id == 0 and r.protocol_version == 2
        and len(r.warnings) == 1 and "v2" in r.warnings[0] and "v1" in r.warnings[0])
    r = R(info, override=4, **ident)
    _ok("resolve: override wins, other protocol warns", r.id == 4 and r.source == "override"
        and len(r.warnings) == 1 and "espp.coredump" in r.warnings[0])
    two = P.DiscoveryInfo(2, "d", "1", [P.ModuleInfo(1, "A", "", "", "espp.ota", 1),
                                       P.ModuleInfo(2, "B", "", "", "espp.ota", 1)])
    r = R(two, **ident)
    _ok("resolve: several candidates -> first + warning", r.id == 1 and len(r.warnings) == 1
        and "A (#1)" in r.warnings[0] and "B (#2)" in r.warnings[0])
    r = R(P.DiscoveryInfo(3, "d", "1", [P.ModuleInfo(0, "OTA", "", "", "espp.ota", 1)]), **ident)
    _ok("resolve: newer payload version warns", r.id == 0 and len(r.warnings) == 1
        and "v3" in r.warnings[0])


def test_client_adopts_discovered_module():
    # the device serves OTA on module 9: the whole session runs on 9
    dev = MockDevice(module=9)
    image = bytes(range(256)) * 20
    client = OtaClient(dev, chunk_size=1024)
    client.flash(image)
    _ok("flash on the discovered module", dev.finished and bytes(dev.image) == image
        and client.module == 9 and client.resolution.source == "protocol"
        and dev.request_modules == {9} and dev.discovery_requests == 1)
    st = client.get_status()
    _ok("later requests reuse the resolved id (no second probe)",
        st.rollback_supported and dev.discovery_requests == 1)
    # a v1 device (no protocol id) is found by its app filename
    dev = MockDevice(module=9, discovery_version=1)
    client = OtaClient(dev)
    client.get_status()
    _ok("v1 device found by app", client.module == 9 and client.resolution.source == "app")
    # no discovery at all -> the default id after the (short) probe timeout
    dev = MockDevice(answer_discovery=False)
    client = OtaClient(dev, discover_timeout_ms=50)
    client.get_status()
    _ok("silent device -> default id", client.module == 0
        and client.resolution.source == "default" and client.discovered is None)
    # an explicit module skips discovery and is used as given
    dev = MockDevice(module=9)
    client = OtaClient(dev, module=9)
    client.get_status()
    _ok("explicit module: no discovery", dev.discovery_requests == 0 and client.module == 9)
    r = client.resolve_module()
    _ok("explicit module reported as override", r.source == "override" and r.id == 9
        and not r.warnings and client.module == 9)
    try:
        OtaClient(dev, module=0xFF)
        _ok("module 0xFF refused", False)
    except ValueError:
        _ok("module 0xFF refused", True)


def test_cli_module_option():
    import espp_ota.cli as cli
    parser = cli.build_parser()
    _ok("cli: --module parses on every sub-command (hex ok)",
        parser.parse_args(["status", "--module", "0x9"]).module == 9
        and parser.parse_args(["flash", "x.bin", "--module", "3"]).module == 3
        and parser.parse_args(["discover"]).module is None)
    # environment defaults go through the option's own parser: a valid value
    # is adopted, an invalid one is a usage error, not a crash building the parser
    saved = {k: os.environ.pop(k) for k in ("ESPP_OTA_MODULE", "ESPP_OTA_VID")
             if k in os.environ}
    try:
        os.environ["ESPP_OTA_MODULE"] = "0x9"
        os.environ["ESPP_OTA_VID"] = "0x1234"
        ns = cli.build_parser().parse_args(["status"])
        _ok("cli: ESPP_OTA_MODULE / _VID supply defaults", ns.module == 9 and ns.vid == 0x1234)
        ns = cli.build_parser().parse_args(["status", "--module", "3"])
        _ok("cli: --module overrides ESPP_OTA_MODULE", ns.module == 3)
        os.environ["ESPP_OTA_MODULE"] = "nine"
        try:
            parser = cli.build_parser()  # must not raise
            with contextlib.redirect_stderr(io.StringIO()) as err:
                parser.parse_args(["status"])
            usage_error = False
        except SystemExit as exc:
            usage_error = exc.code == 2 and "--module" in err.getvalue() \
                and "nine" in err.getvalue()
        _ok("cli: an invalid ESPP_OTA_MODULE is a usage error naming --module", usage_error)
        os.environ["ESPP_OTA_MODULE"] = ""
        _ok("cli: an empty ESPP_OTA_MODULE means unset",
            cli.build_parser().parse_args(["status"]).module is None)
    finally:
        for k in ("ESPP_OTA_MODULE", "ESPP_OTA_VID"):
            os.environ.pop(k, None)
        os.environ.update(saved)


def test_transport_timeout_classification():
    from espp_ota.transport import UsbVendorTransport

    class USBError(Exception):
        def __init__(self, errno=None, backend_error_code=None):
            super().__init__("usb")
            self.errno = errno
            self.backend_error_code = backend_error_code

    class USBTimeoutError(USBError):
        pass

    Core = type("Core", (), {"USBError": USBError, "USBTimeoutError": USBTimeoutError})
    OldCore = type("OldCore", (), {"USBError": USBError})  # pyusb < 1.1: no USBTimeoutError
    is_timeout = UsbVendorTransport.is_usb_timeout
    _ok("transport: pyusb's USBTimeoutError is a timeout", is_timeout(Core, USBTimeoutError()))
    import errno as _errno
    _ok("transport: this platform's ETIMEDOUT is a timeout",
        is_timeout(Core, USBError(errno=_errno.ETIMEDOUT)))
    wsa = getattr(_errno, "WSAETIMEDOUT", None)
    _ok("transport: WSAETIMEDOUT counts only where it exists (Windows)",
        wsa is None or is_timeout(Core, USBError(errno=wsa)))
    foreign = [e for e in (110, 60, 10060) if e not in (_errno.ETIMEDOUT, wsa)]
    _ok("transport: another OS's ETIMEDOUT number is not a timeout here",
        all(not is_timeout(Core, USBError(errno=e)) for e in foreign))
    _ok("transport: libusb LIBUSB_ERROR_TIMEOUT is a timeout",
        is_timeout(Core, USBError(backend_error_code=-7)))
    _ok("transport: other errors are not timeouts",
        not is_timeout(Core, USBError(errno=5)) and not is_timeout(Core, USBError()))
    _ok("transport: old pyusb without USBTimeoutError still classifies by errno",
        is_timeout(OldCore, USBError(errno=_errno.ETIMEDOUT))
        and not is_timeout(OldCore, USBError(errno=19)))


def test_component_loader():
    """components/ota/idf_ext.py (what idf.py imports) loads the package from
    its path without touching sys.path."""
    import importlib.util

    loader_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), "idf_ext.py")
    spec = importlib.util.spec_from_file_location("idf_ext_ota_test", loader_path)
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    before = list(sys.path)
    ext = loader.action_extensions({}, "/proj")
    _ok("loader: registers ota-usb", "ota-usb" in ext.get("actions", {}))
    _ok("loader: leaves sys.path alone", sys.path == before)
    _ok("loader: reuses an already imported package",
        loader._load_package() is sys.modules["espp_ota"])


def test_idf_extension():
    import json
    import tempfile

    from espp_ota import idf_ext as X

    ext = X.action_extensions({}, "/proj")
    action = ext["actions"][X.ACTION_NAME]
    names = {n for opt in action["options"] for n in opt["names"]}
    _ok("idf_ext registers ota-usb with its options",
        action["callback"] is not None and action["dependencies"] == ["all"]
        and {"--binary", "--chunk-size", "--no-verify", "--verify-timeout", "--quiet",
             "--status", "--mark-valid", "--rollback", "--vid", "--pid", "--serial",
             "--interface", "--module"} <= names)
    _ok("idf_ext declares the extension version", "version" in ext)
    _ok("idf_ext skips a second registration",
        X.action_extensions({"actions": {X.ACTION_NAME: {}}}, "/proj") == {})

    with tempfile.TemporaryDirectory() as build_dir:
        try:
            X.project_bin(build_dir)
            _ok("idf_ext: unconfigured build dir is a FatalError", False)
        except X.FatalError:
            _ok("idf_ext: unconfigured build dir is a FatalError", True)
        desc_path = os.path.join(build_dir, "project_description.json")
        with open(desc_path, "w", encoding="utf-8") as f:
            f.write("{not json")
        try:
            X.project_bin(build_dir)
            _ok("idf_ext: a corrupt project description is a FatalError", False)
        except X.FatalError as exc:
            _ok("idf_ext: a corrupt project description is a FatalError",
                "project_description.json" in str(exc))
        with open(desc_path, "w", encoding="utf-8") as f:
            json.dump({"build_dir": build_dir, "app_bin": "my_app.bin"}, f)
        binary = os.path.join(build_dir, "my_app.bin")
        _ok("idf_ext: .bin from project_description.json", X.project_bin(build_dir) == binary)
        _ok("idf_ext: argv for a plain flash",
            X.build_tool_argv(build_dir) == ["flash", binary])
        _ok("idf_ext: argv for flash with every option + device ids",
            X.build_tool_argv(build_dir, binary="o.bin", chunk_size="2048", no_verify=True,
                              verify_timeout="5", quiet=True, vid="0x1209", pid="0x1234",
                              serial="S1", interface="2")
            == ["flash", "o.bin", "--vid", "0x1209", "--pid", "0x1234", "--serial", "S1",
                "--interface", "2", "--chunk-size", "2048", "--no-verify",
                "--verify-timeout", "5", "--quiet"])
        _ok("idf_ext: mode flags run the sub-command instead (no .bin needed)",
            X.build_tool_argv("/nonexistent", status=True, pid="-1") == ["status", "--pid", "-1"]
            and X.build_tool_argv("/nonexistent", mark_valid=True) == ["mark-valid"]
            and X.build_tool_argv("/nonexistent", rollback=True, serial="S1")
            == ["rollback", "--serial", "S1"])
        _ok("idf_ext: --module is forwarded with the device options",
            X.build_tool_argv("/nonexistent", status=True, module="9") == ["status", "--module", "9"]
            and X.build_tool_argv(build_dir, module="0x9") == ["flash", binary, "--module", "0x9"])
        try:
            X.build_tool_argv(build_dir, status=True, rollback=True)
            _ok("idf_ext: two mode flags are refused", False)
        except X.FatalError as exc:
            _ok("idf_ext: two mode flags are refused", "mutually exclusive" in str(exc))
        # what the action builds must be what the CLI accepts
        import espp_ota.cli as cli
        parser = cli.build_parser()
        ns = parser.parse_args(X.build_tool_argv(build_dir, chunk_size="2048", no_verify=True,
                                                 verify_timeout="5", quiet=True, pid="0x1234",
                                                 serial="S1"))
        _ok("idf_ext: CLI parses the flash argv",
            ns.binary == binary and ns.chunk_size == 2048 and ns.no_verify and ns.quiet
            and ns.serial == "S1")
        ns = parser.parse_args(X.build_tool_argv("/nonexistent", rollback=True, pid="-1"))
        _ok("idf_ext: CLI parses a mode argv", ns.pid == -1)

        # the action callback drives the CLI in-process and maps a non-zero exit to FatalError
        calls = []
        real_main = cli.main
        cli.main = lambda argv=None: (calls.append(list(argv)), 0)[1]
        try:
            class Args:
                pass

            args = Args()
            args.build_dir = build_dir
            action["callback"]("ota-usb", None, args, no_verify=True)
            _ok("idf_ext: callback flashes the project .bin",
                calls == [["flash", binary, "--no-verify"]])
            calls.clear()
            action["callback"]("ota-usb", None, args, mark_valid=True, pid="0x1234")
            _ok("idf_ext: callback runs a mode sub-command with the device ids",
                calls == [["mark-valid", "--pid", "0x1234"]])
            cli.main = lambda argv=None: 1
            failed = False
            try:
                action["callback"]("ota-usb", None, args)
            except X.FatalError:
                failed = True  # the tool's non-zero exit is the expected failure
            _ok("idf_ext: non-zero tool exit is a FatalError", failed)

            def _exit(code):
                raise SystemExit(code)

            cli.main = lambda argv=None: _exit(0)
            action["callback"]("ota-usb", None, args)
            _ok("idf_ext: SystemExit(0) from the CLI is a clean run", True)
            cli.main = lambda argv=None: _exit(2)
            try:
                action["callback"]("ota-usb", None, args)
                _ok("idf_ext: SystemExit(2) from the CLI is a FatalError", False)
            except X.FatalError as exc:
                _ok("idf_ext: SystemExit(2) from the CLI is a FatalError", "status 2" in str(exc))
            cli.main = lambda argv=None: None
            action["callback"]("ota-usb", None, args)
            _ok("idf_ext: a None return counts as success", True)
        finally:
            cli.main = real_main


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
    test_discovery_and_module_resolution()
    test_client_adopts_discovered_module()
    test_cli_module_option()
    test_transport_timeout_classification()
    test_component_loader()
    test_idf_extension()
    print("all host tests passed")
