"""OdriveNative Python-binding test.

In-process loopback between the bound C++ protocol server (espp.OdriveNative,
the same wire core the firmware runs) and the pure-python espp_odrive client
helpers (components/odrive_native/python) -- the packets built by the client
codec are fed straight into the server's process_bytes(), which is exactly
what travels over USB/UART on hardware. Also freezes the CRC goldens and the
UART stream framing through the bindings.

Exit code 0 on full pass, 1 on any failure.
"""

import json
import os
import struct
import sys
from typing import List, Tuple

import espp

# The pure-python ODrive client codec (espp_odrive) ships alongside the espp
# package in the wheel / installed prefix; fall back to its source-of-truth
# location in the odrive_native component for source-tree runs.
try:
    import espp_odrive  # noqa: F401
except ImportError:
    sys.path.insert(
        0,
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "components",
                     "odrive_native", "python"))
from espp_odrive import PROTOCOL_VERSION, crc8, crc16  # noqa: E402
from espp_odrive.protocol import build_packet, parse_response  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

results: List[Tuple[str, bool]] = []


def check(test: str, condition: bool, desc: str) -> bool:
    if condition:
        print(f"  PASS [{test}]: {desc}")
    else:
        print(f"  FAIL [{test}]: {desc}")
    results.append((test, condition))
    return condition


# ---------------------------------------------------------------------------
# 1. CRC goldens: bindings vs the pure-python client codec
# ---------------------------------------------------------------------------
print("1. CRC goldens (bindings vs espp_odrive)")
check("crc", espp.odrive_crc16(b"123456789") == 0xAA01, "crc16 golden 0xaa01")
check("crc", espp.odrive_crc8(b"123456789") == 0x8C, "crc8 golden 0x8c")
check("crc", espp.odrive_crc16(b"123456789") == crc16(b"123456789"),
      "crc16 matches espp_odrive.crc16")
check("crc", espp.odrive_crc8(b"123456789") == crc8(b"123456789"),
      "crc8 matches espp_odrive.crc8")

# ---------------------------------------------------------------------------
# 2. Server construction + property registration from Python
# ---------------------------------------------------------------------------
print("2. Server + registration")
state = {"vbus": 24.0, "pos": 0.0, "error": 0, "locked": 7.0}

dev = espp.OdriveNative()
dev.register_float_property("vbus_voltage", lambda: state["vbus"])
dev.register_float_property(
    "axis0.controller.input_pos", lambda: state["pos"],
    lambda v: state.__setitem__("pos", v) or True)
dev.register_uint32_property("axis0.error", lambda: state["error"])
# a setter that REJECTS every write (returns False)
dev.register_float_property("axis0.locked", lambda: state["locked"], lambda v: False)

descriptor = dev.json()
jcrc = dev.json_crc()
check("reg", jcrc != 0, f"json_crc nonzero (0x{jcrc:04x})")
check("reg", jcrc == crc16(descriptor.encode("ascii"), PROTOCOL_VERSION),
      "json_crc == espp_odrive crc16(json, init=PROTOCOL_VERSION)")

tree = json.loads(descriptor)
check("reg", isinstance(tree, list) and len(tree) > 0, "descriptor parses as JSON")


def find_endpoint(nodes, dotted):
    parts = dotted.split(".")
    for part in parts[:-1]:
        nodes = next(n for n in nodes if n["name"] == part)["members"]
    return next(n for n in nodes if n["name"] == parts[-1])


ep_pos = find_endpoint(tree, "axis0.controller.input_pos")
ep_locked = find_endpoint(tree, "axis0.locked")
check("reg", ep_pos["access"] == "rw", "input_pos advertises rw")

# ---------------------------------------------------------------------------
# 3. Endpoint-0 chunked JSON download through the wire path
# ---------------------------------------------------------------------------
print("3. Endpoint-0 JSON download")
blob = b""
for _ in range(64):
    req = build_packet(1, 0, 512, struct.pack("<I", len(blob)), True, jcrc)
    resp = dev.process_bytes(req)
    seq_no, data = parse_response(resp)
    if not data:
        break
    blob += data
check("ep0", blob.decode("ascii") == descriptor, "downloaded JSON == dev.json()")

# ---------------------------------------------------------------------------
# 4. Typed write-then-read via the client codec
# ---------------------------------------------------------------------------
print("4. Typed read/write")
wrote = 3.14159
req = build_packet(2, ep_pos["id"], 0, struct.pack("<f", wrote), True, jcrc)
resp = dev.process_bytes(req)
check("rw", len(resp) == 2, "write gets a bare 2-byte ack")
check("rw", abs(state["pos"] - wrote) < 1e-6, "python setter saw the value")

req = build_packet(3, ep_pos["id"], 4, b"", True, jcrc)
seq_no, data = parse_response(dev.process_bytes(req))
check("rw", seq_no == (3 | 0x8000), "response echoes seq | 0x8000")
check("rw", abs(struct.unpack("<f", data)[0] - wrote) < 1e-6, "read returns written value")

# rejected write: setter returns False -> value unchanged
req = build_packet(4, ep_locked["id"], 0, struct.pack("<f", 99.0), True, jcrc)
dev.process_bytes(req)
check("rw", state["locked"] == 7.0, "rejecting setter leaves value unchanged")

# bad canary -> request ignored entirely
req = build_packet(5, ep_pos["id"], 4, b"", True, jcrc ^ 0xFFFF)
check("rw", dev.process_bytes(req) == b"", "canary mismatch is ignored")

# unknown endpoint -> ignored even with the expect-response bit
req = build_packet(6, 999, 4, b"", True, jcrc)
check("rw", dev.process_bytes(req) == b"", "unknown endpoint is ignored")

# ---------------------------------------------------------------------------
# 5. UART stream framing helpers
# ---------------------------------------------------------------------------
print("5. Stream framing")
pkt = build_packet(7, ep_pos["id"], 4, b"", True, jcrc)
framed = espp.odrive_stream_frame(pkt)
check("stream", len(framed) == len(pkt) + 5, "frame adds sync+len+crc8+crc16")
deframer = espp.OdriveStreamDeframer()
mid = len(framed) // 2
out = deframer.push(framed[:mid])
check("stream", out == [], "partial frame yields nothing")
out = deframer.push(framed[mid:])
check("stream", out == [pkt], "completed frame yields the original packet")
check("stream", deframer.buffered() == 0, "deframer drained")
check("stream", espp.odrive_stream_frame(b"\x00" * 200) == b"",
      "oversize packet (>127) refused")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
failed = [t for t, ok in results if not ok]
print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
if failed:
    print("FAILED:", ", ".join(sorted(set(failed))))
    sys.exit(1)
print("ODRIVE_NATIVE PYTHON BINDINGS: ALL PASSED")
sys.exit(0)
