"""Dispatcher / stream_frame Python binding test.

Exercises the espp.stream_frame codec (build_frame / StreamParser / crc32 /
flags) and espp.Dispatcher (module routing, register/unregister, reset) — the
Python mirror of components/{stream_frame,dispatcher}/test.

Exit code 0 on full pass, 1 on any failure.
"""

import sys

import espp

sf = espp.stream_frame

failures = 0


def check(desc: str, condition: bool) -> None:
    global failures
    if condition:
        print(f"  PASS: {desc}")
    else:
        print(f"  FAIL: {desc}")
        failures += 1


# ---------------------------------------------------------------------------
# stream_frame codec
# ---------------------------------------------------------------------------
print("--- stream_frame codec ---")
check("crc32 golden", sf.crc32(b"123456789") == 0xCBF43926)
check("crc32 chaining", sf.crc32(b"6789", sf.crc32(b"12345")) == 0xCBF43926)
check("make_flags/flags_is_reply", (not sf.flags_is_reply(sf.make_flags(False)))
      and sf.flags_is_reply(sf.make_flags(True)))
check("version in flags", sf.flags_version(sf.make_flags(True)) == sf.VERSION)

# Encode a request and a reply, parse them back.
req = sf.build_frame(module=4, type=0x42, payload=b"\x01\x02\x03")
rep = sf.build_frame(module=4, type=0xC2, payload=b"", reply=True)
check("encoded frame magic 'TO'", req[0] == 0x54 and req[1] == 0x4F)
check("build_frame rejects oversized payload",
      sf.build_frame(module=0, type=0, payload=b"\x00" * (sf.MAX_PAYLOAD_SIZE + 1)) == b"")

parser = sf.StreamParser()
frames = parser.feed(req + rep)
check("parsed both frames", len(frames) == 2)
if len(frames) == 2:
    check("frame[0] module/type/reply", frames[0].module == 4 and frames[0].type == 0x42
          and not frames[0].is_reply())
    check("frame[0] payload", frames[0].payload == b"\x01\x02\x03")
    check("frame[1] reply flag", frames[1].module == 4 and frames[1].type == 0xC2
          and frames[1].is_reply())
check("parser drained", parser.buffered() == 0 and parser.dropped_bytes() == 0)

# Optional correlation id (flag-gated header field).
plain = sf.build_frame(module=4, type=0x10, payload=b"\x01")
withc = sf.build_frame(module=4, type=0xC0, payload=b"\x01", reply=True, correlation=0xBEEF)
check("no-correlation frame is unchanged size", len(plain) == sf.HEADER_SIZE + 1 + sf.CRC_SIZE)
check("correlation grows the header", len(withc) == sf.HEADER_SIZE + sf.CORRELATION_SIZE + 1 + sf.CRC_SIZE)
cframes = sf.StreamParser().feed(plain + withc)
check("both parse", len(cframes) == 2)
if len(cframes) == 2:
    check("plain has no correlation", cframes[0].correlation is None and not cframes[0].has_correlation())
    check("correlated id round-trips", cframes[1].correlation == 0xBEEF and cframes[1].has_correlation())

# Split delivery (one byte at a time) still yields the frames.
parser2 = sf.StreamParser()
got = []
for b in req:
    got += parser2.feed(bytes([b]))
check("split-across-bytes round-trip", len(got) == 1 and got[0].type == 0x42)

# ---------------------------------------------------------------------------
# Dispatcher routing
# ---------------------------------------------------------------------------
print("--- Dispatcher routing ---")
d = espp.Dispatcher()
seen = {0: [], 4: [], 200: []}
d.register_module(0, lambda f: seen[0].append(f.type))
d.register_module(4, lambda f: seen[4].append(f.type))
# a full-byte module id well beyond the old nibble range
d.register_module(200, lambda f: seen[200].append(f.type))
check("has_module", d.has_module(0) and d.has_module(4) and d.has_module(200)
      and not d.has_module(7))

stream = (
    sf.build_frame(module=0, type=0x01)
    + sf.build_frame(module=4, type=0x42, payload=b"\xaa")
    + sf.build_frame(module=7, type=0x01)  # unregistered -> ignored
    + sf.build_frame(module=200, type=0x99)
    + sf.build_frame(module=0, type=0x02)
)
d.feed(stream)
check("module 0 routed twice", seen[0] == [0x01, 0x02])
check("module 4 routed once", seen[4] == [0x42])
check("module 200 (full byte) routed", seen[200] == [0x99])
check("unregistered module ignored", d.dropped_bytes() == 0)

# unregister stops routing
d.unregister_module(4)
seen[4].clear()
d.feed(sf.build_frame(module=4, type=0x42))
check("unregister_module stops routing", seen[4] == [] and not d.has_module(4))

# reset() drops a partial frame so a stale prefix can't stitch onto later bytes
d2 = espp.Dispatcher()
count = []
d2.register_module(0, lambda f: count.append(1))
frame = sf.build_frame(module=0, type=0x03)
d2.feed(frame[:3])  # header prefix only
check("partial frame buffered", d2.buffered() > 0)
d2.reset()
d2.feed(frame[3:])  # remainder alone
check("reset() dropped the split frame", count == [] and d2.buffered() == 0)

# module_of static helper
f = sf.StreamParser().feed(sf.build_frame(module=5, type=0x50))[0]
check("module_of(frame)", espp.Dispatcher.module_of(f) == 5)

# register_module(id, None) unregisters (a callback can't be None otherwise)
d3 = espp.Dispatcher()
d3.register_module(1, lambda f: None)
d3.register_module(1, None)
check("register_module(id, None) unregisters", not d3.has_module(1))

# a handler may retain the frame: it must be an independent Python-owned copy,
# not a wrapper over the C++ Frame that only lives for the dispatch call.
kept = []
d3.register_module(2, lambda f: kept.append(f))
d3.feed(sf.build_frame(module=2, type=0x01, payload=b"hi"))
d3.feed(sf.build_frame(module=2, type=0x02, payload=b"other"))  # churns feed()'s vector
check("retained frame survives later feeds", kept and kept[0].payload == b"hi")

# a handler may re-entrantly unregister its own module mid-dispatch (the handler
# copy must not be destroyed under the running call).
d4 = espp.Dispatcher()
hits = []


def self_unregister(frame):
    hits.append(frame.type)
    d4.register_module(6, None)


d4.register_module(6, self_unregister)
d4.feed(sf.build_frame(module=6, type=0x11) + sf.build_frame(module=6, type=0x22))
check("re-entrant self-unregister is safe", hits == [0x11] and not d4.has_module(6))

# ---------------------------------------------------------------------------
print()
if failures == 0:
    print("ALL TESTS PASSED")
    sys.exit(0)
print(f"{failures} FAILURE(S)")
sys.exit(1)
