"""espp.Dispatcher / stream_frame example.

Shows how to multiplex several framed protocols over one byte stream: build
frames with espp.stream_frame.build_frame(), then route them to per-module
handlers with an espp.Dispatcher. Mirrors the C++ dispatcher usage (OTA on
module 0, crash-dump on module 4, ...).

Run:  python python/dispatcher.py
"""

import espp
from espp import stream_frame as sf

# --- Two toy protocols sharing one stream --------------------------------
# Module 0: a "control" protocol. Module 4: a "telemetry" protocol.
MODULE_CONTROL = 0
MODULE_TELEMETRY = 4

dispatcher = espp.Dispatcher()


def on_control(frame: sf.Frame) -> None:
    kind = "reply" if frame.is_reply() else "request"
    print(f"[control] {kind} type=0x{frame.type:02X} payload={frame.payload!r}")


def on_telemetry(frame: sf.Frame) -> None:
    # type 0x01 = a little-endian u32 sensor reading
    value = int.from_bytes(frame.payload, "little") if frame.payload else None
    print(f"[telemetry] type=0x{frame.type:02X} value={value}")


dispatcher.register_module(MODULE_CONTROL, on_control)
dispatcher.register_module(MODULE_TELEMETRY, on_telemetry)

# --- Build a mixed stream (as a device/host peer would send it) ----------
stream = b"".join([
    sf.build_frame(module=MODULE_CONTROL, type=0x01, payload=b"start"),
    sf.build_frame(module=MODULE_TELEMETRY, type=0x01, payload=(42).to_bytes(4, "little")),
    # a frame for an unregistered module is silently ignored
    sf.build_frame(module=9, type=0x00, payload=b"ignored"),
    sf.build_frame(module=MODULE_CONTROL, type=0x81, payload=b"ok", reply=True),
])

print(f"feeding {len(stream)} bytes ({MODULE_CONTROL=}, {MODULE_TELEMETRY=})...")
# Feed it in two arbitrary chunks to show the parser reassembles split frames.
half = len(stream) // 2
dispatcher.feed(stream[:half])
dispatcher.feed(stream[half:])
print(f"done ({dispatcher.dropped_bytes()} bytes dropped while resyncing).")
