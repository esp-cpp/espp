"""WDI (Wheelchair Digital Interface) Python binding test.

Exercises the espp.wdi protocol core (ControlReport / FeedbackReport / HostUuid
serialize+parse, bitfields, enums) -- the Python mirror of
components/wdi/test/wdi_protocol_host_test.cpp. This is also how a WDI *host* (the
wheelchair side) is built/tested from Python: parse the Control reports an
accessory sends, and build the Feedback reports to send back.

Exit code 0 on full pass, 1 on any failure.
"""

import sys

import espp

wdi = espp.wdi

failures = 0


def check(desc: str, condition: bool) -> None:
    global failures
    if condition:
        print(f"  PASS: {desc}")
    else:
        print(f"  FAIL: {desc}")
        failures += 1


print("--- sizes ---")
check("control size 18", wdi.kControlSize == 18)
check("feedback size 19", wdi.kFeedbackSize == 19)
check("keepalive response size 16", wdi.kKeepaliveResponseSize == 16)

print("--- control report round-trip ---")
c = wdi.ControlReport()
c.x = -127
c.y = 100
c.set(wdi.ControlBit.DriveEnable)
c.set(wdi.ControlBit.SpeedUp)
c.vendor1 = 0xDEADBEEF
data = c.serialize()
check("control serializes to 18 bytes", len(data) == 18)
check("x is signed -127", data[0] == 0x81)  # -127 as u8
check("vendor1 little-endian", data[10:14] == b"\xef\xbe\xad\xde")

parsed = wdi.ControlReport.parse(data)
check("control parses", parsed is not None)
if parsed is not None:
    check("parsed x/y", parsed.x == -127 and parsed.y == 100)
    check("parsed DriveEnable", parsed.has(wdi.ControlBit.DriveEnable))
    check("parsed SpeedUp", parsed.has(wdi.ControlBit.SpeedUp))
    check("parsed not Stop", not parsed.has(wdi.ControlBit.Stop))
    check("parsed vendor1", parsed.vendor1 == 0xDEADBEEF)
    check("not a release", not parsed.is_release())

check("release is all-zero", wdi.ControlReport().serialize() == b"\x00" * 18)
check("wrong-size control rejected", wdi.ControlReport.parse(b"\x00" * 17) is None)

print("--- feedback report round-trip ---")
f = wdi.FeedbackReport()
f.set(wdi.FeedbackBit.DriveEnabled)
f.set(wdi.FeedbackBit.LimitedSpeed)
f.speed = 5
f.profile = 2
f.velocity_whole = 3
f.velocity_tenths = 7
f.odometer = 42
fdata = f.serialize()
check("feedback serializes to 19 bytes", len(fdata) == 19)
check("speed/profile nibble packing", fdata[12] == 0x52)
check("velocity nibble packing", fdata[13] == 0x37)
check("odometer byte", fdata[14] == 42)

fp = wdi.FeedbackReport.parse(fdata)
check("feedback parses", fp is not None)
if fp is not None:
    check("parsed DriveEnabled", fp.has(wdi.FeedbackBit.DriveEnabled))
    check("parsed LimitedSpeed", fp.has(wdi.FeedbackBit.LimitedSpeed))
    check("parsed speed/profile", fp.speed == 5 and fp.profile == 2)
    check("velocity mph", abs(fp.velocity_mph() - 3.7) < 0.01)
    check("parsed odometer", fp.odometer == 42)

print("--- host uuid ---")
raw = bytes([0x00, 0x0B]) + bytes(14)  # manufacturer id 0x000B = LUCI, big-endian
u = wdi.HostUuid.parse(raw)
check("host uuid parses", u is not None)
if u is not None:
    check("manufacturer id big-endian", u.manufacturer_id() == 0x000B)
    check("manufacturer id == LUCI", u.manufacturer_id() == int(wdi.ManufacturerId.LuciMobility))
    check("uuid serializes verbatim", u.serialize() == raw)
check("wrong-size uuid rejected", wdi.HostUuid.parse(b"\x00" * 15) is None)

print("--- host round-trip: parse Control, build Feedback (the wheelchair side) ---")
# An accessory drives forward with drive enabled; the "host" parses it and replies.
accessory = wdi.ControlReport()
accessory.y = -100
accessory.set(wdi.ControlBit.DriveEnable)
on_wire = accessory.serialize()
got = wdi.ControlReport.parse(on_wire)
check("host received forward + drive-enable", got is not None and got.y == -100
      and got.has(wdi.ControlBit.DriveEnable))
reply = wdi.FeedbackReport()
reply.set(wdi.FeedbackBit.DriveEnabled)
reply.speed = 4
check("host feedback round-trips", wdi.FeedbackReport.parse(reply.serialize()) is not None)

if failures == 0:
    print("ALL WDI PYTHON BINDING TESTS PASSED")
    sys.exit(0)
print(f"{failures} FAILURE(S)")
sys.exit(1)
