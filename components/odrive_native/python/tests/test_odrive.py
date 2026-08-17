#!/usr/bin/env python3
"""End-to-end + CRC self-test for the espp_odrive native client.

Runs two things:

1. **CRC self-test** -- the golden vector ``crc16("123456789", init=0x1337) ==
   0xaa01`` plus a stream-frame round-trip through the deframer.
2. **End-to-end interop** -- builds the C++ device shim, spawns it on a PTY,
   connects THIS client (not the reference fibre package), enumerates the tree,
   reads ``vbus_voltage``/``serial_number``, and write-then-reads
   ``axis0.controller.input_pos`` and ``axis0.controller.config.vel_limit``.

Runnable directly (``python3 tests/test_odrive.py``); also exposes ``test_*``
functions for pytest if it happens to be available.
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)                    # .../python
COMPONENT = os.path.dirname(PKG_ROOT)               # .../odrive_native
INCLUDE = os.path.join(COMPONENT, "include")
DEVICE_SRC = os.path.join(COMPONENT, "interop", "odrive_native_interop_device.cpp")

sys.path.insert(0, PKG_ROOT)

from espp_odrive import connect  # noqa: E402
from espp_odrive.crc import crc16, crc8, CRC16_INIT, CRC8_INIT  # noqa: E402
from espp_odrive.transport import StreamDeframer, stream_frame  # noqa: E402


def log(msg):
    print("[test] " + msg, flush=True)


# --------------------------------------------------------------------------- #
# 1. CRC self-test
# --------------------------------------------------------------------------- #
def test_crc_golden():
    got = crc16(b"123456789", CRC16_INIT)
    assert got == 0xAA01, "crc16('123456789', 0x1337) = 0x%04x, expected 0xaa01" % got
    # CRC8 init/self-consistency: crc8 over [sync,len,crc8] is 0.
    hdr = bytes([0xAA, 5])
    hdr += bytes([crc8(hdr, CRC8_INIT)])
    assert crc8(hdr, CRC8_INIT) == 0, "crc8 header self-check failed"
    log("CRC self-test PASSED (crc16('123456789')=0x%04x)" % got)


def test_stream_roundtrip():
    packet = bytes(range(20))
    framed = stream_frame(packet)
    deframer = StreamDeframer()
    # Feed it in two arbitrary splits + some leading garbage to test resync.
    out = deframer.push(b"\x00\x01" + framed[:3])
    out += deframer.push(framed[3:])
    assert out == [packet], "stream round-trip failed: %r" % out
    log("stream frame/deframe round-trip PASSED")


# --------------------------------------------------------------------------- #
# 2. End-to-end interop against the device shim
# --------------------------------------------------------------------------- #
def _build_device(workdir):
    out = os.path.join(workdir, "odrive_native_device")
    cxx = os.environ.get("CXX", "c++")
    cmd = [cxx, "-std=c++20", "-I", INCLUDE, DEVICE_SRC, "-o", out]
    log("building device shim: " + " ".join(cmd))
    subprocess.run(cmd, check=True)
    return out


def _spawn_device(device_bin):
    proc = subprocess.Popen([device_bin], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, bufsize=1, text=True)
    pty = None
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                break
            continue
        m = re.match(r"PTY_SLAVE (\S+)", line.strip())
        if m:
            pty = m.group(1)
            break
    return proc, pty


def test_end_to_end():
    workdir = os.environ.get("TMPDIR", "/tmp")
    device_bin = _build_device(workdir)
    proc, pty = _spawn_device(device_bin)
    try:
        assert pty, "device shim did not report a PTY slave"
        log("device PTY slave = %s" % pty)

        dev = connect(pty, timeout=5.0)
        log("CONNECTED. json_crc = 0x%04x, descriptor = %d bytes"
            % (dev.json_crc, len(dev._json_bytes)))

        # Enumerate the tree.
        log("endpoint tree:")
        tree = dev.dump()
        props = set(re.findall(r"^\s*(.*?) = ", tree, re.M))  # leaf names only (indented)

        # Read values.
        vbus = dev.vbus_voltage
        log("READ  vbus_voltage = %r" % vbus)
        assert abs(vbus - 24.37) < 1e-3, "vbus mismatch: %r" % vbus

        sn = dev.serial_number
        log("READ  serial_number = 0x%X" % sn)
        assert sn == 0x00A1B2C3D4E5, "serial_number mismatch: 0x%X" % sn

        err = dev.axis0.error
        log("READ  axis0.error = %r" % err)
        assert err == 0

        # Write then read back (attribute style).
        dev.axis0.controller.input_pos = 3.14159
        rb = dev.axis0.controller.input_pos
        log("WRITE/READ axis0.controller.input_pos = %r" % rb)
        assert abs(rb - 3.14159) < 1e-4, "input_pos read-back mismatch: %r" % rb

        # Write then read back (get/set path style).
        dev.set("axis0.controller.config.vel_limit", 42.5)
        vlim = dev.get("axis0.controller.config.vel_limit")
        log("WRITE/READ axis0.controller.config.vel_limit = %r" % vlim)
        assert abs(vlim - 42.5) < 1e-4, "vel_limit read-back mismatch: %r" % vlim

        log("ALL END-TO-END ASSERTIONS PASSED (espp_odrive client <-> espp device)")
        dev.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()


def main():
    rc = 0
    for fn in (test_crc_golden, test_stream_roundtrip, test_end_to_end):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            log("FAILED: %s: %s" % (fn.__name__, e))
            rc = 1
    print("\n==================== SUMMARY ====================")
    print("RESULT: %s" % ("PASS" if rc == 0 else "FAIL"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
