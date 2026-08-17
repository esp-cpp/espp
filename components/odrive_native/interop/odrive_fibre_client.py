#!/usr/bin/env python3
"""Real-tool interop CLIENT for the espp odrive_native device shim.

This drives the GENUINE legacy pure-python ``fibre`` library shipped in the ODrive
firmware (``odriverobotics/ODrive`` @ ``fw-v0.5.1``,
``Firmware/fibre/python/fibre``) -- the exact reference implementation the espp
``OdriveNativeCore`` wire codec was built against. It connects to the device shim
over a serial port / PTY, downloads endpoint 0, enumerates the object tree, reads a
value, and writes-then-reads-back a value, asserting each step.

Exit 0 on success, non-zero + diagnostics on failure.

Usage:
    odrive_fibre_client.py <serial_port> [--fibre-path DIR] [--timeout SECONDS]
"""
import argparse
import struct
import sys
import time


def log(msg):
    print("[client] " + msg, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="serial port / PTY slave path the device shim printed")
    ap.add_argument("--fibre-path", default=None,
                    help="path to Firmware/fibre/python (the legacy fibre package)")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    if args.fibre_path:
        sys.path.insert(0, args.fibre_path)

    try:
        import fibre  # noqa: F401
        from fibre import find_any
        from fibre.utils import Logger
    except Exception as e:  # pragma: no cover - environment issue
        log("FAILED to import the reference fibre library: %r" % e)
        log("Provide it with --fibre-path <ODrive>/Firmware/fibre/python")
        return 3

    log("fibre reference library: %s" % fibre.__file__)
    # A serial: path spec makes fibre's serial backend scan for a port whose name
    # matches the (regex-anchored) path -- our PTY slave, e.g. /dev/ttys011.
    path_spec = "serial:" + args.port
    log("connecting via find_any(path=%r, timeout=%ss)..." % (path_spec, args.timeout))

    dev = find_any(path=path_spec, timeout=args.timeout, logger=Logger(verbose=False))
    if dev is None:
        log("FAILED: no device discovered on %s within %ss" % (args.port, args.timeout))
        return 1

    log("CONNECTED. Enumerating endpoint tree downloaded from endpoint 0:")

    # Walk the endpoint-0 JSON member tree that fibre downloaded + parsed.
    def walk(members, prefix=""):
        names = []
        for m in members:
            name = m.get("name")
            full = (prefix + "." + name) if prefix else name
            if m.get("type") == "object":
                names.append((full, "object", None))
                names.extend(walk(m.get("members", []), full))
            else:
                names.append((full, m.get("type"), m.get("access", "")))
        return names

    tree = walk(dev.__dict__.get("_json_data", []))
    for full, typ, access in tree:
        if typ == "object":
            log("  %-40s (object)" % full)
        else:
            log("  %-40s %-8s %s" % (full, typ, access))

    props = {full for (full, typ, _a) in tree if typ != "object"}
    required = {
        "vbus_voltage",
        "axis0.error",
        "axis0.controller.input_pos",
        "axis0.controller.config.vel_limit",
        "serial_number",
    }
    missing = required - props
    if missing:
        log("FAILED: endpoint tree is missing %s" % sorted(missing))
        return 1
    log("endpoint tree contains all %d expected properties" % len(required))

    def get(path):
        obj = dev
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        return getattr(obj, parts[-1])

    def set_(path, value):
        obj = dev
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        setattr(obj, parts[-1], value)

    # 1) Read a value.
    vbus = get("vbus_voltage")
    log("READ  vbus_voltage = %r" % vbus)
    if abs(vbus - 24.37) > 1e-3:
        log("FAILED: vbus_voltage expected ~24.37, got %r" % vbus)
        return 1

    sn = get("serial_number")
    log("READ  serial_number = 0x%X" % sn)
    if sn != 0x00A1B2C3D4E5:
        log("FAILED: serial_number mismatch, got 0x%X" % sn)
        return 1

    err = get("axis0.error")
    log("READ  axis0.error = %r" % err)

    # 2) Write then read back a value.
    new_pos = 3.14159
    log("WRITE axis0.controller.input_pos <- %r" % new_pos)
    set_("axis0.controller.input_pos", new_pos)
    time.sleep(0.1)
    readback = get("axis0.controller.input_pos")
    log("READ  axis0.controller.input_pos = %r" % readback)
    if abs(readback - new_pos) > 1e-4:
        log("FAILED: input_pos read-back %r != written %r" % (readback, new_pos))
        return 1

    # 3) Write-then-read a second rw property to be thorough.
    new_vlim = 42.5
    log("WRITE axis0.controller.config.vel_limit <- %r" % new_vlim)
    set_("axis0.controller.config.vel_limit", new_vlim)
    time.sleep(0.1)
    vlim = get("axis0.controller.config.vel_limit")
    log("READ  axis0.controller.config.vel_limit = %r" % vlim)
    if abs(vlim - new_vlim) > 1e-4:
        log("FAILED: vel_limit read-back %r != written %r" % (vlim, new_vlim))
        return 1

    log("ALL INTEROP ASSERTIONS PASSED (real fibre client <-> espp device)")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except Exception:
        import traceback
        traceback.print_exc()
        rc = 2
    sys.exit(rc)
