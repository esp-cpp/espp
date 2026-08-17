#!/usr/bin/env python3
"""Hardware probe: talk to the flashed espp ODrive USB example over its native
(Fibre) *vendor* interface, using the GENUINE legacy pure-python ``fibre`` library
from ODrive ``fw-v0.5.1`` -- the exact reference the espp ``odrive_native`` codec
was built against, and the code odrivetool uses.

This is the USB sibling of ``components/odrive_native/interop/odrive_fibre_client.py``
(which uses a serial/PTY loopback): here we discover the real board over USB, download
endpoint 0, enumerate the object tree, read values, and write-then-read-back.

Prereqs (host):
  - libusb (macOS: ``brew install libusb``; Linux: ``apt install libusb-1.0-0``)
  - a venv with ``pyusb`` + ``appdirs`` (the reference fibre USB backend uses pyusb)
  - the reference fibre package (pass ``--fibre-path <ODrive>/Firmware/fibre/python``;
    the interop harness already clones it to
    ``components/odrive_native/interop/odrive-ref/Firmware/fibre/python``)
  - on Linux you may need a udev rule / sudo to claim the vendor interface.

Usage:
  python odrive_usb_probe.py \
      --fibre-path ../../odrive_native/interop/odrive-ref/Firmware/fibre/python
"""
import argparse
import sys
import time


def log(msg):
    print("[probe] " + msg, flush=True)


def main():
    ap = argparse.ArgumentParser()
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
    except Exception as e:
        log("FAILED to import the reference fibre library: %r" % e)
        log("Provide it with --fibre-path <ODrive>/Firmware/fibre/python")
        return 3

    log("fibre reference library: %s" % fibre.__file__)
    log("discovering over USB (find_any(path='usb', timeout=%ss))..." % args.timeout)
    log("(if this hangs: check libusb is installed and the board isn't held by another")
    log(" process; on Linux you may need sudo / a udev rule to claim the vendor interface)")

    dev = find_any(path="usb", timeout=args.timeout, logger=Logger(verbose=False))
    if dev is None:
        log("FAILED: no ODrive-like USB device discovered within %ss" % args.timeout)
        return 1

    log("CONNECTED over USB. Endpoint tree downloaded from endpoint 0:")

    def walk(members, prefix=""):
        out = []
        for m in members:
            name = m.get("name")
            full = (prefix + "." + name) if prefix else name
            if m.get("type") == "object":
                out.append((full, "object", None))
                out.extend(walk(m.get("members", []), full))
            else:
                out.append((full, m.get("type"), m.get("access", "")))
        return out

    tree = walk(dev.__dict__.get("_json_data", []))
    for full, typ, access in tree:
        log("  %-40s %s" % (full, "(object)" if typ == "object" else "%-8s %s" % (typ, access)))

    props = {full for (full, typ, _a) in tree if typ != "object"}
    required = {
        "vbus_voltage", "axis0.error",
        "axis0.encoder.pos_estimate", "axis0.encoder.vel_estimate",
        "axis0.controller.input_pos", "axis0.controller.config.vel_limit",
        "serial_number",
    }
    missing = required - props
    if missing:
        log("FAILED: endpoint tree missing %s" % sorted(missing))
        return 1
    log("endpoint tree contains all %d expected properties" % len(required))

    def get(path):
        obj = dev
        for p in path.split(".")[:-1]:
            obj = getattr(obj, p)
        return getattr(obj, path.split(".")[-1])

    def set_(path, value):
        obj = dev
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        setattr(obj, parts[-1], value)

    vbus = get("vbus_voltage")
    log("READ  vbus_voltage = %r" % vbus)
    if abs(vbus - 24.0) > 0.5:
        log("FAILED: vbus_voltage expected ~24.0, got %r" % vbus)
        return 1
    log("READ  serial_number = 0x%X" % get("serial_number"))
    log("READ  axis0.error = %r" % get("axis0.error"))
    log("READ  axis0.encoder.vel_estimate = %r (animated on the device)" %
        get("axis0.encoder.vel_estimate"))

    for path, val in (("axis0.controller.input_pos", 3.14159),
                      ("axis0.controller.config.vel_limit", 42.5)):
        log("WRITE %s <- %r" % (path, val))
        set_(path, val)
        time.sleep(0.1)
        rb = get(path)
        log("READ  %s = %r" % (path, rb))
        if abs(rb - val) > 1e-4:
            log("FAILED: %s read-back %r != written %r" % (path, rb, val))
            return 1

    log("ALL PROBE ASSERTIONS PASSED (real fibre client <-> flashed espp board over USB)")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except Exception:
        import traceback
        traceback.print_exc()
        rc = 2
    sys.exit(rc)
