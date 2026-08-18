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
import os
import subprocess
import sys
import time

# The reference legacy fibre (pure python) lives in the ODrive repo at fw-v0.5.1.
FIBRE_REPO = "https://github.com/odriverobotics/ODrive.git"
FIBRE_TAG = "fw-v0.5.1"
FIBRE_SUBPATH = os.path.join("Firmware", "fibre", "python")


def log(msg):
    print("[probe] " + msg, flush=True)


def find_fibre_path(explicit):
    """Return a dir containing the `fibre` package, or None.

    Checks the explicit --fibre-path first, then well-known locations: the clone
    the odrive_native interop harness makes, a clone next to this script, and /tmp.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if explicit:
        candidates.append(explicit)
    candidates += [
        # cloned by components/odrive_native/interop/run.sh (git-ignored):
        os.path.join(here, "..", "..", "odrive_native", "interop", "odrive-ref", *FIBRE_SUBPATH.split(os.sep)),
        os.path.join(here, "odrive-ref", *FIBRE_SUBPATH.split(os.sep)),
        os.path.join(here, "ODrive", *FIBRE_SUBPATH.split(os.sep)),
        os.path.join("/tmp", "odrive-ref", *FIBRE_SUBPATH.split(os.sep)),
    ]
    for c in candidates:
        if os.path.isdir(os.path.join(c, "fibre")):
            return os.path.abspath(c)
    return None


def clone_fibre():
    """Shallow-clone the reference fibre into ./odrive-ref next to this script."""
    here = os.path.dirname(os.path.abspath(__file__))
    dest = os.path.join(here, "odrive-ref")
    log("cloning reference fibre: %s @ %s -> %s" % (FIBRE_REPO, FIBRE_TAG, dest))
    subprocess.check_call([
        "git", "clone", "--depth", "1", "--branch", FIBRE_TAG,
        "--filter=blob:none", "--sparse", FIBRE_REPO, dest,
    ])
    subprocess.check_call(["git", "-C", dest, "sparse-checkout", "set", FIBRE_SUBPATH])
    return os.path.join(dest, *FIBRE_SUBPATH.split(os.sep))


def main():
    ap = argparse.ArgumentParser(
        description="Probe the flashed espp ODrive USB device over its native (Fibre) "
                    "vendor interface using the reference legacy fibre library.")
    ap.add_argument("--fibre-path", default=None,
                    help="path to Firmware/fibre/python (auto-detected if omitted)")
    ap.add_argument("--clone", action="store_true",
                    help="git-clone the reference fibre next to this script if not found")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    fibre_path = find_fibre_path(args.fibre_path)
    if fibre_path is None and args.clone:
        try:
            fibre_path = clone_fibre()
        except Exception as e:
            log("clone failed: %r" % e)
    if fibre_path is None:
        log("reference fibre library not found.")
        log("It is pure-python (only needs pyserial/pyusb). Get it either way:")
        log("  1) re-run with --clone   (clones it next to this script), or")
        log("  2) git clone --depth 1 -b %s %s /tmp/odrive-ref" % (FIBRE_TAG, FIBRE_REPO))
        log("     then: --fibre-path /tmp/odrive-ref/%s" % FIBRE_SUBPATH)
        log("  (if you have run components/odrive_native/interop/run.sh, it is auto-detected)")
        return 3
    sys.path.insert(0, fibre_path)

    try:
        import fibre  # noqa: F401
        from fibre import find_any
        from fibre.utils import Logger
    except Exception as e:
        log("FAILED to import the reference fibre from %s: %r" % (fibre_path, e))
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
