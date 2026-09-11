"""Command-line interface: ``python -m espp_ota <command>``.

Commands:
  flash <binary>   BEGIN -> stream DATA -> FINISH an image over USB.
  list             List matching USB devices.
  discover         Probe the device (dispatcher ListModules) and report reply.

VID/PID default to the espp UsbDevice default (0x1209:0x0d32) but can be
overridden (also via the ESPP_OTA_VID / ESPP_OTA_PID env vars, which the CMake
``ota-usb`` target forwards).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

from . import __version__, ui
from .client import OtaClient
from .protocol import OtaError
from .transport import DEFAULT_PID, DEFAULT_VID, TransportError, UsbVendorTransport, list_devices

CON = ui.Console()


def _auto_int(text: str) -> int:
    return int(text, 0)  # accepts 0x1209, 4617, etc.


def _human_size(n: int) -> str:
    size = float(n)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if size < 1024 or unit == "GiB":
            return f"{int(size)} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return f"{n} B"


def _env_int(name: str, default: int) -> int:
    val = os.environ.get(name)
    return _auto_int(val) if val else default


def _add_device_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--vid", type=_auto_int, default=_env_int("ESPP_OTA_VID", DEFAULT_VID),
                   help="USB vendor id (default 0x%04x)" % DEFAULT_VID)
    p.add_argument("--pid", type=_auto_int, default=_env_int("ESPP_OTA_PID", DEFAULT_PID),
                   help="USB product id (default 0x%04x; pass -1 to match any)" % DEFAULT_PID)
    p.add_argument("--serial", default=os.environ.get("ESPP_OTA_SERIAL"),
                   help="match a specific device serial number")
    p.add_argument("--interface", type=_auto_int, default=None,
                   help="force a specific vendor interface number")


def _make_transport(args) -> UsbVendorTransport:
    """Build an (unopened) transport; use it as a context manager (`with`)."""
    pid = None if args.pid is not None and args.pid < 0 else args.pid
    return UsbVendorTransport(vid=args.vid, pid=pid, serial=args.serial,
                              interface=args.interface)


def _reconnect(args, timeout_s: float = 20.0):
    """Reopen the device after it reboots (same VID/PID, re-enumerates). Retries
    until it appears or the timeout elapses; returns the opened transport or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            return _make_transport(args)
        except TransportError:
            time.sleep(0.5)
    return None


def _cmd_flash(args) -> int:
    with open(args.binary, "rb") as fh:
        image = fh.read()
    if not image:
        CON.error("image is empty")
        return 2
    size = 0 if args.unknown_size else len(image)

    before = None  # firmware running before the flash
    with _make_transport(args) as t:
        if not args.quiet:
            CON.note(f"● Connected to {t.description}")
        client = OtaClient(t)
        try:
            before = client.get_status()
            if not args.quiet:
                CON.info(f"  Currently running: {before.firmware_str()}")
        except OtaError:
            pass  # older device without GET_STATUS; carry on
        if not args.quiet:
            CON.info(f"  Flashing {args.binary} ({_human_size(len(image))})")
        start = time.monotonic()
        with ui.Progress(len(image), label="Flashing", quiet=args.quiet) as prog:
            client = OtaClient(
                t,
                chunk_size=args.chunk_size,
                progress=prog.update,
                begin_timeout_ms=args.begin_timeout,
                data_timeout_ms=args.data_timeout,
                finish_timeout_ms=args.finish_timeout,
            )
            client.flash(image, image_size=size)
        if not args.quiet:
            dt = time.monotonic() - start
            rate = len(image) / dt / 1024 if dt else 0
            CON.success(f"OTA complete in {dt:.1f}s ({rate:.0f} KiB/s). "
                        f"The device is activating the new image and rebooting.")

    if args.no_verify:
        CON.info("Auto-verify disabled. Once you have checked the device, confirm the "
                 "image with `espp-ota mark-valid` (or it rolls back on the next reset).")
        return 0
    return _auto_verify(args, before)


def _auto_verify(args, before) -> int:
    """Reconnect after the reboot and confirm the running image can respond before
    marking it valid. This is the safety check: a pending-verify image that can
    answer OTA commands has booted, so the host confirms it."""
    CON.info("Waiting for the device to reboot and reconnect…")
    time.sleep(2.0)  # give it a moment to drop off the bus before we scan
    t = _reconnect(args, timeout_s=args.verify_timeout)
    if t is None:
        CON.warn("device did not reappear after the reboot. If it booted correctly, "
                 "confirm the image with `espp-ota mark-valid`; otherwise it will roll "
                 "back on the next reset.")
        return 1
    with t:
        client = OtaClient(t)
        try:
            st = client.get_status()
        except OtaError as exc:
            CON.error(f"reconnected but the device did not report status: {exc}. "
                      "Not confirming — it will roll back on the next reset.")
            return 1
        # Report the transition.
        if before is not None:
            CON.note(f"● {before.firmware_str()}  →  {st.firmware_str()}")
        else:
            CON.note(f"● Now running: {st.firmware_str()}")
        if not st.rollback_supported:
            CON.success("Update complete (device has no rollback; nothing to confirm).")
            return 0
        if not st.pending_verify:
            CON.success("Update complete; the running image is already confirmed.")
            return 0
        # It responded to OTA commands AND is pending verify -> it booted; confirm.
        client.mark_valid()
        CON.success("The new image booted and responded, so it has been marked valid "
                    "(rollback cancelled).")
    return 0


def _cmd_list(args) -> int:
    pid = None if args.pid is not None and args.pid < 0 else args.pid
    found = list_devices(vid=args.vid, pid=pid)
    if not found:
        CON.warn("no matching USB devices found")
        return 1
    for vid, pid_, desc in found:
        print(f"0x{vid:04x}:0x{pid_:04x}  {desc}")
    return 0


def _cmd_discover(args) -> int:
    with _make_transport(args) as t:
        frames = OtaClient(t).discover(timeout_ms=args.timeout)
        if not frames:
            CON.warn("no discovery reply (device may not run a Dispatcher on the "
                     "vendor interface)")
            return 1
        for fr in frames:
            CON.info(f"reply module=0x{fr.module:02x} type=0x{fr.type:02x} "
                     f"reply={fr.is_reply} payload={len(fr.payload)} bytes")
    return 0


def _cmd_status(args) -> int:
    with _make_transport(args) as t:
        st = OtaClient(t).get_status()
    CON.note(f"● Running: {st.firmware_str()}")
    if not st.rollback_supported:
        CON.info("rollback: not supported (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE off)")
    elif st.pending_verify:
        CON.warn("running image is PENDING VERIFY — confirm it with `mark-valid` "
                 "(or it rolls back on the next reset)")
    else:
        CON.success("running image is confirmed (not pending verify)")
    return 0


def _cmd_mark_valid(args) -> int:
    with _make_transport(args) as t:
        client = OtaClient(t)
        try:
            fw = client.get_status().firmware_str()
        except OtaError:
            fw = None
        client.mark_valid()
    CON.success("running image marked valid; rollback cancelled"
                + (f" ({fw})" if fw else ""))
    return 0


def _cmd_rollback(args) -> int:
    with _make_transport(args) as t:
        OtaClient(t).mark_invalid()
    CON.success("device rolling back to the previous image and rebooting")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="espp_ota", description=__doc__.split("\n")[0])
    p.add_argument("--version", action="version", version=f"espp_ota {__version__}")
    sub = p.add_subparsers(dest="command", required=True)

    f = sub.add_parser("flash", help="OTA-update a binary over USB")
    f.add_argument("binary", help="path to the app .bin to flash")
    _add_device_args(f)
    f.add_argument("--chunk-size", type=_auto_int, default=4096,
                   help="DATA payload bytes per frame (1..4096, default 4096)")
    f.add_argument("--unknown-size", action="store_true",
                   help="stream with size 0 (device erases the whole partition)")
    f.add_argument("--begin-timeout", type=int, default=60000, help="ms (default 60000)")
    f.add_argument("--data-timeout", type=int, default=5000, help="ms (default 5000)")
    f.add_argument("--finish-timeout", type=int, default=60000, help="ms (default 60000)")
    f.add_argument("-q", "--quiet", action="store_true", help="suppress progress output")
    f.add_argument("--no-verify", action="store_true",
                   help="don't auto-verify: skip the reconnect + mark-valid after reboot")
    f.add_argument("--verify-timeout", type=float, default=20.0,
                   help="seconds to wait for the device to reappear after reboot (default 20)")
    f.set_defaults(func=_cmd_flash)

    lst = sub.add_parser("list", help="list matching USB devices")
    _add_device_args(lst)
    lst.set_defaults(func=_cmd_list)

    d = sub.add_parser("discover", help="probe the device's dispatcher (ListModules)")
    _add_device_args(d)
    d.add_argument("--timeout", type=int, default=2000, help="ms (default 2000)")
    d.set_defaults(func=_cmd_discover)

    st = sub.add_parser("status", help="query rollback status (is the image pending verify?)")
    _add_device_args(st)
    st.set_defaults(func=_cmd_status)

    mv = sub.add_parser("mark-valid",
                        help="confirm the running image (cancel rollback) after verifying it")
    _add_device_args(mv)
    mv.set_defaults(func=_cmd_mark_valid)

    rb = sub.add_parser("rollback", help="reject the running image: roll back + reboot")
    _add_device_args(rb)
    rb.set_defaults(func=_cmd_rollback)
    return p


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (OtaError, TransportError) as exc:
        CON.error(exc)
        return 1
    except FileNotFoundError as exc:
        CON.error(exc)
        return 2
    except OSError as exc:
        # pyusb's USBError derives from OSError/IOError, so routine USB failures
        # (unplug mid-flash, permission denied, missing libusb backend) land here
        # instead of raising an ugly traceback. Report them cleanly.
        CON.error(exc)
        return 1
    except KeyboardInterrupt:
        CON.warn("interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
