"""Command-line interface: ``python -m espp_coredump <command>``.

Commands:
  list             List matching USB devices.
  discover         Probe the device (dispatcher ListModules) and list its modules.
  summary          Print the device's crash report (the last core dump's summary).
  size             Print the stored core-dump image size (0 = none).
  download         Download the core dump to a file (core.elf by default).
  erase            Erase the stored core dump.
  debug <app.elf>  Download the core dump and decode it against the app ELF
                   (esp-coredump info_corefile; --gdb opens GDB on it instead).

VID/PID default to the coredump example's ids (0x1209:0x0d36) but can be
overridden (also via the ESPP_COREDUMP_VID / ESPP_COREDUMP_PID env vars, which
the CMake ``coredump-usb`` target forwards).
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

from . import __version__, decoder, ui
from .client import DEFAULT_CHUNK, CoreDumpClient
from .elf import extract_elf
from .protocol import MAX_READ_LENGTH, MODULE, CoreDumpError
from .transport import DEFAULT_PID, DEFAULT_VID, TransportError, UsbVendorTransport, list_devices

CON = ui.Console()

DEFAULT_ELF_OUT = "core.elf"
DEFAULT_RAW_OUT = "coredump_raw.bin"


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
    p.add_argument("--vid", type=_auto_int, default=_env_int("ESPP_COREDUMP_VID", DEFAULT_VID),
                   help="USB vendor id (default 0x%04x)" % DEFAULT_VID)
    p.add_argument("--pid", type=_auto_int, default=_env_int("ESPP_COREDUMP_PID", DEFAULT_PID),
                   help="USB product id (default 0x%04x; pass -1 to match any)" % DEFAULT_PID)
    p.add_argument("--serial", default=os.environ.get("ESPP_COREDUMP_SERIAL"),
                   help="match a specific device serial number")
    p.add_argument("--interface", type=_auto_int, default=None,
                   help="force a specific vendor interface number")


def _add_transfer_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--chunk-size", type=_auto_int, default=DEFAULT_CHUNK,
                   help=f"READ bytes per frame (1..{MAX_READ_LENGTH}, default {DEFAULT_CHUNK})")
    p.add_argument("--timeout", type=int, default=5000, help="per-request reply timeout, ms (default 5000)")
    p.add_argument("--retries", type=int, default=2,
                   help="re-send a request this many times on a reply timeout (default 2)")
    p.add_argument("-q", "--quiet", action="store_true", help="suppress progress output")


def _make_transport(args) -> UsbVendorTransport:
    """Build an (unopened) transport; use it as a context manager (`with`)."""
    pid = None if args.pid is not None and args.pid < 0 else args.pid
    return UsbVendorTransport(vid=args.vid, pid=pid, serial=args.serial,
                              interface=args.interface)


def _make_client(args, t, progress=None) -> CoreDumpClient:
    return CoreDumpClient(
        t,
        chunk_size=getattr(args, "chunk_size", DEFAULT_CHUNK),
        progress=progress,
        timeout_ms=getattr(args, "timeout", 5000),
        retries=getattr(args, "retries", 2),
    )


# -- the download itself, shared by `download` and `debug` --------------------
def _download(args, t) -> bytes:
    """Fetch the stored image with a progress bar; returns b"" when there is none."""
    size = _make_client(args, t).size()
    if size == 0:
        return b""
    if not args.quiet:
        CON.info(f"  Core dump: {_human_size(size)}")
    with ui.Progress(size, label="Downloading", quiet=args.quiet) as prog:
        return _make_client(args, t, progress=prog.update).read_image(size=size)


def _save_image(image: bytes, out: Optional[str], raw: bool) -> tuple:
    """Write the ELF (or the raw image) to disk. Returns (path, core_format)."""
    elf = None if raw else extract_elf(image)
    if elf is not None:
        path = out or DEFAULT_ELF_OUT
        data, fmt = elf, "elf"
    else:
        path = out or DEFAULT_RAW_OUT
        data, fmt = image, "raw"
    with open(path, "wb") as fh:
        fh.write(data)
    return path, fmt


# -- commands -----------------------------------------------------------------
def _cmd_list(args) -> int:
    pid = None if args.pid is not None and args.pid < 0 else args.pid
    found = list_devices(vid=args.vid, pid=pid, serial=args.serial)
    if not found:
        CON.warn("no matching USB devices found")
        return 1
    for vid, pid_, desc, sn in found:
        print(f"0x{vid:04x}:0x{pid_:04x}  {desc}" + (f"  serial={sn}" if sn else ""))
    return 0


def _cmd_discover(args) -> int:
    with _make_transport(args) as t:
        info = _make_client(args, t).discover(timeout_ms=args.discover_timeout)
    if info is None:
        CON.warn("no discovery reply (device may not run a Dispatcher on the vendor interface)")
        return 1
    CON.note(f"● {info.device_name or '(unnamed device)'}"
             + (f"  firmware {info.firmware}" if info.firmware else ""))
    for m in info.modules:
        print(f"  module {m.id:3d}  {m.name:<24} {m.app:<28} {m.description}")
    if not info.has_module(MODULE):
        CON.warn(f"the device does not advertise the Core Dump module (id {MODULE})")
        return 1
    return 0


def _cmd_summary(args) -> int:
    with _make_transport(args) as t:
        text = _make_client(args, t).summary()
    if not text:
        CON.success("no crash recorded (clean boot history)")
        return 0
    print(text.rstrip("\n"))
    return 0


def _cmd_size(args) -> int:
    with _make_transport(args) as t:
        n = _make_client(args, t).size()
    print(n)  # always the number on stdout (0 = no core dump), for scripts
    if n == 0:
        print("no core dump stored", file=sys.stderr)
    return 0


def _cmd_download(args) -> int:
    with _make_transport(args) as t:
        if not args.quiet:
            CON.note(f"● Connected to {t.description}")
        image = _download(args, t)
    if not image:
        CON.warn("no core dump stored on the device")
        return 1
    path, fmt = _save_image(image, args.out, args.raw)
    if fmt == "elf":
        CON.success(f"saved ELF core file to {path} ({_human_size(os.path.getsize(path))})")
    elif args.raw:
        CON.success(f"saved raw core-dump image to {path} ({_human_size(len(image))})")
    else:
        CON.warn(f"no ELF magic in the image (binary core-dump format?); saved the raw image to "
                 f"{path}. Decode it with: esp-coredump info_corefile --core {path} "
                 f"--core-format raw <app.elf>")
    return 0


def _cmd_erase(args) -> int:
    if not args.yes:
        try:
            answer = input("Erase the stored core dump? [y/N] ")
        except EOFError:
            answer = ""
        if answer.strip().lower() not in ("y", "yes"):
            CON.info("not erased")
            return 1
    with _make_transport(args) as t:
        _make_client(args, t).erase()
    CON.success("core dump erased")
    return 0


def _cmd_debug(args) -> int:
    if not os.path.isfile(args.app_elf):
        raise FileNotFoundError(f"app ELF not found: {args.app_elf}")
    with _make_transport(args) as t:
        if not args.quiet:
            CON.note(f"● Connected to {t.description}")
        image = _download(args, t)
    if not image:
        CON.warn("no core dump stored on the device (nothing to debug)")
        return 1
    path, fmt = _save_image(image, args.out, raw=False)
    if not args.quiet:
        CON.info(f"  Core file: {path} ({fmt})")
    rc = decoder.run_decoder(path, args.app_elf, gdb=args.gdb, core_format=fmt)
    if rc == -1:
        CON.warn("no core-dump decoder found (`pip install esp-coredump`, or set IDF_PATH). "
                 "Run it yourself:")
        print("  " + decoder.suggested_command(path, args.app_elf, gdb=args.gdb, core_format=fmt))
        return 1
    return rc


def build_parser() -> argparse.ArgumentParser:
    # `python -OO` strips docstrings (__doc__ is None): fall back to a fixed line
    description = (__doc__ or "Read an espp device's core dump over USB.").splitlines()[0]
    p = argparse.ArgumentParser(prog="espp_coredump", description=description)
    p.add_argument("--version", action="version", version=f"espp_coredump {__version__}")
    sub = p.add_subparsers(dest="command", required=True)

    lst = sub.add_parser("list", help="list matching USB devices")
    _add_device_args(lst)
    lst.set_defaults(func=_cmd_list)

    d = sub.add_parser("discover", help="probe the device's dispatcher (ListModules)")
    _add_device_args(d)
    d.add_argument("--discover-timeout", type=int, default=2000, help="ms (default 2000)")
    d.set_defaults(func=_cmd_discover)

    s = sub.add_parser("summary", help="print the crash report of the stored core dump")
    _add_device_args(s)
    s.set_defaults(func=_cmd_summary)

    z = sub.add_parser("size", help="print the stored core-dump image size in bytes")
    _add_device_args(z)
    z.set_defaults(func=_cmd_size)

    dl = sub.add_parser("download", help="download the core dump (core.elf by default)")
    _add_device_args(dl)
    _add_transfer_args(dl)
    dl.add_argument("--out", default=None,
                    help=f"output path (default {DEFAULT_ELF_OUT}, or {DEFAULT_RAW_OUT} "
                         "when the image holds no ELF)")
    dl.add_argument("--raw", action="store_true",
                    help="keep the flash image as-is (header + ELF + checksum) instead of "
                         "extracting the ELF")
    dl.set_defaults(func=_cmd_download)

    e = sub.add_parser("erase", help="erase the stored core dump")
    _add_device_args(e)
    e.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")
    e.set_defaults(func=_cmd_erase)

    dbg = sub.add_parser("debug", help="download the core dump and decode it against the app ELF")
    dbg.add_argument("app_elf", help="the app .elf the device is running (build/<app>.elf)")
    _add_device_args(dbg)
    _add_transfer_args(dbg)
    dbg.add_argument("--out", default=None,
                     help=f"where to save the core file (default {DEFAULT_ELF_OUT})")
    dbg.add_argument("--gdb", action="store_true",
                     help="open GDB on the core file (dbg_corefile) instead of printing "
                          "the decoded crash (info_corefile)")
    dbg.set_defaults(func=_cmd_debug)
    return p


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (CoreDumpError, TransportError) as exc:
        CON.error(exc)
        return 1
    except FileNotFoundError as exc:
        CON.error(exc)
        return 2
    except OSError as exc:
        # pyusb's USBError derives from OSError/IOError, so routine USB failures
        # (unplug mid-transfer, permission denied, missing libusb backend) land
        # here instead of raising an ugly traceback. Report them cleanly.
        CON.error(exc)
        return 1
    except KeyboardInterrupt:
        CON.warn("interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
