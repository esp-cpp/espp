"""idf.py extension: ``idf.py coredump-usb`` with real options.

Registers an idf.py *action* (the same mechanism idf.py's own ``flash`` /
``coredump-info`` use), so unlike a CMake custom target it can take flags::

    idf.py coredump-usb                 # build, download the core dump, decode it
    idf.py coredump-usb --gdb           # ... open GDB on the core file instead
    idf.py coredump-usb --summary       # just the crash report the device stores
    idf.py coredump-usb --erase         # decode, then erase the dump from the device
    idf.py coredump-usb --summary --erase   # report it, then erase it (no download)
    idf.py coredump-usb --out crash.elf --pid 0x1234 --serial ABC123

idf.py loads it two ways:

* from ``components/coredump/idf_ext.py`` (a thin loader for this module) when
  the ``coredump`` component is in the build. idf.py only trusts component
  extensions from ESP-IDF itself, the project's own components,
  ``EXTRA_COMPONENT_DIRS`` and registry components under ``espressif/``; a
  registry install of ``espp/coredump`` therefore needs
  ``IDF_EXTENSION_ALLOW_UNTRUSTED=1`` (idf.py says so);
* from the ``idf_extension`` Python entry point declared by the ``espp`` wheel,
  whenever that wheel is installed in the ESP-IDF Python environment (no trust
  check applies to entry points).

Both may be active at once; the second registration is skipped, so there is no
duplicate-action warning. The CMake targets ``coredump-usb`` /
``coredump-usb-debug`` from ``project_include.cmake`` stay as the fallback for
builds where neither extension is loaded (an idf.py action shadows a CMake
target of the same name).
"""

from __future__ import annotations

import json
import os
from typing import Any

try:  # only importable inside idf.py; tests import this module without it
    from idf_py_actions.errors import FatalError
except ImportError:  # pragma: no cover - exercised by the tests below

    class FatalError(RuntimeError):  # type: ignore[no-redef]
        """Stand-in for idf.py's FatalError when not running under idf.py."""


ACTION_NAME = "coredump-usb"

_HELP = (
    "Build, then download the stored core dump over the USB vendor interface and "
    "decode it against the app ELF (esp-coredump info_corefile). --gdb opens GDB on "
    "the core file instead; --summary prints the device's crash report only; --erase "
    "erases the stored dump from the device afterwards. Needs pyusb (and esp-coredump "
    "for decoding). Device selection defaults to the espp coredump example's USB ids; "
    "see `python -m espp_coredump --help`."
)


def device_argv(
    vid: str | None = None,
    pid: str | None = None,
    serial: str | None = None,
    interface: str | None = None,
) -> list[str]:
    """The device-selection part of an ``espp_coredump`` command line."""
    argv: list[str] = []
    for name, value in (
        ("--vid", vid),
        ("--pid", pid),
        ("--serial", serial),
        ("--interface", interface),
    ):
        if value is not None and value != "":
            argv += [name, str(value)]
    return argv


def project_elf(build_dir: str) -> str:
    """The app ELF of the project built in ``build_dir``, from its project description.

    Raises FatalError when the build directory has not been configured (no
    ``project_description.json``) or the description names no ELF.
    """
    desc_path = os.path.join(build_dir, "project_description.json")
    if not os.path.isfile(desc_path):
        raise FatalError(
            f"{desc_path} not found: configure/build the project first (idf.py build)"
        )
    try:
        with open(desc_path, encoding="utf-8") as f:
            desc = json.load(f)
    except (OSError, ValueError) as exc:  # unreadable, or not JSON (a broken build dir)
        raise FatalError(
            f"{desc_path} could not be read ({exc}); reconfigure the project (idf.py reconfigure)"
        ) from exc
    if not isinstance(desc, dict):
        raise FatalError(f"{desc_path} is not a JSON object; reconfigure the project")
    app_elf = desc.get("app_elf")
    if not app_elf:
        raise FatalError(f"{desc_path} names no app ELF (app_elf)")
    return os.path.join(desc.get("build_dir") or build_dir, app_elf)


def build_tool_argv(
    build_dir: str,
    *,
    gdb: bool = False,
    summary: bool = False,
    erase: bool = False,
    out: str | None = None,
    vid: str | None = None,
    pid: str | None = None,
    serial: str | None = None,
    interface: str | None = None,
) -> list[str]:
    """The ``espp_coredump`` command line for one ``idf.py coredump-usb``
    invocation. ``--erase`` is passed through to the tool's ``summary`` /
    ``debug`` command, which erases on the same USB connection it read the
    dump from, and only after the report was printed / the decode succeeded
    (never a second device selection, so with several boards attached the
    dump erased is the one just reported / decoded)."""
    if summary and gdb:
        raise FatalError("--summary and --gdb are mutually exclusive (--summary prints the "
                         "device's report without downloading; --gdb needs the core file)")
    argv = device_argv(vid, pid, serial, interface)
    if summary:
        argv.append("summary")
    else:
        argv += ["debug", project_elf(build_dir)]
        # default the core file into the build directory (the tool's own default
        # is the current directory, which under idf.py is the project source tree)
        argv += ["--out", out or os.path.join(build_dir, "core.elf")]
        if gdb:
            argv.append("--gdb")
    if erase:
        argv.append("--erase")
    return argv


def run_tool(argv: list[str]) -> None:
    """Run the espp_coredump CLI in-process; anything but a clean exit becomes
    a FatalError. The CLI returns an int, but argparse exits via SystemExit
    (e.g. on a usage error), so that is normalized too."""
    from espp_coredump.cli import main as tool_main

    try:
        rc = tool_main(argv)
    except SystemExit as exc:  # argparse --help / usage errors
        rc = exc.code
    if rc is None:
        rc = 0
    if not isinstance(rc, int):
        # SystemExit("message") style: the message was printed, treat as failure
        raise FatalError(f"espp_coredump failed: {rc}")
    if rc != 0:
        raise FatalError(f"espp_coredump exited with status {rc}")


def action_extensions(base_actions: dict[str, Any] | None, project_path: str) -> dict[str, Any]:
    """idf.py's extension hook: return the ``coredump-usb`` action (once)."""
    del project_path  # the build dir comes from idf.py's args at call time
    existing = (base_actions or {}).get("actions") or {}
    if ACTION_NAME in existing:
        return {}  # already registered by the other loading path

    def coredump_usb(
        action: str,
        ctx: Any,
        args: Any,
        gdb: bool = False,
        summary: bool = False,
        erase: bool = False,
        out: str | None = None,
        vid: str | None = None,
        pid: str | None = None,
        serial: str | None = None,
        interface: str | None = None,
    ) -> None:
        del action, ctx
        run_tool(
            build_tool_argv(
                args.build_dir,
                gdb=gdb,
                summary=summary,
                erase=erase,
                out=out,
                vid=vid,
                pid=pid,
                serial=serial,
                interface=interface,
            )
        )

    return {
        # idf.py requires a truthy "version" on custom extensions
        "version": "1",
        "actions": {
            ACTION_NAME: {
                "callback": coredump_usb,
                "help": _HELP,
                "options": [
                    {
                        "names": ["--gdb"],
                        "is_flag": True,
                        "default": False,
                        "help": "Open GDB on the core file (esp-coredump dbg_corefile) instead of "
                        "printing the report.",
                    },
                    {
                        "names": ["--summary"],
                        "is_flag": True,
                        "default": False,
                        "help": "Only print the crash report stored on the device; no download, "
                        "no decoding.",
                    },
                    {
                        "names": ["--erase"],
                        "is_flag": True,
                        "default": False,
                        "help": "Erase the stored core dump from the device once the report "
                        "is printed / the decode succeeded, on the same USB connection "
                        "(the flag is the confirmation; a failed decode leaves it in place).",
                    },
                    {
                        "names": ["--out", "-o"],
                        "default": None,
                        "help": "Where to save the core file (default: core.elf in the build "
                        "directory).",
                    },
                    {
                        "names": ["--vid"],
                        "default": None,
                        "help": "USB vendor id to match (default 0x1209, or $ESPP_COREDUMP_VID).",
                    },
                    {
                        "names": ["--pid"],
                        "default": None,
                        "help": "USB product id to match (default 0x0d36, or $ESPP_COREDUMP_PID; "
                        "-1 = any).",
                    },
                    {
                        "names": ["--serial"],
                        "default": None,
                        "help": "USB serial number to match (or $ESPP_COREDUMP_SERIAL).",
                    },
                    {
                        "names": ["--interface"],
                        "default": None,
                        "help": "Force a specific USB interface number instead of the first "
                        "vendor (0xFF) one.",
                    },
                ],
                # build first, so the ELF the dump is decoded against is the one on the device
                "dependencies": ["all"],
            }
        }
    }
