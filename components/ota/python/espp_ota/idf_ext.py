"""idf.py extension: ``idf.py ota-usb`` with real options.

Registers an idf.py *action* (the same mechanism idf.py's own ``flash`` uses),
so unlike a CMake custom target it can take flags::

    idf.py ota-usb                        # build, then OTA the app .bin over USB
    idf.py ota-usb --no-verify            # ... without the reconnect + mark-valid
    idf.py ota-usb --binary other.bin     # OTA some other image instead
    idf.py ota-usb --status               # is the running image pending verification?
    idf.py ota-usb --mark-valid           # confirm the running image (cancel rollback)
    idf.py ota-usb --rollback             # reject it: roll back + reboot
    idf.py ota-usb --pid 0x1234 --serial ABC123 --chunk-size 2048

idf.py loads it two ways:

* from ``components/ota/idf_ext.py`` (a thin loader for this module) when the
  ``ota`` component is in the build. idf.py only trusts component extensions
  from ESP-IDF itself, the project's own components, ``EXTRA_COMPONENT_DIRS``
  and registry components under ``espressif/``; a registry install of
  ``espp/ota`` therefore needs ``IDF_EXTENSION_ALLOW_UNTRUSTED=1`` (idf.py says
  so);
* from the ``idf_extension`` Python entry point declared by the ``espp`` wheel,
  whenever that wheel is installed in the ESP-IDF Python environment (no trust
  check applies to entry points).

Both may be active at once; the second registration is skipped, so there is no
duplicate-action warning. The CMake target ``ota-usb`` from
``project_include.cmake`` stays as the fallback for builds where neither
extension is loaded (an idf.py action shadows a CMake target of the same name).
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


ACTION_NAME = "ota-usb"

_HELP = (
    "Build, then OTA-update the app .bin over the USB vendor interface (BEGIN -> "
    "DATA -> FINISH, then reconnect and mark the new image valid unless --no-verify). "
    "--status / --mark-valid / --rollback query or drive the rollback state instead "
    "of flashing. Needs pyusb. Device selection defaults to the espp UsbDevice ids; "
    "see `python -m espp_ota --help`."
)

# the mode flags, in the order they are reported; only one may be given
_MODES = ("status", "mark-valid", "rollback")


def device_argv(
    vid: str | None = None,
    pid: str | None = None,
    serial: str | None = None,
    interface: str | None = None,
) -> list[str]:
    """The device-selection part of an ``espp_ota`` command line."""
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


def project_bin(build_dir: str) -> str:
    """The app .bin of the project built in ``build_dir``, from its project description.

    Raises FatalError when the build directory has not been configured (no
    ``project_description.json``) or the description names no binary.
    """
    desc_path = os.path.join(build_dir, "project_description.json")
    if not os.path.isfile(desc_path):
        raise FatalError(
            f"{desc_path} not found: configure/build the project first (idf.py build)"
        )
    with open(desc_path, encoding="utf-8") as f:
        desc = json.load(f)
    app_bin = desc.get("app_bin")
    if not app_bin:
        raise FatalError(f"{desc_path} names no app binary (app_bin)")
    return os.path.join(desc.get("build_dir") or build_dir, app_bin)


def selected_mode(status: bool = False, mark_valid: bool = False, rollback: bool = False) -> str | None:
    """Which of the mutually exclusive mode flags is set (None = flash).

    Raises FatalError when more than one is given.
    """
    chosen = [name for name, on in zip(_MODES, (status, mark_valid, rollback)) if on]
    if len(chosen) > 1:
        raise FatalError("--" + " and --".join(chosen) + " are mutually exclusive")
    return chosen[0] if chosen else None


def build_tool_argv(
    build_dir: str,
    *,
    binary: str | None = None,
    chunk_size: str | None = None,
    no_verify: bool = False,
    verify_timeout: str | None = None,
    quiet: bool = False,
    status: bool = False,
    mark_valid: bool = False,
    rollback: bool = False,
    vid: str | None = None,
    pid: str | None = None,
    serial: str | None = None,
    interface: str | None = None,
) -> list[str]:
    """The ``espp_ota`` command line for one ``idf.py ota-usb`` invocation."""
    mode = selected_mode(status, mark_valid, rollback)
    argv: list[str] = []
    if mode is not None:
        # the sub-command comes first; device args follow it (argparse
        # attaches them to the sub-parser)
        return [mode] + device_argv(vid, pid, serial, interface)
    argv += ["flash", binary or project_bin(build_dir)]
    argv += device_argv(vid, pid, serial, interface)
    if chunk_size is not None and chunk_size != "":
        argv += ["--chunk-size", str(chunk_size)]
    if no_verify:
        argv.append("--no-verify")
    if verify_timeout is not None and verify_timeout != "":
        argv += ["--verify-timeout", str(verify_timeout)]
    if quiet:
        argv.append("--quiet")
    return argv


def run_tool(argv: list[str]) -> None:
    """Run the espp_ota CLI in-process; a non-zero exit becomes a FatalError."""
    from espp_ota.cli import main as tool_main

    rc = tool_main(argv)
    if rc != 0:
        raise FatalError(f"espp_ota exited with status {rc}")


def action_extensions(base_actions: dict[str, Any] | None, project_path: str) -> dict[str, Any]:
    """idf.py's extension hook: return the ``ota-usb`` action (once)."""
    del project_path  # the build dir comes from idf.py's args at call time
    existing = (base_actions or {}).get("actions") or {}
    if ACTION_NAME in existing:
        return {}  # already registered by the other loading path

    def ota_usb(
        action: str,
        ctx: Any,
        args: Any,
        binary: str | None = None,
        chunk_size: str | None = None,
        no_verify: bool = False,
        verify_timeout: str | None = None,
        quiet: bool = False,
        status: bool = False,
        mark_valid: bool = False,
        rollback: bool = False,
        vid: str | None = None,
        pid: str | None = None,
        serial: str | None = None,
        interface: str | None = None,
    ) -> None:
        del action, ctx
        run_tool(
            build_tool_argv(
                args.build_dir,
                binary=binary,
                chunk_size=chunk_size,
                no_verify=no_verify,
                verify_timeout=verify_timeout,
                quiet=quiet,
                status=status,
                mark_valid=mark_valid,
                rollback=rollback,
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
                "callback": ota_usb,
                "help": _HELP,
                "options": [
                    {
                        "names": ["--binary", "-b"],
                        "default": None,
                        "help": "The .bin to OTA instead of the project's app binary "
                        "(build/<project>.bin).",
                    },
                    {
                        "names": ["--chunk-size"],
                        "default": None,
                        "help": "DATA payload bytes per frame (1..4096, default 4096).",
                    },
                    {
                        "names": ["--no-verify"],
                        "is_flag": True,
                        "default": False,
                        "help": "Do not reconnect after the reboot to mark the new image valid "
                        "(leave it pending; the device rolls back unless it is confirmed).",
                    },
                    {
                        "names": ["--verify-timeout"],
                        "default": None,
                        "help": "Seconds to wait for the device to reappear after the reboot "
                        "(default 20).",
                    },
                    {
                        "names": ["--quiet", "-q"],
                        "is_flag": True,
                        "default": False,
                        "help": "Suppress the progress output.",
                    },
                    {
                        "names": ["--status"],
                        "is_flag": True,
                        "default": False,
                        "help": "Instead of flashing: query the rollback status (is the running "
                        "image pending verification?).",
                    },
                    {
                        "names": ["--mark-valid"],
                        "is_flag": True,
                        "default": False,
                        "help": "Instead of flashing: confirm the running image (cancel the "
                        "rollback).",
                    },
                    {
                        "names": ["--rollback"],
                        "is_flag": True,
                        "default": False,
                        "help": "Instead of flashing: reject the running image (roll back and "
                        "reboot).",
                    },
                    {
                        "names": ["--vid"],
                        "default": None,
                        "help": "USB vendor id to match (default 0x1209, or $ESPP_OTA_VID).",
                    },
                    {
                        "names": ["--pid"],
                        "default": None,
                        "help": "USB product id to match (default 0x0d32, or $ESPP_OTA_PID; "
                        "-1 = any).",
                    },
                    {
                        "names": ["--serial"],
                        "default": None,
                        "help": "USB serial number to match (or $ESPP_OTA_SERIAL).",
                    },
                    {
                        "names": ["--interface"],
                        "default": None,
                        "help": "Force a specific USB interface number instead of the first "
                        "vendor (0xFF) one.",
                    },
                ],
                # build first, so the .bin that goes over is the one just built
                "dependencies": ["all"],
            }
        }
    }
