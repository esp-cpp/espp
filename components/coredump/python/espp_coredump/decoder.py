"""Finding and running ESP-IDF's core-dump decoder on the host.

Two ways it may be installed, tried in order:

1. the ``esp-coredump`` console script (``pip install esp-coredump``, which the
   ESP-IDF Python environment ships);
2. ``$IDF_PATH/components/espcoredump/espcoredump.py`` run with the current
   interpreter (older IDF layouts).

Both take the same sub-commands: ``info_corefile`` prints the decoded crash
(registers, backtrace with symbols, threads), ``dbg_corefile`` opens GDB on it.
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from typing import List, Optional


def find_decoder() -> Optional[List[str]]:
    """The command prefix to run the decoder, or None when none is installed."""
    exe = shutil.which("esp-coredump")
    if exe:
        return [exe]
    idf = os.environ.get("IDF_PATH")
    if idf:
        script = os.path.join(idf, "components", "espcoredump", "espcoredump.py")
        if os.path.isfile(script):
            return [sys.executable, script]
    return None


def decoder_args(core_path: str, app_elf: str, gdb: bool = False,
                 core_format: str = "elf") -> List[str]:
    """The decoder sub-command + arguments (without the command prefix)."""
    return [
        "dbg_corefile" if gdb else "info_corefile",
        "--core", core_path,
        "--core-format", core_format,
        app_elf,
    ]


def suggested_command(core_path: str, app_elf: str, gdb: bool = False,
                      core_format: str = "elf") -> str:
    """The exact command to run by hand when no decoder is installed, quoted
    for the host's shell (paths with spaces survive a copy-paste)."""
    argv = ["esp-coredump"] + decoder_args(core_path, app_elf, gdb, core_format)
    if os.name == "nt":
        return subprocess.list2cmdline(argv)
    return shlex.join(argv)


def run_decoder(core_path: str, app_elf: str, gdb: bool = False,
                core_format: str = "elf") -> int:
    """Run the decoder in the foreground (its output goes to the terminal, and
    ``dbg_corefile`` is interactive). Returns its exit code, or -1 when no
    decoder is installed."""
    prefix = find_decoder()
    if prefix is None:
        return -1
    cmd = prefix + decoder_args(core_path, app_elf, gdb, core_format)
    return subprocess.call(cmd)
