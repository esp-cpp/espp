#!/usr/bin/env python3
"""Patch the prebuilt ESP-IDF NimBLE controller library to accept a sub-spec
5 ms BLE connection interval, which the Nintendo Switch 2 console requires of
its controllers.

The check that rejects intervals below 7.5 ms (6 units of 1.25 ms) is compiled
into `ble_ll_conn.c.o` inside the closed `libble_app.a` shipped with ESP-IDF for
the RISC-V targets. This flips the immediate `-6` to `-4` (5 ms), i.e.

    addi a5, a4, -6   (93 07 a7 ff)  ->  addi a5, a4, -4   (93 07 c7 ff)

WARNING: this modifies files inside your global $IDF_PATH install, affecting
every project that uses that IDF. A `.original` backup is written next to the
patched archive; `--restore` puts it back. Only RISC-V targets with the open
NimBLE controller (esp32c6/c61/c2/h2) are supported — S3/C3 use a different
closed controller and are not handled here.

Approach adapted from the MIT-licensed zhantss/ESP32-BLE5-NSController-Emulator;
the reverse-engineered requirement is documented in ndeadly/switch2_controller_research.
This script ships no Espressif or Nintendo binaries — it only edits the archive
already present in the user's local ESP-IDF.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

OBJECT = "ble_ll_conn.c.o"
OLD = bytes([0x93, 0x07, 0xA7, 0xFF])  # min interval 6 units (7.5 ms)
NEW = bytes([0x93, 0x07, 0xC7, 0xFF])  # min interval 4 units (5 ms)


def resolve_ar(explicit: str | None) -> str:
    """The controller archives are GNU-format (long-name symbol/string tables).
    macOS's BSD `ar` cannot extract them, so prefer the RISC-V toolchain's GNU
    `ar` (on PATH after the ESP-IDF export script), then llvm-ar, then `ar`."""
    if explicit:
        return explicit
    for cand in ("riscv32-esp-elf-ar", "llvm-ar"):
        if shutil.which(cand):
            return cand
    return "ar"

# Target -> relative path of libble_app.a under $IDF_PATH.
LIBS = {
    "esp32c6": "components/bt/controller/lib_esp32c6/esp32c6-bt-lib/esp32c6/libble_app.a",
    "esp32c61": "components/bt/controller/lib_esp32c6/esp32c6-bt-lib/esp32c61/libble_app.a",
    "esp32c2": "components/bt/controller/lib_esp32c2/esp32c2-bt-lib/libble_app.a",
    "esp32h2": "components/bt/controller/lib_esp32h2/esp32h2-bt-lib/libble_app.a",
}


def lib_path(idf_path: str, target: str) -> str:
    rel = LIBS.get(target)
    if rel is None:
        sys.exit(f"unsupported target '{target}'; supported: {', '.join(LIBS)}")
    path = os.path.join(idf_path, rel)
    if not os.path.isfile(path):
        sys.exit(f"library not found: {path}")
    return path


def read_object(ar: str, lib: str) -> bytes:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([ar, "x", lib, OBJECT], cwd=tmp, check=True)
        with open(os.path.join(tmp, OBJECT), "rb") as f:
            return f.read()


def write_object(ar: str, lib: str, data: bytes) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        obj = os.path.join(tmp, OBJECT)
        with open(obj, "wb") as f:
            f.write(data)
        subprocess.run([ar, "r", lib, obj], cwd=os.path.dirname(obj) or ".", check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--idf-path", default=os.environ.get("IDF_PATH"), help="ESP-IDF root")
    ap.add_argument("--target", required=True, help="esp32c6 / esp32c61 / esp32c2 / esp32h2")
    ap.add_argument("--verify-only", action="store_true", help="report state, change nothing")
    ap.add_argument("--restore", action="store_true", help="restore the .original backup")
    ap.add_argument("--ar", default=None,
                    help="archiver to use (default: riscv32-esp-elf-ar / llvm-ar / ar). "
                         "macOS BSD ar cannot read these GNU-format archives.")
    args = ap.parse_args()
    if not args.idf_path:
        sys.exit("set --idf-path or the IDF_PATH environment variable")

    ar = resolve_ar(args.ar)
    lib = lib_path(args.idf_path, args.target)
    backup = lib + ".original"

    if args.restore:
        if not os.path.isfile(backup):
            sys.exit(f"no backup to restore: {backup}")
        shutil.copy2(backup, lib)
        print(f"restored {lib} from backup")
        return 0

    data = read_object(ar, lib)
    n_old, n_new = data.count(OLD), data.count(NEW)
    if args.verify_only:
        print(f"{OBJECT} (via {ar}): unpatched-pattern={n_old} patched-pattern={n_new}")
        return 0
    if n_new > 0 and n_old == 0:
        print("already patched; nothing to do")
        return 0
    if n_old == 0:
        sys.exit("expected byte pattern not found — IDF version may differ; not patching")

    if not os.path.isfile(backup):
        shutil.copy2(lib, backup)
        print(f"backed up -> {backup}")
    write_object(ar, lib, data.replace(OLD, NEW))
    print(f"patched {n_old} occurrence(s); {lib} now accepts a 5 ms connection interval")
    return 0


if __name__ == "__main__":
    sys.exit(main())
