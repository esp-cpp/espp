#!/usr/bin/env python3
"""Patch the prebuilt ESP-IDF BLE controller library to accept a sub-spec 5 ms
connection interval, which the Nintendo Switch 2 console requires of its
controllers.

BLE's minimum connection interval is 7.5 ms (6 units of 1.25 ms). The console
drives its controllers at 5 ms (4 units), so a stock controller rejects it. The
7.5 ms floor is compiled into the closed controller library ESP-IDF ships, and
differs by chip family:

  * RISC-V NimBLE controller (esp32c6/c61/c2/h2) — `libble_app.a`, object
    `ble_ll_conn.c.o`. The floor is an `addi a5, a4, -6`; flip the immediate to
    -4:  93 07 a7 ff  ->  93 07 c7 ff.

  * BTDM / RivieraWaves controller (esp32s3 Xtensa, esp32c3 RISC-V) —
    `libbtdm_app.a` (and the `_flash` variant), object `llc_con_upd.o`, function
    `r_llc_con_upd_param_in_range`. The floor is a compare of the requested
    min-interval against 6:
        S3 (Xtensa):  bltui a4, 6  (b6 64 01)  ->  bltui a4, 4  (b6 44 01)
        C3 (RISC-V):  li a6,5;bgeu a6,a2 (15 48) -> li a6,3;bgeu a6,a2 (0d 48)
    Both give a new floor of 4 units = 5 ms. This is the peripheral-side
    validator the console's LL_CONNECTION_PARAM_REQ / _UPDATE_IND path runs
    through (verified: its only caller is the RivieraWaves ip_funcs jump table,
    and its siblings are ll_connection_param_req_handler /
    ll_connection_update_ind_handler).

WARNING: this modifies files inside your global $IDF_PATH install, affecting
every project that uses that IDF. A `.original` backup is written next to each
patched archive; `--restore` puts them back.

Approach adapted from the MIT-licensed zhantss/ESP32-BLE5-NSController-Emulator
(RISC-V/NimBLE); the S3/C3 BTDM equivalent was reverse-engineered here from the
same reject-below-6 semantics. The reverse-engineered requirement is documented
in ndeadly/switch2_controller_research. This script ships no Espressif or
Nintendo binaries — it only edits the archives already present in the user's
local ESP-IDF.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

# Per-target patch spec. `arch` selects the toolchain archiver/objdump (the
# archives are GNU-format; macOS BSD `ar` cannot read them). `archives` is a
# list because the BTDM family ships two variants (IRAM + flash-only, chosen by
# CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY) — we patch whichever are present. `old`/`new`
# are the byte patterns inside `object`; `disasm_old`/`disasm_new` are the
# human-readable instruction each corresponds to (used by smoke_test_5ms.py).
TARGETS = {
    # --- RISC-V NimBLE controller: libble_app.a, ble_ll_conn.c.o ---
    "esp32c6": {
        "arch": "riscv",
        "archives": ["components/bt/controller/lib_esp32c6/esp32c6-bt-lib/esp32c6/libble_app.a"],
        "object": "ble_ll_conn.c.o",
        "old": bytes([0x93, 0x07, 0xA7, 0xFF]),  # addi a5,a4,-6  (min 6 units / 7.5 ms)
        "new": bytes([0x93, 0x07, 0xC7, 0xFF]),  # addi a5,a4,-4  (min 4 units / 5 ms)
        "disasm_old": r"addi\s+a5,a4,-6",
        "disasm_new": r"addi\s+a5,a4,-4",
    },
    "esp32c61": {
        "arch": "riscv",
        "archives": ["components/bt/controller/lib_esp32c6/esp32c6-bt-lib/esp32c61/libble_app.a"],
        "object": "ble_ll_conn.c.o",
        "old": bytes([0x93, 0x07, 0xA7, 0xFF]),
        "new": bytes([0x93, 0x07, 0xC7, 0xFF]),
        "disasm_old": r"addi\s+a5,a4,-6",
        "disasm_new": r"addi\s+a5,a4,-4",
    },
    "esp32c2": {
        "arch": "riscv",
        "archives": ["components/bt/controller/lib_esp32c2/esp32c2-bt-lib/libble_app.a"],
        "object": "ble_ll_conn.c.o",
        "old": bytes([0x93, 0x07, 0xA7, 0xFF]),
        "new": bytes([0x93, 0x07, 0xC7, 0xFF]),
        "disasm_old": r"addi\s+a5,a4,-6",
        "disasm_new": r"addi\s+a5,a4,-4",
    },
    "esp32h2": {
        "arch": "riscv",
        "archives": ["components/bt/controller/lib_esp32h2/esp32h2-bt-lib/libble_app.a"],
        "object": "ble_ll_conn.c.o",
        "old": bytes([0x93, 0x07, 0xA7, 0xFF]),
        "new": bytes([0x93, 0x07, 0xC7, 0xFF]),
        "disasm_old": r"addi\s+a5,a4,-6",
        "disasm_new": r"addi\s+a5,a4,-4",
    },
    # --- BTDM / RivieraWaves controller: libbtdm_app.a[+_flash], llc_con_upd.o ---
    "esp32s3": {
        "arch": "xtensa-esp32s3",
        "archives": [
            "components/bt/controller/lib_esp32c3_family/esp32s3/libbtdm_app.a",
            "components/bt/controller/lib_esp32c3_family/esp32s3/libbtdm_app_flash.a",
        ],
        "object": "llc_con_upd.o",
        "old": bytes([0xB6, 0x64, 0x01]),  # bltui a4,6  (reject min < 6 / 7.5 ms)
        "new": bytes([0xB6, 0x44, 0x01]),  # bltui a4,4  (reject min < 4 / 5 ms)
        "disasm_old": r"bltui\s+a4, ?6,",
        "disasm_new": r"bltui\s+a4, ?4,",
    },
    "esp32c3": {
        "arch": "riscv",
        "archives": [
            "components/bt/controller/lib_esp32c3_family/esp32c3/libbtdm_app.a",
            "components/bt/controller/lib_esp32c3_family/esp32c3/libbtdm_app_flash.a",
        ],
        "object": "llc_con_upd.o",
        "old": bytes([0x15, 0x48]),  # c.li a6,5; bgeu a6,a2 -> reject min <= 5 (floor 6)
        "new": bytes([0x0D, 0x48]),  # c.li a6,3; bgeu a6,a2 -> reject min <= 3 (floor 4)
        "disasm_old": r"li\s+a6,5",
        "disasm_new": r"li\s+a6,3",
    },
}


def resolve_tool(kind: str, arch: str, explicit: str | None) -> str:
    """Resolve the GNU `ar`/`objdump` for the target arch. The controller
    archives are GNU-format (long-name symbol/string tables); macOS's BSD `ar`
    cannot extract them, so prefer the ESP toolchain's GNU tools (on PATH after
    the ESP-IDF export script), then llvm-*, then the bare tool."""
    if explicit:
        return explicit
    prefixes = {
        "riscv": ["riscv32-esp-elf-"],
        "xtensa-esp32s3": ["xtensa-esp32s3-elf-"],
    }.get(arch, [])
    cands = [p + kind for p in prefixes] + [f"llvm-{kind}", kind]
    for cand in cands:
        if shutil.which(cand):
            return cand
    return kind


def spec_for(target: str) -> dict:
    spec = TARGETS.get(target)
    if spec is None:
        sys.exit(f"unsupported target '{target}'; supported: {', '.join(TARGETS)}")
    return spec


def archive_paths(idf_path: str, spec: dict) -> list[str]:
    """Absolute paths of the target's archives that actually exist on disk."""
    paths = []
    for rel in spec["archives"]:
        p = os.path.join(idf_path, rel)
        if os.path.isfile(p):
            paths.append(p)
    if not paths:
        sys.exit(f"no controller archive found under {idf_path} for this target:\n  "
                 + "\n  ".join(spec["archives"]))
    return paths


def read_object(ar: str, lib: str, obj: str) -> bytes:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([ar, "x", lib, obj], cwd=tmp, check=True)
        with open(os.path.join(tmp, obj), "rb") as f:
            return f.read()


def write_object(ar: str, lib: str, obj: str, data: bytes) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, obj)
        with open(path, "wb") as f:
            f.write(data)
        subprocess.run([ar, "r", lib, path], cwd=os.path.dirname(path) or ".", check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--idf-path", default=os.environ.get("IDF_PATH"), help="ESP-IDF root")
    ap.add_argument("--target", required=True, help="/ ".join(TARGETS))
    ap.add_argument("--verify-only", action="store_true", help="report state, change nothing")
    ap.add_argument("--restore", action="store_true", help="restore the .original backups")
    ap.add_argument("--ar", default=None,
                    help="archiver to use (default: the ESP toolchain GNU ar for the target). "
                         "macOS BSD ar cannot read these GNU-format archives.")
    args = ap.parse_args()
    if not args.idf_path:
        sys.exit("set --idf-path or the IDF_PATH environment variable")

    spec = spec_for(args.target)
    ar = resolve_tool("ar", spec["arch"], args.ar)
    libs = archive_paths(args.idf_path, spec)
    obj, old, new = spec["object"], spec["old"], spec["new"]

    if args.restore:
        n = 0
        for lib in libs:
            backup = lib + ".original"
            if os.path.isfile(backup):
                shutil.copy2(backup, lib)
                print(f"restored {lib}")
                n += 1
        if n == 0:
            sys.exit("no .original backups found to restore")
        return 0

    any_patched_now = False
    for lib in libs:
        data = read_object(ar, lib, obj)
        n_old, n_new = data.count(old), data.count(new)
        tag = os.path.basename(lib)
        if args.verify_only:
            state = "PATCHED (5 ms)" if (n_new and not n_old) else \
                    "unpatched (7.5 ms)" if (n_old and not n_new) else "UNKNOWN"
            print(f"{tag}: {obj} unpatched-pattern={n_old} patched-pattern={n_new} -> {state}")
            continue
        if n_new > 0 and n_old == 0:
            print(f"{tag}: already patched; nothing to do")
            continue
        if n_old == 0:
            sys.exit(f"{tag}: expected byte pattern not found in {obj} — IDF version may "
                     f"differ; not patching")
        if n_old > 1:
            sys.exit(f"{tag}: pattern appears {n_old}x in {obj} (expected 1) — refusing to "
                     f"patch ambiguously")
        backup = lib + ".original"
        if not os.path.isfile(backup):
            shutil.copy2(lib, backup)
            print(f"{tag}: backed up -> {os.path.basename(backup)}")
        write_object(ar, lib, obj, data.replace(old, new))
        print(f"{tag}: patched {n_old} occurrence(s) — now accepts a 5 ms connection interval")
        any_patched_now = True

    if not args.verify_only and any_patched_now:
        print(f"done ({args.target}). Run tools/smoke_test_5ms.py --target {args.target} to verify.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
