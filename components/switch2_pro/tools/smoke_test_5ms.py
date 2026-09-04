#!/usr/bin/env python3
"""Smoke-test the 5 ms BLE connection-interval patch — no hardware required.

For the given target it locates the controller archive(s) in $IDF_PATH, extracts
the object that holds the min-interval floor, disassembles the relevant function
with the target's GNU objdump, and reports the *actual instruction* that enforces
the floor:

    unpatched -> the 7.5 ms floor instruction (e.g. `bltui a4, 6`)  => 5 ms REJECTED
    patched   -> the 5 ms floor instruction  (e.g. `bltui a4, 4`)   => 5 ms ACCEPTED

This proves the patch does what it claims at the disassembly level, independent
of the byte-pattern match the patcher uses. Exit code 0 = patched (5 ms capable),
1 = unpatched, 2 = indeterminate / error.

It reuses the per-target spec from patch_nimble_5ms.py (same directory), so the
two tools can never drift. Run it before and after the patcher to see the floor
change from 6 (7.5 ms) to 4 (5 ms).

On-hardware confirmation (the second half of "does the console accept it"):
flash a BLE peripheral built with the patched IDF, then on the GAP connect /
connection-update event log the negotiated interval. With esp-nimble-cpp:

    void onConnect(NimBLEConnInfo& info) {
        ESP_LOGI("smoke", "conn interval = %u units (%.2f ms)",
                 info.getConnInterval(), info.getConnInterval() * 1.25f);
    }

Point a central that drives a fast interval at it (the Switch 2, or a BlueZ host
with its own min-interval floor lowered). A patched controller logs 4 units
(5.00 ms); a stock one never goes below 6 (7.50 ms) or drops the link.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from patch_nimble_5ms import TARGETS, archive_paths, resolve_tool, spec_for  # noqa: E402

# Function that contains the floor check, per stack.
FLOOR_FUNC = {
    "nimble": None,  # NimBLE object has no single obvious symbol; scan the whole object
    "btdm": "r_llc_con_upd_param_in_range",
}


def stack_of(spec: dict) -> str:
    return "btdm" if spec["object"] == "llc_con_upd.o" else "nimble"


def disassemble(objdump: str, obj_path: str, func: Optional[str]) -> str:
    out = subprocess.run([objdump, "-d", obj_path], capture_output=True, text=True).stdout
    if not func:
        return out
    # keep only the named function body (up to the next symbol header)
    lines, keep, buf = out.splitlines(), False, []
    for ln in lines:
        if re.search(rf"<{re.escape(func)}>:", ln):
            keep = True
        elif keep and re.match(r"^[0-9a-f]{8} <", ln):
            break
        if keep:
            buf.append(ln)
    return "\n".join(buf)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--idf-path", default=os.environ.get("IDF_PATH"), help="ESP-IDF root")
    ap.add_argument("--target", required=True, help="/ ".join(TARGETS))
    ap.add_argument("--objdump", default=None, help="override the objdump binary")
    args = ap.parse_args()
    if not args.idf_path:
        sys.exit("set --idf-path or the IDF_PATH environment variable")

    spec = spec_for(args.target)
    ar = resolve_tool("ar", spec["arch"], None)
    objdump = resolve_tool("objdump", spec["arch"], args.objdump)
    libs = archive_paths(args.idf_path, spec)
    obj = spec["object"]
    func = FLOOR_FUNC[stack_of(spec)]
    re_old, re_new = re.compile(spec["disasm_old"]), re.compile(spec["disasm_new"])

    print(f"target {args.target}  ({spec['arch']}, {stack_of(spec)} controller)")
    print(f"objdump: {objdump}   object: {obj}"
          + (f"   function: {func}" if func else ""))
    print("-" * 68)

    verdicts = []
    for lib in libs:
        tag = os.path.basename(lib)
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run([ar, "x", lib, obj], cwd=tmp, check=True)
            disasm = disassemble(objdump, os.path.join(tmp, obj), func)
        old_lines = [l.strip() for l in disasm.splitlines() if re_old.search(l)]
        new_lines = [l.strip() for l in disasm.splitlines() if re_new.search(l)]
        if new_lines and not old_lines:
            verdict, floor = "PATCHED  (5 ms ACCEPTED)", new_lines[0]
        elif old_lines and not new_lines:
            verdict, floor = "unpatched (5 ms REJECTED)", old_lines[0]
        else:
            verdict, floor = "INDETERMINATE", (old_lines + new_lines or ["<floor instruction not found>"])[0]
        verdicts.append(verdict)
        print(f"  {tag}")
        print(f"    floor instruction : {floor}")
        print(f"    verdict           : {verdict}")

    print("-" * 68)
    if all(v.startswith("PATCHED") for v in verdicts):
        print(f"PASS — {args.target} controller accepts a 5 ms connection interval.")
        return 0
    if all(v.startswith("unpatched") for v in verdicts):
        print(f"unpatched — run tools/patch_nimble_5ms.py --target {args.target} to enable 5 ms.")
        return 1
    print("INDETERMINATE — archives disagree or the floor instruction moved (IDF version?).")
    return 2


if __name__ == "__main__":
    sys.exit(main())
