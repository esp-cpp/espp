#!/usr/bin/env python3
"""Apply clang's nested-scope qualification fixes to the generated pybind file.

litgen 0.22 emits *unqualified* nested-type / nested-static names across classes that have nested
types (e.g. `frame_callback_t` instead of `espp::RtspClient::frame_callback_t`, `GuidPrefix` instead
of `espp::RtpsParticipant::GuidPrefix`, `Kind` instead of `espp::RtpsParticipant::Locator::Kind`).
These were previously fixed by hand after every regeneration (the RTSP/rtps ones were never even
documented).

clang reports each one with a precise `... 'Y'; did you mean 'espp::X::Y'?` suggestion. This script
compiles the generated file (syntax-only, all errors at once), applies those suggestions
positionally, and repeats until the file compiles cleanly. Run it after autogenerate_bindings.py
(which autogenerate invokes automatically).
"""
from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
PYDEF = os.path.join(HERE, "python_bindings", "pybind_espp.cpp")
# cmake writes the exact compile command (with every include dir / define) here. Generate it with:
#   cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. (from lib/build)
COMPILE_DB = os.path.join(HERE, "build", "compile_commands.json")


def _compile_command() -> list[str]:
    if not os.path.exists(COMPILE_DB):
        sys.exit(
            f"{COMPILE_DB} not found. Configure the build once with\n"
            f"  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..\n"
            f"from lib/build so the exact compile flags are available."
        )
    db = json.load(open(COMPILE_DB))
    entry = next(c for c in db if c["file"].endswith("pybind_espp.cpp"))
    argv = shlex.split(entry["command"])
    # Drop output (-o x.o) and the input file; add syntax-only + all-errors.
    out: list[str] = []
    skip = False
    for tok in argv:
        if skip:
            skip = False
            continue
        if tok == "-o":
            skip = True
            continue
        if tok.endswith("pybind_espp.cpp.o") or tok.endswith("pybind_espp.cpp") or tok == "-c":
            continue
        out.append(tok)
    out += ["-fsyntax-only", "-ferror-limit=0", PYDEF]
    return out


def _compile_errors() -> str:
    result = subprocess.run(_compile_command(), capture_output=True, text=True)
    return result.stderr


_SUGGEST_RE = re.compile(
    r"pybind_espp\.cpp:(\d+):(\d+): error: [^\n]*?'(\w+)'; did you mean '([^']+)'\?"
)
_UNDECL_RE = re.compile(
    r"pybind_espp\.cpp:(\d+):(\d+): error: (?:use of undeclared identifier|unknown type name) '(\w+)'"
)
# A def_readwrite on a non-copyable member (e.g. a UdpSocket field) fails to compile; the note
# points back at the offending `.def_readwrite(...)` line, which must become `.def_readonly(...)`.
_READWRITE_NOTE_RE = re.compile(
    r"pybind_espp\.cpp:(\d+):\d+: note: in instantiation of .*def_readwrite.* requested here"
)


_ENCLOSING_CLASS_RE = re.compile(r"py::class_<espp::(\w+)")


def _enclosing_class(lines: list[str], line_index: int) -> str | None:
    """Find the espp class whose binding block contains line_index (nearest preceding py::class_)."""
    for i in range(line_index, -1, -1):
        m = _ENCLOSING_CLASS_RE.search(lines[i])
        if m:
            return m.group(1)
    return None


def _fix_once() -> tuple[int, int]:
    """Apply one round of fixes. Returns (remaining_error_count, edits_applied)."""
    err = _compile_errors()
    error_count = err.count(": error:")
    lines = open(PYDEF).read().split("\n")

    # Learn the bare->qualified map from clang's suggestions (only accept espp:: scope suggestions
    # that genuinely qualify the same identifier, e.g. Y -> espp::X::Y; skip noise like
    # "did you mean 'bind'?").
    qualified: dict[str, str] = {}
    edits: list[tuple[int, int, str, str]] = []
    for m in _SUGGEST_RE.finditer(err):
        ln, col, bare, qual = int(m.group(1)), int(m.group(2)), m.group(3), m.group(4)
        if qual.startswith("espp::") and qual.endswith("::" + bare):
            qualified[bare] = qual
            edits.append((ln, col, bare, qual))
    # Undeclared/unknown identifiers without a (usable) suggestion: reuse the learned mapping, or
    # fall back to the enclosing class. Some nested typedefs (e.g. frame_callback_t, defined in both
    # RtpDepacketizer and RtspClient) get no clang suggestion and are ambiguous, so we qualify them
    # with the class whose binding block contains the error: `espp::<EnclosingClass>::<name>`.
    for m in _UNDECL_RE.finditer(err):
        ln, col, bare = int(m.group(1)), int(m.group(2)), m.group(3)
        if bare in qualified:
            edits.append((ln, col, bare, qualified[bare]))
        else:
            enclosing = _enclosing_class(lines, ln - 1)
            if enclosing is not None:
                edits.append((ln, col, bare, f"espp::{enclosing}::{bare}"))

    # Apply positionally, rightmost-first within a line so earlier columns don't shift.
    applied = 0
    for ln, col, bare, qual in sorted(set(edits), key=lambda e: (e[0], e[1]), reverse=True):
        i, c = ln - 1, col - 1
        if 0 <= i < len(lines) and lines[i][c:c + len(bare)] == bare:
            lines[i] = lines[i][:c] + qual + lines[i][c + len(bare):]
            applied += 1

    # Demote def_readwrite -> def_readonly for non-copyable members the compiler flagged.
    for m in _READWRITE_NOTE_RE.finditer(err):
        i = int(m.group(1)) - 1
        if 0 <= i < len(lines) and "def_readwrite" in lines[i]:
            lines[i] = lines[i].replace("def_readwrite", "def_readonly", 1)
            applied += 1

    if applied:
        open(PYDEF, "w").write("\n".join(lines))
    return error_count, applied


def main() -> int:
    for iteration in range(60):
        error_count, applied = _fix_once()
        print(f"  iter {iteration}: errors={error_count} qualifications_applied={applied}")
        if error_count == 0:
            print("Generated bindings compile cleanly.")
            return 0
        if applied == 0:
            print("No more qualification fixes to apply; remaining errors are not scope issues:")
            remaining = [l for l in _compile_errors().splitlines() if ": error:" in l]
            print("\n".join(remaining[:25]))
            return 1
    print("Did not converge within iteration limit.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
