#!/usr/bin/env python3
"""The QEMU memory suite's report.

Merges, per limits profile, what the suite measured on the device
(``memory.json`` next to the image: the sizeof of the engine's objects, the heap
by stage, the heap after each host-driven step) with what the linker put in the
image (``idf.py size-components --format json2``: the rtps library's flash, IRAM
and DRAM), and prints Markdown tables. Costs are negative deltas of FREE heap.

    memory_report.py memory.json
        one profile, the tables on stdout (what run_qemu_interop.sh does)
    memory_report.py --summary $GITHUB_STEP_SUMMARY --json out.json \\
        embedded:path/memory.json:path/size-components.json host:...
        several profiles side by side; the Markdown appended to --summary, the
        merged numbers written to --json (the rtps-memory artifact)

A profile whose files are missing (a firmware that did not start, say) is
reported as such rather than failing the report.

    memory_report.py --baseline main.json --baseline-label "main @ abc1234" ...
        every table gains a column with the delta against the same number in
        the baseline (the --json of an earlier run, main's latest one on CI), so
        a pull request's cost shows as such, not as another absolute figure
"""

import argparse
import json
import os
import sys

SIZEOF_ROWS = [
    ("domain", "rtps::Domain (holds every pool below)"),
    ("participant", "rtps::Participant"),
    ("stateful_writer", "rtps::StatefulWriter"),
    ("stateless_writer", "rtps::StatelessWriter"),
    ("stateful_reader", "rtps::StatefulReader"),
    ("stateless_reader", "rtps::StatelessReader"),
    ("facade", "espp::RtpsParticipant"),
]
LIBRARY_ROWS = [
    ("Flash Code", "flash code"),
    ("Flash Data", "flash data"),
    ("DRAM", "DRAM (.data + .bss)"),
    ("IRAM", "IRAM"),
]


def load_json(path):
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print(f"memory_report: cannot read {path}: {e}", file=sys.stderr)
        return None


def library_sizes(size_components):
    """The rtps archive's size per memory type, from the json2 report."""
    if not size_components:
        return None
    for name, entry in size_components.items():
        if entry.get("abbrev_name") == "librtps.a" or name.endswith("/librtps.a"):
            # a memory type the report does not list is unknown (n/a), not zero
            types = entry.get("memory_types", {})
            return {key: types[key].get("size") if key in types else None for key, _ in LIBRARY_ROWS}
    return None


def fmt(n):
    return "n/a" if n is None else f"{n:,}"


def delta(now, before):
    if now is None or before is None:
        return "n/a"
    d = now - before
    return f"{d:+,}"


def vs_baseline(now, before):
    """The delta of a FREE-heap figure against the baseline's: a lower free
    heap than the baseline is a cost, printed negative; equal prints as 0."""
    if before is None:
        return "n/a"
    return delta(now, before)


def profile_tables(name, mem, base=None):
    """The stage and step tables of one profile; with `base` (the same
    profile's numbers from the baseline run) each row gains "vs baseline"
    columns for internal RAM and PSRAM."""
    out = []
    base_stages = {s["stage"]: s for s in (base or {}).get("stages") or []}
    base_steps = {s["step"]: s for s in (base or {}).get("steps") or []}
    vs = base is not None
    vs_head = " vs baseline | vs baseline (PSRAM) |" if vs else ""
    vs_sep = "---:|---:|" if vs else ""

    def vs_cells(s, b):
        if not vs:
            return ""
        if b is None:
            return " n/a | n/a |"
        return f" {vs_baseline(s['internal'], b.get('internal'))} | {vs_baseline(s['psram'], b.get('psram'))} |"

    stages = {s["stage"]: s for s in (mem or {}).get("stages", [])}
    if stages:
        out.append(f"#### {name}: heap by stage (free bytes; a cost is a negative delta)\n")
        out.append(
            "| stage | internal | delta | internal low-water mark | largest internal block | PSRAM | delta |"
            + vs_head
        )
        out.append("|---|---:|---:|---:|---:|---:|---:|" + vs_sep)
        prev = None
        for key in ("boot", "started", "endpoints"):
            s = stages.get(key)
            if s is None:
                out.append(f"| {key} | n/a | | | | | |" + (" | |" if vs else ""))
                continue
            out.append(
                f"| {key} | {fmt(s['internal'])} | {delta(s['internal'], prev['internal']) if prev else ''} "
                f"| {fmt(s['internal_min'])} | {fmt(s['internal_largest'])} | {fmt(s['psram'])} "
                f"| {delta(s['psram'], prev['psram']) if prev else ''} |" + vs_cells(s, base_stages.get(key))
            )
            prev = s
        out.append("")
    steps = (mem or {}).get("steps", [])
    if steps:
        out.append(
            f"#### {name}: heap after each host-driven step (free bytes; deltas against the previous row; "
            "the pools are preallocated, so a peer costs little heap - the low-water mark shows the transients)\n"
        )
        out.append(
            "| step | internal | delta | internal low-water mark | largest internal block | PSRAM | delta |"
            + vs_head
        )
        out.append("|---|---:|---:|---:|---:|---:|---:|" + vs_sep)
        prev = None
        for s in steps:
            out.append(
                f"| {s['step']} | {fmt(s['internal'])} | {delta(s['internal'], prev['internal']) if prev else ''} "
                f"| {fmt(s['internal_min'])} | {fmt(s['internal_largest'])} | {fmt(s['psram'])} "
                f"| {delta(s['psram'], prev['psram']) if prev else ''} |" + vs_cells(s, base_steps.get(s["step"]))
            )
            prev = s
        out.append("")
    return out


def with_vs(value, base_value, vs):
    """A cell: the value, and when a baseline is given its delta in parentheses
    (a size: bigger is a cost, printed positive)."""
    if not vs or value is None:
        return fmt(value)
    if base_value is None:
        return f"{fmt(value)} (n/a)"
    return f"{fmt(value)} ({delta(value, base_value)})"


def report(profiles, baseline=None, baseline_label=None):
    """profiles: list of (name, memory dict or None, library dict or None).
    baseline: the {"profiles": {...}} of an earlier run's --json, whose numbers
    every table is compared against (deltas in parentheses, or in the "vs
    baseline" columns of the heap tables)."""
    out = ["### rtps memory footprint (QEMU, esp32)\n"]
    names = [p[0] for p in profiles]
    base_profiles = (baseline or {}).get("profiles") or {}
    vs = baseline is not None
    if vs:
        out.append(
            f"Deltas in parentheses and in the \"vs baseline\" columns are against {baseline_label or 'the baseline run'}: "
            "a bigger size or a smaller free heap than there is a cost of this change.\n"
        )

    def base_of(name):
        return base_profiles.get(name) or {}

    # the rtps library in the image
    if any(p[2] for p in profiles):
        out.append("#### librtps.a in the image (bytes, `idf.py size-components`)\n")
        out.append("| | " + " | ".join(names) + " |")
        out.append("|---|" + "---:|" * len(names))
        for key, label in LIBRARY_ROWS:
            out.append(
                f"| {label} | "
                + " | ".join(
                    with_vs(p[2].get(key), (base_of(p[0]).get("library") or {}).get(key), vs) if p[2] else "n/a"
                    for p in profiles
                )
                + " |"
            )
        out.append("")
    # sizeof
    if any(p[1] and p[1].get("sizeof") for p in profiles):
        out.append("#### engine objects, `sizeof` (bytes; the Domain is allocated from the heap at `start()`)\n")
        out.append("| object | " + " | ".join(names) + " |")
        out.append("|---|" + "---:|" * len(names))
        for key, label in SIZEOF_ROWS:
            out.append(
                f"| {label} | "
                + " | ".join(
                    with_vs(
                        (p[1] or {}).get("sizeof", {}).get(key),
                        (base_of(p[0]).get("sizeof") or {}).get(key),
                        vs,
                    )
                    for p in profiles
                )
                + " |"
            )
        out.append("")
    for name, mem, _ in profiles:
        if mem is None:
            out.append(f"#### {name}: no measurements (the firmware did not report; did it start?)\n")
            continue
        if mem.get("started") is False:
            reason = mem.get("start_error") or "no error logged"
            out.append(
                f"#### {name}: the participant did not start on this target - the profile does not fit "
                f"(`{reason}`); its sizeof is in the table above, the boot heap below\n"
            )
        out.extend(profile_tables(name, mem, base_of(name) if vs else None))
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="memory.json, or profile:memory.json:size-components.json")
    ap.add_argument("--summary", help="append the Markdown to this file as well")
    ap.add_argument("--json", help="write the merged numbers to this file")
    ap.add_argument("--baseline", help="the --json of an earlier run to show deltas against")
    ap.add_argument("--baseline-label", help="how to name the baseline in the report (a commit, a run)")
    args = ap.parse_args()
    baseline = load_json(args.baseline) if args.baseline else None
    if args.baseline and baseline is None:
        print(f"memory_report: no usable baseline at {args.baseline}; reporting without deltas", file=sys.stderr)

    profiles = []
    merged = {}
    for item in args.inputs:
        if ":" in item:
            # two separators at most: a path may itself contain a colon
            name, mem_path, size_path = (item.split(":", 2) + [None, None])[:3]
        else:
            name, mem_path, size_path = os.path.basename(os.path.dirname(os.path.abspath(item))) or "device", item, None
        mem = load_json(mem_path)
        lib = library_sizes(load_json(size_path))
        profiles.append((name, mem, lib))
        merged[name] = {"sizeof": (mem or {}).get("sizeof"), "stages": (mem or {}).get("stages"),
                        "steps": (mem or {}).get("steps"), "library": lib}

    text = report(profiles, baseline, args.baseline_label)
    print(text)
    if args.summary:
        with open(args.summary, "a") as f:
            f.write(text)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"profiles": merged}, f, indent=1)
    return 0 if any(p[1] for p in profiles) else 1


if __name__ == "__main__":
    sys.exit(main())
