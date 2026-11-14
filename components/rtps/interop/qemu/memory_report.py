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
        every number is followed by its delta against the same number in the
        baseline (the --json of an earlier run, main's latest one on CI), in
        parentheses, so a pull request's cost shows as such, not as another
        absolute figure; --baseline-label alone (no usable baseline) makes the
        report say why there are no deltas
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


def cell(value, base_value, vs):
    """A number and, with a baseline, its delta against the baseline's in
    parentheses: for a size, bigger is a cost (positive); for a free-heap
    figure, smaller is a cost (negative). (n/a) when the baseline lacks it."""
    if not vs or value is None:
        return fmt(value)
    if base_value is None:
        return f"{fmt(value)} (n/a)"
    return f"{fmt(value)} ({delta(value, base_value)})"


def heap_row(label, s, prev, base, vs):
    """One row of a heap table: the free figures (each with its baseline
    delta), the deltas against the previous row, the largest internal block."""
    b = base or {}
    return (
        f"| {label} | {cell(s['internal'], b.get('internal'), vs)} "
        f"| {delta(s['internal'], prev['internal']) if prev else ''} "
        f"| {cell(s['internal_min'], b.get('internal_min'), vs)} | {fmt(s['internal_largest'])} "
        f"| {cell(s['psram'], b.get('psram'), vs)} | {delta(s['psram'], prev['psram']) if prev else ''} |"
    )


def keyed(items, key):
    """{item[key]: item} over the dicts of a list that carry `key`; tolerant
    of a list that is not one, or items that are not dicts."""
    if not isinstance(items, list):
        return {}
    return {it[key]: it for it in items if isinstance(it, dict) and key in it}


def profile_tables(name, mem, base=None):
    """The stage and step tables of one profile; with `base` (the same
    profile's numbers from the baseline run) every free figure carries its
    delta against the baseline in parentheses."""
    out = []
    vs = base is not None
    base_stages = keyed((base or {}).get("stages"), "stage")
    base_steps = keyed((base or {}).get("steps"), "step")
    header = "| {} | internal | delta | internal low-water mark | largest internal block | PSRAM | delta |"
    stages = keyed((mem or {}).get("stages") or [], "stage")
    if stages:
        out.append(f"#### {name}: heap by stage (free bytes; a cost is a negative delta)\n")
        out.append(header.format("stage"))
        out.append("|---|---:|---:|---:|---:|---:|---:|")
        prev = None
        for key in ("boot", "started", "endpoints"):
            s = stages.get(key)
            if s is None:
                out.append(f"| {key} | n/a | | | | | |")
                continue
            out.append(heap_row(key, s, prev, base_stages.get(key), vs))
            prev = s
        out.append("")
    steps = [st for st in ((mem or {}).get("steps") or []) if isinstance(st, dict) and "step" in st]
    if steps:
        out.append(
            f"#### {name}: heap after each host-driven step (free bytes; deltas against the previous row; "
            "the pools are preallocated, so a peer costs little heap - the low-water mark shows the transients)\n"
        )
        out.append(header.format("step"))
        out.append("|---|---:|---:|---:|---:|---:|---:|")
        prev = None
        for s in steps:
            out.append(heap_row(s["step"], s, prev, base_steps.get(s["step"]), vs))
            prev = s
        gone = [k for k in base_steps if k not in {s["step"] for s in steps}]
        if gone:
            out.append("")
            out.append(f"Steps in the baseline this run did not have: {', '.join(gone)}.")
        out.append("")
    return out


def report(profiles, baseline=None, baseline_label=None):
    """profiles: list of (name, memory dict or None, library dict or None).
    baseline: the {"profiles": {...}} of an earlier run's --json, whose numbers
    every table is compared against (deltas in parentheses); baseline_label
    names it - or, without a baseline, says why there is none."""
    out = ["### rtps memory footprint (QEMU, esp32)\n"]
    names = [p[0] for p in profiles]
    base_profiles = (baseline or {}).get("profiles") or {}
    vs = baseline is not None
    if vs:
        out.append(
            f"Deltas in parentheses are against {baseline_label or 'the baseline run'}: "
            "a bigger size or a smaller free heap than there is a cost of this change.\n"
        )
    elif baseline_label:
        out.append(f"No baseline to compare against ({baseline_label}): absolute numbers only.\n")

    def base_of(name):
        b = base_profiles.get(name)
        return b if isinstance(b, dict) else {}

    # the rtps library in the image
    if any(p[2] for p in profiles):
        out.append("#### librtps.a in the image (bytes, `idf.py size-components`)\n")
        out.append("| | " + " | ".join(names) + " |")
        out.append("|---|" + "---:|" * len(names))
        for key, label in LIBRARY_ROWS:
            out.append(
                f"| {label} | "
                + " | ".join(
                    cell(p[2].get(key), (base_of(p[0]).get("library") or {}).get(key), vs) if p[2] else "n/a"
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
                    cell(
                        ((p[1] or {}).get("sizeof") or {}).get(key),
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
    ap.add_argument("--baseline-label", help="how to name the baseline in the report (a commit, a run); "
                    "without --baseline: why there is none, printed in the report")
    args = ap.parse_args()
    baseline = load_json(args.baseline) if args.baseline else None
    if args.baseline and not (isinstance(baseline, dict) and isinstance(baseline.get("profiles"), dict)):
        # not this script's --json (a per-profile memory.json, say): no comparison
        print(f"memory_report: no usable baseline at {args.baseline} (not a --json of this script); "
              "reporting without deltas", file=sys.stderr)
        baseline = None

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
