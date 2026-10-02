#!/usr/bin/env python3
"""Generate the hosted web-app landing page and the app registry.

Run by .github/workflows/build_and_publish_docs.yml AFTER the per-component
web apps (components/*/web/*.html) are copied into docs/apps/. It writes,
next to the apps:

  index.html    the landing page: every app as a card, grouped by category,
                with a live search box (title / description / protocol id /
                transport / category) and a name-or-category sort
  registry.js   `window.ESPP_APPS = {...}` -- the same app table for pages
                that are published beside it (the Device Hub loads it as an
                optional sibling script to link every app that speaks a
                device's protocols, not only the one the firmware advertises)
  registry.json the same object, for tooling

Each app describes itself with tags in its <head>:

  <title>…</title>                                 card title
  <meta name="description" content="…">            card blurb
  <meta name="espp-category" content="…">          one of CATEGORIES
  <meta name="espp-protocols" content="…">         space-separated `id:version`
                                                   entries; a trailing `?` marks
                                                   an optional protocol (the app
                                                   works without it); omit when
                                                   the app speaks no espp
                                                   dispatcher protocol
  <meta name="espp-transports" content="…">        space-separated subset of
                                                   TRANSPORTS

so a new app is listed (and matched by the hub) by having those tags -- there
is no hand-maintained list. A page without a category, with an unknown
category / transport, or with a malformed protocol entry fails the build.

Usage: generate_apps_index.py <apps_dir>
"""

import html
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

# Section order on the landing page.
CATEGORIES = ["device management", "motor control", "bus tools", "input devices", "utilities"]
TRANSPORTS = ["webusb", "webserial", "webhid"]
TRANSPORT_LABEL = {"webusb": "WebUSB", "webserial": "Web Serial", "webhid": "WebHID"}
# A protocol id is a dotted, namespaced identifier (e.g. espp.can-bridge).
PROTOCOL_RE = re.compile(r"^([A-Za-z0-9_-]+(?:\.[A-Za-z0-9_-]+)+):([0-9]+)(\?)?$")


class AppError(Exception):
    pass


def meta(text: str, name: str):
    # each attribute value ends at the SAME quote character it opened with, so an
    # apostrophe inside a double-quoted description does not end it
    m = re.search(
        r'<meta\s+name=(["\'])' + re.escape(name) + r'\1\s+content=(["\'])(.*?)\2', text,
        re.S | re.I)
    return html.unescape(m.group(3).strip()) if m else None


def parse_protocols(spec: str, where: str):
    out = []
    for token in spec.split():
        m = PROTOCOL_RE.match(token)
        if not m:
            raise AppError(f"{where}: malformed espp-protocols entry {token!r} "
                           "(expected id:version, e.g. espp.ota:1, optional: espp.monitor:1?)")
        out.append({"id": m.group(1), "version": int(m.group(2)), "optional": m.group(3) == "?"})
    return out


def extract(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    title_m = re.search(r"<title>(.*?)</title>", text, re.S | re.I)
    title = html.unescape(title_m.group(1).strip()) if title_m else path.stem
    desc = meta(text, "description") or ""
    category = meta(text, "espp-category")
    if category is None:
        raise AppError(f"{path.name}: missing <meta name=\"espp-category\"> "
                       f"(one of: {', '.join(CATEGORIES)})")
    if category not in CATEGORIES:
        raise AppError(f"{path.name}: unknown espp-category {category!r} "
                       f"(one of: {', '.join(CATEGORIES)})")
    protocols = parse_protocols(meta(text, "espp-protocols") or "", path.name)
    transports = (meta(text, "espp-transports") or "").split()
    for t in transports:
        if t not in TRANSPORTS:
            raise AppError(f"{path.name}: unknown espp-transports entry {t!r} "
                           f"(one of: {', '.join(TRANSPORTS)})")
    return {"file": path.name, "title": title, "description": desc, "category": category,
            "protocols": protocols, "transports": transports}


def card(app: dict) -> str:
    chips = []
    for p in app["protocols"]:
        chips.append(f'<span class="chip proto{" opt" if p["optional"] else ""}" '
                     f'title="{"optional: the app works without it" if p["optional"] else "required"}">'
                     f'{html.escape(p["id"])} v{p["version"]}{"?" if p["optional"] else ""}</span>')
    for t in app["transports"]:
        chips.append(f'<span class="chip transport">{html.escape(TRANSPORT_LABEL[t])}</span>')
    search = " ".join([app["title"], app["description"], app["category"],
                       *[p["id"] for p in app["protocols"]],
                       *[TRANSPORT_LABEL[t] for t in app["transports"]], *app["transports"]])
    return (f'      <a class="card" href="{html.escape(app["file"])}" data-category="{html.escape(app["category"])}" '
            f'data-title="{html.escape(app["title"].casefold())}" data-search="{html.escape(search.casefold())}">\n'
            f'        <h2>{html.escape(app["title"])}</h2>\n'
            f'        <p>{html.escape(app["description"]) if app["description"] else "&nbsp;"}</p>\n'
            f'        <div class="chips">{"".join(chips)}</div>\n'
            f'      </a>')


def page(apps: list) -> str:
    sections = []
    for cat in CATEGORIES:
        members = sorted((a for a in apps if a["category"] == cat), key=lambda a: a["title"].casefold())
        if not members:
            continue
        sections.append(
            f'    <section class="group" data-category="{html.escape(cat)}">\n'
            f'      <h2 class="group-title">{html.escape(cat)} <span class="count" data-total="{len(members)}">{len(members)}</span></h2>\n'
            f'      <div class="grid">\n' + "\n".join(card(a) for a in members) + "\n"
            f'      </div>\n'
            f'    </section>')
    count = len(apps)
    body = "\n".join(sections)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>espp Web Apps</title>
<meta name="description" content="Browser tools hosted with the espp documentation: Web Serial / WebUSB / WebHID consoles, control panels, and flashers.">
<style>
  :root {{
    --bg: #ffffff; --fg: #1a1a2e; --muted: #666; --card: #f6f7f9;
    --border: #d9dce1; --accent: #2563eb; --chip: #e8edf5; --chip-fg: #334;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --bg: #14161a; --fg: #e6e6e6; --muted: #9aa0a6; --card: #1d2127;
      --border: #333842; --accent: #7aa2ff; --chip: #2a313c; --chip-fg: #cfd6e0;
    }}
  }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; padding: 2rem 1rem; background: var(--bg); color: var(--fg);
         font: 16px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }}
  main {{ max-width: 60rem; margin: 0 auto; }}
  .title-row {{ display: flex; flex-wrap: wrap; align-items: center; gap: .5rem 1rem; }}
  h1 {{ margin: 0 0 .25rem; }}
  .hub-link {{ display: inline-block; background: var(--accent); color: #fff; font-weight: 600; text-decoration: none;
         padding: .45rem .9rem; border-radius: .5rem; white-space: nowrap; }}
  .hub-link:hover {{ filter: brightness(1.1); }}
  .sub {{ color: var(--muted); margin: 0 0 1.25rem; }}
  .controls {{ display: flex; flex-wrap: wrap; gap: .75rem; align-items: center; margin: 0 0 1.5rem; }}
  .controls label {{ color: var(--muted); font-size: .92rem; display: flex; gap: .4rem; align-items: center; }}
  .controls input[type=search] {{ flex: 1 1 16rem; min-width: 12rem; padding: .5rem .7rem; font: inherit;
         color: var(--fg); background: var(--card); border: 1px solid var(--border); border-radius: .5rem; }}
  .controls select {{ padding: .45rem .6rem; font: inherit; color: var(--fg); background: var(--card);
         border: 1px solid var(--border); border-radius: .5rem; }}
  .controls input:focus-visible, .controls select:focus-visible, .card:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 2px; }}
  .group {{ margin: 0 0 1.75rem; }}
  .group-title {{ font-size: 1rem; text-transform: capitalize; color: var(--muted); margin: 0 0 .6rem;
         border-bottom: 1px solid var(--border); padding-bottom: .3rem; }}
  .group-title .count {{ font-weight: normal; }}
  .group[hidden] {{ display: none; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(16rem, 1fr)); gap: 1rem; }}
  body.flat .group-title {{ display: none; }}
  body.flat .group {{ margin: 0; }}
  body.flat .grid {{ display: contents; }}
  body.flat .group {{ display: none; }} /* the cards moved out; an empty wrapper must not take a grid slot */
  body.flat #all {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(16rem, 1fr)); gap: 1rem; }}
  .card {{ display: block; padding: 1rem 1.25rem; background: var(--card);
          border: 1px solid var(--border); border-radius: .6rem;
          color: inherit; text-decoration: none; }}
  .card:hover {{ border-color: var(--accent); }}
  .card[hidden] {{ display: none; }}
  .card h2 {{ margin: 0 0 .4rem; font-size: 1.05rem; color: var(--accent); }}
  .card p {{ margin: 0 0 .6rem; color: var(--muted); font-size: .92rem; }}
  .chips {{ display: flex; flex-wrap: wrap; gap: .3rem; }}
  .chip {{ font-size: .72rem; padding: .1rem .45rem; border-radius: .6rem; background: var(--chip); color: var(--chip-fg);
          font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }}
  .chip.transport {{ font-family: inherit; }}
  .chip.proto.opt {{ opacity: .75; border: 1px dashed var(--border); }}
  .empty {{ color: var(--muted); font-style: italic; }}
  .empty[hidden] {{ display: none; }}
  footer {{ margin-top: 2.5rem; color: var(--muted); font-size: .85rem; }}
  footer a {{ color: var(--accent); }}
</style>
</head>
<body>
  <main>
    <div class="title-row">
      <h1>espp Web Apps</h1>
      <a class="hub-link" href="dispatcher_hub.html" title="Connect a device and see which of these apps it can use">Device Hub &rarr;</a>
    </div>
    <p class="sub">{count} self-contained browser tools hosted with the espp
    documentation. They use the Web&nbsp;Serial / WebUSB / WebHID APIs
    (Chromium-based browsers) and talk directly to your hardware &mdash; nothing
    to install. Chips show the espp protocols an app speaks (a trailing
    <code>?</code> marks an optional one) and its transports.</p>
    <div class="controls" role="search">
      <input type="search" id="filter" placeholder="Filter by name, description, protocol, transport or category" aria-label="Filter apps">
      <label for="sort">Sort <select id="sort"><option value="category" selected>by category</option><option value="name">by name</option></select></label>
      <span class="count" id="shown" aria-live="polite"></span>
    </div>
    <div id="all">
{body}
    </div>
    <p class="empty" id="empty" hidden>No app matches the filter.</p>
    <footer>Part of the <a href="../index.html">espp documentation</a> &middot;
    <a href="https://github.com/esp-cpp/espp">esp-cpp/espp</a> &middot;
    <a href="registry.json">registry.json</a></footer>
  </main>
  <script>
    (function () {{
      const filter = document.getElementById("filter");
      const sort = document.getElementById("sort");
      const shown = document.getElementById("shown");
      const empty = document.getElementById("empty");
      const all = document.getElementById("all");
      const groups = Array.from(document.querySelectorAll(".group"));
      const cards = Array.from(document.querySelectorAll(".card"));
      const total = cards.length;
      // remember the filter / sort per browser (a convenience only)
      try {{
        filter.value = localStorage.getItem("espp.apps.filter") || "";
        sort.value = localStorage.getItem("espp.apps.sort") || "category";
      }} catch (_) {{}}
      function apply() {{
        const q = filter.value.trim().toLowerCase();
        const terms = q ? q.split(/\\s+/) : [];
        let visible = 0;
        for (const c of cards) {{
          const hay = c.dataset.search;
          const hit = terms.every((t) => hay.includes(t));
          c.hidden = !hit;
          if (hit) visible++;
        }}
        // Place the cards first, count the groups after: in name order the
        // cards sit directly under #all as one flat alphabetical list and every
        // group wrapper is hidden (an emptied wrapper would otherwise take a
        // grid slot); in category order they go back into their group's grid.
        // The card elements are moved, not copied, so their hidden state follows.
        const flat = sort.value === "name";
        document.body.classList.toggle("flat", flat);
        if (flat) {{
          const sorted = cards.slice().sort((a, b) => a.dataset.title.localeCompare(b.dataset.title));
          for (const c of sorted) all.appendChild(c);
        }} else {{
          for (const g of groups) {{
            const grid = g.querySelector(".grid");
            const mine = cards.filter((c) => c.dataset.category === g.dataset.category)
                              .sort((a, b) => a.dataset.title.localeCompare(b.dataset.title));
            for (const c of mine) grid.appendChild(c);
            all.appendChild(g);
          }}
        }}
        for (const g of groups) {{
          const n = g.querySelectorAll(".card:not([hidden])").length;
          g.hidden = flat || n === 0;
          const count = g.querySelector(".count");
          count.textContent = n === Number(count.dataset.total) ? String(n) : n + " / " + count.dataset.total;
        }}
        shown.textContent = visible === total ? total + " apps" : visible + " of " + total + " apps";
        empty.hidden = visible !== 0;
        try {{
          localStorage.setItem("espp.apps.filter", filter.value);
          localStorage.setItem("espp.apps.sort", sort.value);
        }} catch (_) {{}}
      }}
      filter.addEventListener("input", apply);
      sort.addEventListener("change", apply);
      filter.addEventListener("keydown", (e) => {{ if (e.key === "Escape") {{ filter.value = ""; apply(); }} }});
      apply();
    }})();
  </script>
</body>
</html>
"""


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    apps_dir = Path(sys.argv[1])
    pages = sorted(p for p in apps_dir.glob("*.html") if p.name != "index.html")
    if not pages:
        print(f"no apps found in {apps_dir}", file=sys.stderr)
        return 1
    try:
        apps = [extract(p) for p in pages]
    except AppError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    apps.sort(key=lambda a: (CATEGORIES.index(a["category"]), a["title"].casefold()))
    for a in apps:
        protos = " ".join(f'{p["id"]}:{p["version"]}{"?" if p["optional"] else ""}' for p in a["protocols"])
        print(f'  indexed: {a["file"]} -> {a["title"]} [{a["category"]}]'
              + (f" {{{protos}}}" if protos else ""))
    registry = {"generated": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
                "categories": CATEGORIES, "apps": apps}
    (apps_dir / "index.html").write_text(page(apps), encoding="utf-8")
    (apps_dir / "registry.json").write_text(json.dumps(registry, indent=2) + "\n", encoding="utf-8")
    (apps_dir / "registry.js").write_text(
        "// Generated by doc/generate_apps_index.py: the hosted espp web apps and the\n"
        "// protocols each one speaks. Loaded as an optional sibling script by pages\n"
        "// published beside it (the Device Hub); do not edit.\n"
        "window.ESPP_APPS = " + json.dumps(registry, indent=2) + ";\n", encoding="utf-8")
    print(f"wrote {apps_dir / 'index.html'}, registry.js, registry.json ({len(apps)} apps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
