#!/usr/bin/env python3
"""Generate docs/apps/index.html from the hosted single-file web apps.

Run by .github/workflows/build_and_publish_docs.yml AFTER the per-component
web apps (components/*/web/*.html) are copied into docs/apps/. Each app's
card title comes from its <title> and its blurb from
<meta name="description" content="...">, so a new app is listed automatically
by simply having those two tags -- no registry to maintain.

Usage: generate_apps_index.py <apps_dir>
"""

import html
import re
import sys
from pathlib import Path


def extract(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    title_m = re.search(r"<title>(.*?)</title>", text, re.S | re.I)
    desc_m = re.search(
        r'<meta\s+name=["\']description["\']\s+content=["\'](.*?)["\']', text, re.S | re.I)
    title = html.unescape(title_m.group(1).strip()) if title_m else path.stem
    desc = html.unescape(desc_m.group(1).strip()) if desc_m else ""
    return title, desc


def main() -> int:
    apps_dir = Path(sys.argv[1])
    apps = [p for p in apps_dir.glob("*.html") if p.name != "index.html"]
    if not apps:
        print(f"no apps found in {apps_dir}", file=sys.stderr)
        return 1

    # Sort by display title (case-insensitive) so the grid reads alphabetically
    # regardless of file name.
    entries = sorted(((extract(p), p.name) for p in apps),
                     key=lambda e: e[0][0].casefold())
    cards = []
    for (title, desc), name in entries:
        cards.append(
            f'      <a class="card" href="{html.escape(name)}">\n'
            f"        <h2>{html.escape(title)}</h2>\n"
            f"        <p>{html.escape(desc) if desc else '&nbsp;'}</p>\n"
            f"      </a>")
        print(f"  indexed: {name} -> {title}")
    count = len(entries)

    page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>espp Web Apps</title>
<meta name="description" content="Browser tools hosted with the espp documentation: Web Serial / WebUSB / WebHID consoles, control panels, and flashers.">
<style>
  :root {{
    --bg: #ffffff; --fg: #1a1a2e; --muted: #666; --card: #f6f7f9;
    --border: #d9dce1; --accent: #2563eb;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --bg: #14161a; --fg: #e6e6e6; --muted: #9aa0a6; --card: #1d2127;
      --border: #333842; --accent: #7aa2ff;
    }}
  }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; padding: 2rem 1rem; background: var(--bg); color: var(--fg);
         font: 16px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }}
  main {{ max-width: 60rem; margin: 0 auto; }}
  h1 {{ margin: 0 0 .25rem; }}
  .sub {{ color: var(--muted); margin: 0 0 2rem; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(16rem, 1fr)); gap: 1rem; }}
  .card {{ display: block; padding: 1rem 1.25rem; background: var(--card);
          border: 1px solid var(--border); border-radius: .6rem;
          color: inherit; text-decoration: none; }}
  .card:hover {{ border-color: var(--accent); }}
  .card h2 {{ margin: 0 0 .4rem; font-size: 1.05rem; color: var(--accent); }}
  .card p {{ margin: 0; color: var(--muted); font-size: .92rem; }}
  footer {{ margin-top: 2.5rem; color: var(--muted); font-size: .85rem; }}
  footer a {{ color: var(--accent); }}
</style>
</head>
<body>
  <main>
    <h1>espp Web Apps</h1>
    <p class="sub">{count} self-contained browser tools hosted with the espp
    documentation. They use the Web&nbsp;Serial / WebUSB / WebHID APIs
    (Chromium-based browsers) and talk directly to your hardware &mdash; nothing
    to install.</p>
    <div class="grid">
{chr(10).join(cards)}
    </div>
    <footer>Part of the <a href="../index.html">espp documentation</a> &middot;
    <a href="https://github.com/esp-cpp/espp">esp-cpp/espp</a></footer>
  </main>
</body>
</html>
"""
    (apps_dir / "index.html").write_text(page, encoding="utf-8")
    print(f"wrote {apps_dir / 'index.html'} ({len(apps)} apps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
