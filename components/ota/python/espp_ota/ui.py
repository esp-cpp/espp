"""Terminal UI: a nice progress bar + colorized messages, with graceful fallback.

Two rendering paths so it looks good both standalone and under ``idf.py``:

* **Standalone, real terminal** — if `rich` is available (it ships in the
  ESP-IDF Python environment, and `pip install "espp[usb]"` pulls it in) we draw
  a `rich` progress bar (spinner, bar, %, bytes, transfer speed, ETA) and print
  colorized status/error lines.
* **Captured (e.g. under `idf.py ota-usb`)** — idf.py reads the target's output
  line-by-line and re-renders any line ending in ``(NN %)`` *in place* (the same
  mechanism that makes esptool's progress animate under `idf.py flash`). So there
  we emit throttled ``… (NN %)`` lines, which idf.py turns into a live in-place
  bar. `rich`'s own live display can't animate through that line capture, so it's
  intentionally only used on a real TTY.

Everything degrades to plain text; `rich` is optional.
"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional


def _isatty() -> bool:
    try:
        return sys.stderr.isatty()
    except Exception:
        return False


def _have_rich() -> bool:
    try:
        import rich  # noqa: F401
        return True
    except Exception:
        return False


def _ansi_enabled() -> bool:
    if os.environ.get("NO_COLOR") is not None:
        return False
    if os.environ.get("CLICOLOR_FORCE") or os.environ.get("FORCE_COLOR"):
        return True
    return _isatty()


class Console:
    """Styled status/error output (stderr). Uses rich when available."""

    def __init__(self) -> None:
        self._rich = None
        if _have_rich():
            try:
                from rich.console import Console as RichConsole
                self._rich = RichConsole(file=sys.stderr, highlight=False)
            except Exception:
                self._rich = None

    def _emit(self, text: str, rich_style: Optional[str], ansi: Optional[str]) -> None:
        if self._rich is not None:
            self._rich.print(text, style=rich_style, soft_wrap=True)
            return
        if ansi and _ansi_enabled():
            text = f"\033[{ansi}m{text}\033[0m"
        sys.stderr.write(text + "\n")
        sys.stderr.flush()

    def info(self, text: str) -> None:
        self._emit(text, None, None)

    def success(self, text: str) -> None:
        self._emit(text, "bold green", "1;32")

    def warn(self, text: str) -> None:
        self._emit(text, "yellow", "33")

    def error(self, text: str) -> None:
        # Distinct prefix so it stands out even when color is stripped (idf.py).
        self._emit(f"espp_ota ERROR: {text}", "bold red", "1;31")


class Progress:
    """OTA progress reporter; use as a context manager, feed :meth:`update`.

        with Progress(total, "Flashing", quiet) as p:
            client.flash(image)   # progress=p.update
    """

    def __init__(self, total: int, label: str = "Flashing", quiet: bool = False) -> None:
        self._total = total or 0
        self._label = label
        self._quiet = quiet
        self._rich = None
        self._task = None
        self._last_pct = -1000
        self._last_t = 0.0
        # rich's live bar can't animate through idf.py's line capture, so use it
        # only on a real terminal; otherwise emit "(NN %)" lines idf.py renders.
        self._use_rich = (not quiet) and _isatty() and _have_rich()

    def __enter__(self) -> "Progress":
        if self._use_rich:
            try:
                from rich.console import Console as RichConsole
                from rich.progress import (BarColumn, DownloadColumn, Progress as RichProgress,
                                           SpinnerColumn, TaskProgressColumn, TextColumn,
                                           TimeRemainingColumn, TransferSpeedColumn)
                self._rich = RichProgress(
                    SpinnerColumn(),
                    TextColumn("[bold blue]{task.description}"),
                    BarColumn(),
                    TaskProgressColumn(),
                    DownloadColumn(),
                    TransferSpeedColumn(),
                    TimeRemainingColumn(),
                    console=RichConsole(file=sys.stderr),
                )
                self._rich.start()
                self._task = self._rich.add_task(self._label, total=self._total or None)
            except Exception:
                self._rich = None  # fall back to text lines
        return self

    def update(self, written: int, total: int) -> None:
        if self._quiet:
            return
        if self._rich is not None:
            self._rich.update(self._task, completed=written, total=total or None)
            return
        now = time.monotonic()
        done = bool(total) and written >= total
        if total:
            pct = int(100 * written / total)
            # every 1% (and always the final frame). The line ends in "(NN %)" so
            # idf.py re-renders it in place; standalone it prints one line per %.
            if done or pct >= self._last_pct + 1:
                self._last_pct = 100 if done else pct
                bar = self._text_bar(self._last_pct)
                sys.stderr.write(f"  {self._label} {bar} {written // 1024:>5}/"
                                 f"{total // 1024} KB ({self._last_pct} %)\n")
                sys.stderr.flush()
        elif done or now - self._last_t >= 0.5:
            self._last_t = now
            sys.stderr.write(f"  {self._label} {written // 1024} KB\n")
            sys.stderr.flush()

    @staticmethod
    def _text_bar(pct: int, width: int = 24) -> str:
        filled = min(width, max(0, pct * width // 100))
        return "[" + "#" * filled + "-" * (width - filled) + "]"

    def __exit__(self, *exc) -> None:
        if self._rich is not None:
            try:
                self._rich.stop()
            except Exception:
                pass
