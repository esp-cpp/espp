"""Terminal UI: a nice progress bar + colorized messages, with graceful fallback.

The progress bar is drawn on the controlling terminal, so it animates in place
both standalone and under ``idf.py ota-usb`` (where the tool's stdout/stderr are
captured pipes — see ``_open_progress_stream``, which opens ``/dev/tty`` /
``CONOUT$`` to bypass the capture). If `rich` is available it draws a rich bar
(spinner, bar, %, bytes, transfer speed, ETA) and colorizes status/error lines;
otherwise it falls back to a manual ``\r`` bar, and to periodic plain-text lines
when there is no terminal at all (CI / redirected output).

`rich` is optional. It ships in the ESP-IDF Python environment (so
``idf.py ota-usb`` already has it) and is pulled in by ``pip install "espp[usb-ui]"``.
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


def _open_progress_stream():
    """A writable stream connected to the real terminal, plus an 'owned' flag.

    A live progress bar needs a terminal to animate on. Under ``idf.py ota-usb``
    the tool's stdout/stderr are captured pipes (so it forwards our lines one at a
    time, scrolling), but the process still has a *controlling terminal* — so we
    open ``/dev/tty`` (``CONOUT$`` on Windows) and draw the bar straight to it,
    bypassing the capture. Returns ``(stream, owned)`` or ``(None, False)`` when
    there is no terminal at all (CI, fully redirected)."""
    try:
        if sys.stderr.isatty():
            return sys.stderr, False
    except Exception:
        pass  # stderr may not support isatty() (e.g. a wrapped stream); fall through
    for name in ("/dev/tty", "CONOUT$"):
        try:
            return open(name, "w"), True
        except Exception:
            continue
    return None, False


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

    def note(self, text: str) -> None:
        """A prominent status line (e.g. the connected device) — bold cyan."""
        self._emit(text, "bold cyan", "1;36")

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
        self._rich = None       # rich Progress, when available
        self._task = None
        self._term = None       # a real-terminal stream for an in-place bar
        self._own_term = False
        self._plain = False     # draw a manual \r bar on self._term
        self._last_pct = -1000
        self._last_t = 0.0
        self._newline_done = False

    def __enter__(self) -> "Progress":
        if self._quiet:
            return self
        self._term, self._own_term = _open_progress_stream()
        if self._term is not None and _have_rich():
            try:
                from rich.console import Console as RichConsole
                from rich.progress import (BarColumn, DownloadColumn, Progress as RichProgress,
                                           SpinnerColumn, TaskProgressColumn, TextColumn,
                                           TimeRemainingColumn, TransferSpeedColumn)
                # force_terminal: the stream is a real tty (possibly /dev/tty) even
                # though our stdout/stderr were captured by idf.py.
                self._rich = RichProgress(
                    SpinnerColumn(),
                    TextColumn("[bold blue]{task.description}"),
                    BarColumn(),
                    TaskProgressColumn(),
                    DownloadColumn(),
                    TransferSpeedColumn(),
                    TimeRemainingColumn(),
                    console=RichConsole(file=self._term, force_terminal=True),
                )
                self._rich.start()
                self._task = self._rich.add_task(self._label, total=self._total or None)
            except Exception:
                self._rich = None
        if self._rich is None and self._term is not None:
            self._plain = True  # manual in-place bar on the terminal
        return self

    def update(self, written: int, total: int) -> None:
        if self._quiet:
            return
        if self._rich is not None:
            self._rich.update(self._task, completed=written, total=total or None)
            return
        now = time.monotonic()
        done = bool(total) and written >= total
        if self._plain:
            # in-place carriage-return bar on the real terminal (throttled ~10 Hz)
            if now - self._last_t < 0.1 and not done:
                return
            self._last_t = now
            if total:
                pct = min(100, int(100 * written / total))
                self._term.write(f"\r  {self._label} {self._text_bar(pct)} "
                                 f"{written // 1024}/{total // 1024} KB {pct:3d}%")
            else:
                self._term.write(f"\r  {self._label} {written // 1024} KB")
            if done:
                self._term.write("\n")
            self._term.flush()
            return
        # No terminal at all (CI / fully redirected): throttled newline lines.
        if total:
            pct = int(100 * written / total)
            if done or pct >= self._last_pct + 5:
                self._last_pct = 100 if done else pct
                sys.stderr.write(f"  {self._label} {written // 1024}/{total // 1024} KB "
                                 f"({self._last_pct} %)\n")
                sys.stderr.flush()
        elif done or now - self._last_t >= 1.0:
            self._last_t = now
            sys.stderr.write(f"  {self._label} {written // 1024} KB\n")
            sys.stderr.flush()

    @staticmethod
    def _text_bar(pct: int, width: int = 28) -> str:
        filled = min(width, max(0, pct * width // 100))
        return "[" + "#" * filled + "-" * (width - filled) + "]"

    def __exit__(self, *exc) -> None:
        if self._rich is not None:
            try:
                self._rich.stop()
            except Exception:
                pass  # tearing down the display must never raise
        try:
            if self._own_term and self._term is not None:
                self._term.close()
        except Exception:
            pass  # closing the borrowed /dev/tty handle is best-effort
