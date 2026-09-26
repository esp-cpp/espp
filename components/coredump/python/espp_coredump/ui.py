"""Terminal UI: a nice progress bar + colorized messages, with graceful fallback.

The progress bar is drawn on the controlling terminal, so it animates in place
both standalone and under ``idf.py coredump-usb`` (where the tool's stdout/stderr are
captured pipes — see ``_open_progress_stream``, which opens ``/dev/tty`` /
``CONOUT$`` to bypass the capture). If `rich` is available it draws a rich bar
(spinner, bar, %, bytes, transfer speed, ETA) and colorizes status/error lines;
otherwise it falls back to a manual ``\r`` bar, and to periodic plain-text lines
when there is no terminal at all (CI / redirected output).

`rich` is optional. It ships in the ESP-IDF Python environment (so
``idf.py coredump-usb`` already has it) and is pulled in by ``pip install "espp[usb-ui]"``.
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

    A live progress bar needs a terminal to animate on. Under ``idf.py coredump-usb``
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
    # Try only the terminal device for THIS platform. Using the wrong name (e.g.
    # "CONOUT$" on POSIX) would create a stray regular file in the cwd and write
    # progress there instead of falling back to stderr.
    term_name = "CONOUT$" if os.name == "nt" else "/dev/tty"
    try:
        return open(term_name, "w"), True
    except Exception:
        return None, False  # no controlling terminal (CI / redirected) -> stderr fallback


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


# How a crash-report line is styled inside a panel, by its "key:" prefix (the
# device's format_report() layout). (rich style, ANSI SGR code).
_REPORT_LINE_STYLES = (
    ("crashed task:", "bold red", "1;31"),
    ("backtrace:", "bold yellow", "1;33"),
    ("core dump:", "magenta", "35"),
    ("last reset:", "cyan", "36"),
    ("decode with:", "dim", "2"),
)


def report_line_style(line: str) -> tuple[Optional[str], Optional[str]]:
    """(rich style, ANSI code) for one crash-report line, (None, None) if plain."""
    for prefix, rich_style, ansi in _REPORT_LINE_STYLES:
        if line.startswith(prefix):
            return rich_style, ansi
    return None, None


def _stream_can_encode(stream, text: str) -> bool:
    """Whether ``text`` survives ``stream``'s encoding (a cp1252 / ASCII console
    cannot draw box characters; misconfigured locales report None)."""
    encoding = getattr(stream, "encoding", None) or "ascii"
    try:
        text.encode(encoding)
        return True
    except (UnicodeEncodeError, LookupError):
        return False


_BOX_UNICODE = ("╭─ ", "╮", "│ ", " │", "╰", "╯", "─")
_BOX_ASCII = ("+- ", "+", "| ", " |", "+", "+", "-")


def render_panel_plain(title: str, lines: list, color: bool = False, width: int = 0,
                       ascii_only: bool = False) -> str:
    """A framed, titled block as text (the rich-less fallback). Pure: testable.

    ``color`` adds ANSI styling per :func:`report_line_style`; ``width`` caps
    the frame (0 = fit the content). A line longer than the frame is wrapped
    (continuation rows indented), so every row is exactly as wide as the
    borders. ``ascii_only`` draws the frame with ``+ - |`` for streams whose
    encoding cannot represent the box-drawing characters."""
    import textwrap

    tl, tr, left, right, bl, br, h = _BOX_ASCII if ascii_only else _BOX_UNICODE
    inner = max([len(title) + 2] + [len(line) for line in lines]) if lines else len(title) + 2
    if width:
        inner = min(inner, max(width - 4, len(title) + 2))
    out = [tl + title + " " + h * max(0, inner - len(title) - 1) + tr]
    for line in lines:
        _, ansi = report_line_style(line) if color else (None, None)
        rows = textwrap.wrap(line, inner, subsequent_indent="  ", break_long_words=True,
                             break_on_hyphens=False) or [""]
        for row in rows:
            pad = " " * max(0, inner - len(row))
            text = f"\033[{ansi}m{row}\033[0m" if ansi else row
            out.append(left + text + pad + right)
    out.append(bl + h * (inner + 2) + br)
    return "\n".join(out)


class Console:
    """Styled status/error output (stderr) and framed results (stdout).
    Uses rich when available."""

    def __init__(self) -> None:
        self._rich = None
        self._rich_out = None
        if _have_rich():
            try:
                from rich.console import Console as RichConsole
                self._rich = RichConsole(file=sys.stderr, highlight=False)
                self._rich_out = RichConsole(file=sys.stdout, highlight=False)
            except Exception:
                self._rich = None
                self._rich_out = None

    def panel(self, title: str, body: str, border: str = "cyan") -> None:
        """Print a command's result as a framed, titled block on STDOUT, with
        the crash-report lines colorized, so it stands out from the build noise
        around it under ``idf.py coredump-usb``. Blank lines before and after
        separate it from ninja's last line and idf.py's post-build hints."""
        lines = body.rstrip("\n").split("\n") if body.strip() else []
        if self._rich_out is not None:
            try:
                from rich.panel import Panel
                from rich.text import Text
                text = Text()
                for i, line in enumerate(lines):
                    rich_style, _ = report_line_style(line)
                    text.append(line, style=rich_style)
                    if i + 1 < len(lines):
                        text.append("\n")
                self._rich_out.print()
                self._rich_out.print(Panel(text, title=f"[bold]{title}[/bold]", title_align="left",
                                           border_style=border, expand=False, padding=(0, 1)))
                self._rich_out.print()
                return
            except Exception:
                pass  # fall through to the plain frame
        try:
            width = os.get_terminal_size().columns
        except Exception:
            width = 0
        sys.stdout.write("\n" + render_panel_plain(title, lines, color=_ansi_enabled(), width=width,
                                                   ascii_only=not _stream_can_encode(sys.stdout, "╭│╯"))
                         + "\n\n")
        sys.stdout.flush()

    def rule(self, title: str = "") -> None:
        """A horizontal rule with an optional title on STDOUT (section divider)."""
        if self._rich_out is not None:
            try:
                from rich.rule import Rule
                self._rich_out.print(Rule(f"[bold]{title}[/bold]" if title else "", style="cyan"))
                return
            except Exception:
                pass
        try:
            width = os.get_terminal_size().columns
        except Exception:
            width = 80
        h = "─" if _stream_can_encode(sys.stdout, "─") else "-"
        line = f"{h}{h} {title} " if title else ""
        sys.stdout.write(line + h * max(0, width - len(line)) + "\n")
        sys.stdout.flush()

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
        self._emit(f"espp_coredump ERROR: {text}", "bold red", "1;31")


class Progress:
    """Transfer progress reporter; use as a context manager, feed :meth:`update`.

        with Progress(total, "Downloading", quiet) as p:
            client.read_image(progress=p.update)
    """

    def __init__(self, total: int, label: str = "Downloading", quiet: bool = False) -> None:
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
