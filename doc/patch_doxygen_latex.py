#!/usr/bin/env python3
from pathlib import Path
import sys


OLD_BLOCK = r"""\NewDocumentEnvironment{DoxyEnumFields}{O{2} m +b}{%
  \par
  \ifnum#1>2
    \DoxyParamTable{3}{lr}{#2}{#3}
  \else
    \DoxyParamTable{2}{l}{#2}{#3}
  \fi
}{}"""

NEW_BLOCK = r"""\NewDocumentEnvironment{DoxyEnumFields}{O{2} m +b}{%
  \par
  \begin{longtable}{|p{0.34\linewidth}|p{0.58\linewidth}|}
  \hline
  \textbf{#2} & \textbf{Description} \\
  \hline
  \endfirsthead
  \hline
  \textbf{#2} & \textbf{Description} \\
  \hline
  \endhead
  #3
  \end{longtable}
  \par\addvspace{6pt}%
}{}"""


def main() -> int:
    if len(sys.argv) != 2:
      print("usage: patch_doxygen_latex.py <latex_dir>", file=sys.stderr)
      return 2

    latex_dir = Path(sys.argv[1])
    style_path = latex_dir / "doxygen.sty"
    if not style_path.exists():
      print(f"doxygen.sty not found in {latex_dir}", file=sys.stderr)
      return 1

    text = style_path.read_text()
    if NEW_BLOCK in text:
      return 0
    if OLD_BLOCK not in text:
      print("expected DoxyEnumFields block not found in doxygen.sty", file=sys.stderr)
      return 1

    style_path.write_text(text.replace(OLD_BLOCK, NEW_BLOCK, 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
