#!/usr/bin/env python3
from pathlib import Path
import re
import sys


OLD_NEWDOCUMENT_PATTERN = re.compile(
   r"""\\NewDocumentEnvironment\{DoxyEnumFields\}\{O\{2\} m \+b\}\{\%
.*?
\}\{\}""",
   re.DOTALL,
)

NEW_NEWDOCUMENT_BLOCK = r"""\NewDocumentEnvironment{DoxyEnumFields}{O{2} m +b}{%
 \par
 \begin{longtable}{|p{0.34\linewidth}|p{0.58\linewidth}|}
 \hline
 \multicolumn{2}{|l|}{\textbf{#2}} \\
 \hline
 \endfirsthead
 \hline
 \multicolumn{2}{|l|}{\textbf{#2}} \\
 \hline
 \endhead
 #3
 \end{longtable}
 \par\addvspace{6pt}%
}{}"""

OLD_NEWENV_PATTERN = re.compile(
   r"""\\newenvironment\{DoxyEnumFields\}\[1\]\{\%
.*?
\}\{\%
.*?
\}""",
   re.DOTALL,
)

NEW_NEWENV_BLOCK = r"""\newenvironment{DoxyEnumFields}[1]{%
   \par%
   \begin{longtable}{|p{0.34\linewidth}|p{0.58\linewidth}|}%
   \hline%
   \multicolumn{2}{|l|}{\textbf{#1}}\\%
   \hline%
   \endfirsthead%
   \hline%
   \multicolumn{2}{|l|}{\textbf{#1}}\\%
   \hline%
   \endhead%
}{%
   \end{longtable}%
   \vspace{6pt}%
}"""

ETOC_LINE = (
   r"\IfFormatAtLeastTF{2023/05/01}{\usepackage[deeplevels]{etoc}}"
   r"{\usepackage[deeplevels]{etoc_doxygen}}"
)
ETOC_REPLACEMENT = r"\usepackage[deeplevels]{etoc_doxygen}"
TOC_PRELUDE_PREFIX = r"\@ifundefined {etoctocstyle}"
ETOC_BLOCK_PATTERN = re.compile(
   r"""  % in page table of contents
  .*?
  \\etocsetlevel\{subparagraph\}\{9\}
""",
   re.DOTALL,
)
ETOC_BLOCK_REPLACEMENT = """  % printed table of contents disabled for PDF stability on large manuals
"""
TABLE_OF_CONTENTS_LINE = r"  \tableofcontents"
TABLE_OF_CONTENTS_REPLACEMENT = (
   "  % printed table of contents disabled for PDF stability on large manuals"
)


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
    if NEW_NEWDOCUMENT_BLOCK in text or NEW_NEWENV_BLOCK in text:
      replacements = 0
      updated_text = text
    else:
      updated_text, replacements = OLD_NEWDOCUMENT_PATTERN.subn(
          lambda _: NEW_NEWDOCUMENT_BLOCK, text, count=1
      )
      if replacements == 0:
        updated_text, replacements = OLD_NEWENV_PATTERN.subn(
            lambda _: NEW_NEWENV_BLOCK, text, count=1
        )
      if replacements == 0:
        print("expected DoxyEnumFields block not found in doxygen.sty", file=sys.stderr)
        return 1

    style_path.write_text(updated_text)

    refman_path = latex_dir / "refman.tex"
    if refman_path.exists():
      refman_text = refman_path.read_text()
      if ETOC_LINE in refman_text:
        refman_path.write_text(refman_text.replace(ETOC_LINE, ETOC_REPLACEMENT, 1))
        refman_text = refman_path.read_text()
      refman_text, _ = ETOC_BLOCK_PATTERN.subn(ETOC_BLOCK_REPLACEMENT, refman_text, count=1)
      if TABLE_OF_CONTENTS_LINE in refman_text:
        refman_text = refman_text.replace(
            TABLE_OF_CONTENTS_LINE, TABLE_OF_CONTENTS_REPLACEMENT, 1
        )
      refman_path.write_text(refman_text)

    toc_path = latex_dir / "refman.toc"
    if toc_path.exists():
      toc_lines = toc_path.read_text().splitlines(keepends=True)
      if toc_lines and toc_lines[0].startswith(TOC_PRELUDE_PREFIX):
        toc_path.write_text("".join(toc_lines[1:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
