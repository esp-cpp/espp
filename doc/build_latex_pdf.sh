#!/bin/sh
set -eu

if [ "$#" -ne 1 ]; then
  echo "usage: build_latex_pdf.sh <latex_dir>" >&2
  exit 2
fi

latex_dir=$1
if [ ! -d "$latex_dir" ]; then
  echo "latex directory not found: $latex_dir" >&2
  exit 1
fi
latex_dir=$(
  CDPATH= cd -- "$latex_dir" && pwd
)

script_dir=$(
  CDPATH= cd -- "$(dirname -- "$0")" && pwd
)

success=0
cleanup() {
  if [ "$success" -ne 1 ]; then
    rm -f "$latex_dir/refman.pdf" "$latex_dir/espp_documentation.pdf"
  fi
}
trap cleanup EXIT HUP INT TERM

rm -f \
  "$latex_dir/refman.pdf" \
  "$latex_dir/espp_documentation.pdf" \
  "$latex_dir/refman.aux" \
  "$latex_dir/refman.fdb_latexmk" \
  "$latex_dir/refman.fls" \
  "$latex_dir/refman.idx" \
  "$latex_dir/refman.ilg" \
  "$latex_dir/refman.ind" \
  "$latex_dir/refman.log" \
  "$latex_dir/refman.lof" \
  "$latex_dir/refman.lot" \
  "$latex_dir/refman.out" \
  "$latex_dir/refman.synctex.gz" \
  "$latex_dir/refman.toc"

python3 "$script_dir/patch_doxygen_latex.py" "$latex_dir"

cd "$latex_dir"
pdflatex -interaction=batchmode -halt-on-error refman.tex
python3 "$script_dir/patch_doxygen_latex.py" "$latex_dir"
pdflatex -interaction=batchmode -halt-on-error refman.tex

if [ ! -s refman.pdf ]; then
  echo "refman.pdf was not generated" >&2
  exit 1
fi

cp refman.pdf espp_documentation.pdf
success=1
