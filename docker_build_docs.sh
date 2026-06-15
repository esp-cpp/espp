#!/bin/sh
# add doc to the python path
export PYTHONPATH=$PYTHONPATH:/project/doc
# ensure we can run git commands
git config --global --add safe.directory /project
# install documentation extensions not guaranteed to be present in the base image
python -m pip install --user -r /project/doc/requirements.txt
# build the docs
build-docs -bs html -t esp32 -l en --project-path /project/ --source-dir /project/doc/ --doxyfile_dir /project/doc/
if ! sh /project/doc/build_latex_pdf.sh /project/_build/en/esp32/latex; then
  rm -f /project/_build/en/esp32/latex/refman.pdf
  rm -f /project/_build/en/esp32/latex/espp_documentation.pdf
  echo "Warning: PDF documentation was not generated successfully." >&2
fi
mkdir -p /project/docs
# copy the docs to the docs folder
cp -rf /project/_build/en/esp32/html/* /project/docs/.
cp -rf /project/_build/en/esp32/sphinx-warning-log.txt /project/.
cp -rf /project/_build/en/esp32/doxygen-warning-log.txt /project/.
