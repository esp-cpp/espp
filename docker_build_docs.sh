#!/bin/sh
# add doc to the python path
export PYTHONPATH=$PYTHONPATH:/project/doc
# ensure we can run git commands
git config --global --add safe.directory /project
# build the docs
build-docs -t esp32 -l en --project-path /project/ --source-dir /project/doc/ --doxyfile_dir /project/doc/
mkdir -p /project/docs
# copy the docs to the docs folder
cp -rf /project/_build/en/esp32/html/* /project/docs/.
