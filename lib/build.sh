#!/bin/bash
set -e

# Build espp and install the find_package-able package (C++ static library +
# headers + esppConfig.cmake) together with the python `espp` package into a
# local staging prefix (<repo>/install). Point ../pc (and any external consumer)
# at it with -DCMAKE_PREFIX_PATH=<repo>/install; put <repo>/install on PYTHONPATH
# to `import espp`.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PREFIX="$REPO_ROOT/install"

# ESPP_BUILD_PYTHON=ON also builds/installs the python package (what CI
# publishes). ESPP_INSTALL=ON installs the find_package package into PREFIX.
cmake -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DESPP_INSTALL=ON \
  -DESPP_BUILD_PYTHON=ON \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$SCRIPT_DIR/build" --config Release --target install --parallel 4
