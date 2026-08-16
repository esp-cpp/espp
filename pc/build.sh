#!/bin/bash
set -e

# Build the pc/ example tests against the INSTALLED espp package. Run
# ../lib/build.sh first; it installs espp into <repo>/install, which we point
# find_package(espp) at via CMAKE_PREFIX_PATH.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PREFIX="$REPO_ROOT/install"

cmake -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$SCRIPT_DIR/build" --config Release --parallel 4
