#!/usr/bin/env bash
# Host-side entry point: build the harness image and run the interop matrix.
# Usage: ./run.sh   (from components/rtps_embedded/interop)
set -euo pipefail
cd "$(dirname "$0")"
REPO_ROOT="$(cd ../../.. && pwd)"
docker build -t espp-rtps-interop .
exec docker run --rm -v "$REPO_ROOT":/work espp-rtps-interop \
  bash /work/components/rtps_embedded/interop/run_interop.sh
