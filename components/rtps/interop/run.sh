#!/usr/bin/env bash
# Host-side entry point: build the harness image and run the interop matrix.
# Usage: ./run.sh   (from components/rtps/interop)
set -euo pipefail
cd "$(dirname "$0")"
REPO_ROOT="$(cd ../../.. && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}" # jazzy (default) or humble, see the Dockerfile
docker build --build-arg "ROS_DISTRO=$ROS_DISTRO" -t "espp-rtps-interop-$ROS_DISTRO" .
exec docker run --rm -v "$REPO_ROOT":/work "espp-rtps-interop-$ROS_DISTRO" \
  bash /work/components/rtps/interop/run_interop.sh
