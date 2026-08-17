#!/usr/bin/env bash
# Host-side entry point for the ODrive native serial-loopback interop test.
# Usage: ./run.sh   (from components/odrive_native/interop)
#
# Unlike the rtps interop (which needs a ROS 2 / FastDDS container), this test
# needs only a C++20 compiler, python3, and network access to fetch the reference
# fibre client, so it runs directly on the host -- no Docker required.
set -euo pipefail
cd "$(dirname "$0")"
exec bash ./run_interop.sh
