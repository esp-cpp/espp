#!/usr/bin/env bash
# Host-side entry point for the QEMU interop matrix: build the harness image and
# run the embedded RTPS engine (the esp32 firmware in qemu/, built beforehand)
# in QEMU inside the container, against FastDDS, CycloneDDS / ROS 2 and the espp
# host engine on a tap device.
#
# Usage: ./run_qemu.sh [path/to/qemu_flash.bin] [matrix|soak]   (from components/rtps/interop)
#   matrix (default): the interop cases + fault injection; soak: minutes of churn, heap asserted
#
# The firmware image defaults to qemu/build/qemu_flash.bin, i.e.
#   cd qemu && idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ci.defaults" build \
#     && (cd build && python -m esptool --chip esp32 merge-bin --fill-flash-size 4MB \
#         -o qemu_flash.bin @flash_args)
#
# --network none: the tap device the runner creates is then the container's only
# network interface, so every DDS implementation picks it without per-vendor
# interface configuration. NET_ADMIN + /dev/net/tun: to create the tap and to
# shape it (tc netem) for the fault-injection cases.
set -euo pipefail
cd "$(dirname "$0")"
REPO_ROOT="$(cd ../../.. && pwd)"
FLASH="${1:-$PWD/qemu/build/qemu_flash.bin}"
[ -s "$FLASH" ] || { echo "firmware image not found: $FLASH (build components/rtps/interop/qemu first)"; exit 2; }
case "$FLASH" in "$REPO_ROOT"/*) FLASH_IN="/work${FLASH#"$REPO_ROOT"}" ;; *) echo "the image must live inside the repo ($REPO_ROOT)"; exit 2 ;; esac
ROS_DISTRO="${ROS_DISTRO:-jazzy}" # jazzy (default) or humble, see the Dockerfile
docker build --build-arg "ROS_DISTRO=$ROS_DISTRO" -t "espp-rtps-interop-$ROS_DISTRO" .
# MEMORY_STAGES_ONLY=1 (the memory suite on a profile that only boots for its
# markers) must reach the harness inside the container.
exec docker run --rm --network none --cap-add NET_ADMIN --device /dev/net/tun \
  ${MEMORY_STAGES_ONLY:+-e "MEMORY_STAGES_ONLY=$MEMORY_STAGES_ONLY"} \
  -v "$REPO_ROOT":/work "espp-rtps-interop-$ROS_DISTRO" \
  bash /work/components/rtps/interop/run_qemu_interop.sh "$FLASH_IN" "${2:-matrix}"
