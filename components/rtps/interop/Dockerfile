# FastDDS / ROS 2 interop harness image for the embeddedRTPS engine.
# ros:jazzy-ros-base ships FastDDS + rmw_fastrtps + the ros2 CLI, giving both the
# raw-DDS and ROS 2 sides of the interop matrix in one image. rmw_cyclonedds_cpp is
# added so the ROS 2 pairs also run against CycloneDDS. The espp repo is
# bind-mounted read-only at /work, copied to a container-local tree, and built
# there (the repo's lib/pc install dir carries the developer's host-platform
# artifacts and must not be clobbered with linux ones). See run_interop.sh.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git rsync python3-dev python3-pip python3-venv pybind11-dev \
    ros-jazzy-example-interfaces ros-jazzy-rmw-cyclonedds-cpp \
    curl xz-utils iproute2 iputils-ping tcpdump \
    libpixman-1-0 libgcrypt20 libsdl2-2.0-0 libslirp0 zlib1g \
    && rm -rf /var/lib/apt/lists/*

# Espressif's QEMU (the esp32 machine) for run_qemu_interop.sh: the same build
# ESP-IDF's idf_tools.py installs, straight from its release tarball, so the
# image needs no ESP-IDF. The firmware is built outside (idf.py) and handed in
# as a merged flash image. The libraries above are what the binary links.
ARG QEMU_XTENSA_URL=https://github.com/espressif/qemu/releases/download/esp-develop-9.2.2-20260417/qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-x86_64-linux-gnu.tar.xz
# the checksum ESP-IDF pins for the same tarball (tools/tools.json, qemu-xtensa linux-amd64)
ARG QEMU_XTENSA_SHA256=0eecb2a34a5586c0e59110f77b9343b7b336e82fdb0e1a30e1dc1bab8a547e35
RUN curl -sSL -o /tmp/qemu-xtensa.tar.xz "$QEMU_XTENSA_URL" \
    && echo "$QEMU_XTENSA_SHA256  /tmp/qemu-xtensa.tar.xz" | sha256sum -c - \
    && tar -xJ -C /opt -f /tmp/qemu-xtensa.tar.xz \
    && rm /tmp/qemu-xtensa.tar.xz \
    && /opt/qemu/bin/qemu-system-xtensa --version

# A venv with the espp wheel build backend, so run_interop.sh can build the
# bindings in-container and functionally test them (python/rtps_rpc_demo.py). A
# venv (not --break-system-packages) avoids clashing with Ubuntu's debian-managed
# packaging. --system-site-packages so the built espp module can still see any
# system deps if needed; the demo itself is pure espp (no rclpy).
RUN python3 -m venv --system-site-packages /opt/espp-venv \
    && /opt/espp-venv/bin/pip install --no-cache-dir \
       scikit-build-core pybind11 setuptools-scm pycdr2

WORKDIR /work
