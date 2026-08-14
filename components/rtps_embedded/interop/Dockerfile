# FastDDS / ROS 2 interop harness image for the embeddedRTPS engine.
# ros:jazzy-ros-base ships FastDDS + rmw_fastrtps + the ros2 CLI, giving both the
# raw-DDS and ROS 2 sides of the interop matrix in one image. The espp repo is
# bind-mounted read-only at /work, copied to a container-local tree, and built
# there (the repo's lib/pc install dir carries the developer's host-platform
# artifacts and must not be clobbered with linux ones). See run_interop.sh.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git rsync python3-dev python3-pip python3-venv pybind11-dev \
    ros-jazzy-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

# A venv with the espp wheel build backend, so run_interop.sh can build the
# bindings in-container and functionally test them (python/rtps_rpc_demo.py). A
# venv (not --break-system-packages) avoids clashing with Ubuntu's debian-managed
# packaging. --system-site-packages so the built espp module can still see any
# system deps if needed; the demo itself is pure espp (no rclpy).
RUN python3 -m venv --system-site-packages /opt/espp-venv \
    && /opt/espp-venv/bin/pip install --no-cache-dir \
       scikit-build-core pybind11 setuptools-scm

WORKDIR /work
