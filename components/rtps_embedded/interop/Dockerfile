# FastDDS / ROS 2 interop harness image for the embeddedRTPS engine.
# ros:jazzy-ros-base ships FastDDS + rmw_fastrtps + the ros2 CLI, giving both the
# raw-DDS and ROS 2 sides of the interop matrix in one image. The espp repo is
# bind-mounted read-only at /work, copied to a container-local tree, and built
# there (the repo's lib/pc install dir carries the developer's host-platform
# artifacts and must not be clobbered with linux ones). See run_interop.sh.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git rsync python3-dev pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
