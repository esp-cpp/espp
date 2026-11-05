# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager pulls the virtual RGB panel driver the qemu BSP
# depends on (unused here, but part of the BSP); the espp components come
# from the local tree.
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Only the components this firmware needs (the manager validates every
# component's manifest against the target, so the whole tree cannot be added).
set(EXTRA_COMPONENT_DIRS
  "../../../../components/base_component"
  "../../../../components/cdr"
  "../../../../components/display"
  "../../../../components/ethernet"
  "../../../../components/format"
  "../../../../components/led"
  "../../../../components/logger"
  "../../../../components/lvgl"
  "../../../../components/qemu"
  "../../../../components/reflect_cpp"
  "../../../../components/rtps"
  "../../../../components/sdcard"
  "../../../../components/socket"
  "../../../../components/task"
  "../../../../components/thread_pool"
  "../../../../components/timer"
)

set(
  COMPONENTS
  "main esptool_py logger cdr timer rtps qemu"
  CACHE STRING
  "List of components to include"
  )

project(rtps_qemu)

set(CMAKE_CXX_STANDARD 23)
