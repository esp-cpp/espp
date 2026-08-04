# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager must be enabled to fetch the epdiy dependency declared by
# the lilygo-t5-47 BSP. The espp components are provided locally via
# EXTRA_COMPONENT_DIRS below.
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use. List the specific espp
# component dirs (not the whole components/ tree) so the component manager does
# not try to validate esp32p4-only BSPs against this esp32s3 target.
set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/bm8563"
  "../../../components/format"
  "../../../components/gt911"
  "../../../components/i2c"
  "../../../components/input_drivers"
  "../../../components/interrupt"
  "../../../components/logger"
  "../../../components/lvgl"
  "../../../components/spi"
  "../../../components/task"
  "../../../components/touch"
)

set(
  COMPONENTS
  "main esptool_py lilygo-t5-47 lvgl"
  CACHE STRING
  "List of components to include"
  )

project(lilygo_t5_47_example)

set(CMAKE_CXX_STANDARD 20)
