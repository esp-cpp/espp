# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# NOTE: -Wdouble-promotion (to keep this float-only example honest on targets
# without hardware double support) is applied to the example's own code in
# main/CMakeLists.txt, NOT globally: applying it project-wide also compiled it
# into ESP-IDF's own components, whose code trips it under IDF v6's newer GCC
# (e.g. esp_hw_support/port/esp32/rtc_clk.c) and fails their -Werror build.

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py logger math"
  CACHE STRING
  "List of components to include"
  )

project(math_example)

set(CMAKE_CXX_STANDARD 20)
