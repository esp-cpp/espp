# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/codec"
  "../../../components/display"
  "../../../components/display_drivers"
  "../../../components/filters"
  "../../../components/gt911"
  "../../../components/i2c"
  "../../../components/ina226"
  "../../../components/input_drivers"
  "../../../components/math"
  "../../../components/interrupt"
  "../../../components/pi4ioe5v"
  "../../../components/task"
  "../../../components/bmi270"
  "../../../components/base_peripheral"
)

set(
  COMPONENTS
  "main esptool_py m5stack-tab5 filters display lvgl"
  CACHE STRING
  "List of components to include"
  )

project(m5stack_tab5_example)

set(CMAKE_CXX_STANDARD 20)
