# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/base_peripheral"
  # "../../../components/ble_gatt_server"
  "../../../components/bmi270"
  # "../../../components/esp-nimble-cpp"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/cli"
  "../../../components/codec"
  "../../../components/display"
  "../../../components/display_drivers"
  "../../../components/esp-dsp"
  "../../../components/filters"
  "../../../components/gt911"
  "../../../components/i2c"
  "../../../components/ina226"
  "../../../components/input_drivers"
  "../../../components/interrupt"
  "../../../components/led"
  "../../../components/lvgl"
  "../../../components/math"
  "../../../components/rx8130ce"
  "../../../components/pi4ioe5v"
  "../../../components/task"
)

set(
  COMPONENTS
  "main esptool_py m5stack-tab5 filters display lvgl"
  CACHE STRING
  "List of components to include"
  )

project(m5stack_tab5_example)

set(CMAKE_CXX_STANDARD 20)
