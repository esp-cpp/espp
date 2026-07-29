# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager is needed to pull the managed camera components
# (espressif/esp_video + espressif/esp_cam_sensor). They are declared as
# dependencies of the m5stack-tab5 BSP (components/m5stack-tab5/idf_component.yml)
# and fetched transitively, so they do not need to be listed here. The espp
# components are provided locally via EXTRA_COMPONENT_DIRS below.
#
# Enable the component manager explicitly so this example configures even if the
# build environment defaults it off (it is on by default in a normal ESP-IDF).
set(ENV{IDF_COMPONENT_MANAGER} "1")
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
  "../../../components/spi"
  "../../../components/st7123touch"
  "../../../components/task"
  "../../../components/touch"
)

set(
  COMPONENTS
  "main esptool_py m5stack-tab5 filters display lvgl"
  CACHE STRING
  "List of components to include"
  )

project(m5stack_tab5_example)

set(CMAKE_CXX_STANDARD 20)
