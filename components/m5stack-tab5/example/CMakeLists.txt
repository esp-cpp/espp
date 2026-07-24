# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager is needed to pull the managed camera components
# (espressif/esp_video + espressif/esp_cam_sensor) declared in main/idf_component.yml;
# the espp components are still provided locally via EXTRA_COMPONENT_DIRS below.
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
  "main esptool_py m5stack-tab5 filters display lvgl esp_video"
  CACHE STRING
  "List of components to include"
  )

project(m5stack_tab5_example)

set(CMAKE_CXX_STANDARD 20)
