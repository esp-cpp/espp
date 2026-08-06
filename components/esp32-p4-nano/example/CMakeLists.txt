cmake_minimum_required(VERSION 3.20)

# The component manager is needed to pull the managed camera components
# (espressif/esp_video + espressif/esp_cam_sensor). They are declared as
# dependencies of the esp32-p4-nano BSP (components/esp32-p4-nano/idf_component.yml)
# and fetched transitively. Enable it explicitly so this example configures even
# if the build environment defaults it off (it is on by default in ESP-IDF).
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Enumerate the specific espp component dirs (rather than globbing the whole
# components/ tree): with the component manager enabled, every discovered
# component's manifest is target-checked, and some espp components (e.g. byte90)
# declare targets that exclude esp32p4. Listing only what this BSP needs avoids
# those false incompatibilities.
set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/base_peripheral"
  "../../../components/cli"
  "../../../components/codec"
  "../../../components/display"
  "../../../components/display_drivers"
  "../../../components/ethernet"
  "../../../components/format"
  "../../../components/gt911"
  "../../../components/i2c"
  "../../../components/input_drivers"
  "../../../components/interrupt"
  "../../../components/led"
  "../../../components/logger"
  "../../../components/lvgl"
  "../../../components/spi"
  "../../../components/task"
  "../../../components/touch"
)

set(
  COMPONENTS
  "main esptool_py esp32-p4-nano display lvgl"
  CACHE STRING
  "List of components to include"
  )

project(esp32_p4_nano_example)

set(CMAKE_CXX_STANDARD 20)
