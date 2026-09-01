# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# NOTE: the IDF component manager is intentionally left ENABLED here (unlike most
# espp examples) so it can fetch the managed `espressif/esp_tinyusb` dependency
# declared by the usb_device component. To avoid the manager scanning every espp
# component manifest (some board components declare target-specific constraints
# that would fail on esp32s3), EXTRA_COMPONENT_DIRS is narrowed to just the
# components this example uses; the in-repo espp components there satisfy the
# `espp/*` dependencies locally.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/dispatcher"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/stream_frame"
  "../../../components/task"
  "../../../components/twai"
  "../../../components/usb_device"
)

set(
  COMPONENTS
  "main esptool_py base_component dispatcher format logger stream_frame task twai usb_device esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(can_bridge_example)

set(CMAKE_CXX_STANDARD 20)
