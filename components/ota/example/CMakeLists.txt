# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# NOTE: the IDF component manager is intentionally left ENABLED here (unlike most
# espp examples) so that it can fetch the managed `espressif/esp_tinyusb`
# dependency declared by the usb_device component's idf_component.yml. To avoid
# the component manager scanning every espp component manifest (some board
# components declare target-specific constraints that would fail on esp32s3),
# EXTRA_COMPONENT_DIRS is narrowed to just the components this example uses; the
# in-repo espp components there satisfy the `espp/*` dependencies locally (no
# example manifest / override_path needed).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/cli"
  "../../../components/dispatcher"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/nvs"
  "../../../components/ota"
  "../../../components/stream_frame"
  "../../../components/task"
  "../../../components/usb_device"
  "../../../components/wifi"
)

set(
  COMPONENTS
  "main esptool_py base_component cli dispatcher format logger nvs ota stream_frame task usb_device wifi esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(ota_example)

set(CMAKE_CXX_STANDARD 20)
