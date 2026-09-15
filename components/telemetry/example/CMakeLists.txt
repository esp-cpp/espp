# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# Build the example (and the espp components it pulls in) as C++20 (the repo
# standard; the telemetry service needs no C++23 feature). Must be set before
# the project.cmake include / project() call so IDF picks it up.
set(CMAKE_CXX_STANDARD 20)

# NOTE: the IDF component manager is intentionally left ENABLED here (unlike most
# espp examples) so that it can fetch the managed `espressif/esp_tinyusb`
# dependency declared by the usb_device component's idf_component.yml. To avoid
# the component manager scanning every espp component manifest (some board
# components declare target-specific constraints that would fail on esp32s3),
# EXTRA_COMPONENT_DIRS is narrowed to just the components this example uses; the
# in-repo espp components there satisfy the `espp/*` dependencies locally.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/dispatcher"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/telemetry"
  "../../../components/stream_frame"
  "../../../components/task"
  "../../../components/timer"
  "../../../components/usb_device"
)

# With the component manager disabled (IDF_COMPONENT_MANAGER=0, e.g. in CI so the
# build does not need the as-yet unpublished espp/* components in the registry),
# esp_tinyusb/tinyusb are not fetched; add the vendored submodule copies under
# external/ to the search path. esp_tinyusb's CMakeLists adds `tinyusb` to its
# REQUIRES when the manager is off, so both directories must be discoverable.
if(DEFINED ENV{IDF_COMPONENT_MANAGER} AND "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  list(APPEND EXTRA_COMPONENT_DIRS
    "../../../external/esp-usb/device/esp_tinyusb"
    "../../../external/tinyusb"
  )
endif()

set(
  COMPONENTS
  "main esptool_py base_component dispatcher format logger telemetry stream_frame task timer usb_device esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(telemetry_example)
