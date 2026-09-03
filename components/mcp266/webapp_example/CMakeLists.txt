# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# This example needs the managed `espressif/esp_tinyusb` component (required by
# usb_device). It supports two build modes:
#
#  * DEFAULT (component manager ON) - how an end user builds it from the
#    component registry: the manager fetches esp_tinyusb (and its tinyusb
#    dependency) and the espp/* dependencies from the registry. EXTRA_COMPONENT_DIRS
#    is narrowed to just the components this example uses so the manager does not
#    scan every espp manifest (some board components declare target-specific
#    constraints that would fail on esp32s3).
#
#  * MANAGER OFF (IDF_COMPONENT_MANAGER=0) - used by CI so the build does not need
#    the (as-yet unpublished) espp/* components in the registry: every espp
#    dependency resolves locally from EXTRA_COMPONENT_DIRS, and esp_tinyusb /
#    tinyusb come from the vendored git submodules under external/ (esp_tinyusb
#    ships inside the espressif/esp-usb monorepo, so its component subdir is added).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/canopen"
  "../../../components/dispatcher"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/mcp266"
  "../../../components/stream_frame"
  "../../../components/task"
  "../../../components/twai"
  "../../../components/usb_device"
)

# With the component manager disabled, esp_tinyusb/tinyusb are not fetched from
# the registry; add the vendored submodule copies to the component search path.
# esp_tinyusb's own CMakeLists adds `tinyusb` to its REQUIRES when the manager is
# off, so both component directories must be discoverable here.
if(DEFINED ENV{IDF_COMPONENT_MANAGER} AND "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  list(APPEND EXTRA_COMPONENT_DIRS
    "../../../external/esp-usb/device/esp_tinyusb"
    "../../../external/tinyusb"
  )
endif()

set(
  COMPONENTS
  "main esptool_py base_component canopen dispatcher format logger mcp266 stream_frame task twai usb_device esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(mcp266_webapp_example)

set(CMAKE_CXX_STANDARD 20)
