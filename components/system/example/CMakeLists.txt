# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

# This example needs the managed `espressif/esp_tinyusb` component (required by
# usb_device). It supports two build modes:
#
#  * DEFAULT (component manager ON) - how an end user builds it from the
#    component registry: the manager fetches esp_tinyusb (and its tinyusb
#    dependency) and the espp/* dependencies from the registry.
#    EXTRA_COMPONENT_DIRS is narrowed to just the components this example uses.
#
#  * MANAGER OFF (IDF_COMPONENT_MANAGER=0) - used by CI: every espp dependency
#    resolves locally from EXTRA_COMPONENT_DIRS, and esp_tinyusb / tinyusb come
#    from the vendored git submodules under external/.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/dispatcher"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/monitor"
  "../../../components/stream_frame"
  "../../../components/system"
  "../../../components/task"
  "../../../components/usb_device"
)

if(DEFINED ENV{IDF_COMPONENT_MANAGER} AND "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  list(APPEND EXTRA_COMPONENT_DIRS
    "../../../external/esp-usb/device/esp_tinyusb"
    "../../../external/tinyusb"
  )
endif()

set(
  COMPONENTS
  "main esptool_py base_component dispatcher format logger monitor stream_frame system task usb_device esp_tinyusb esp_psram"
  CACHE STRING
  "List of components to include"
  )

project(system_example)
