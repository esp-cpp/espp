# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/task"
  "../../../components/timer"
  "../../../components/hid-rp"
  "../../../components/usb_device"
  "../../../components/wdi"
)

# With the component manager disabled (IDF_COMPONENT_MANAGER=0, e.g. in CI so the
# build does not need the as-yet unpublished espp/* components in the registry),
# esp_tinyusb/tinyusb are not fetched from the registry; add the vendored
# submodule copies under external/ to the search path. esp_tinyusb's CMakeLists
# adds `tinyusb` to its REQUIRES when the manager is off, so both directories
# must be discoverable.
if(DEFINED ENV{IDF_COMPONENT_MANAGER} AND "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  list(APPEND EXTRA_COMPONENT_DIRS
    "../../../external/esp-usb/device/esp_tinyusb"
    "../../../external/tinyusb"
  )
endif()

set(
  COMPONENTS
  "main esptool_py base_component format logger task timer hid-rp usb_device wdi esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(wdi_usb_example)

set(CMAKE_CXX_STANDARD 20)
