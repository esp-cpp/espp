# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

# This example needs the managed `espressif/esp_tinyusb` component (required by
# usb_device) and `joltwallet/littlefs` (required by file_system). It supports
# two build modes:
#
#  * DEFAULT (component manager ON) - how an end user builds it from the
#    component registry: the manager fetches esp_tinyusb (and its tinyusb
#    dependency), littlefs and the espp/* dependencies from the registry.
#    EXTRA_COMPONENT_DIRS is narrowed to just the components this example uses.
#
#  * MANAGER OFF (IDF_COMPONENT_MANAGER=0) - used by CI: every espp dependency
#    resolves locally from EXTRA_COMPONENT_DIRS, esp_tinyusb / tinyusb come
#    from the vendored git submodules under external/ and littlefs from the
#    components/littlefs submodule.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include($ENV{IDF_PATH}/tools/cmake/version.cmake)

# The CANopen app (simulated node or TWAI peripheral) is built on espp's twai
# component, which needs the IDF >= 6.0 esp_driver_twai node API (the simulated
# bus is a Twai drop-in and shares its Message type). The rest of the example
# builds on IDF 5.5, so the canopen / twai components are only pulled in when
# the IDF supports them -- or when overridden on the command line:
#   idf.py -DDESKTOP_EXAMPLE_CANOPEN=OFF build   (leave the CANopen app out)
# With it OFF, CONFIG_DESKTOP_EXAMPLE_ENABLE_CANOPEN has no effect (the app is
# not compiled; app_main logs a warning if it is enabled).
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
  set(DESKTOP_EXAMPLE_CANOPEN_DEFAULT ON)
else()
  set(DESKTOP_EXAMPLE_CANOPEN_DEFAULT OFF)
endif()
option(DESKTOP_EXAMPLE_CANOPEN "Build the CANopen / DS402 app (needs IDF >= 6.0 for twai)"
       ${DESKTOP_EXAMPLE_CANOPEN_DEFAULT})
# main/CMakeLists.txt needs the decision for its REQUIRES, which IDF evaluates
# in a separate script process (no CMake cache there): hand it over as a build
# property, which IDF writes into that process.
idf_build_set_property(DESKTOP_EXAMPLE_CANOPEN "${DESKTOP_EXAMPLE_CANOPEN}")
if(DESKTOP_EXAMPLE_CANOPEN)
  set(DESKTOP_EXAMPLE_CANOPEN_DIRS "../../../components/canopen" "../../../components/twai")
  set(DESKTOP_EXAMPLE_CANOPEN_COMPONENTS "canopen twai")
  message(STATUS "desktop example: CANopen / DS402 app ON (IDF ${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR})")
else()
  set(DESKTOP_EXAMPLE_CANOPEN_DIRS "")
  set(DESKTOP_EXAMPLE_CANOPEN_COMPONENTS "")
  message(STATUS "desktop example: CANopen / DS402 app OFF (DESKTOP_EXAMPLE_CANOPEN; twai needs IDF >= 6.0)")
endif()

set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  ${DESKTOP_EXAMPLE_CANOPEN_DIRS}
  "../../../components/cli"
  "../../../components/coredump"
  "../../../components/desktop"
  "../../../components/dispatcher"
  "../../../components/ethernet"
  "../../../components/file_system"
  "../../../components/format"
  "../../../components/i2c"
  "../../../components/logger"
  "../../../components/monitor"
  "../../../components/nvs"
  "../../../components/ota"
  "../../../components/stream_frame"
  "../../../components/system"
  "../../../components/task"
  "../../../components/usb_device"
  "../../../components/wifi"
)

if(DEFINED ENV{IDF_COMPONENT_MANAGER} AND "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  list(APPEND EXTRA_COMPONENT_DIRS
    "../../../components/littlefs"
    "../../../external/esp-usb/device/esp_tinyusb"
    "../../../external/tinyusb"
  )
endif()

# The component list is a cache entry like in the other examples, so a caller's
# -DCOMPONENTS=... wins; the CANopen-dependent default is only computed when
# the caller did not supply one. (main's REQUIRES still resolve canopen / twai
# from EXTRA_COMPONENT_DIRS whenever DESKTOP_EXAMPLE_CANOPEN is ON, whatever
# the list says: that decision travels as a build property, not through
# COMPONENTS.)
if(NOT DEFINED COMPONENTS)
  set(
    COMPONENTS
    "main esptool_py base_component desktop dispatcher file_system format logger monitor nvs stream_frame system task usb_device esp_tinyusb esp_psram coredump ota espcoredump littlefs i2c wifi ethernet cli ${DESKTOP_EXAMPLE_CANOPEN_COMPONENTS}"
    CACHE STRING
    "List of components to include"
    )
endif()

project(desktop_example)
