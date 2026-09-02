# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# NOTE: the IDF component manager is intentionally left ENABLED here (unlike
# most espp examples) so that it can fetch the managed `espressif/esp_tinyusb`
# dependency declared by the usb_device component's idf_component.yml. To avoid
# the component manager scanning every espp component manifest (some board
# components declare target-specific constraints that would fail on esp32s3),
# EXTRA_COMPONENT_DIRS is narrowed to just the components this example uses
# (plus their transitive dependencies); the in-repo espp components there
# satisfy the `espp/*` dependencies locally.

# Build this example as C++23; must be set before including project.cmake so
# the IDF build system picks it up for all components.
set(CMAKE_CXX_STANDARD 23)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/adc"
  "../../../components/base_component"
  "../../../components/base_peripheral"
  "../../../components/bldc_driver"
  "../../../components/bldc_haptics"
  "../../../components/bldc_motor"
  "../../../components/bldc_types"
  "../../../components/cli"
  "../../../components/esp-dsp"
  "../../../components/filters"
  "../../../components/format"
  "../../../components/i2c"
  "../../../components/interrupt"
  "../../../components/led"
  "../../../components/logger"
  "../../../components/lsm6dso"
  "../../../components/magnetic_encoder"
  "../../../components/math"
  "../../../components/motorgo-axis"
  "../../../components/motorgo-mini"
  "../../../components/mt6701"
  "../../../components/ota"
  "../../../components/pid"
  "../../../components/spi"
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
  "main esptool_py bldc_driver bldc_haptics bldc_motor i2c motorgo-axis motorgo-mini mt6701 ota stream_frame task usb_device esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(bldc_haptics_example)
