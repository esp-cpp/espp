# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# Build the example (and the espp components it pulls in) as C++23. Must be
# set before the project.cmake include / project() call so IDF picks it up.
set(CMAKE_CXX_STANDARD 23)

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
  "../../../components/coredump"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/ota"
  "../../../components/task"
  "../../../components/usb_device"
)

set(
  COMPONENTS
  "main esptool_py base_component coredump format logger ota task usb_device esp_tinyusb espcoredump"
  CACHE STRING
  "List of components to include"
  )

project(coredump_example)
