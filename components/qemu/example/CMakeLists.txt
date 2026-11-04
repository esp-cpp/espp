# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager pulls the virtual RGB panel driver
# (espressif/esp_lcd_qemu_rgb), declared as a dependency of the qemu BSP
# (components/qemu/idf_component.yml). The espp components come from the
# local tree via EXTRA_COMPONENT_DIRS.
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/base_peripheral"
  "../../../components/display"
  "../../../components/ethernet"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/lvgl"
  "../../../components/sdcard"
  "../../../components/task"
)

set(
  COMPONENTS
  "main esptool_py qemu"
  CACHE STRING
  "List of components to include"
  )

project(qemu_example)

set(CMAKE_CXX_STANDARD 23)
