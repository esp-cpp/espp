cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../.."
)

set(
  COMPONENTS
  "main esptool_py esp_driver_uart esp_driver_usb_serial_jtag vfs odrive_ascii"
  CACHE STRING
  "List of components to include"
  )

project(odrive_ascii_example)

set(CMAKE_CXX_STANDARD 20)
