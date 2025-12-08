# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  # NOTE: we add esp_vfs_console to be able to see the log output on the console
  # if we are running on ESP32-S2 and have enabled USB-CDC console output
  "main esptool_py logger esp_vfs_console"
  CACHE STRING
  "List of components to include"
  )

project(logger_example)

set(CMAKE_CXX_STANDARD 20)
