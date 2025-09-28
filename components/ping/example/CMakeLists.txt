cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py cli ping wifi"
  CACHE STRING
  "List of components to include"
  )

project(ping_example)

set(CMAKE_CXX_STANDARD 20)
