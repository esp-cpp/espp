cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py task i2c lp5817"
  CACHE STRING
  "List of components to include"
  )

project(lp5817_example)

set(CMAKE_CXX_STANDARD 20)
