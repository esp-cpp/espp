# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/cli"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/i2c"
  "../../../components/task"
  "../../../components/base_peripheral"
)

set(
  COMPONENTS
  "main esptool_py rx8130ce i2c"
  CACHE STRING
  "List of components to include"
  )

project(rx8130ce_example)

set(CMAKE_CXX_STANDARD 20)
