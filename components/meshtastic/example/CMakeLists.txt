# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# NOTE: we don't use the IDF component manager for the examples in this
# repository since the examples use the local versions of the components
set(ENV{IDF_COMPONENT_MANAGER} "0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py meshtastic sx126x logger task t-deck m5stack-cardputer"
  CACHE STRING
  "List of components to include"
  )

project(meshtastic_example)

set(CMAKE_CXX_STANDARD 20)
