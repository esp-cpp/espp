# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py bt nvs_flash filters driver task st25dv"
  CACHE STRING
  "List of components to include"
  )

project(st25dv_example)

set(CMAKE_CXX_STANDARD 20)
