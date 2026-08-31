# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(ENV{IDF_COMPONENT_MANAGER} "0")

# Set the C++ standard before including project.cmake / project() so the IDF
# build system picks it up for all components.
set(CMAKE_CXX_STANDARD 23)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py esp32-p4-wifi6-poe-eth"
  CACHE STRING
  "List of components to include"
  )

project(esp32_p4_wifi6_poe_eth_example)
