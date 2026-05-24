# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED IDF_TARGET)
  set(IDF_TARGET esp32s3 CACHE STRING "ESP-IDF target")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/cli"
  "../../../components/format"
  "../../../components/home_assistant"
  "../../../components/logger"
  "../../../components/wifi"
)

set(
  COMPONENTS
  "main esptool_py nvs_flash home_assistant wifi"
  CACHE STRING
  "List of components to include"
  )

project(home_assistant_example)

set(CMAKE_CXX_STANDARD 20)
