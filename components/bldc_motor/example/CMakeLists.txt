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
  "main esptool_py filters task monitor mt6701 bldc_motor bldc_driver i2c timer"
  CACHE STRING
  "List of components to include"
  )

project(bldc_motor_example)

set(CMAKE_CXX_STANDARD 20)
