cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/android_tv_remote"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/task"
  "../../../components/wifi"
)

set(
  COMPONENTS
  "main esptool_py nvs_flash android_tv_remote wifi mdns"
  CACHE STRING
  "List of components to include"
  )

project(android_tv_remote_example)

set(CMAKE_CXX_STANDARD 20)
