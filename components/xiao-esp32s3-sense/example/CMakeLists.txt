# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/cli"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/monitor"
  "../../../components/rtsp"
  "../../../components/sdcard"
  "../../../components/socket"
  "../../../components/task"
  "../../../components/thread_pool"
  "../../../components/wifi"
  "../../../components/xiao-esp32s3-sense"
)

set(
  COMPONENTS
  "main esptool_py cli esp32-camera mdns monitor rtsp task thread_pool wifi xiao-esp32s3-sense"
  CACHE STRING
  "List of components to include"
  )

project(xiao_esp32s3_sense_example)

set(CMAKE_CXX_STANDARD 20)

# The mDNS component (espressif/mdns from the registry, or the vendored copy)
# trips -Werror=stringop-truncation under the ESP-IDF v6 toolchain (GCC 15). It
# is third-party code this example only consumes, so keep the warning but do not
# let it fail the build. Scoped to that one component; espp code stays -Werror.
idf_build_get_property(_build_components BUILD_COMPONENTS)
foreach(_mdns_name IN ITEMS mdns espressif__mdns)
  if("${_mdns_name}" IN_LIST _build_components)
    idf_component_get_property(_mdns_lib ${_mdns_name} COMPONENT_LIB)
    target_compile_options(${_mdns_lib} PRIVATE -Wno-error=stringop-truncation)
  endif()
endforeach()
