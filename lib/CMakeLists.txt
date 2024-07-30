# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

# building PC c++ library and python binding
message(STATUS "Building for PC: C++ & Python")

project(espp)

set(CMAKE_COLOR_MAKEFILE   ON)

set(COMPONENTS "../components")
set(EXTERNAL "../external")

set(EXTERNAL_INCLUDES
  ${EXTERNAL}/alpaca/include
  ${EXTERNAL}/cli/include
  ${EXTERNAL}/csv2/include
  ${EXTERNAL}/hid-rp/hid-rp/
  ${EXTERNAL}/fmt/include
  ${EXTERNAL}/magic_enum/include/magic_enum/
  ${EXTERNAL}/tabulate/include
)

set(ESPP_INCLUDES
  ${COMPONENTS}/base_component/include
  ${COMPONENTS}/base_peripheral/include
  ${COMPONENTS}/color/include
  ${COMPONENTS}/csv/include
  ${COMPONENTS}/event_manager/include
  ${COMPONENTS}/file_system/include
  ${COMPONENTS}/ftp/include
  ${COMPONENTS}/format/include
  ${COMPONENTS}/hid-rp/include
  ${COMPONENTS}/logger/include
  ${COMPONENTS}/math/include
  ${COMPONENTS}/rtsp/include
  ${COMPONENTS}/serialization/include
  ${COMPONENTS}/tabulate/include
  ${COMPONENTS}/task/include
  ${COMPONENTS}/timer/include
  ${COMPONENTS}/socket/include
  ${COMPONENTS}/state_machine/include
)

set(ESPP_SOURCES
  ${COMPONENTS}/color/src/color.cpp
  ${COMPONENTS}/event_manager/src/event_manager.cpp
  ${COMPONENTS}/logger/src/logger.cpp
  ${COMPONENTS}/file_system/src/file_system.cpp
  ${COMPONENTS}/rtsp/src/rtsp_client.cpp
  ${COMPONENTS}/rtsp/src/rtsp_server.cpp
  ${COMPONENTS}/task/src/task.cpp
  ${COMPONENTS}/timer/src/timer.cpp
  ${COMPONENTS}/socket/src/socket.cpp
  ${COMPONENTS}/socket/src/tcp_socket.cpp
  ${COMPONENTS}/socket/src/udp_socket.cpp
  espp.cpp
)

include_directories(. ${EXTERNAL_INCLUDES} ${ESPP_INCLUDES})

# add /utf-8 flag for MSVC
if(MSVC)
  add_compile_options(/utf-8)

set(TARGET_NAME "espp_pc")
# to build for python we need to have the python developer libraries installed
find_package (Python3 COMPONENTS Interpreter Development)
if(APPLE)
  set(LINK_ARG "-all_load")
else()
  set(LINK_ARG "--whole-archive")
endif()

# main library (which can be built for pc, android, and iOS)
add_library( # Specifies the name of the library.
             ${TARGET_NAME}
             # Sets the library as a static (.a) library.
             STATIC
             # Provides a relative path to your source file(s).
             ${ESPP_SOURCES} )
set_property(TARGET ${TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_options(${TARGET_NAME} PRIVATE "${LINK_ARG}")
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_20)

# and install it into the hardware_tests/pc folder for use by the python scripts there
install(TARGETS ${TARGET_NAME}
        ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/pc)
# and install the headers for use by the c++ code there
install(DIRECTORY ${EXTERNAL_INCLUDES} DESTINATION ${PROJECT_SOURCE_DIR}/pc)
install(DIRECTORY ${ESPP_INCLUDES} DESTINATION ${PROJECT_SOURCE_DIR}/pc/)
# we have to make sure to install the magic_enum includes differently, since they're in different folders
install(DIRECTORY ${EXTERNAL}/magic_enum/include/magic_enum/ DESTINATION ${PROJECT_SOURCE_DIR}/pc/include/)
# and install the espp.hpp file for use by the python scripts there
install(FILES espp.hpp DESTINATION ${PROJECT_SOURCE_DIR}/pc/include/)
# Python binding
add_library( # Specifies the name of the library.
             espp
             # Sets the library as a shared (.so) library.
             MODULE
             # Provides a relative path to your source file(s).
             ./python_bindings/module.cpp python_bindings/pybind_espp.cpp )
set_property(TARGET espp PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(espp espp_pc pthread ${Python3_LIBRARIES})
target_include_directories(espp PRIVATE pybind11/include ${Python3_INCLUDE_DIRS})
target_link_options(espp PRIVATE ${Python3_LINK_OPTIONS})
target_compile_features(espp PRIVATE cxx_std_20)
# This changes the filename to `espp.so`
set_target_properties(espp PROPERTIES PREFIX "")
# and install it into the hardware_tests/pc folder for use by the python scripts there
install(TARGETS espp
        LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/pc/)
