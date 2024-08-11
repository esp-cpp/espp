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
  ${COMPONENTS}/rtsp/src/rtcp_packet.cpp
  ${COMPONENTS}/rtsp/src/rtp_packet.cpp
  ${COMPONENTS}/rtsp/src/rtsp_client.cpp
  ${COMPONENTS}/rtsp/src/rtsp_server.cpp
  ${COMPONENTS}/rtsp/src/rtsp_session.cpp
  ${COMPONENTS}/task/src/task.cpp
  ${COMPONENTS}/timer/src/timer.cpp
  ${COMPONENTS}/socket/src/socket.cpp
  ${COMPONENTS}/socket/src/tcp_socket.cpp
  ${COMPONENTS}/socket/src/udp_socket.cpp
  espp.cpp
)

# if we're on windows, we need to add wcswidth.c to the sources
if(MSVC)
  list(APPEND ESPP_SOURCES wcswidth.c)
endif()

# if we're on Windows, we need to link against ws2_32
if(WIN32)
  set(EXTERNAL_LIBS ws2_32)
endif()

include_directories(. ${EXTERNAL_INCLUDES} ${ESPP_INCLUDES})

set(LINK_ARG "--whole-archive")

# settings for Windows / MSVC
if(MSVC)
  add_compile_options(/utf-8 /D_USE_MATH_DEFINES /bigobj)
endif()

# settings for MacOS
if(APPLE)
  set(LINK_ARG "-all_load")
endif()

set(TARGET_NAME "espp_pc")

# main library (which can be built for pc, android, and iOS)
add_library( # Specifies the name of the library.
             ${TARGET_NAME}
             # Sets the library as a static (.a) library.
             STATIC
             # Provides a relative path to your source file(s).
             ${ESPP_SOURCES} )
set_property(TARGET ${TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_options(${TARGET_NAME} PRIVATE "${LINK_ARG}")
target_link_libraries(${TARGET_NAME} ${EXTERNAL_LIBS})
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
install(FILES espp.hpp wcswidth.h DESTINATION ${PROJECT_SOURCE_DIR}/pc/include/)

# and we need to include the pybind11 library
add_subdirectory(pybind11)

# NOTE: this is an alternate WIP way
# python_add_library(_core MODULE
#   ./python_bindings/module.cpp ./python_bindings/pybind_espp.cpp ${ESPP_SOURCES} WITH_SOABI)
# target_link_libraries(_core PRIVATE pybind11::headers)
# install(TARGETS _core DESTINATION ${PROJECT_SOURCE_DIR}/pc/)

# Python binding
pybind11_add_module(espp
  ./python_bindings/module.cpp ./python_bindings/pybind_espp.cpp ${ESPP_SOURCES})
target_compile_features(espp PRIVATE cxx_std_20)
install(TARGETS espp
        LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/pc/)
