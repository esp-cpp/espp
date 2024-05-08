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

set(ESPP_INCLUDES
  ${EXTERNAL}/fmt/include
  ${EXTERNAL}/alpaca/include
  ${COMPONENTS}/base_component/include
  ${COMPONENTS}/base_peripheral/include
  ${COMPONENTS}/ftp/include
  ${COMPONENTS}/format/include
  ${COMPONENTS}/logger/include
  ${COMPONENTS}/rtsp/include
  ${COMPONENTS}/serialization/include
  ${COMPONENTS}/task/include
  ${COMPONENTS}/timer/include
  ${COMPONENTS}/socket/include
)

set(ESPP_SOURCES
  ${COMPONENTS}/logger/src/logger.cpp
  lib.cpp
)

include_directories(${ESPP_INCLUDES})

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
install(DIRECTORY ${ESPP_INCLUDES} DESTINATION ${PROJECT_SOURCE_DIR}/pc)
# Python binding
add_library( # Specifies the name of the library.
             espp
             # Sets the library as a shared (.so) library.
             MODULE
             # Provides a relative path to your source file(s).
             ./bind.cpp )
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
