cmake_minimum_required(VERSION 3.11)

# building PC c++ library and python binding
message(STATUS "Building for PC: C++ & Python")

project(espp)

set(CMAKE_COLOR_MAKEFILE   ON)

include(espp.cmake)

include_directories(${ESPP_INCLUDES})

set(LINK_ARG "--whole-archive")

# settings for Windows / MSVC
if(MSVC)
  add_compile_options(/utf-8 /D_USE_MATH_DEFINES /bigobj)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
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
target_link_libraries(${TARGET_NAME} ${ESPP_EXTERNAL_LIBS})
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_20)

# install build output and headers
install(TARGETS ${TARGET_NAME}
        ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/pc)
install(DIRECTORY ${ESPP_INCLUDES} DESTINATION ${PROJECT_SOURCE_DIR}/pc/)
# we have to make sure to install the magic_enum includes differently, since they're in different folders
install(DIRECTORY ${ESPP_EXTERNAL}/magic_enum/include/magic_enum/ DESTINATION ${PROJECT_SOURCE_DIR}/pc/include/)

# and we need to include the pybind11 library
add_subdirectory(pybind11)

# NOTE: this is an alternate WIP way
# python_add_library(_core MODULE
#   ./python_bindings/module.cpp ./python_bindings/pybind_espp.cpp ${ESPP_SOURCES} WITH_SOABI)
# target_link_libraries(_core PRIVATE pybind11::headers)
# install(TARGETS _core DESTINATION ${PROJECT_SOURCE_DIR}/pc/)

# Python binding
pybind11_add_module(espp ${ESPP_PYTHON_SOURCES})
target_compile_features(espp PRIVATE cxx_std_20)
target_link_libraries(espp PRIVATE ${ESPP_EXTERNAL_LIBS})
install(TARGETS espp
        LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/pc/)
