cmake_minimum_required(VERSION 3.20)

# building PC c++ library and python binding
message(STATUS "Building for PC: C++ & Python")

project(espp)

# Prefer an installed pybind11 (provided as a build requirement when building
# python wheels via pip / scikit-build-core); fall back to fetching it with CPM
# for standalone CMake builds (e.g. ./build.sh).
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  include(cmake/CPM.cmake)
  CPMAddPackage(
      NAME pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      VERSION 3.0.4
  )
  if(NOT pybind11_ADDED)
    message(FATAL_ERROR "pybind11 not found. Please ensure it is available in the specified version.")
  endif()
endif()

set(CMAKE_COLOR_MAKEFILE   ON)

include(espp.cmake)

include_directories(${ESPP_INCLUDE_DIRS})

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

if(SKBUILD)
  # Building a python wheel via scikit-build-core (pip install). Only build the
  # espp._espp extension module and install it into the `espp` package; the
  # pure-python package files come from lib/python_bindings/espp via the
  # `wheel.packages` setting in pyproject.toml.
  espp_add_python_module()
  install(TARGETS _espp
          LIBRARY DESTINATION espp
          RUNTIME DESTINATION espp)
else()
  # Standalone build (./build.sh / ./build.ps1): build the C++ static library
  # and the python package, and install both into ./pc for local use.
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

  espp_install_includes(${PROJECT_SOURCE_DIR}/pc)

  # Build and install the python package (espp/ with the _espp extension inside)
  espp_install_python_module(${PROJECT_SOURCE_DIR}/pc)
endif()
