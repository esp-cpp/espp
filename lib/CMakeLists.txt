# 3.21+ for a reliable PROJECT_IS_TOP_LEVEL (used to guard the source-tree
# lib/pc install so it never fires when espp is consumed via FetchContent/CPM).
cmake_minimum_required(VERSION 3.21)

# Build the PC (host) C++ static library. Python bindings are opt-in (see the
# ESPP_BUILD_PYTHON option below); a plain `cmake -S lib` is C++-only.
project(espp VERSION 1.1.8)

# Build the espp Python bindings (_espp)? Default OFF so a plain
# `cmake -S lib` produces just the C++ static library + its install/export
# (find_package(espp) / espp::espp) with no pybind11 dependency. The standalone
# scripts (build.sh / build.ps1) pass -DESPP_BUILD_PYTHON=ON to also build and
# install the python package into lib/pc (this is what CI publishes). Wheel
# builds via scikit-build-core take the SKBUILD path below regardless of this
# option.
option(ESPP_BUILD_PYTHON "Build the espp Python bindings module (_espp)" OFF)

# Install the standard, relocatable CMake package (install(TARGETS ... EXPORT),
# esppConfig.cmake, flattened headers) so a separate project can
# `find_package(espp)` and link `espp::espp`. OFF by default so build.sh's plain
# `cmake --build . --target install` (which sets no CMAKE_INSTALL_PREFIX) does
# NOT write the export into the default system prefix (/usr/local). A real
# consumer opts in explicitly: `-DESPP_INSTALL=ON -DCMAKE_INSTALL_PREFIX=<prefix>`.
option(ESPP_INSTALL "Install espp as a find_package-able package (standard install/export under CMAKE_INSTALL_PREFIX)" OFF)

# Report what this configure will actually build (Python is opt-in).
if(SKBUILD OR ESPP_BUILD_PYTHON)
  message(STATUS "Building espp for PC: C++ static library + Python bindings (_espp)")
else()
  message(STATUS "Building espp for PC: C++ static library only (set -DESPP_BUILD_PYTHON=ON for Python bindings)")
endif()

# Prefer an installed pybind11 (provided as a build requirement when building
# python wheels via pip / scikit-build-core); fall back to fetching it with CPM
# for standalone CMake builds (e.g. ./build.sh). Only needed when the python
# bindings are actually being built.
if(SKBUILD OR ESPP_BUILD_PYTHON)
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
  # Namespaced ALIAS so build-tree (FetchContent / CPM add_subdirectory) and
  # install-tree (find_package) consumers link the SAME name: espp::espp.
  add_library(espp::espp ALIAS ${TARGET_NAME})
  # Export the target as `espp` (not `espp_pc`) so find_package consumers link
  # `espp::espp` too -- the SAME name as the build-tree alias above. The on-disk
  # archive name stays libespp_pc.a (OUTPUT_NAME unchanged) for pc/ and CI.
  set_target_properties(${TARGET_NAME} PROPERTIES EXPORT_NAME espp)
  set_property(TARGET ${TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_options(${TARGET_NAME} PRIVATE "${LINK_ARG}")
  # PUBLIC so the system link deps (pthread, or ws2_32/winmm/iphlpapi on Windows)
  # propagate to consumers of the exported target.
  target_link_libraries(${TARGET_NAME} PUBLIC ${ESPP_EXTERNAL_LIBS})
  if(WIN32)
    target_link_libraries(${TARGET_NAME} PUBLIC winmm)
  endif()
  # PUBLIC/INTERFACE so consumers of espp::espp automatically compile as C++23
  # (the espp headers require it).
  target_compile_features(${TARGET_NAME} PUBLIC cxx_std_23)

  # ---------------------------------------------------------------------------
  # Usage requirements (modern, target-based). Expose every include dir espp.cmake
  # collects as a BUILD_INTERFACE dir (so FetchContent/CPM consumers using the
  # build tree get them), and <prefix>/include as the INSTALL_INTERFACE dir.
  # ---------------------------------------------------------------------------
  include(GNUInstallDirs)
  foreach(_dir IN LISTS ESPP_INCLUDE_DIRS)
    target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${_dir}>)
  endforeach()
  target_include_directories(${TARGET_NAME}
    PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

  # ---------------------------------------------------------------------------
  # Legacy local install into lib/pc (kept for the pc/ test build and for the
  # libespp_* CI artifacts, which consume lib/pc/{include,libespp_pc.a}). This
  # writes into the espp SOURCE tree, so guard it to the standalone/CI build
  # (build.sh, top-level) and the python-package build -- it must NOT fire when
  # espp is a subproject and a parent project runs `cmake --install` (that would
  # mutate the espp source tree of a FetchContent/CPM consumer).
  # ---------------------------------------------------------------------------
  if(PROJECT_IS_TOP_LEVEL OR ESPP_BUILD_PYTHON)
    install(TARGETS ${TARGET_NAME}
            ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/pc)
    espp_install_includes(${PROJECT_SOURCE_DIR}/pc)
  endif()

  # ---------------------------------------------------------------------------
  # Proper install + export so `find_package(espp)` works from a separate
  # project. Installs into GNUInstallDirs under CMAKE_INSTALL_PREFIX. Opt-in via
  # -DESPP_INSTALL=ON so the plain build.sh install (no CMAKE_INSTALL_PREFIX)
  # does not write to the default system prefix (/usr/local).
  # ---------------------------------------------------------------------------
  if(ESPP_INSTALL)
    espp_install_cmake_package(${TARGET_NAME})
  endif()

  # Build and install the python package (espp/ with the _espp extension inside)
  if(ESPP_BUILD_PYTHON)
    espp_install_python_module(${PROJECT_SOURCE_DIR}/pc)
  endif()
endif()
