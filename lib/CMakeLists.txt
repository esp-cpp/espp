# 3.21+ for a reliable PROJECT_IS_TOP_LEVEL (used to default ESPP_INSTALL to ON
# only for a top-level build, so a FetchContent/CPM consumer's `cmake --install`
# never installs espp into the parent's prefix unless it opts in).
cmake_minimum_required(VERSION 3.21)

# ---------------------------------------------------------------------------
# Package version, derived from the latest git tag (strip a leading 'v' and keep
# the MAJOR.MINOR.PATCH numeric core). This flows into PROJECT_VERSION and thus
# esppConfigVersion.cmake, so `find_package(espp X.Y.Z)` version checks work.
# Falls back to 0.0.0 when git or a tag is unavailable (e.g. tarball builds) so
# configure still succeeds.
#
# NOTE: the python wheel gets its version independently from setuptools_scm (see
# pyproject.toml [tool.setuptools_scm] / SKBUILD_PROJECT_VERSION); this git-tag
# derivation is only for the CMake / find_package package version.
# ---------------------------------------------------------------------------
set(ESPP_VERSION "0.0.0")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE _espp_git_tag
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _espp_git_result)
  if(_espp_git_result EQUAL 0 AND _espp_git_tag)
    string(REGEX REPLACE "^v" "" _espp_git_tag "${_espp_git_tag}")
    if(_espp_git_tag MATCHES "^([0-9]+)(\\.[0-9]+)?(\\.[0-9]+)?")
      set(ESPP_VERSION "${CMAKE_MATCH_0}")
    endif()
  endif()
endif()

# Build the PC (host) C++ static library. Python bindings are opt-in (see the
# ESPP_BUILD_PYTHON option below); a plain `cmake -S lib` is C++-only.
project(espp VERSION ${ESPP_VERSION})
message(STATUS "espp package version: ${PROJECT_VERSION} (from git tag; 0.0.0 = no tag)")

# Build the espp Python bindings (_espp)? Default OFF so a plain
# `cmake -S lib` produces just the C++ static library + its install/export
# (find_package(espp) / espp::espp) with no pybind11 dependency. The standalone
# scripts (build.sh / build.ps1) pass -DESPP_BUILD_PYTHON=ON to also build and
# install the python package (the `espp/` package) alongside the C++ package
# under CMAKE_INSTALL_PREFIX (this is what CI publishes). Wheel builds via
# scikit-build-core take the SKBUILD path below regardless of this option.
option(ESPP_BUILD_PYTHON "Build the espp Python bindings module (_espp)" OFF)

# Install the standard, relocatable CMake package (install(TARGETS ... EXPORT),
# esppConfig.cmake, flattened headers) so a separate project can
# `find_package(espp)` and link `espp::espp`. This is THE install path (there is
# no longer a legacy source-tree lib/pc install). Defaults to ON for a top-level
# build (build.sh passes an explicit CMAKE_INSTALL_PREFIX, so no /usr/local
# surprise) and OFF when espp is a subproject, so a FetchContent/CPM consumer's
# `cmake --install` does not drag espp into the parent's prefix unless asked.
option(ESPP_INSTALL "Install espp as a find_package-able package (standard install/export under CMAKE_INSTALL_PREFIX)" ${PROJECT_IS_TOP_LEVEL})

# `SKBUILD` is set for ANY scikit-build-core configure in the tree - including
# when a DOWNSTREAM scikit-build-core project pulls espp in via FetchContent. In
# that case espp is not the top-level project and must still build the normal C++
# static library + the advertised `espp::espp` target, NOT just the `_espp` wheel
# module. So the wheel-only path is taken only when espp itself is the top-level
# scikit-build project (i.e. `pip wheel .` of espp).
if(SKBUILD AND PROJECT_IS_TOP_LEVEL)
  set(ESPP_WHEEL_BUILD ON)
else()
  set(ESPP_WHEEL_BUILD OFF)
endif()

# Report what this configure will actually build (Python is opt-in).
if(ESPP_WHEEL_BUILD OR ESPP_BUILD_PYTHON)
  message(STATUS "Building espp for PC: C++ static library + Python bindings (_espp)")
else()
  message(STATUS "Building espp for PC: C++ static library only (set -DESPP_BUILD_PYTHON=ON for Python bindings)")
endif()

# Prefer an installed pybind11 (provided as a build requirement when building
# python wheels via pip / scikit-build-core); fall back to fetching it with CPM
# for standalone CMake builds (e.g. ./build.sh). Only needed when the python
# bindings are actually being built.
if(ESPP_WHEEL_BUILD OR ESPP_BUILD_PYTHON)
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

# settings for Windows / MSVC
if(MSVC)
  add_compile_options(/utf-8 /D_USE_MATH_DEFINES /bigobj)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# NOTE: whole-archive linking (so global-ctor / registration code such as the
# Windows timer-period adjustment in espp.hpp is not stripped) is now the
# CONSUMER's responsibility, applied where espp::espp is linked (see
# pc/CMakeLists.txt, which wraps it with $<LINK_LIBRARY:WHOLE_ARCHIVE,...>). It
# is a no-op on the static archive itself (ar ignores link flags), so it does
# not belong here.

if(ESPP_WHEEL_BUILD)
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
  # (and, with -DESPP_BUILD_PYTHON=ON, the python package) and install the
  # find_package-able package into CMAKE_INSTALL_PREFIX.
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
  # PUBLIC so the system link deps (pthread, or ws2_32/winmm/iphlpapi on Windows)
  # propagate to consumers of the exported target.
  target_link_libraries(${TARGET_NAME} PUBLIC ${ESPP_EXTERNAL_LIBS})
  if(WIN32)
    target_link_libraries(${TARGET_NAME} PUBLIC winmm)
  endif()
  # PUBLIC/INTERFACE so consumers of espp::espp automatically compile as C++23
  # (the espp headers require it).
  target_compile_features(${TARGET_NAME} PUBLIC cxx_std_23)
  # PUBLIC so find_package(espp) consumers compile the rtps headers with the SAME
  # limits/fragmentation profile the archive was built with (ABI-critical; see
  # espp.cmake). These export into esppTargets.cmake as INTERFACE definitions.
  target_compile_definitions(${TARGET_NAME} PUBLIC ${ESPP_RTPS_COMPILE_DEFINITIONS})
  # PUBLIC MSVC usage requirements the exported target must carry so find_package /
  # FetchContent consumers on Windows compile espp's public headers. The
  # directory-level flags at the top only cover espp's own build (not the exported
  # INTERFACE), which is why the consumer smoke test caught these:
  #  - _USE_MATH_DEFINES: MSVC's <cmath> only defines M_PI / M_PI_2 when it is set
  #    before the include, and espp headers (math/fast_math.hpp, filters) use M_PI.
  #  - /utf-8: the vendored fmt (pulled in by logger.hpp etc.) static_asserts that
  #    Unicode support requires compiling with /utf-8.
  # (`MSVC` is also true for clang-cl, which needs both too.)
  if(MSVC)
    target_compile_definitions(${TARGET_NAME} PUBLIC _USE_MATH_DEFINES)
    target_compile_options(${TARGET_NAME} PUBLIC /utf-8)
  endif()

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
  # Proper install + export so `find_package(espp)` works from a separate
  # project. Installs into GNUInstallDirs under CMAKE_INSTALL_PREFIX. This is THE
  # install path; ESPP_INSTALL defaults ON for a top-level build (build.sh passes
  # an explicit prefix) and OFF for a subproject.
  # ---------------------------------------------------------------------------
  if(ESPP_INSTALL)
    espp_install_cmake_package(${TARGET_NAME})
  endif()

  # Build and install the python package (espp/ with the _espp extension inside)
  # into CMAKE_INSTALL_PREFIX (put <prefix> on PYTHONPATH to `import espp`).
  if(ESPP_BUILD_PYTHON)
    espp_install_python_module()
  endif()
endif()
