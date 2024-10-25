cmake_minimum_required(VERSION 3.11)

# building PC c++ library and python binding
message(STATUS "Building for PC: C++ & Python")

project(espp)

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

# Build and install the python binding, which is built using pybind11, so we
# need to include the pybind11 library
add_subdirectory(pybind11)
espp_install_python_module(${PROJECT_SOURCE_DIR}/pc)
