# 3.24+ for the $<LINK_LIBRARY:WHOLE_ARCHIVE,...> generator expression used
# below (portable whole-archive linking across GNU ld / Apple / MSVC).
cmake_minimum_required (VERSION 3.24)

# Enable CXX at the top level so find_package(espp) -> find_dependency(Threads)
# (FindThreads needs an enabled C/CXX language) resolves. Individual tests still
# declare their own project()/name inside GEN_TESTS below.
project(espp_pc_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

# Consume the INSTALLED espp package. Configure with
#   -DCMAKE_PREFIX_PATH=<espp install prefix>
# where <espp install prefix> is where `cmake --install` put espp (see
# ../lib/build.sh, which installs into <repo>/install). The exported espp::espp
# target already carries its include dirs, C++23 requirement, and PUBLIC system
# libraries (pthread, or ws2_32/winmm/iphlpapi on Windows), so nothing else is
# needed here.
find_package(espp REQUIRED)

MACRO(GEN_TESTS curdir)
  # get test files
  FILE(GLOB tests RELATIVE ${curdir} ${curdir}/tests/*.cpp)
  # create test executables
  FOREACH(test_file ${tests})
    get_filename_component(TEST_NAME ${test_file} NAME_WE)
    project(${TEST_NAME}
      LANGUAGES CXX
      VERSION 1.0.0
    )
    # settings for Windows / MSVC
    if(MSVC)
      add_compile_options(/utf-8 /D_USE_MATH_DEFINES /bigobj)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    endif()
    add_executable(${TEST_NAME} ${test_file})

    # The odrive_native golden is header-only (host-buildable detail/ headers) and
    # is NOT part of the espp lib build, so give its target the component include
    # dir directly. It still links espp::espp below (harmless), matching the macro.
    if(TEST_NAME MATCHES "^odrive_native")
      target_include_directories(${TEST_NAME}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../components/odrive_native/include)
    endif()

    # Link the WHOLE archive so global-ctor / registration code is not stripped
    # from the static library -- e.g. the Windows timer-period adjustment in
    # espp.hpp, which otherwise gets dropped and caps the timer at ~64 Hz. The
    # $<LINK_LIBRARY:WHOLE_ARCHIVE,...> genex expands to the correct per-platform
    # flags around espp::espp only (GNU: -Wl,--whole-archive/--no-whole-archive,
    # Apple: -force_load, MSVC: /WHOLEARCHIVE), and pulls in the target's PUBLIC
    # usage requirements (includes, C++23, system libs) too.
    target_link_libraries(${TEST_NAME}
      PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,espp::espp>")
  ENDFOREACH()
ENDMACRO()

GEN_TESTS(${CMAKE_CURRENT_SOURCE_DIR})
