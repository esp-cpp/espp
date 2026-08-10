cmake_minimum_required (VERSION 3.11)

set(CMAKE_CXX_STANDARD 20)

include(${CMAKE_CURRENT_SOURCE_DIR}/../lib/espp.cmake)


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


    target_include_directories(${TEST_NAME} PRIVATE
      ${curdir}/../lib/pc/include)
    target_link_directories(${TEST_NAME} PRIVATE
      ${curdir}/../lib/pc)
    target_link_libraries(${TEST_NAME}
      PRIVATE espp_pc
      PRIVATE ${ESPP_EXTERNAL_LIBS}
    )
    # /WHOLEARCHIVE is an MSVC/link.exe flag (gate on MSVC, not WIN32, so
    # MinGW/clang Windows builds don't receive it). It ensures the whole archive
    # is linked in, otherwise the Windows timer-period adjustment code (from
    # espp.hpp) is stripped and the timer runs at a max of ~64 Hz.
    if(MSVC)
      target_link_options(${TEST_NAME} PRIVATE "/WHOLEARCHIVE:espp_pc.lib")
    endif()
  ENDFOREACH()
ENDMACRO()

GEN_TESTS(${CMAKE_CURRENT_SOURCE_DIR})
