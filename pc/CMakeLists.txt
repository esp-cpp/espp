
cmake_minimum_required (VERSION 3.11)

set(CMAKE_CXX_STANDARD 20)

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
	  add_executable(${TEST_NAME} ${test_file})
    target_include_directories(${TEST_NAME} PRIVATE
      ${curdir}/../lib/pc/include)
    target_link_directories(${TEST_NAME} PRIVATE
      ${curdir}/../lib/pc)
    target_link_libraries(${TEST_NAME}
      PRIVATE espp_pc
      PRIVATE pthread
    )
    install(TARGETS ${TEST_NAME} RUNTIME DESTINATION "bin")
  ENDFOREACH()
ENDMACRO()

GEN_TESTS(${CMAKE_CURRENT_SOURCE_DIR})
