# The HFSM's C++ is GENERATED from Complex.json, not checked in: the
# model is the source, the C++ is a build product.
#
# espp_generate_hfsm comes from the state_machine component's
# project_include.cmake. ESP-IDF does not load those during its early
# requirements pass, so the call is guarded -- SRCS and INCLUDE_DIRS
# are not read in that pass either, and the empty variables below are
# what it expects to see.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  espp_generate_hfsm(
    MODEL "${CMAKE_CURRENT_LIST_DIR}/Complex.json"
    OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/hfsm"
    SOURCES_VAR hfsm_srcs
    INCLUDE_DIR_VAR hfsm_inc)
endif()

idf_component_register(
  SRCS "hfsm_example.cpp" ${hfsm_srcs}
  INCLUDE_DIRS "." ${hfsm_inc})
