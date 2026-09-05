# The HFSM's C++ is GENERATED from Complex.json, not checked in.
#
# The model is the source; the C++ is a build product, the same as an
# object file. Keeping both in git meant they could disagree, and they
# did: the committed copy was generated in 2023 and had drifted from
# what the generator produces by roughly two thousand lines.
#
# Requires node (>= 18). Everything else comes from npx on demand.

set(HFSM_MODEL "${CMAKE_CURRENT_LIST_DIR}/Complex.json")

# Pinned so a build is reproducible. This is a git ref rather than an
# npm version because the packaging fix that makes `npx webgme-hfsm`
# work at all is not in a release yet; switch to "webgme-hfsm@^1.8.0"
# once it is published.
set(HFSM_GEN_SPEC "github:finger563/webgme-hfsm#3f4652e5b3c89b80e1ba7befb856f22aa959c02e"
    CACHE STRING "npm/npx spec for the webgme-hfsm code generator")

# Set this to skip npx entirely -- e.g. a local checkout while working
# on the generator:
#   idf.py -DHFSM_GEN_COMMAND="node;/path/to/webgme-hfsm/bin/hfsm-gen.js" build
set(HFSM_GEN_COMMAND "" CACHE STRING
    "Override the generator invocation (a ;-separated command)")

# Re-run cmake when the model changes, so editing it regenerates the
# C++ on the next build rather than on the next clean.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HFSM_MODEL}")

set(HFSM_DIR "${CMAKE_CURRENT_BINARY_DIR}/hfsm")

if(HFSM_GEN_COMMAND)
  set(_hfsm_cmd ${HFSM_GEN_COMMAND})
else()
  find_program(NPX_EXECUTABLE npx)
  if(NOT NPX_EXECUTABLE)
    message(FATAL_ERROR
      "This example generates its state machine from ${HFSM_MODEL} and needs "
      "node (>= 18) with npx on PATH.\n"
      "  Install node:  https://nodejs.org\n"
      "  Or point at a local generator checkout:\n"
      "    idf.py -DHFSM_GEN_COMMAND=\"node;/path/to/webgme-hfsm/bin/hfsm-gen.js\" build")
  endif()
  set(_hfsm_cmd "${NPX_EXECUTABLE}" -y -p "${HFSM_GEN_SPEC}" hfsm-gen)
endif()

# --no-support: generate THIS machine and not the shared runtime.
#
# The generator otherwise also emits state_base.hpp,
# deep_history_state.hpp, shallow_history_state.hpp and magic_enum.hpp,
# which are the same for every machine. This component already provides
# them -- espp's are the ones the rest of the codebase is built against,
# and its magic_enum is newer than the generator's -- so a second copy
# beside the generated machine would land on the include path and
# shadow them.
message(STATUS "hfsm: generating C++ from ${HFSM_MODEL}")
execute_process(
  COMMAND ${_hfsm_cmd} "${HFSM_MODEL}" -o "${HFSM_DIR}" --no-support
  RESULT_VARIABLE _hfsm_result
  OUTPUT_VARIABLE _hfsm_out
  ERROR_VARIABLE _hfsm_err)
if(NOT _hfsm_result EQUAL 0)
  message(FATAL_ERROR "hfsm generation failed:\n${_hfsm_out}${_hfsm_err}")
endif()

idf_component_register(
  SRCS "hfsm_example.cpp" "${HFSM_DIR}/Complex_generated_states.cpp"
  INCLUDE_DIRS "." "${HFSM_DIR}")
