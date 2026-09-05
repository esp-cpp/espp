# espp_generate_hfsm() -- turn an HFSM model into C++ at configure time.
#
# Included by the ESP-IDF build system before any component's
# CMakeLists is processed, so every component can call it.
#
# The model is the source and the C++ is a build product, like an
# object file. Checking generated code in lets the two disagree, and
# they do: this component's example carried a copy generated in 2023
# that had drifted from the generator by roughly two thousand lines.
#
# Requires node (>= 18). The generator itself comes from npx on demand.
#
#   espp_generate_hfsm(
#     MODEL             <model.json>          # required
#     OUTPUT_DIR        <dir>                 # required, wiped each configure
#     [SOURCES_VAR      <var>]                # -> generated .cpp files
#     [INCLUDE_DIR_VAR  <var>]                # -> dir for INCLUDE_DIRS
#     [NAMESPACE        <ns>]                 # -n, else the model's own
#     [WITH_SUPPORT]                          # also emit the shared runtime
#   )
#
# Typical use, from a component's CMakeLists.txt:
#
#   espp_generate_hfsm(
#     MODEL "${CMAKE_CURRENT_LIST_DIR}/Complex.json"
#     OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/hfsm"
#     SOURCES_VAR hfsm_srcs
#     INCLUDE_DIR_VAR hfsm_inc)
#
#   idf_component_register(SRCS "main.cpp" ${hfsm_srcs}
#                          INCLUDE_DIRS "." ${hfsm_inc})

# Pinned to a minor range: reproducible enough for a build that
# regenerates from a checked-in model, and picks up generator fixes
# without a commit here. 1.8.0 is the first release the CLI can be
# installed from at all, and the first with --no-support.
set(ESPP_HFSM_GEN_SPEC "webgme-hfsm@^1.8.0"
    CACHE STRING "npm/npx spec for the webgme-hfsm code generator")

# Skip npx entirely -- a local checkout while working on the
# generator, and the way to build with no network:
#   idf.py -DESPP_HFSM_GEN_COMMAND="node;/path/to/webgme-hfsm/bin/hfsm-gen.js" build
set(ESPP_HFSM_GEN_COMMAND "" CACHE STRING
    "Override the generator invocation (a ;-separated command)")

function(espp_generate_hfsm)
  cmake_parse_arguments(HFSM
    "WITH_SUPPORT"
    "MODEL;OUTPUT_DIR;SOURCES_VAR;INCLUDE_DIR_VAR;NAMESPACE"
    ""
    ${ARGN})

  if(HFSM_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "espp_generate_hfsm: unknown argument(s): ${HFSM_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT HFSM_MODEL)
    message(FATAL_ERROR "espp_generate_hfsm: MODEL is required")
  endif()
  if(NOT HFSM_OUTPUT_DIR)
    message(FATAL_ERROR "espp_generate_hfsm: OUTPUT_DIR is required")
  endif()
  if(NOT EXISTS "${HFSM_MODEL}")
    message(FATAL_ERROR "espp_generate_hfsm: no such model: ${HFSM_MODEL}")
  endif()

  # Re-run cmake when the model changes, so editing it regenerates on
  # the next build rather than the next clean. A function does not open
  # a directory scope, so this lands on the caller's directory.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${HFSM_MODEL}")

  if(ESPP_HFSM_GEN_COMMAND)
    set(_cmd ${ESPP_HFSM_GEN_COMMAND})
  else()
    find_program(ESPP_NPX_EXECUTABLE npx)
    if(NOT ESPP_NPX_EXECUTABLE)
      message(FATAL_ERROR
        "espp_generate_hfsm needs node (>= 18) with npx on PATH to generate "
        "${HFSM_MODEL}.\n"
        "  Install node:  https://nodejs.org\n"
        "  Or point at a local generator checkout:\n"
        "    idf.py -DESPP_HFSM_GEN_COMMAND=\"node;/path/to/webgme-hfsm/bin/hfsm-gen.js\" build")
    endif()
    # --yes so npx never stops to ask whether to install the package,
    # which in CI would hang rather than fail
    set(_cmd "${ESPP_NPX_EXECUTABLE}" --yes -p "${ESPP_HFSM_GEN_SPEC}" hfsm-gen)
  endif()

  set(_args "${HFSM_MODEL}" -o "${HFSM_OUTPUT_DIR}")
  if(NOT HFSM_WITH_SUPPORT)
    # Generate THIS machine and not the shared runtime: state_base.hpp,
    # the history states and magic_enum.hpp are the same for every
    # machine, and this component already provides them. espp's are the
    # ones the rest of the codebase is built against -- and its
    # magic_enum is newer than the generator's -- so a second copy
    # beside the generated machine would shadow them on the include
    # path.
    list(APPEND _args --no-support)
  endif()
  if(HFSM_NAMESPACE)
    list(APPEND _args -n "${HFSM_NAMESPACE}")
  endif()

  # Clear stale artifacts first. The generator only overwrites what it
  # emits, so a machine renamed or removed in the model would leave an
  # obsolete .cpp behind in the (persistent) binary dir for CMake to go
  # on compiling.
  file(REMOVE_RECURSE "${HFSM_OUTPUT_DIR}")
  file(MAKE_DIRECTORY "${HFSM_OUTPUT_DIR}")

  message(STATUS "hfsm: generating C++ from ${HFSM_MODEL}")
  execute_process(
    COMMAND ${_cmd} ${_args}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR "hfsm generation failed for ${HFSM_MODEL}:\n${_out}${_err}")
  endif()

  # Found rather than assumed: the file names come from the machine's
  # name in the model, so a caller would otherwise have to know what
  # its own model is called and keep that in step by hand.
  file(GLOB _sources "${HFSM_OUTPUT_DIR}/*.cpp")
  list(SORT _sources)
  if(NOT _sources)
    message(FATAL_ERROR
      "hfsm: ${HFSM_MODEL} generated no .cpp -- does it contain a State Machine?")
  endif()

  if(HFSM_SOURCES_VAR)
    set(${HFSM_SOURCES_VAR} "${_sources}" PARENT_SCOPE)
  endif()
  if(HFSM_INCLUDE_DIR_VAR)
    set(${HFSM_INCLUDE_DIR_VAR} "${HFSM_OUTPUT_DIR}" PARENT_SCOPE)
  endif()
endfunction()
