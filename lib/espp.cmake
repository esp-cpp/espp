set(ESPP_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../components")

# ---------------------------------------------------------------------------
# RTPS static-limits profile selection.
#
# The rtps engine keeps a fully-static (deterministic) allocation model
# whose compile-time capacity caps are chosen by a profile header (see
# components/rtps/include/rtps/config.hpp). Profiles:
#   embedded   - tight MCU caps (rtps/config_esp32.hpp)
#   host       - relaxed static caps, DEFAULT for non-ESP builds
#                (rtps/config_desktop.hpp)
#   host_large - generous static caps for large DDS graphs
#                (rtps/config_host_large.hpp)
#
# Select with -DRTPS_LIMITS_PROFILE=embedded|host|host_large. Default is "host"
# (this file is only used for non-ESP / host builds; ESP/IDF builds pick the
# profile via Kconfig in components/rtps/Kconfig).
# ---------------------------------------------------------------------------
if(NOT DEFINED RTPS_LIMITS_PROFILE)
  set(RTPS_LIMITS_PROFILE "host")
endif()
set(RTPS_LIMITS_PROFILE "${RTPS_LIMITS_PROFILE}" CACHE STRING
    "RTPS static limits profile: embedded|host|host_large")
set_property(CACHE RTPS_LIMITS_PROFILE PROPERTY STRINGS embedded host host_large)

if(RTPS_LIMITS_PROFILE STREQUAL "embedded")
  set(RTPS_CONFIG_HEADER_FILE "rtps/config_esp32.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 262144)   # 256 KB (matches config_esp32.hpp)
elseif(RTPS_LIMITS_PROFILE STREQUAL "host")
  set(RTPS_CONFIG_HEADER_FILE "rtps/config_desktop.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 8388608)  # 8 MB (matches config_desktop.hpp)
elseif(RTPS_LIMITS_PROFILE STREQUAL "host_large")
  set(RTPS_CONFIG_HEADER_FILE "rtps/config_host_large.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 8388608)  # 8 MB (matches config_host_large.hpp)
else()
  message(FATAL_ERROR
    "Invalid RTPS_LIMITS_PROFILE '${RTPS_LIMITS_PROFILE}' "
    "(expected: embedded | host | host_large)")
endif()
add_compile_definitions(RTPS_CONFIG_HEADER="${RTPS_CONFIG_HEADER_FILE}")
message(STATUS "RTPS limits profile: ${RTPS_LIMITS_PROFILE}")

# ---------------------------------------------------------------------------
# RTPS per-limit capacity overrides (fine-grained alternative to switching
# profiles): a semicolon list of NAME=VALUE entries, each overriding ONE
# capacity cap of the selected profile via its RTPS_CFG_<NAME> macro - e.g.
#   -DRTPS_LIMIT_OVERRIDES="NUM_STATELESS_WRITERS=16;HISTORY_SIZE_STATEFUL=20"
# See the RTPS_CFG_* blocks in include/rtps/config_*.hpp for the knob names.
# Applied as GLOBAL compile definitions so the engine sources (which size the
# pools) and every consumer translation unit agree on the values - defining
# them for only a consumer TU would silently disagree with the library.
# Capacity-only: no bytes on the wire change.
# ---------------------------------------------------------------------------
set(RTPS_LIMIT_OVERRIDES "" CACHE STRING
    "Semicolon list of RTPS capacity overrides, e.g. NUM_STATELESS_WRITERS=16;HISTORY_SIZE_STATEFUL=20")
# The supported knobs (must match the RTPS_CFG_* blocks in the profile headers).
# All of them back uint8_t constants, so values are bounded to 1..255 - an
# unvalidated 256 would silently truncate to a ZERO-capacity pool, and a typoed
# name would silently define an unused macro (i.e. no override at all).
set(RTPS_LIMIT_KNOB_NAMES
    NUM_STATELESS_WRITERS NUM_STATELESS_READERS NUM_STATEFUL_WRITERS NUM_STATEFUL_READERS
    MAX_NUM_PARTICIPANTS NUM_WRITERS_PER_PARTICIPANT NUM_READERS_PER_PARTICIPANT
    NUM_WRITER_PROXIES_PER_READER NUM_READER_PROXIES_PER_WRITER
    MAX_NUM_UNMATCHED_REMOTE_WRITERS MAX_NUM_UNMATCHED_REMOTE_READERS
    MAX_NUM_READER_CALLBACKS HISTORY_SIZE_STATELESS HISTORY_SIZE_STATEFUL
    MAX_TYPENAME_LENGTH MAX_TOPICNAME_LENGTH)
foreach(override ${RTPS_LIMIT_OVERRIDES})
  if(NOT override MATCHES "^([A-Z_]+)=([0-9]+)$")
    message(FATAL_ERROR "Invalid RTPS_LIMIT_OVERRIDES entry '${override}' (expected NAME=VALUE)")
  endif()
  string(REGEX REPLACE "^([A-Z_]+)=[0-9]+$" "\\1" _rtps_knob "${override}")
  string(REGEX REPLACE "^[A-Z_]+=([0-9]+)$" "\\1" _rtps_value "${override}")
  if(NOT _rtps_knob IN_LIST RTPS_LIMIT_KNOB_NAMES)
    message(FATAL_ERROR
      "Unknown RTPS limit knob '${_rtps_knob}' in RTPS_LIMIT_OVERRIDES. Supported knobs: "
      "${RTPS_LIMIT_KNOB_NAMES}")
  endif()
  if(_rtps_value LESS 1 OR _rtps_value GREATER 255)
    message(FATAL_ERROR
      "RTPS limit override '${override}' out of range: all knobs are uint8_t, valid range 1..255")
  endif()
  add_compile_definitions("RTPS_CFG_${override}")
  message(STATUS "RTPS limit override: ${override}")
endforeach()

# ---------------------------------------------------------------------------
# RTPS best-effort DATA_FRAG fragmentation (Slice C).
#
# On host builds fragmentation is ALWAYS enabled: samples larger than a single
# DATA submessage are split into DATA_FRAG submessages and reassembled by the
# peer (interoperates with FastDDS / ROS 2). RTPS_MAX_SAMPLE_SIZE is the
# large-sample reassembly cap and is kept in sync with the profile's
# Config::MAX_SAMPLE_SIZE. On ESP32/IDF fragmentation is opt-in via Kconfig (see
# components/rtps/Kconfig / CMakeLists.txt), default off, so the MCU
# pays nothing for it by default.
# ---------------------------------------------------------------------------
add_compile_definitions(RTPS_ENABLE_FRAGMENTATION RTPS_MAX_SAMPLE_SIZE=${RTPS_MAX_SAMPLE_SIZE})
message(STATUS "RTPS fragmentation: ON (max sample size ${RTPS_MAX_SAMPLE_SIZE} bytes)")

# The RTPS static-limits + fragmentation config is compiled into libespp_pc.a AND
# is part of the public ABI: any consumer that compiles rtps headers (templates /
# inline code) must use the SAME profile, else it sees a different Config and a
# different set of guarded declarations (e.g. addSubMessageDataFrag). Expose them
# as PUBLIC compile definitions on the exported target (see lib/CMakeLists.txt) so
# find_package(espp) consumers inherit them automatically. The above
# add_compile_definitions still covers the in-tree python module / SKBUILD build.
set(ESPP_RTPS_COMPILE_DEFINITIONS
  RTPS_CONFIG_HEADER="${RTPS_CONFIG_HEADER_FILE}"
  RTPS_ENABLE_FRAGMENTATION
  RTPS_MAX_SAMPLE_SIZE=${RTPS_MAX_SAMPLE_SIZE})

set(ESPP_EXTERNAL_INCLUDES
  ${ESPP_COMPONENTS}/serialization/detail/alpaca/include
  ${ESPP_COMPONENTS}/cli/detail/cli/include
  ${ESPP_COMPONENTS}/csv/detail/csv2/include
  ${ESPP_COMPONENTS}/format/detail/fmt/include
  ${ESPP_COMPONENTS}/tabulate/detail/tabulate/include
)

# NOTE: these are separate because they do not follow the standard format of
# having their include files be in the "include" directory, so when we install
# them we need to handle them separately
set(ESPP_EXTERNAL_INCLUDES_SEPARATE
  ${ESPP_COMPONENTS}/hid-rp/detail/hid-rp/hid-rp/
  ${ESPP_COMPONENTS}/state_machine/detail/magic_enum/include/magic_enum/
)

set(ESPP_INCLUDES
  ${ESPP_COMPONENTS}/base_component/include
  ${ESPP_COMPONENTS}/base_peripheral/include
  ${ESPP_COMPONENTS}/cdr/include
  ${ESPP_COMPONENTS}/cdr/detail/cdr/include
  ${ESPP_COMPONENTS}/reflect_cpp/detail/reflect-cpp/include
  ${ESPP_COMPONENTS}/cobs/include
  ${ESPP_COMPONENTS}/color/include
  ${ESPP_COMPONENTS}/csv/include
  ${ESPP_COMPONENTS}/event_manager/include
  ${ESPP_COMPONENTS}/file_system/include
  ${ESPP_COMPONENTS}/filters/include
  ${ESPP_COMPONENTS}/ftp/include
  ${ESPP_COMPONENTS}/format/include
  ${ESPP_COMPONENTS}/hid-rp/include
  ${ESPP_COMPONENTS}/joystick/include
  ${ESPP_COMPONENTS}/logger/include
  ${ESPP_COMPONENTS}/math/include
  ${ESPP_COMPONENTS}/ndef/include
  ${ESPP_COMPONENTS}/odrive_native/include
  ${ESPP_COMPONENTS}/pid/include
  ${ESPP_COMPONENTS}/rtps/include
  ${ESPP_COMPONENTS}/rtsp/include
  ${ESPP_COMPONENTS}/serialization/include
  ${ESPP_COMPONENTS}/tabulate/include
  ${ESPP_COMPONENTS}/task/include
  ${ESPP_COMPONENTS}/thread_pool/include
  ${ESPP_COMPONENTS}/timer/include
  ${ESPP_COMPONENTS}/trajectory_planner/include
  ${ESPP_COMPONENTS}/socket/include
  ${ESPP_COMPONENTS}/state_machine/include
  ${CMAKE_CURRENT_LIST_DIR}/include
)

set(ESPP_SOURCES
  ${ESPP_COMPONENTS}/cobs/src/cobs.cpp
  ${ESPP_COMPONENTS}/cobs/src/cobs_stream.cpp
  ${ESPP_COMPONENTS}/color/src/color.cpp
  ${ESPP_COMPONENTS}/event_manager/src/event_manager.cpp
  ${ESPP_COMPONENTS}/logger/src/logger.cpp
  ${ESPP_COMPONENTS}/file_system/src/file_system.cpp
  ${ESPP_COMPONENTS}/filters/src/lowpass_filter.cpp
  ${ESPP_COMPONENTS}/filters/src/simple_lowpass_filter.cpp
  ${ESPP_COMPONENTS}/joystick/src/joystick.cpp
  ${ESPP_COMPONENTS}/rtps/src/rtps_participant.cpp
  ${ESPP_COMPONENTS}/rtps/src/communication/EsppTransport.cpp
  ${ESPP_COMPONENTS}/rtps/src/discovery/ParticipantProxyData.cpp
  ${ESPP_COMPONENTS}/rtps/src/discovery/SEDPAgent.cpp
  ${ESPP_COMPONENTS}/rtps/src/discovery/SPDPAgent.cpp
  ${ESPP_COMPONENTS}/rtps/src/discovery/TopicData.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/Domain.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/Participant.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/Reader.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/StatefulReader.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/StatefulWriter.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/StatelessReader.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/StatelessWriter.cpp
  ${ESPP_COMPONENTS}/rtps/src/entities/Writer.cpp
  ${ESPP_COMPONENTS}/rtps/src/messages/MessageReceiver.cpp
  ${ESPP_COMPONENTS}/rtps/src/messages/MessageTypes.cpp
  ${ESPP_COMPONENTS}/rtps/src/utils/Diagnostics.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtcp_packet.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtp_packet.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_client.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_server.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_session.cpp
  ${ESPP_COMPONENTS}/rtsp/src/generic_depacketizer.cpp
  ${ESPP_COMPONENTS}/rtsp/src/generic_packetizer.cpp
  ${ESPP_COMPONENTS}/rtsp/src/h264_packetizer.cpp
  ${ESPP_COMPONENTS}/rtsp/src/h264_depacketizer.cpp
  ${ESPP_COMPONENTS}/rtsp/src/mjpeg_depacketizer.cpp
  ${ESPP_COMPONENTS}/rtsp/src/mjpeg_packetizer.cpp
  ${ESPP_COMPONENTS}/task/src/task.cpp
  ${ESPP_COMPONENTS}/thread_pool/src/thread_pool.cpp
  ${ESPP_COMPONENTS}/timer/src/timer.cpp
  ${ESPP_COMPONENTS}/trajectory_planner/src/trajectory_planner.cpp
  ${ESPP_COMPONENTS}/socket/src/socket.cpp
  ${ESPP_COMPONENTS}/socket/src/tcp_socket.cpp
  ${ESPP_COMPONENTS}/socket/src/udp_socket.cpp
  ${ESPP_COMPONENTS}/socket/src/socket_reactor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/espp.cpp
)

set(ESPP_INCLUDE_DIRS
  ${ESPP_INCLUDES}
  ${ESPP_EXTERNAL_INCLUDES}
  ${ESPP_EXTERNAL_INCLUDES_SEPARATE}
)

# if we're on windows, we need to add wcswidth.c to the sources
if(MSVC)
  list(APPEND ESPP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/wcswidth.c)
endif()

# On Windows link against ws2_32 (sockets), winmm (timeBeginPeriod, used by the
# TimerResolution helper in espp.hpp), and iphlpapi (GetAdaptersAddresses, used
# by RtpsParticipant interface auto-detection). Centralizing these here keeps
# linkage consistent across the static library, tests, and the _espp module, and
# works on all Windows toolchains (MSVC, MinGW, clang) rather than relying on
# MSVC-only #pragma comment(lib, ...).
if(WIN32)
  set(ESPP_EXTERNAL_LIBS ws2_32 winmm iphlpapi)
else()
  set(ESPP_EXTERNAL_LIBS pthread)
endif()

set(ESPP_PYTHON_BINDINGS_DIR ${CMAKE_CURRENT_LIST_DIR}/python_bindings)

set(ESPP_PYTHON_SOURCES
  ${ESPP_PYTHON_BINDINGS_DIR}/module.cpp
  ${ESPP_PYTHON_BINDINGS_DIR}/pybind_espp.cpp
  ${ESPP_PYTHON_BINDINGS_DIR}/odrive_native_bindings.cpp
  ${ESPP_PYTHON_BINDINGS_DIR}/rtps_bindings.cpp
  ${ESPP_PYTHON_BINDINGS_DIR}/socket_reactor_bindings.cpp
  ${ESPP_SOURCES}
)

# make an espp_install_cmake_package command that gives the C++ library target a
# proper install + export so a separate project can `find_package(espp)` and link
# `espp::espp`. Installs into GNUInstallDirs under CMAKE_INSTALL_PREFIX:
#   <prefix>/lib/libespp_pc.a
#   <prefix>/include/...                (all component + vendored headers, flat)
#   <prefix>/lib/cmake/espp/esppTargets.cmake, esppConfig.cmake, ...Version.cmake
#
# All third-party deps (reflect-cpp, magic_enum, tabulate, fmt, alpaca, cli,
# csv2, hid-rp, cdr) are VENDORED: their headers are installed under
# <prefix>/include and their objects are compiled into libespp_pc.a, so a
# consumer needs no extra find_package for them. The only non-bundled PUBLIC
# dependency is the system threads library (handled via find_dependency(Threads)
# in esppConfig.cmake.in; on Windows the ws2_32/winmm/iphlpapi system libs
# resolve automatically).
function(espp_install_cmake_package TARGET_NAME)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS ${TARGET_NAME}
          EXPORT esppTargets
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
          INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # Install the public headers flat into <prefix>/include so every quote-include
  # espp uses resolves against a single -I<prefix>/include. All three lists are
  # already on the build include path, and their headers are referenced flat
  # relative to those roots, so installing each dir's CONTENTS (the trailing
  # slash) yields the exact layout consumers compile against:
  #  - ESPP_INCLUDES / ESPP_EXTERNAL_INCLUDES: ".../include" dirs -> "logger.hpp".
  #  - ESPP_EXTERNAL_INCLUDES_SEPARATE: dirs NOT named "include" but still on the
  #    -I path, referenced flat -> "magic_enum.hpp", "hid/...", "sized_unsigned.hpp".
  # This intentionally differs from the old lib/pc helper (which preserved the
  # SEPARATE dir names under include/): that only worked because the pc build also
  # had the source dirs on its -I path; a pure find_package consumer needs flat.
  # Verified by building a find_package consumer that includes both a regular and
  # a SEPARATE header.
  foreach(_inc IN LISTS ESPP_INCLUDES ESPP_EXTERNAL_INCLUDES ESPP_EXTERNAL_INCLUDES_SEPARATE)
    install(DIRECTORY ${_inc}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  endforeach()

  install(EXPORT esppTargets
          NAMESPACE espp::
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/espp
          FILE esppTargets.cmake)

  configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/esppConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/esppConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/espp)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/esppConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/esppConfig.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/esppConfigVersion.cmake
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/espp)
endfunction()

# make an espp_add_python_module command that defines the `_espp` pybind11
# extension module target (the native part of the `espp` python package)
function(espp_add_python_module)
  pybind11_add_module(_espp ${ESPP_PYTHON_SOURCES})
  target_compile_features(_espp PRIVATE cxx_std_23)
  # disable certain compiler warnings for this module, but only if we're not on
  # Windows
  if(NOT MSVC)
    target_compile_options(_espp PRIVATE -Wno-braced-scalar-init -Wno-unused-variable -Wno-unused-parameter)
  endif()
  target_link_libraries(_espp PRIVATE ${ESPP_EXTERNAL_LIBS})
  # embed the package version (set by scikit-build-core when building wheels);
  # prefer the full PEP 440 version (includes .devN+local parts) over the
  # CMake-compatible X.Y.Z truncation
  if(DEFINED SKBUILD_PROJECT_VERSION_FULL)
    target_compile_definitions(_espp PRIVATE VERSION_INFO=${SKBUILD_PROJECT_VERSION_FULL})
  elseif(DEFINED SKBUILD_PROJECT_VERSION)
    target_compile_definitions(_espp PRIVATE VERSION_INFO=${SKBUILD_PROJECT_VERSION})
  endif()
endfunction()

# make an espp_install_python_module command that installs the full `espp` python
# package (pure-python files + the compiled `_espp` extension) into the standard
# install prefix as <prefix>/espp, so CMAKE_INSTALL_PREFIX can be put on sys.path
# / PYTHONPATH to `import espp`. Relative DESTINATIONs are interpreted against
# CMAKE_INSTALL_PREFIX.
function(espp_install_python_module)
  espp_add_python_module()
  install(DIRECTORY ${ESPP_PYTHON_BINDINGS_DIR}/espp
    DESTINATION .
    PATTERN "__pycache__" EXCLUDE
    PATTERN ".mypy_cache" EXCLUDE)
  # Also stage espp_odrive (the pure-python ODrive client that lives in the
  # odrive_native component) next to the espp package, mirroring the wheel's
  # `wheel.packages` in pyproject.toml -- so PYTHONPATH=<prefix> gives both
  # `import espp` and `import espp_odrive` (and espp.odrive works).
  install(DIRECTORY ${ESPP_COMPONENTS}/odrive_native/python/espp_odrive
    DESTINATION .
    PATTERN "__pycache__" EXCLUDE)
  install(TARGETS _espp
    LIBRARY DESTINATION espp/
    RUNTIME DESTINATION espp/)
endfunction()
