set(ESPP_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../components")

# ---------------------------------------------------------------------------
# RTPS static-limits profile selection.
#
# The rtps_embedded engine keeps a fully-static (deterministic) allocation model
# whose compile-time capacity caps are chosen by a profile header (see
# components/rtps_embedded/include/rtps/config.hpp). Profiles:
#   embedded   - tight MCU caps (rtps/config_esp32.hpp)
#   host       - relaxed static caps, DEFAULT for non-ESP builds
#                (rtps/config_desktop.hpp)
#   host_large - generous static caps for large DDS graphs
#                (rtps/config_host_large.hpp)
#
# Select with -DRTPS_LIMITS_PROFILE=embedded|host|host_large. Default is "host"
# (this file is only used for non-ESP / host builds; ESP/IDF builds pick the
# profile via Kconfig in components/rtps_embedded/Kconfig).
# ---------------------------------------------------------------------------
if(NOT DEFINED RTPS_LIMITS_PROFILE)
  set(RTPS_LIMITS_PROFILE "host")
endif()
set(RTPS_LIMITS_PROFILE "${RTPS_LIMITS_PROFILE}" CACHE STRING
    "RTPS static limits profile: embedded|host|host_large")
set_property(CACHE RTPS_LIMITS_PROFILE PROPERTY STRINGS embedded host host_large)

if(RTPS_LIMITS_PROFILE STREQUAL "embedded")
  add_compile_definitions(RTPS_CONFIG_HEADER="rtps/config_esp32.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 262144)   # 256 KB (matches config_esp32.hpp)
elseif(RTPS_LIMITS_PROFILE STREQUAL "host")
  add_compile_definitions(RTPS_CONFIG_HEADER="rtps/config_desktop.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 8388608)  # 8 MB (matches config_desktop.hpp)
elseif(RTPS_LIMITS_PROFILE STREQUAL "host_large")
  add_compile_definitions(RTPS_CONFIG_HEADER="rtps/config_host_large.hpp")
  set(RTPS_MAX_SAMPLE_SIZE 8388608)  # 8 MB (matches config_host_large.hpp)
else()
  message(FATAL_ERROR
    "Invalid RTPS_LIMITS_PROFILE '${RTPS_LIMITS_PROFILE}' "
    "(expected: embedded | host | host_large)")
endif()
message(STATUS "RTPS limits profile: ${RTPS_LIMITS_PROFILE}")

# ---------------------------------------------------------------------------
# RTPS best-effort DATA_FRAG fragmentation (Slice C).
#
# On host builds fragmentation is ALWAYS enabled: samples larger than a single
# DATA submessage are split into DATA_FRAG submessages and reassembled by the
# peer (interoperates with FastDDS / ROS 2). RTPS_MAX_SAMPLE_SIZE is the
# large-sample reassembly cap and is kept in sync with the profile's
# Config::MAX_SAMPLE_SIZE. On ESP32/IDF fragmentation is opt-in via Kconfig (see
# components/rtps_embedded/Kconfig / CMakeLists.txt), default off, so the MCU
# pays nothing for it by default.
# ---------------------------------------------------------------------------
add_compile_definitions(RTPS_ENABLE_FRAGMENTATION RTPS_MAX_SAMPLE_SIZE=${RTPS_MAX_SAMPLE_SIZE})
message(STATUS "RTPS fragmentation: ON (max sample size ${RTPS_MAX_SAMPLE_SIZE} bytes)")

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
  ${ESPP_COMPONENTS}/pid/include
  ${ESPP_COMPONENTS}/rtps_embedded/include
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
  ${ESPP_COMPONENTS}/rtps_embedded/src/rtps_participant.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/communication/EsppTransport.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/discovery/ParticipantProxyData.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/discovery/SEDPAgent.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/discovery/SPDPAgent.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/discovery/TopicData.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/Domain.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/Participant.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/Reader.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/StatefulReader.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/StatefulWriter.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/StatelessReader.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/StatelessWriter.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/entities/Writer.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/messages/MessageReceiver.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/messages/MessageTypes.cpp
  ${ESPP_COMPONENTS}/rtps_embedded/src/utils/Diagnostics.cpp
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
  ${ESPP_PYTHON_BINDINGS_DIR}/rtps_bindings.cpp
  ${ESPP_PYTHON_BINDINGS_DIR}/socket_reactor_bindings.cpp
  ${ESPP_SOURCES}
)

# make an espp_install_includes command that can be used by other scripts, where
# they just need to specify the folder they want to install into
function(espp_install_includes FOLDER)
  install(DIRECTORY ${ESPP_INCLUDES} DESTINATION ${FOLDER}/)
  install(DIRECTORY ${ESPP_EXTERNAL_INCLUDES} DESTINATION ${FOLDER}/)
  install(DIRECTORY ${ESPP_EXTERNAL_INCLUDES_SEPARATE} DESTINATION ${FOLDER}/include/)
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

# make an espp_install_python_module command that can be used by other scripts,
# where they just need to specify the folder they want to install into. This
# installs the full `espp` python package (pure-python files + the compiled
# `_espp` extension) into FOLDER/espp so FOLDER can be put on sys.path.
function(espp_install_python_module FOLDER)
  espp_add_python_module()
  install(DIRECTORY ${ESPP_PYTHON_BINDINGS_DIR}/espp
    DESTINATION ${FOLDER}/
    PATTERN "__pycache__" EXCLUDE
    PATTERN ".mypy_cache" EXCLUDE)
  install(TARGETS _espp
    LIBRARY DESTINATION ${FOLDER}/espp/
    RUNTIME DESTINATION ${FOLDER}/espp/)
endfunction()
