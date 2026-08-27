idf_component_register(
  SRCS
    "src/rtps_participant.cpp"
    "src/communication/EsppTransport.cpp"
    "src/discovery/ParticipantProxyData.cpp"
    "src/discovery/SEDPAgent.cpp"
    "src/discovery/SPDPAgent.cpp"
    "src/discovery/TopicData.cpp"
    "src/entities/Domain.cpp"
    "src/entities/Participant.cpp"
    "src/entities/Reader.cpp"
    "src/entities/StatefulReader.cpp"
    "src/entities/StatefulWriter.cpp"
    "src/entities/StatelessReader.cpp"
    "src/entities/StatelessWriter.cpp"
    "src/entities/Writer.cpp"
    "src/messages/MessageReceiver.cpp"
    "src/messages/MessageTypes.cpp"
    "src/utils/Diagnostics.cpp"
  INCLUDE_DIRS
    "include"
  REQUIRES
    base_component cdr task thread_pool timer socket
)

# Select the RTPS static-limits profile from Kconfig (see Kconfig in this
# component). The "embedded" profile is the default and is byte-identical to the
# historical behavior: it defines no RTPS_CONFIG_HEADER, so config.hpp selects
# rtps/config_esp32.hpp via ESP_PLATFORM. The relaxed "host" / "host_large"
# profiles override the profile header. These are capacity-only caps and do not
# change any bytes on the wire.
if(CONFIG_RTPS_LIMITS_PROFILE_HOST)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC RTPS_CONFIG_HEADER="rtps/config_desktop.hpp")
elseif(CONFIG_RTPS_LIMITS_PROFILE_HOST_LARGE)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC RTPS_CONFIG_HEADER="rtps/config_host_large.hpp")
endif()

# Storage policy is orthogonal to the limits profile above: dynamic (heap,
# grow-on-full) storage is an explicit ESP opt-in (default static) so a relaxed
# limits profile never silently switches the MCU to heap-backed history. The
# limits headers no longer define RTPS_STORAGE_DYNAMIC themselves; it is set here
# on ESP and defaulted on in config.hpp for host/PC builds.
# Per-limit capacity overrides (Kconfig "Custom capacity overrides"): a
# nonzero value overrides that single cap of the selected profile via its
# RTPS_CFG_* macro. PUBLIC so application translation units see the same
# values as the engine (the pools are sized in this component's sources).
set(RTPS_LIMIT_KNOBS
    NUM_STATELESS_WRITERS
    NUM_STATELESS_READERS
    NUM_STATEFUL_WRITERS
    NUM_STATEFUL_READERS
    MAX_NUM_PARTICIPANTS
    NUM_WRITERS_PER_PARTICIPANT
    NUM_READERS_PER_PARTICIPANT
    NUM_WRITER_PROXIES_PER_READER
    NUM_READER_PROXIES_PER_WRITER
    MAX_NUM_UNMATCHED_REMOTE_WRITERS
    MAX_NUM_UNMATCHED_REMOTE_READERS
    MAX_NUM_READER_CALLBACKS
    HISTORY_SIZE_STATELESS
    HISTORY_SIZE_STATEFUL
    MAX_TYPENAME_LENGTH
    MAX_TOPICNAME_LENGTH
)
foreach(knob ${RTPS_LIMIT_KNOBS})
  if(DEFINED CONFIG_RTPS_LIMIT_${knob} AND NOT "${CONFIG_RTPS_LIMIT_${knob}}" STREQUAL "" AND NOT "${CONFIG_RTPS_LIMIT_${knob}}" STREQUAL "0")
    # Builtin-aware minima (0 stays the "use profile default" sentinel): the
    # discovery endpoints draw from these pools, so a nonzero value below the
    # reservation would crash startup (the second SEDP allocation returns null)
    # or silently omit the builtins (per-participant caps < 3).
    if((knob STREQUAL "NUM_STATEFUL_WRITERS" OR knob STREQUAL "NUM_STATEFUL_READERS")
       AND CONFIG_RTPS_LIMIT_${knob} LESS 2)
      message(FATAL_ERROR
        "CONFIG_RTPS_LIMIT_${knob}=${CONFIG_RTPS_LIMIT_${knob}} is below the minimum of 2 "
        "(2 slots are reserved for the builtin SEDP endpoints)")
    endif()
    if((knob STREQUAL "NUM_WRITERS_PER_PARTICIPANT" OR knob STREQUAL "NUM_READERS_PER_PARTICIPANT")
       AND CONFIG_RTPS_LIMIT_${knob} LESS 3)
      message(FATAL_ERROR
        "CONFIG_RTPS_LIMIT_${knob}=${CONFIG_RTPS_LIMIT_${knob}} is below the minimum of 3 "
        "(the 3 builtin discovery endpoints count against the per-participant cap)")
    endif()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "RTPS_CFG_${knob}=${CONFIG_RTPS_LIMIT_${knob}}")
    set(RTPS_EFFECTIVE_${knob} ${CONFIG_RTPS_LIMIT_${knob}})
  endif()
endforeach()

# Cross-limit capacity check (mirrors lib/espp.cmake): every participant
# consumes 1 stateless writer/reader + 2 stateful writers/readers from the
# GLOBAL pools for its builtin discovery endpoints. Effective value = the
# Kconfig override if nonzero, else the selected profile's default.
if(CONFIG_RTPS_LIMITS_PROFILE_HOST)
  set(_rtps_defaults 8 16 16 32 32)
elseif(CONFIG_RTPS_LIMITS_PROFILE_HOST_LARGE)
  set(_rtps_defaults 32 64 64 128 128)
else() # embedded (default)
  set(_rtps_defaults 1 5 5 5 5)
endif()
list(GET _rtps_defaults 0 _rtps_d_participants)
list(GET _rtps_defaults 1 _rtps_d_stateless_w)
list(GET _rtps_defaults 2 _rtps_d_stateless_r)
list(GET _rtps_defaults 3 _rtps_d_stateful_w)
list(GET _rtps_defaults 4 _rtps_d_stateful_r)
foreach(pair
    "MAX_NUM_PARTICIPANTS;_rtps_d_participants"
    "NUM_STATELESS_WRITERS;_rtps_d_stateless_w"
    "NUM_STATELESS_READERS;_rtps_d_stateless_r"
    "NUM_STATEFUL_WRITERS;_rtps_d_stateful_w"
    "NUM_STATEFUL_READERS;_rtps_d_stateful_r")
  list(GET pair 0 _rtps_k)
  list(GET pair 1 _rtps_dvar)
  if(NOT DEFINED RTPS_EFFECTIVE_${_rtps_k})
    set(RTPS_EFFECTIVE_${_rtps_k} ${${_rtps_dvar}})
  endif()
endforeach()
math(EXPR _rtps_need_stateful "2 * ${RTPS_EFFECTIVE_MAX_NUM_PARTICIPANTS}")
if(RTPS_EFFECTIVE_NUM_STATELESS_WRITERS LESS RTPS_EFFECTIVE_MAX_NUM_PARTICIPANTS
   OR RTPS_EFFECTIVE_NUM_STATELESS_READERS LESS RTPS_EFFECTIVE_MAX_NUM_PARTICIPANTS
   OR RTPS_EFFECTIVE_NUM_STATEFUL_WRITERS LESS _rtps_need_stateful
   OR RTPS_EFFECTIVE_NUM_STATEFUL_READERS LESS _rtps_need_stateful)
  message(FATAL_ERROR
    "RTPS limits cannot host the participant budget: MAX_NUM_PARTICIPANTS="
    "${RTPS_EFFECTIVE_MAX_NUM_PARTICIPANTS} needs >= ${RTPS_EFFECTIVE_MAX_NUM_PARTICIPANTS} "
    "stateless writers/readers (have ${RTPS_EFFECTIVE_NUM_STATELESS_WRITERS}/"
    "${RTPS_EFFECTIVE_NUM_STATELESS_READERS}) and >= ${_rtps_need_stateful} stateful "
    "writers/readers (have ${RTPS_EFFECTIVE_NUM_STATEFUL_WRITERS}/"
    "${RTPS_EFFECTIVE_NUM_STATEFUL_READERS}) for the builtin discovery endpoints "
    "(1 stateless W/R + 2 stateful W/R per participant). Raise the pool overrides "
    "(menuconfig 'Custom capacity overrides') or lower MAX_NUM_PARTICIPANTS.")
endif()

if(CONFIG_RTPS_STORAGE_DYNAMIC)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC RTPS_STORAGE_DYNAMIC)
endif()

# Best-effort DATA_FRAG fragmentation is opt-in on ESP targets (Kconfig, default
# off) so the MCU pays nothing for it by default. When enabled, define
# RTPS_ENABLE_FRAGMENTATION (compiles the fragment send/reassembly paths) and the
# reassembly cap RTPS_MAX_SAMPLE_SIZE (256 KB on the embedded profile). The
# facade's max payload size rises to this when fragmentation is enabled.
if(CONFIG_RTPS_ENABLE_FRAGMENTATION)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC
    RTPS_ENABLE_FRAGMENTATION RTPS_MAX_SAMPLE_SIZE=262144)
endif()

# The RPC layer (services + actions) is compiled in by default; the facade
# header defines RTPS_WITH_RPC unless RTPS_NO_RPC is set. When the Kconfig option
# is turned OFF, define RTPS_NO_RPC so the whole services/actions surface (and its
# std::thread/std::future use) is excluded, saving flash. ESP-only (this file is
# the ESP-IDF component build); host builds always keep RPC on.
if(NOT CONFIG_RTPS_ENABLE_RPC)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC RTPS_NO_RPC)
endif()

