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
    base_component cdr task thread_pool socket
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

