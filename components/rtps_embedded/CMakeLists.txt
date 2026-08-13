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

