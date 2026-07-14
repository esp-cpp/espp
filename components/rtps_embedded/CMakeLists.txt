idf_component_register(
  SRCS
    "src/ThreadPool.cpp"
    "src/communication/EsppTransport.cpp"
    "src/discovery/ParticipantProxyData.cpp"
    "src/discovery/SEDPAgent.cpp"
    "src/discovery/SPDPAgent.cpp"
    "src/discovery/TopicData.cpp"
    "src/entities/Domain.cpp"
    "src/entities/Participant.cpp"
    "src/entities/Reader.cpp"
    "src/entities/StatelessReader.cpp"
    "src/entities/Writer.cpp"
    "src/messages/MessageReceiver.cpp"
    "src/messages/MessageTypes.cpp"
    "src/utils/Diagnostics.cpp"
    "thirdparty/Micro-CDR/src/c/common.c"
    "thirdparty/Micro-CDR/src/c/types/array.c"
    "thirdparty/Micro-CDR/src/c/types/basic.c"
    "thirdparty/Micro-CDR/src/c/types/sequence.c"
    "thirdparty/Micro-CDR/src/c/types/string.c"
  INCLUDE_DIRS
    "include"
    "thirdparty/Micro-CDR/include"
  REQUIRES
    base_component cdr task thread_pool socket
)

# target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
# target_compile_definitions(${COMPONENT_LIB} PUBLIC RTPS_CONFIG_HEADER=\"rtps/config_esp32.h\")
