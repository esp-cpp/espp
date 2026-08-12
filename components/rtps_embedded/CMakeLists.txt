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
    "src/entities/StatelessReader.cpp"
    "src/entities/Writer.cpp"
    "src/messages/MessageReceiver.cpp"
    "src/messages/MessageTypes.cpp"
    "src/utils/Diagnostics.cpp"
  INCLUDE_DIRS
    "include"
  REQUIRES
    base_component cdr task thread_pool socket
)

