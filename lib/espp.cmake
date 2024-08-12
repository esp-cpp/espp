set(ESPP_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../components")
set(ESPP_EXTERNAL "${CMAKE_CURRENT_LIST_DIR}/../external")

set(ESPP_EXTERNAL_INCLUDES
  ${ESPP_EXTERNAL}/alpaca/include
  ${ESPP_EXTERNAL}/cli/include
  ${ESPP_EXTERNAL}/csv2/include
  ${ESPP_EXTERNAL}/hid-rp/hid-rp/
  ${ESPP_EXTERNAL}/fmt/include
  ${ESPP_EXTERNAL}/magic_enum/include/magic_enum/
  ${ESPP_EXTERNAL}/tabulate/include
)

set(ESPP_INCLUDES
  ${ESPP_EXTERNAL_INCLUDES}
  ${ESPP_COMPONENTS}/base_component/include
  ${ESPP_COMPONENTS}/base_peripheral/include
  ${ESPP_COMPONENTS}/color/include
  ${ESPP_COMPONENTS}/csv/include
  ${ESPP_COMPONENTS}/event_manager/include
  ${ESPP_COMPONENTS}/file_system/include
  ${ESPP_COMPONENTS}/ftp/include
  ${ESPP_COMPONENTS}/format/include
  ${ESPP_COMPONENTS}/hid-rp/include
  ${ESPP_COMPONENTS}/joystick/include
  ${ESPP_COMPONENTS}/logger/include
  ${ESPP_COMPONENTS}/math/include
  ${ESPP_COMPONENTS}/rtsp/include
  ${ESPP_COMPONENTS}/serialization/include
  ${ESPP_COMPONENTS}/tabulate/include
  ${ESPP_COMPONENTS}/task/include
  ${ESPP_COMPONENTS}/timer/include
  ${ESPP_COMPONENTS}/socket/include
  ${ESPP_COMPONENTS}/state_machine/include
  ${CMAKE_CURRENT_LIST_DIR}/include
)

set(ESPP_SOURCES
  ${ESPP_COMPONENTS}/color/src/color.cpp
  ${ESPP_COMPONENTS}/event_manager/src/event_manager.cpp
  ${ESPP_COMPONENTS}/logger/src/logger.cpp
  ${ESPP_COMPONENTS}/file_system/src/file_system.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtcp_packet.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtp_packet.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_client.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_server.cpp
  ${ESPP_COMPONENTS}/rtsp/src/rtsp_session.cpp
  ${ESPP_COMPONENTS}/task/src/task.cpp
  ${ESPP_COMPONENTS}/timer/src/timer.cpp
  ${ESPP_COMPONENTS}/socket/src/socket.cpp
  ${ESPP_COMPONENTS}/socket/src/tcp_socket.cpp
  ${ESPP_COMPONENTS}/socket/src/udp_socket.cpp
  ${CMAKE_CURRENT_LIST_DIR}/espp.cpp
)

# if we're on windows, we need to add wcswidth.c to the sources
if(MSVC)
  list(APPEND ESPP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/wcswidth.c)
endif()

# if we're on Windows, we need to link against ws2_32
if(WIN32)
  set(ESPP_EXTERNAL_LIBS ws2_32)
else()
  set(ESPP_EXTERNAL_LIBS pthread)
endif()

set(ESPP_PYTHON_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/python_bindings/module.cpp
  ${CMAKE_CURRENT_LIST_DIR}/python_bindings/pybind_espp.cpp
  ${ESPP_SOURCES}
)
