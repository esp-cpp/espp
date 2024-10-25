set(ESPP_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../components")
set(ESPP_EXTERNAL "${CMAKE_CURRENT_LIST_DIR}/../external")

set(ESPP_EXTERNAL_INCLUDES
  ${ESPP_EXTERNAL}/alpaca/include
  ${ESPP_EXTERNAL}/cli/include
  ${ESPP_EXTERNAL}/csv2/include
  ${ESPP_EXTERNAL}/fmt/include
  ${ESPP_EXTERNAL}/tabulate/include
)

# NOTE: these are separate because they do not follow the standard format of
# having their include files be in the "include" directory, so when we install
# them we need to handle them separately
set(ESPP_EXTERNAL_INCLUDES_SEPARATE
  ${ESPP_EXTERNAL}/hid-rp/hid-rp/
  ${ESPP_EXTERNAL}/magic_enum/include/magic_enum/
)

set(ESPP_INCLUDES
  ${ESPP_COMPONENTS}/base_component/include
  ${ESPP_COMPONENTS}/base_peripheral/include
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
  ${ESPP_COMPONENTS}/filters/src/lowpass_filter.cpp
  ${ESPP_COMPONENTS}/filters/src/simple_lowpass_filter.cpp
  ${ESPP_COMPONENTS}/joystick/src/joystick.cpp
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

set(ESPP_INCLUDE_DIRS
  ${ESPP_INCLUDES}
  ${ESPP_EXTERNAL_INCLUDES}
  ${ESPP_EXTERNAL_INCLUDES_SEPARATE}
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

# make an espp_install_includes command that can be used by other scripts, where
# they just need to specify the folder they want to install into
function(espp_install_includes FOLDER)
  install(DIRECTORY ${ESPP_INCLUDES} DESTINATION ${FOLDER}/)
  install(DIRECTORY ${ESPP_EXTERNAL_INCLUDES} DESTINATION ${FOLDER}/)
  install(DIRECTORY ${ESPP_EXTERNAL_INCLUDES_SEPARATE} DESTINATION ${FOLDER}/include/)
endfunction()

# make an espp_install_python_module command that can be used by other scripts, where
# they just need to specify the folder they want to install into
function(espp_install_python_module FOLDER)
  pybind11_add_module(espp ${ESPP_PYTHON_SOURCES})
  target_compile_features(espp PRIVATE cxx_std_20)
  target_link_libraries(espp PRIVATE ${ESPP_EXTERNAL_LIBS})
  install(TARGETS espp
    LIBRARY DESTINATION ${FOLDER}/)
endfunction()
