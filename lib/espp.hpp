#pragma once

#include <cli/cli.h>
#include <cli/clifilesession.h>

#include "base_component.hpp"
#include "bezier.hpp"
#include "color.hpp"
#include "csv.hpp"
#include "event_manager.hpp"
#include "fast_math.hpp"
#include "file_system.hpp"
#include "ftp_client_session.hpp"
#include "ftp_server.hpp"
#include "gaussian.hpp"
// TODO: these are not working
// #include "hid-rp.hpp"
// #include "hid-rp-gamepad.hpp"
#include "logger.hpp"
#include "range_mapper.hpp"
#include "rtsp_client.hpp"
#include "rtsp_server.hpp"
#include "serialization.hpp"
#include "tabulate.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "timer.hpp"
#include "udp_socket.hpp"
#include "vector2d.hpp"

// state machine includes
#include "deep_history_state.hpp"
#include "magic_enum.hpp"
#include "shallow_history_state.hpp"
#include "state_base.hpp"

#include <tabulate/markdown_exporter.hpp>