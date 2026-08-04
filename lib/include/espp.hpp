#pragma once

#include "socket_msvc.hpp"

#ifdef _MSC_VER
extern "C" {
#include <windows.h>
// NOTE: needed for tabulate
#include "wcswidth.h"
}
#endif

#include <cli/cli.h>
#include <cli/clifilesession.h>

#include "base_component.hpp"
#include "bezier.hpp"
#include "butterworth_filter.hpp"
#include "cdr.hpp"
#include "cobs.hpp"
#include "cobs_stream.hpp"
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
#include "generic_depacketizer.hpp"
#include "generic_packetizer.hpp"
#include "h264_depacketizer.hpp"
#include "h264_packetizer.hpp"
#include "joystick.hpp"
#include "logger.hpp"
#include "lowpass_filter.hpp"
#include "mjpeg_depacketizer.hpp"
#include "mjpeg_packetizer.hpp"
#include "ndef.hpp"
#include "pid.hpp"
#include "range_mapper.hpp"
#include "rtp_depacketizer.hpp"
#include "rtp_packetizer.hpp"
#include "rtp_types.hpp"
#include "rtps.hpp"
#include "rtsp_client.hpp"
#include "rtsp_server.hpp"
#include "serialization.hpp"
#include "simple_lowpass_filter.hpp"
#include "tabulate.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "thread_pool.hpp"
#include "timer.hpp"
#include "udp_socket.hpp"
#include "vector2d.hpp"

// state machine includes
#include "deep_history_state.hpp"
#include "magic_enum.hpp"
#include "shallow_history_state.hpp"
#include "state_base.hpp"

#include <tabulate/markdown_exporter.hpp>

#ifdef _MSC_VER

#include <mmsystem.h>
#include <windows.h>

// we want to ensure that the timer resolution is set to 1ms, otherwise the
// timer will not be accurate. To do this we need to call timeBeginPeriod(1) at
// the start of the program and timeEndPeriod(1) at the end of the program.
class TimerResolution {
  espp::Logger logger{{.tag = "TimerResolution", .level = espp::Logger::Verbosity::INFO}};

public:
  TimerResolution() {
    logger.info("Setting timeBeginPeriod(1)");
    if (timeBeginPeriod(1) == TIMERR_NOERROR) {
      logger.info("Success");
    } else {
      logger.error("failed to set timeBeginPeriod(1)");
    }
  }
  ~TimerResolution() {
    logger.info("Setting timeEndPeriod(1)");
    timeEndPeriod(1);
  }
};

// we create a global instance of the TimerResolution class to ensure that the
// timer resolution is set to 1ms for the duration of the program.
extern TimerResolution timer_resolution;

#endif
