#pragma once

#include "socket_win32.hpp"

#ifdef _MSC_VER
// windows.h is a C++ header and must not be wrapped in extern "C"; only the C
// header (wcswidth) needs it.
#include <windows.h>
extern "C" {
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
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "rtsp_client.hpp"
#include "rtsp_server.hpp"
#include "serialization.hpp"
#include "simple_lowpass_filter.hpp"
#include "tabulate.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "thread_pool.hpp"
#include "timer.hpp"
#include "trajectory_planner.hpp"
#include "udp_socket.hpp"
// NOTE: socket_reactor.hpp must come after tcp_socket/udp_socket/thread_pool/task,
// which it depends on.
#include "socket_reactor.hpp"
#include "vector2d.hpp"

// state machine includes
#include "deep_history_state.hpp"
#include "magic_enum.hpp"
#include "shallow_history_state.hpp"
#include "state_base.hpp"

#include <tabulate/markdown_exporter.hpp>

// The timer-resolution helper uses the Windows multimedia timer API
// (timeBeginPeriod), which is available on all Windows toolchains (MSVC, MinGW,
// clang), so guard on _WIN32 rather than _MSC_VER. winmm is linked via CMake
// (see lib/espp.cmake / pc/CMakeLists.txt).
#ifdef _WIN32

#include <mmsystem.h>
#include <windows.h>

// we want to ensure that the timer resolution is set to 1ms, otherwise the
// timer will not be accurate. To do this we need to call timeBeginPeriod(1) at
// the start of the program and timeEndPeriod(1) at the end of the program.
class TimerResolution {
public:
  TimerResolution() { timeBeginPeriod(1); }
  ~TimerResolution() { timeEndPeriod(1); }
};

// we create a global instance of the TimerResolution class to ensure that the
// timer resolution is set to 1ms for the duration of the program.
extern TimerResolution timer_resolution;

#endif
