// ODrive legacy native (Fibre) interop DEVICE shim.
//
// Emulates an ODrive over a serial link so a REAL fibre client (the pure-python
// legacy fibre from odriverobotics/ODrive @ fw-v0.5.1) can connect, enumerate the
// endpoint tree, and read/write endpoints -- the true real-tool interop gate,
// mirroring how components/rtps is gated against FastDDS / ROS 2.
//
// It uses ONLY the host-buildable detail/ headers (OdriveNativeCore for the packet
// codec + JSON descriptor, StreamDeframer/stream_frame for the UART framing) so it
// builds with a plain `c++ -std=c++20` -- no BaseComponent, no espp lib link.
//
// Transport: opens a pseudo-terminal (posix_openpt/grantpt/unlockpt/ptsname) and
// prints the slave path, OR uses a serial device path given as argv[1]. The read
// loop is: raw stream bytes -> StreamDeframer -> OdriveNativeCore::process_bytes
// -> stream_frame -> write back. Endpoint-0 (JSON) responses are truncated so the
// framed packet stays <= 127 bytes (the stream cap); the client's chunked read
// loop advances by the bytes it actually receives, so truncation is safe.
//
// Build:  c++ -std=c++20 -I../include odrive_native_interop_device.cpp -o device
// Run:    ./device            # opens a PTY, prints the slave path
//         ./device /dev/ttyS0 # uses an existing serial device

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <span>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "detail/odrive_native_core.hpp"
#include "detail/odrive_native_stream.hpp"

using espp::detail::kStreamMaxPacket;
using espp::detail::OdriveNativeCore;
using espp::detail::stream_frame;
using espp::detail::StreamDeframer;

namespace {

// Put a tty/pty into raw mode so the fibre binary stream passes through untouched
// (no CR/LF translation, no XON/XOFF flow control eating 0x11/0x13, no signal
// chars). Applied to either PTY end configures the shared line discipline.
void make_raw(int fd) {
  struct termios t;
  if (tcgetattr(fd, &t) != 0)
    return;
  cfmakeraw(&t);
  t.c_cc[VMIN] = 1;  // block for at least 1 byte
  t.c_cc[VTIME] = 0; // no inter-byte timer
  tcsetattr(fd, TCSANOW, &t);
}

} // namespace

int main(int argc, char **argv) {
  // ---- Build a small demo ODrive-like endpoint tree ----------------------
  OdriveNativeCore core;

  float vbus = 24.37f;
  uint32_t axis0_error = 0;
  float input_pos = 0.0f;
  float vel_limit = 20.0f;
  uint64_t serial_number = 0x00A1B2C3D4E5ULL;

  core.register_float_property("vbus_voltage", [&] { return vbus; });
  core.register_uint32_property("axis0.error", [&] { return axis0_error; });
  core.register_float_property(
      "axis0.controller.input_pos", [&] { return input_pos; },
      [&](float v, std::error_code &ec) {
        input_pos = v;
        ec.clear();
        return true;
      });
  core.register_float_property(
      "axis0.controller.config.vel_limit", [&] { return vel_limit; },
      [&](float v, std::error_code &ec) {
        vel_limit = v;
        ec.clear();
        return true;
      });
  core.register_uint64_property("serial_number", [&] { return serial_number; });
  core.finalize();

  std::fprintf(stderr, "[device] JSON descriptor (%zu bytes, crc=0x%04x): %s\n", core.json().size(),
               core.json_crc(), core.json().c_str());

  // ---- Open the transport (PTY or a given serial path) -------------------
  int fd = -1;
  if (argc > 1) {
    fd = ::open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) {
      std::fprintf(stderr, "[device] failed to open %s: %s\n", argv[1], std::strerror(errno));
      return 1;
    }
    make_raw(fd);
    std::fprintf(stderr, "[device] using serial device %s\n", argv[1]);
  } else {
    fd = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (fd < 0 || ::grantpt(fd) != 0 || ::unlockpt(fd) != 0) {
      std::fprintf(stderr, "[device] failed to open PTY master: %s\n", std::strerror(errno));
      return 1;
    }
    make_raw(fd);
    const char *slave = ::ptsname(fd);
    if (!slave) {
      std::fprintf(stderr, "[device] ptsname failed: %s\n", std::strerror(errno));
      return 1;
    }
    // The runner parses this exact line to learn the port to hand the client.
    std::printf("PTY_SLAVE %s\n", slave);
    std::fflush(stdout);
    std::fprintf(stderr, "[device] PTY slave = %s\n", slave);
  }

  // ---- Serve: stream bytes -> deframe -> process -> reframe -> write ------
  StreamDeframer deframer;
  uint8_t rx[512];
  // Self-terminate after a stretch of inactivity so a crashed client never
  // leaves the shim running forever; the runner also kills it explicitly.
  const int kIdleTimeoutSec = 30;
  time_t last_activity = ::time(nullptr);

  for (;;) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    struct timeval tv {
      1, 0
    };
    int sel = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    if (sel == 0) {
      if (::time(nullptr) - last_activity > kIdleTimeoutSec) {
        std::fprintf(stderr, "[device] idle timeout, exiting\n");
        break;
      }
      continue;
    }

    ssize_t n = ::read(fd, rx, sizeof(rx));
    if (n < 0) {
      // EIO happens on a PTY when the slave side is (re)opened/closed; tolerate.
      if (errno == EIO || errno == EAGAIN || errno == EINTR) {
        usleep(2000);
        continue;
      }
      break;
    }
    if (n == 0) {
      usleep(2000);
      continue;
    }
    last_activity = ::time(nullptr);

    auto packets = deframer.push(std::span<const uint8_t>(rx, static_cast<size_t>(n)));
    for (auto &pkt : packets) {
      std::vector<uint8_t> resp = core.process_bytes(pkt);
      if (resp.empty())
        continue; // fire-and-forget request, no ACK expected
      // Stream cap: keep the framed packet <= 127 bytes. Only the endpoint-0
      // JSON chunk can exceed this; truncating it is safe (client re-reads by
      // offset). The 2-byte response seq header is always preserved.
      if (resp.size() > kStreamMaxPacket)
        resp.resize(kStreamMaxPacket);
      auto framed = stream_frame(resp);
      ssize_t off = 0;
      while (off < static_cast<ssize_t>(framed.size())) {
        ssize_t w = ::write(fd, framed.data() + off, framed.size() - off);
        if (w < 0) {
          if (errno == EINTR || errno == EAGAIN) {
            usleep(1000);
            continue;
          }
          break;
        }
        off += w;
      }
    }
  }

  ::close(fd);
  return 0;
}
