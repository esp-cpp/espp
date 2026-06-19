// Shared helpers for the RTPS host-side tests (pc/tests/rtps_*.cpp).
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "espp.hpp"

namespace rtps_test {

/// Best-effort guess of this machine's primary outbound IPv4 address. RTPS discovery is multicast,
/// so a real interface address (not 127.0.0.1) is needed for cross-host / cross-process discovery.
inline std::string guess_local_ipv4() {
#if defined(_WIN32)
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  std::string ip = "127.0.0.1";
  int sock = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, 0));
  if (sock >= 0) {
    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(53);
    ::inet_pton(AF_INET, "8.8.8.8", &remote.sin_addr);
    if (::connect(sock, reinterpret_cast<sockaddr *>(&remote), sizeof(remote)) == 0) {
      sockaddr_in local{};
      socklen_t len = sizeof(local);
      if (::getsockname(sock, reinterpret_cast<sockaddr *>(&local), &len) == 0) {
        char buf[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf))) {
          ip = buf;
        }
      }
    }
#if defined(_WIN32)
    ::closesocket(sock);
#else
    ::close(sock);
#endif
  }
  return ip;
}

/// Serialize a uint32 as an encapsulated little-endian CDR payload (matches std_msgs/msg/UInt32).
inline std::vector<uint8_t> serialize_uint32(uint32_t value) {
  espp::CdrWriter writer; // defaults: CDR_LE with a 4-byte encapsulation header
  writer.write<uint32_t>(value);
  return writer.take_buffer();
}

/// Parse a uint32 from an encapsulated CDR payload, or std::nullopt if invalid.
inline std::optional<uint32_t> deserialize_uint32(std::span<const uint8_t> cdr) {
  espp::CdrReader reader(cdr);
  uint32_t value = 0;
  if (!reader.valid() || !reader.read<uint32_t>(value)) {
    return std::nullopt;
  }
  return value;
}

} // namespace rtps_test
