#pragma once

#include <string>
#include <vector>

namespace espp {
/// @brief A class to represent a RTCP packet
/// @details This class is used to represent a RTCP packet.
///          It is used as a base class for all RTCP packet types.
class RtcpPacket {
public:
  RtcpPacket() = default;
  virtual ~RtcpPacket() = default;

  std::string_view get_data() const {
    return std::string_view(reinterpret_cast<const char *>(m_buffer), m_bufferSize);
  }

  void serialize() {}

protected:
  uint8_t *m_buffer;
  uint32_t m_bufferSize;
};
} // namespace espp
