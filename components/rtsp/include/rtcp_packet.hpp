#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace espp {
/// @brief A class to represent a RTCP packet
/// @details This class is used to represent a RTCP packet.
///          It is used as a base class for all RTCP packet types.
/// @note At the moment, this class is not used.
class RtcpPacket {
public:
  /// @brief Constructor, default
  RtcpPacket() = default;

  /// @brief Destructor, default
  virtual ~RtcpPacket() = default;

  /// @brief Get the buffer of the packet
  /// @return The buffer of the packet
  std::string_view get_data() const;

protected:
  uint8_t *m_buffer{nullptr};
  uint32_t m_bufferSize{0};
};
} // namespace espp
