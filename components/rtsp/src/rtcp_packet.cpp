#include "rtcp_packet.hpp"

using namespace espp;

std::string_view RtcpPacket::get_data() const {
  return std::string_view(reinterpret_cast<const char *>(m_buffer), m_bufferSize);
}
