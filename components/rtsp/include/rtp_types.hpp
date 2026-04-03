#pragma once

#include <cstdint>
#include <vector>

namespace espp {

/// Describes a media type for RTSP tracks.
enum class MediaType {
  VIDEO, ///< Video media (MJPEG, H264, etc.)
  AUDIO  ///< Audio media (PCM, Opus, AAC, etc.)
};

/// Represents one RTP payload chunk ready to be wrapped in an RtpPacket.
/// Packetizers produce these; the server wraps them with RTP headers.
struct RtpPayloadChunk {
  std::vector<uint8_t> data; ///< The payload data for this chunk
  bool marker{false};        ///< Set on last chunk of a frame/access unit
};

} // namespace espp
