#pragma once

#include "jpeg_header.hpp"
#include "rtp_packetizer.hpp"

namespace espp {

/// MJPEG packetizer that fragments JPEG frames into RFC 2435 RTP payloads.
///
/// This class takes complete JPEG frames and produces RTP payload chunks
/// suitable for MJPEG streaming. Each chunk contains an RFC 2435 MJPEG
/// header, and the first chunk additionally includes quantization tables.
class MjpegPacketizer : public RtpPacketizer {
public:
  /// Configuration for the MJPEG packetizer.
  struct Config {
    size_t max_payload_size{1400}; ///< Maximum payload bytes per RTP packet
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an MJPEG packetizer.
  /// @param config Configuration for the packetizer.
  explicit MjpegPacketizer(const Config &config)
      : RtpPacketizer({.max_payload_size = config.max_payload_size, .log_level = config.log_level},
                      "MjpegPacketizer") {}

  /// Packetize a complete JPEG frame into RFC 2435 RTP payload chunks.
  /// @param frame_data Raw JPEG data including the JPEG header.
  /// @return Vector of payload chunks ready to be wrapped in RTP packets.
  std::vector<RtpPayloadChunk> packetize(std::span<const uint8_t> frame_data) override;

  /// Get the RTP payload type for MJPEG.
  /// @return 26 (static JPEG payload type).
  int get_payload_type() const override;

  /// Get the RTP clock rate for MJPEG.
  /// @return 90000 Hz.
  uint32_t get_clock_rate() const override;

  /// Get the SDP media attributes for MJPEG.
  /// @return SDP rtpmap attribute string.
  std::string get_sdp_media_attributes() const override;

  /// Get the SDP media line for MJPEG.
  /// @return SDP media description line.
  std::string get_sdp_media_line() const override;
};

} // namespace espp
