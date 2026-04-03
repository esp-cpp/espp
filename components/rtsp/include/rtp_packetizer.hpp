#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "rtp_types.hpp"

namespace espp {

/// Abstract base class for splitting media frames into RTP payload chunks.
/// Concrete packetizers (e.g. MJPEG, H.264) override the pure-virtual methods
/// to produce codec-specific payloads. The RTSP server wraps each returned
/// RtpPayloadChunk with an RTP header before sending.
class RtpPacketizer : public BaseComponent {
public:
  /// Configuration for RtpPacketizer.
  struct Config {
    size_t max_payload_size{1400}; ///< Maximum payload bytes per RTP packet
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an RtpPacketizer.
  /// @param config The configuration for this packetizer.
  /// @param name   A human-readable name used for logging.
  explicit RtpPacketizer(const Config &config, const std::string &name)
      : BaseComponent(name, config.log_level)
      , max_payload_size_(config.max_payload_size) {}

  /// Destructor.
  virtual ~RtpPacketizer() = default;

  /// Packetize a complete media frame into RTP payload chunks.
  /// @param frame_data The raw frame bytes to packetize.
  /// @return A vector of RtpPayloadChunk ready to be wrapped in RTP packets.
  virtual std::vector<RtpPayloadChunk> packetize(std::span<const uint8_t> frame_data) = 0;

  /// Get the RTP payload type number for this codec.
  /// @return The RTP payload type (e.g. 26 for MJPEG, 96 for dynamic).
  virtual int get_payload_type() const = 0;

  /// Get the RTP clock rate for timestamp calculation.
  /// @return The clock rate in Hz (e.g. 90000 for video, 8000 for audio).
  virtual uint32_t get_clock_rate() const = 0;

  /// Generate the SDP media-level attributes for this codec.
  /// @return A string containing SDP a= lines (without trailing CRLF).
  virtual std::string get_sdp_media_attributes() const = 0;

  /// Generate the SDP m= line for this codec.
  /// @return A string containing the SDP m= line (without trailing CRLF).
  virtual std::string get_sdp_media_line() const = 0;

protected:
  size_t max_payload_size_; ///< Maximum payload bytes per RTP packet
};

} // namespace espp
