#pragma once

#include <cstdint>
#include <string>

#include "rtp_packetizer.hpp"

namespace espp {

/// A generic RTP packetizer suitable for audio codecs (PCM, G.711, Opus, etc.)
/// or any pre-formatted data that simply needs MTU-based chunking. It splits
/// frame data into chunks of at most max_payload_size bytes and marks the last
/// chunk with the RTP marker bit.
///
/// \section generic_packetizer_ex1 Example
/// \snippet generic_packetizer_example.cpp generic_packetizer example
class GenericPacketizer : public RtpPacketizer {
public:
  /// Configuration for GenericPacketizer.
  struct Config {
    size_t max_payload_size{1400};    ///< Maximum payload bytes per RTP packet
    int payload_type{96};             ///< RTP payload type number
    uint32_t clock_rate{48000};       ///< Clock rate in Hz for RTP timestamps
    std::string encoding_name{"L16"}; ///< Encoding name for SDP rtpmap line
    int channels{1};                  ///< Number of audio channels
    std::string fmtp;                 ///< Optional format parameters for SDP fmtp line
    espp::MediaType media_type{espp::MediaType::AUDIO}; ///< Media type for the SDP m= line
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct a GenericPacketizer.
  /// @param config The configuration for this packetizer.
  explicit GenericPacketizer(const Config &config);

  /// Destructor.
  ~GenericPacketizer() override = default;

  /// Split frame data into RTP payload chunks of at most max_payload_size.
  /// The last (or only) chunk has its marker flag set.
  /// @param frame_data The raw frame bytes to packetize.
  /// @return A vector of RtpPayloadChunk ready to be wrapped in RTP packets.
  std::vector<RtpPayloadChunk> packetize(std::span<const uint8_t> frame_data) override;

  /// Get the RTP payload type number.
  /// @return The configured RTP payload type.
  int get_payload_type() const override;

  /// Get the RTP clock rate.
  /// @return The configured clock rate in Hz.
  uint32_t get_clock_rate() const override;

  /// Generate the SDP media-level attribute lines for this codec.
  /// Produces an a=rtpmap line and, if fmtp is non-empty, an a=fmtp line.
  /// @return A string containing the SDP a= lines.
  std::string get_sdp_media_attributes() const override;

  /// Generate the SDP m= line for this codec.
  /// @return A string such as "m=audio 0 RTP/AVP 96".
  std::string get_sdp_media_line() const override;

protected:
  int payload_type_;           ///< RTP payload type number
  uint32_t clock_rate_;        ///< Clock rate in Hz
  std::string encoding_name_;  ///< Encoding name for SDP
  int channels_;               ///< Number of audio channels
  std::string fmtp_;           ///< Optional format parameters
  espp::MediaType media_type_; ///< Media type (audio/video)
};

} // namespace espp
