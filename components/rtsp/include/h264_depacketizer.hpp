#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "rtp_depacketizer.hpp"
#include "rtp_packet.hpp"

namespace espp {

/// @brief RTP depacketizer for H.264 video per RFC 6184.
///
/// Reassembles H.264 access units from incoming RTP packets. Supports:
///   - **Single NAL unit** packets (NAL type 1–23)
///   - **STAP-A** aggregation packets (NAL type 24)
///   - **FU-A** fragmentation packets (NAL type 28)
///
/// When the RTP marker bit is set, the accumulated NAL units are delivered
/// as one Annex B byte-stream (each NAL prefixed with 0x00 0x00 0x00 0x01)
/// via the frame callback set with set_frame_callback().
///
/// \section h264_depacketizer_ex1 Example
/// \snippet h264_depacketizer_example.cpp h264_depacketizer example
class H264Depacketizer : public RtpDepacketizer {
public:
  /// Configuration for the H264Depacketizer.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an H264Depacketizer.
  /// @param config The configuration for the depacketizer.
  explicit H264Depacketizer(const Config &config);

  /// Destructor.
  ~H264Depacketizer() override = default;

  /// Process an incoming RTP packet containing H.264 payload.
  ///
  /// Handles single NAL, STAP-A, and FU-A packet types. NAL units are
  /// buffered until the RTP marker bit indicates the end of an access unit,
  /// at which point the complete Annex B frame is delivered via the callback.
  ///
  /// @param packet The RTP packet to process.
  void process_packet(const RtpPacket &packet) override;

protected:
  /// Deliver the buffered NAL units as a single Annex B frame.
  void deliver_frame();

  /// Buffer of accumulated NAL units for the current access unit.
  std::vector<std::vector<uint8_t>> nal_buffer_;

  /// In-progress NAL unit being assembled from FU-A fragments.
  std::vector<uint8_t> fua_buffer_;

  /// Whether we are currently assembling FU-A fragments.
  bool fua_in_progress_{false};
};

} // namespace espp
