#pragma once

#include <cstdint>
#include <vector>

#include "rtp_depacketizer.hpp"

namespace espp {

/// A generic RTP depacketizer that reassembles media frames from incoming RTP
/// packets. It accumulates payload data until a packet with the marker bit set
/// is received, then delivers the complete frame via the frame callback. If a
/// packet arrives with a different RTP timestamp than the current accumulation
/// buffer, the old buffer is discarded and a new one is started.
///
/// This is suitable for audio codecs (PCM, G.711, Opus, etc.) or any payload
/// format that uses simple marker-based framing.
///
/// \section generic_depacketizer_ex1 Example
/// \snippet generic_depacketizer_example.cpp generic_depacketizer example
class GenericDepacketizer : public RtpDepacketizer {
public:
  /// Configuration for GenericDepacketizer.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct a GenericDepacketizer.
  /// @param config The configuration for this depacketizer.
  explicit GenericDepacketizer(const Config &config);

  /// Destructor.
  ~GenericDepacketizer() override = default;

  /// Process an incoming RTP packet.
  /// Payload data is accumulated until a packet with the marker bit set is
  /// received. At that point the assembled frame is delivered via the frame
  /// callback and the buffer is reset.
  /// @param packet The RTP packet to process.
  void process_packet(const RtpPacket &packet) override;

protected:
  /// Reset the accumulation buffer.
  void reset_buffer();

  std::vector<uint8_t> buffer_; ///< Accumulation buffer for frame data
  int current_timestamp_{-1};   ///< RTP timestamp of the current buffer
  bool has_timestamp_{false};   ///< Whether the buffer has a valid timestamp
};

} // namespace espp
