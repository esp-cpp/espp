#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "jpeg_frame.hpp"
#include "rtp_depacketizer.hpp"
#include "rtp_jpeg_packet.hpp"

namespace espp {

/// MJPEG depacketizer that reassembles JPEG frames from RTP packets.
///
/// This class receives individual RTP packets containing RFC 2435 MJPEG
/// payloads, reassembles the scan data fragments, reconstructs the JPEG
/// header from the MJPEG header fields, and delivers complete JPEG frames
/// through callbacks.
class MjpegDepacketizer : public RtpDepacketizer {
public:
  /// Callback type for receiving complete JPEG frames as JpegFrame objects.
  using jpeg_frame_callback_t = std::function<void(std::shared_ptr<JpegFrame>)>;

  /// Configuration for the MJPEG depacketizer.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an MJPEG depacketizer.
  /// @param config Configuration for the depacketizer.
  explicit MjpegDepacketizer(const Config &config)
      : RtpDepacketizer({.log_level = config.log_level}, "MjpegDepacketizer") {}

  /// Process an incoming RTP packet containing MJPEG data.
  /// @param packet The RTP packet to process.
  /// @note Packets are parsed as RtpJpegPacket. When a complete frame is
  ///       assembled (marker bit set and no missing sequence numbers), both
  ///       the generic frame callback and the JPEG frame callback are invoked.
  void process_packet(const RtpPacket &packet) override;

  /// Set callback for receiving complete JPEG frames.
  /// @param cb Callback receiving a shared pointer to the completed JpegFrame.
  void set_jpeg_frame_callback(jpeg_frame_callback_t cb);

private:
  /// Check if there are gaps in the received sequence numbers.
  /// @return true if there are missing sequence numbers, false otherwise.
  bool has_missing_sequence_numbers();

  bool assembling_frame_{false};
  std::vector<uint8_t> scan_buffer_;
  int frame_width_{0};
  int frame_height_{0};
  std::vector<uint8_t> q0_table_;
  std::vector<uint8_t> q1_table_;
  std::vector<int> received_sequence_numbers_;
  jpeg_frame_callback_t on_jpeg_frame_;
};

} // namespace espp
