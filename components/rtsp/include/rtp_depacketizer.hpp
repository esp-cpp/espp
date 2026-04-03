#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "rtp_packet.hpp"

namespace espp {

/// Abstract base class for reassembling media frames from incoming RTP packets.
/// Concrete depacketizers (e.g. MJPEG, H.264) override process_packet() to
/// accumulate payload data and invoke the frame callback when a complete frame
/// has been assembled.
class RtpDepacketizer : public BaseComponent {
public:
  /// Callback type invoked when a complete frame has been reassembled.
  /// The frame data is moved into the callback to avoid copies.
  using frame_callback_t = std::function<void(std::vector<uint8_t> &&)>;

  /// Configuration for RtpDepacketizer.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an RtpDepacketizer.
  /// @param config The configuration for this depacketizer.
  /// @param name   A human-readable name used for logging.
  explicit RtpDepacketizer(const Config &config, const std::string &name)
      : BaseComponent(name, config.log_level) {}

  /// Destructor.
  virtual ~RtpDepacketizer() = default;

  /// Process an incoming RTP packet, accumulating payload data.
  /// When a complete frame is assembled the frame callback is invoked.
  /// @param packet The RTP packet to process.
  virtual void process_packet(const RtpPacket &packet) = 0;

  /// Set the callback for completed frames.
  /// @param cb The callback to invoke when a full frame is ready.
  void set_frame_callback(frame_callback_t cb) { on_frame_ = std::move(cb); }

protected:
  frame_callback_t on_frame_; ///< Callback invoked with each reassembled frame
};

} // namespace espp
