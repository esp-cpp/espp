#include "generic_depacketizer.hpp"

using namespace espp;

GenericDepacketizer::GenericDepacketizer(const Config &config)
    : RtpDepacketizer({.log_level = config.log_level}, "GenericDepacketizer") {}

void GenericDepacketizer::process_packet(const RtpPacket &packet) {
  auto payload = packet.get_payload();
  int timestamp = packet.get_timestamp();

  // If the timestamp changed, discard the old buffer and start fresh
  if (has_timestamp_ && timestamp != current_timestamp_) {
    logger_.warn("Timestamp changed ({} -> {}), discarding {} buffered bytes", current_timestamp_,
                 timestamp, buffer_.size());
    reset_buffer();
  }

  current_timestamp_ = timestamp;
  has_timestamp_ = true;

  // Append this packet's payload to the buffer
  buffer_.insert(buffer_.end(), payload.begin(), payload.end());

  logger_.debug("Buffered {} bytes (total: {}, marker: {})", payload.size(), buffer_.size(),
                packet.get_marker());

  // If the marker bit is set, deliver the complete frame
  if (packet.get_marker()) {
    if (on_frame_) {
      logger_.debug("Delivering frame of {} bytes", buffer_.size());
      on_frame_(std::move(buffer_));
    }
    reset_buffer();
  }
}

void GenericDepacketizer::reset_buffer() {
  buffer_.clear();
  has_timestamp_ = false;
  current_timestamp_ = -1;
}
