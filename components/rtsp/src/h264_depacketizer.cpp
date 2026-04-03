#include "h264_depacketizer.hpp"

using namespace espp;

/// Annex B start code: 0x00 0x00 0x00 0x01
static constexpr uint8_t kStartCode[] = {0x00, 0x00, 0x00, 0x01};
static constexpr size_t kStartCodeSize = sizeof(kStartCode);

H264Depacketizer::H264Depacketizer(const Config &config)
    : RtpDepacketizer({.log_level = config.log_level}, "H264Depacketizer") {}

void H264Depacketizer::process_packet(const RtpPacket &packet) {
  auto payload = packet.get_payload();
  if (payload.empty()) {
    logger_.warn("Received RTP packet with empty payload");
    return;
  }

  uint8_t first_byte = payload[0];
  uint8_t nal_type = first_byte & 0x1F;

  if (nal_type >= 1 && nal_type <= 23) {
    // Single NAL unit packet — the entire payload is one NAL.
    logger_.debug("Single NAL: type={}, size={}", nal_type, payload.size());
    nal_buffer_.emplace_back(payload.begin(), payload.end());

  } else if (nal_type == 24) {
    // STAP-A — aggregation packet containing multiple NAL units.
    // Format: STAP-A header (1 byte) + { NAL size (2 bytes, big-endian) + NAL data } ...
    logger_.debug("STAP-A: payload size={}", payload.size());
    size_t offset = 1; // skip STAP-A header byte
    while (offset + 2 <= payload.size()) {
      uint16_t nal_size = (static_cast<uint16_t>(payload[offset]) << 8) |
                          static_cast<uint16_t>(payload[offset + 1]);
      offset += 2;

      if (offset + nal_size > payload.size()) {
        logger_.warn("STAP-A: NAL size {} exceeds remaining payload at offset {}", nal_size,
                     offset);
        break;
      }

      nal_buffer_.emplace_back(payload.data() + offset, payload.data() + offset + nal_size);
      offset += nal_size;
    }

  } else if (nal_type == 28) {
    // FU-A — fragmentation unit.
    if (payload.size() < 2) {
      logger_.warn("FU-A: payload too short ({})", payload.size());
      return;
    }

    uint8_t fu_indicator = payload[0];
    uint8_t fu_header = payload[1];
    bool start_bit = (fu_header & 0x80) != 0;
    bool end_bit = (fu_header & 0x40) != 0;
    uint8_t original_nal_type = fu_header & 0x1F;

    if (start_bit) {
      // Start of a new fragmented NAL unit.
      // Reconstruct the NAL header from the FU indicator NRI and the FU header type.
      uint8_t reconstructed_header = (fu_indicator & 0xE0) | original_nal_type;
      fua_buffer_.clear();
      fua_buffer_.push_back(reconstructed_header);
      fua_buffer_.insert(fua_buffer_.end(), payload.data() + 2, payload.data() + payload.size());
      fua_in_progress_ = true;
      logger_.debug("FU-A start: nal_type={}, fragment_size={}", original_nal_type,
                    payload.size() - 2);
    } else if (fua_in_progress_) {
      // Continuation or end fragment.
      fua_buffer_.insert(fua_buffer_.end(), payload.data() + 2, payload.data() + payload.size());
      logger_.debug("FU-A {}: fragment_size={}, total={}", end_bit ? "end" : "mid",
                    payload.size() - 2, fua_buffer_.size());
    } else {
      logger_.warn("FU-A: received continuation/end fragment without start");
      return;
    }

    if (end_bit) {
      // FU-A reassembly complete — move into the NAL buffer.
      nal_buffer_.push_back(std::move(fua_buffer_));
      fua_buffer_.clear();
      fua_in_progress_ = false;
    }

  } else {
    logger_.warn("Unsupported NAL unit type: {}", nal_type);
    return;
  }

  // If the RTP marker bit is set, the access unit is complete.
  if (packet.get_marker()) {
    deliver_frame();
  }
}

void H264Depacketizer::deliver_frame() {
  if (nal_buffer_.empty()) {
    logger_.debug("Marker set but no NAL units buffered");
    return;
  }

  // Build Annex B frame: each NAL prefixed with 0x00 0x00 0x00 0x01.
  size_t total_size = 0;
  for (const auto &nal : nal_buffer_) {
    total_size += kStartCodeSize + nal.size();
  }

  std::vector<uint8_t> frame;
  frame.reserve(total_size);
  for (const auto &nal : nal_buffer_) {
    frame.insert(frame.end(), kStartCode, kStartCode + kStartCodeSize);
    frame.insert(frame.end(), nal.begin(), nal.end());
  }

  logger_.debug("Delivering frame: {} NAL(s), {} bytes", nal_buffer_.size(), frame.size());
  nal_buffer_.clear();

  if (on_frame_) {
    on_frame_(std::move(frame));
  }
}
