#include "mjpeg_depacketizer.hpp"

#include <algorithm>

namespace espp {

void MjpegDepacketizer::process_packet(const RtpPacket &packet) {
  RtpJpegPacket jpeg_packet(packet.get_data());
  auto frag_offset = jpeg_packet.get_offset();
  auto jpeg_data = jpeg_packet.get_jpeg_data();

  if (frag_offset == 0) {
    // Start of a new frame — reset state and store MJPEG header info
    scan_buffer_.clear();
    received_sequence_numbers_.clear();

    frame_width_ = jpeg_packet.get_width();
    frame_height_ = jpeg_packet.get_height();
    if (jpeg_packet.has_q_tables()) {
      auto q0 = jpeg_packet.get_q_table(0);
      auto q1 = jpeg_packet.get_q_table(1);
      q0_table_.assign(q0.begin(), q0.end());
      q1_table_.assign(q1.begin(), q1.end());
    }

    scan_buffer_.assign(jpeg_data.begin(), jpeg_data.end());
    assembling_frame_ = true;
  } else if (assembling_frame_) {
    // Continuation fragment — place scan data at the correct offset
    size_t required_size = frag_offset + jpeg_data.size();
    if (required_size > scan_buffer_.size()) {
      scan_buffer_.resize(required_size, 0);
    }
    std::copy(jpeg_data.begin(), jpeg_data.end(), scan_buffer_.begin() + frag_offset);
  } else {
    // No frame in progress — discard this fragment
    logger_.debug("Ignoring packet with offset {} — no frame in progress", frag_offset);
    return;
  }

  received_sequence_numbers_.push_back(jpeg_packet.get_sequence_number());

  if (jpeg_packet.get_marker() && !has_missing_sequence_numbers()) {
    // Frame is complete
    assembling_frame_ = false;

    logger_.debug("Frame complete: {}x{}, scan={} bytes", frame_width_, frame_height_,
                  scan_buffer_.size());

    // Reconstruct the full JPEG: header + scan data
    JpegHeader header(frame_width_, frame_height_, std::span<const uint8_t>(q0_table_),
                      std::span<const uint8_t>(q1_table_));
    auto header_data = header.get_data();

    std::vector<uint8_t> jpeg_bytes;
    jpeg_bytes.reserve(header_data.size() + scan_buffer_.size());
    jpeg_bytes.insert(jpeg_bytes.end(), header_data.begin(), header_data.end());
    jpeg_bytes.insert(jpeg_bytes.end(), scan_buffer_.begin(), scan_buffer_.end());

    if (on_jpeg_frame_) {
      auto frame = std::make_shared<JpegFrame>(jpeg_bytes);
      on_jpeg_frame_(frame);
    }

    if (on_frame_) {
      on_frame_(std::move(jpeg_bytes));
    }
  }
}

void MjpegDepacketizer::set_jpeg_frame_callback(jpeg_frame_callback_t cb) {
  on_jpeg_frame_ = std::move(cb);
}

bool MjpegDepacketizer::has_missing_sequence_numbers() {
  if (received_sequence_numbers_.size() <= 1) {
    return false;
  }
  std::sort(received_sequence_numbers_.begin(), received_sequence_numbers_.end());
  int prev_seq = received_sequence_numbers_[0];
  for (size_t i = 1; i < received_sequence_numbers_.size(); ++i) {
    int curr_seq = received_sequence_numbers_[i];
    if (curr_seq != (prev_seq + 1) % 65536) {
      return true;
    }
    prev_seq = curr_seq;
  }
  return false;
}

} // namespace espp
