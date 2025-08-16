#pragma once

#include <span>

#include "jpeg_header.hpp"
#include "rtp_jpeg_packet.hpp"

namespace espp {
/// A class that represents a complete JPEG frame.
///
/// This class is used to collect the JPEG scans that are received in RTP
/// packets and to serialize them into a complete JPEG frame.
class JpegFrame {
public:
  /// Construct a JpegFrame from a RtpJpegPacket.
  ///
  /// This constructor will parse the header of the packet and add the JPEG
  /// data to the frame.
  ///
  /// @param packet The packet to parse.
  explicit JpegFrame(const espp::RtpJpegPacket &packet)
      : data_(std::begin(packet.get_jpeg_data()), std::end(packet.get_jpeg_data()))
      , header_(packet.get_jpeg_data()) {}

  /// Construct a JpegFrame from a vector of jpeg data.
  /// @param data The vector containing the jpeg data.
  /// @note The vector must contain the complete JPEG data, including the JPEG
  ///       header and EOI marker.
  explicit JpegFrame(const std::vector<uint8_t> &data)
      : data_(data)
      , header_(std::span<const uint8_t>(data.data(), data.size())) {}

  /// Construct a JpegFrame from a span of jpeg data.
  /// @param data The span containing the jpeg data.
  /// @note The span must contain the complete JPEG data, including the JPEG
  ///       header and EOI marker.
  explicit JpegFrame(std::span<const uint8_t> data)
      : data_(data.begin(), data.end())
      , header_(data) {}

  /// Construct a JpegFrame from buffer of jpeg data
  /// @param data The buffer containing the jpeg data.
  /// @param size The size of the buffer.
  explicit JpegFrame(const uint8_t *data, size_t size)
      : data_(data, data + size)
      , header_(std::span<const uint8_t>(data, size)) {}

  /// Get a reference to the header.
  /// @return A reference to the header.
  const espp::JpegHeader &get_header() const { return header_; }

  /// Get the width of the frame.
  /// @return The width of the frame.
  int get_width() const { return header_.get_width(); }

  /// Get the height of the frame.
  /// @return The height of the frame.
  int get_height() const { return header_.get_height(); }

  /// Check if the frame is complete.
  /// @return True if the frame is complete, false otherwise.
  bool is_complete() const { return finalized_; }

  /// Append a RtpJpegPacket to the frame.
  /// This will add the JPEG data to the frame.
  /// @param packet The packet containing the scan to append.
  void append(const espp::RtpJpegPacket &packet) { add_scan(packet); }

  /// Append a JPEG scan to the frame.
  /// This will add the JPEG data to the frame.
  /// @note If the packet contains the EOI marker, the frame will be
  ///       finalized, and no further scans can be added.
  /// @param packet The packet containing the scan to append.
  void add_scan(const espp::RtpJpegPacket &packet) {
    add_scan(packet.get_jpeg_data(), packet.get_offset());
    auto sequence_number = packet.get_sequence_number();
    received_sequence_numbers_.push_back(sequence_number);
    if (packet.get_marker()) {
      // check to ensure we got all the packets (sequence numbers) we expected
      // based on start and this packet. What this means is that there should be
      // no gaps in the received sequence numbers.
      if (!has_missing_sequence_numbers()) {
        finalize();
      } else {
        // fmt::print("JpegFrame: Missing sequence numbers in received packets {}\n",
        //            received_sequence_numbers_);
      }
    }
  }

  /// Get the serialized data.
  /// This will return the serialized data.
  /// @return The serialized data.
  std::span<const uint8_t> get_data() const {
    return std::span<const uint8_t>(data_.data(), data_.size());
  }

  /// Get the scan data.
  /// This will return the scan data.
  /// @return The scan data.
  std::span<const uint8_t> get_scan_data() const {
    auto header_data = header_.get_data();
    size_t header_size = header_data.size();
    return std::span<const uint8_t>(data_.data() + header_size, data_.size() - header_size);
  }

protected:
  /// Append a JPEG scan to the frame.
  /// This will add the JPEG data to the frame.
  /// @param scan The jpeg scan to append.
  void add_scan(std::span<const uint8_t> scan) {
    if (finalized_) {
      // TODO: handle this error
      return;
    }
    data_.insert(std::end(data_), std::begin(scan), std::end(scan));
  }

  /// Append a JPEG scan to the frame at a specific offset.
  /// This will add the JPEG data to the frame at the specified offset.
  /// @param scan The jpeg scan to append.
  /// @param frag_offset The offset at which to append the scan.
  /// @note If the offset is greater than the current data size, the data will
  ///       be resized to accommodate the new data.
  void add_scan(std::span<const uint8_t> scan, size_t frag_offset) {
    if (finalized_) {
      // TODO: handle this error
      return;
    }
    // if the required span position is greater than the current data size, we
    // need to pad the data with zeros.
    size_t required_size = frag_offset + scan.size();
    if (required_size >= data_.size()) {
      data_.resize(required_size, 0);
    }
    std::copy(scan.begin(), scan.end(), data_.begin() + frag_offset);
  }

  /// Check if there are any missing sequence numbers.
  /// This will check if there are any missing sequence numbers in the received
  /// sequence numbers.
  /// @return True if there are any missing sequence numbers, false otherwise.
  bool has_missing_sequence_numbers() {
    if (received_sequence_numbers_.empty() || received_sequence_numbers_.size() == 1) {
      return false;
    }
    // first let's sort the sequence numbers
    std::sort(received_sequence_numbers_.begin(), received_sequence_numbers_.end());
    // check for gaps in the sequence numbers. Since we've sorted the list, we
    // can simply iterate and ensure that the difference between consecutive
    // sequence numbers is 1.
    int prev_seq = received_sequence_numbers_[0];
    for (size_t i = 1; i < received_sequence_numbers_.size(); ++i) {
      int curr_seq = received_sequence_numbers_[i];
      if (curr_seq != (prev_seq + 1) % 65536) {
        return true; // found a gap
      }
      prev_seq = curr_seq;
    }
    // if we made it here, there are no gaps
    return false;
  }

  /// Add the EOI marker to the frame.
  /// This will add the EOI marker to the frame. This must be called before
  /// calling get_data().
  /// @note This will prevent any further scans from being added to the frame.
  void finalize() {
    if (!finalized_) {
      finalized_ = true;
    } else {
      // TODO: handle this error
      // already finalized
    }
  }

  std::vector<int> received_sequence_numbers_;
  std::vector<uint8_t> data_;
  JpegHeader header_;
  bool finalized_ = false;
};
} // namespace espp
