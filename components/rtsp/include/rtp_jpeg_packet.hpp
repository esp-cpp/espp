#pragma once

#include "rtp_packet.hpp"

namespace espp {
/// RTP packet for JPEG video.
/// The RTP payload for JPEG is defined in RFC 2435.
class RtpJpegPacket : public RtpPacket {
public:
  /// Construct an RTP packet from a buffer.
  /// @param data The buffer containing the RTP packet.
  explicit RtpJpegPacket(std::string_view data)
      : RtpPacket(data) {
    parse_mjpeg_header();
  }

  /// Construct an RTP packet from fields
  /// @details This will construct a packet with quantization tables, so it
  ///          can only be used for the first packet in a frame.
  /// @param type_specific The type-specific field.
  /// @param frag_type The fragment type field.
  /// @param q The q field.
  /// @param width The width field.
  /// @param height The height field.
  /// @param q0 The first quantization table.
  /// @param q1 The second quantization table.
  /// @param scan_data The scan data.
  explicit RtpJpegPacket(const int type_specific, const int frag_type, const int q, const int width,
                         const int height, std::string_view q0, std::string_view q1,
                         std::string_view scan_data)
      : RtpPacket(PAYLOAD_OFFSET_WITH_QUANT + scan_data.size())
      , type_specific_(type_specific)
      , offset_(0)
      , frag_type_(frag_type)
      , q_(q)
      , width_(width)
      , height_(height) {

    jpeg_data_start_ = PAYLOAD_OFFSET_WITH_QUANT;
    jpeg_data_size_ = scan_data.size();

    serialize_mjpeg_header();
    serialize_q_tables(q0, q1);

    auto &packet = get_packet();
    size_t jpeg_offset = jpeg_data_start_ + get_rtp_header_size();
    memcpy(packet.data() + jpeg_offset, scan_data.data(), scan_data.size());
  }

  /// Construct an RTP packet from fields
  /// @details This will construct a packet without quantization tables, so it
  ///          cannot be used for the first packet in a frame.
  /// @param type_specific The type-specific field.
  /// @param offset The offset field.
  /// @param frag_type The fragment type field.
  /// @param q The q field.
  /// @param width The width field.
  /// @param height The height field.
  /// @param scan_data The scan data.
  explicit RtpJpegPacket(const int type_specific, const int offset, const int frag_type,
                         const int q, const int width, const int height, std::string_view scan_data)
      : RtpPacket(PAYLOAD_OFFSET_NO_QUANT + scan_data.size())
      , type_specific_(type_specific)
      , offset_(offset)
      , frag_type_(frag_type)
      , q_(q)
      , width_(width)
      , height_(height) {
    jpeg_data_start_ = PAYLOAD_OFFSET_NO_QUANT;
    jpeg_data_size_ = scan_data.size();

    serialize_mjpeg_header();

    auto &packet = get_packet();
    size_t jpeg_offset = jpeg_data_start_ + get_rtp_header_size();
    memcpy(packet.data() + jpeg_offset, scan_data.data(), scan_data.size());
  }

  ~RtpJpegPacket() {}

  /// Get the type-specific field.
  /// @return The type-specific field.
  int get_type_specific() const { return type_specific_; }

  /// Get the offset field.
  /// @return The offset field.
  int get_offset() const { return offset_; }

  /// Get the fragment type field.
  /// @return The fragment type field.
  int get_q() const { return q_; }

  /// Get the fragment type field.
  /// @return The fragment type field.
  int get_width() const { return width_; }

  /// Get the fragment type field.
  /// @return The fragment type field.
  int get_height() const { return height_; }

  /// Get the mjepg header.
  /// @return The mjepg header.
  std::string_view get_mjpeg_header() const {
    return std::string_view((char *)get_payload().data(), MJPEG_HEADER_SIZE);
  }

  /// Get whether the packet contains quantization tables.
  /// @note The quantization tables are optional. If they are present, the
  /// number of quantization tables is always 2.
  /// @note This check is based on the value of the q field. If the q field
  ///       is 128-256, the packet contains quantization tables.
  /// @return Whether the packet contains quantization tables.
  bool has_q_tables() const { return q_ >= 128; }

  /// Get the number of quantization tables.
  /// @note The quantization tables are optional. If they are present, the
  /// number of quantization tables is always 2.
  /// @note Only the first packet in a frame contains quantization tables.
  /// @return The number of quantization tables.
  int get_num_q_tables() const { return q_tables_.size(); }

  /// Get the quantization table at the specified index.
  /// @param index The index of the quantization table.
  /// @return The quantization table at the specified index.
  std::string_view get_q_table(int index) const {
    if (index < get_num_q_tables()) {
      return q_tables_[index];
    }
    return {};
  }

  void set_q_table(int index, std::string_view q_table) {
    if (index < get_num_q_tables()) {
      q_tables_[index] = q_table;
    }
  }

  /// Get the JPEG data.
  /// The jpeg data is the payload minus the mjpeg header and quantization
  /// tables.
  /// @return The JPEG data.
  std::string_view get_jpeg_data() const {
    auto payload = get_payload();
    return std::string_view((char *)payload.data() + jpeg_data_start_, jpeg_data_size_);
  }

protected:
  static constexpr int MJPEG_HEADER_SIZE = 8;
  static constexpr int QUANT_HEADER_SIZE = 4;
  static constexpr int NUM_Q_TABLES = 2;
  static constexpr int Q_TABLE_SIZE = 64;

  static constexpr int PAYLOAD_OFFSET_NO_QUANT = MJPEG_HEADER_SIZE;
  static constexpr int PAYLOAD_OFFSET_WITH_QUANT =
      MJPEG_HEADER_SIZE + QUANT_HEADER_SIZE + (NUM_Q_TABLES * Q_TABLE_SIZE);

  void parse_mjpeg_header() {
    auto payload = get_payload();
    type_specific_ = payload[0];
    offset_ = (payload[1] << 16) | (payload[2] << 8) | payload[3];
    frag_type_ = payload[4];
    q_ = payload[5];
    width_ = payload[6] * 8;
    height_ = payload[7] * 8;

    size_t offset = MJPEG_HEADER_SIZE;

    if (has_q_tables()) {
      uint8_t num_quant_bytes = payload[11];
      int expected_num_quant_bytes = NUM_Q_TABLES * Q_TABLE_SIZE;
      if (num_quant_bytes == expected_num_quant_bytes) {
        q_tables_.resize(NUM_Q_TABLES);
        offset += QUANT_HEADER_SIZE;
        for (int i = 0; i < NUM_Q_TABLES; i++) {
          q_tables_[i] = std::string_view((char *)payload.data() + offset, Q_TABLE_SIZE);
          offset += Q_TABLE_SIZE;
        }
      }
    }

    jpeg_data_start_ = offset;
    jpeg_data_size_ = payload.size() - jpeg_data_start_;
  }

  void serialize_mjpeg_header() {
    auto &packet = get_packet();
    size_t offset = get_rtp_header_size();

    packet[offset++] = type_specific_;
    packet[offset++] = (offset_ >> 16) & 0xff;
    packet[offset++] = (offset_ >> 8) & 0xff;
    packet[offset++] = offset_ & 0xff;
    packet[offset++] = frag_type_;
    packet[offset++] = q_;
    packet[offset++] = width_ / 8;
    packet[offset++] = height_ / 8;
  }

  void serialize_q_tables(std::string_view q0, std::string_view q1) {
    q_tables_.resize(NUM_Q_TABLES);
    auto &packet = get_packet();
    int offset = get_rtp_header_size() + MJPEG_HEADER_SIZE;
    packet[offset++] = 0;
    packet[offset++] = 0;
    packet[offset++] = 0;
    packet[offset++] = NUM_Q_TABLES * Q_TABLE_SIZE;

    memcpy(packet.data() + offset, q0.data(), Q_TABLE_SIZE);
    q_tables_[0] = std::string_view((char *)packet.data() + offset, Q_TABLE_SIZE);
    offset += Q_TABLE_SIZE;

    memcpy(packet.data() + offset, q1.data(), Q_TABLE_SIZE);
    q_tables_[1] = std::string_view((char *)packet.data() + offset, Q_TABLE_SIZE);
    // offset += Q_TABLE_SIZE;
  }

  uint8_t type_specific_{0};
  uint32_t offset_{0};
  uint8_t frag_type_{0};
  uint8_t q_{0};
  uint32_t width_{0};
  uint32_t height_{0};
  int jpeg_data_start_{0};
  int jpeg_data_size_{0};
  std::vector<std::string_view> q_tables_;
};
} // namespace espp
