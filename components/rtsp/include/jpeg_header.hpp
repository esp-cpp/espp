#pragma once

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace espp {
/// A class to generate a JPEG header for a given image size and quantization tables.
/// The header is generated once and then cached for future use.
/// The header is generated according to the JPEG standard and is compatible with
/// the ESP32 camera driver.
class JpegHeader {
public:
  /// Create a JPEG header for a given image size and quantization tables.
  /// @param width The image width in pixels.
  /// @param height The image height in pixels.
  /// @param q0_table The quantization table for the Y channel.
  /// @param q1_table The quantization table for the Cb and Cr channels.
  explicit JpegHeader(int width, int height, std::span<const uint8_t> q0_table,
                      std::span<const uint8_t> q1_table)
      : width_(width)
      , height_(height)
      , q0_table_(
            std::string_view(reinterpret_cast<const char *>(q0_table.data()), q0_table.size()))
      , q1_table_(
            std::string_view(reinterpret_cast<const char *>(q1_table.data()), q1_table.size())) {
    serialize();
  }

  /// Create a JPEG header from a given JPEG header data.
  explicit JpegHeader(std::span<const uint8_t> data)
      : data_(data.data(), data.data() + data.size()) {
    parse();
  }

  /// Destructor.
  ~JpegHeader() {}

  /// Get the image width.
  /// @return The image width in pixels.
  int get_width() const { return width_; }

  /// Get the image height.
  /// @return The image height in pixels.
  int get_height() const { return height_; }

  /// Get the size of the JPEG header data.
  /// @return The size of the JPEG header data in bytes.
  /// @note This is the size of the serialized JPEG header, not the image size.
  size_t size() const { return data_.size(); }

  /// Get the JPEG header data.
  /// @return The JPEG header data.
  std::span<const uint8_t> get_data() const {
    return std::span<const uint8_t>(data_.data(), data_.size());
  }

  /// Get the Quantization table at the index.
  /// @param index The index of the quantization table.
  /// @return The quantization table.
  std::span<const uint8_t> get_quantization_table(int index) const {
    return index == 0
               ? std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(q0_table_.data()),
                                          q0_table_.size())
               : std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(q1_table_.data()),
                                          q1_table_.size());
  }

protected:
  static constexpr int SOF0_SIZE = 19;
  static constexpr int DQT_HEADER_SIZE = 5;

  // JFIF APP0 Marker for version 1.2 with 72 DPI and no thumbnail
  static constexpr uint8_t JFIF_APP0_DATA[] = {
      0xFF, 0xE0,                   // APP0 marker
      0x00, 0x10,                   // Length of APP0 data (16 bytes)
      0x4A, 0x46, 0x49, 0x46, 0x00, // Identifier: ASCII "JFIF\0"
      0x01, 0x01,                   // Version number (1.1)

      // NOTE: when comparing, we only care about the above bytes, not the
      //       following bytes

      0x01,       // Units: 1 = dots per inch
      0x00, 0x00, // X density
      0x00, 0x00, // Y density
      0x00, 0x00  // No thumbnail
  };

  static constexpr size_t JFIF_APP0_SIZE = sizeof(JFIF_APP0_DATA);
  static constexpr size_t JFIF_APP0_CMP_LEN = 11; // Length of the JFIF identifier and version

  static constexpr uint8_t HUFFMAN_TABLES[] = {
      // Huffman table DC (luminance)
      0xff,
      0xc4,
      0x00,
      0x1f,
      0x00,
      0x00,
      0x01,
      0x05,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x01,
      0x02,
      0x03,
      0x04,
      0x05,
      0x06,
      0x07,
      0x08,
      0x09,
      0x0a,
      0x0b,
      // Huffman table AC (luminance)
      0xff,
      0xc4,
      0x00,
      0xb5,
      0x10,
      0x00,
      0x02,
      0x01,
      0x03,
      0x03,
      0x02,
      0x04,
      0x03,
      0x05,
      0x05,
      0x04,
      0x04,
      0x00,
      0x00,
      0x01,
      0x7d,
      0x01,
      0x02,
      0x03,
      0x00,
      0x04,
      0x11,
      0x05,
      0x12,
      0x21,
      0x31,
      0x41,
      0x06,
      0x13,
      0x51,
      0x61,
      0x07,
      0x22,
      0x71,
      0x14,
      0x32,
      0x81,
      0x91,
      0xa1,
      0x08,
      0x23,
      0x42,
      0xb1,
      0xc1,
      0x15,
      0x52,
      0xd1,
      0xf0,
      0x24,
      0x33,
      0x62,
      0x72,
      0x82,
      0x09,
      0x0a,
      0x16,
      0x17,
      0x18,
      0x19,
      0x1a,
      0x25,
      0x26,
      0x27,
      0x28,
      0x29,
      0x2a,
      0x34,
      0x35,
      0x36,
      0x37,
      0x38,
      0x39,
      0x3a,
      0x43,
      0x44,
      0x45,
      0x46,
      0x47,
      0x48,
      0x49,
      0x4a,
      0x53,
      0x54,
      0x55,
      0x56,
      0x57,
      0x58,
      0x59,
      0x5a,
      0x63,
      0x64,
      0x65,
      0x66,
      0x67,
      0x68,
      0x69,
      0x6a,
      0x73,
      0x74,
      0x75,
      0x76,
      0x77,
      0x78,
      0x79,
      0x7a,
      0x83,
      0x84,
      0x85,
      0x86,
      0x87,
      0x88,
      0x89,
      0x8a,
      0x92,
      0x93,
      0x94,
      0x95,
      0x96,
      0x97,
      0x98,
      0x99,
      0x9a,
      0xa2,
      0xa3,
      0xa4,
      0xa5,
      0xa6,
      0xa7,
      0xa8,
      0xa9,
      0xaa,
      0xb2,
      0xb3,
      0xb4,
      0xb5,
      0xb6,
      0xb7,
      0xb8,
      0xb9,
      0xba,
      0xc2,
      0xc3,
      0xc4,
      0xc5,
      0xc6,
      0xc7,
      0xc8,
      0xc9,
      0xca,
      0xd2,
      0xd3,
      0xd4,
      0xd5,
      0xd6,
      0xd7,
      0xd8,
      0xd9,
      0xda,
      0xe1,
      0xe2,
      0xe3,
      0xe4,
      0xe5,
      0xe6,
      0xe7,
      0xe8,
      0xe9,
      0xea,
      0xf1,
      0xf2,
      0xf3,
      0xf4,
      0xf5,
      0xf6,
      0xf7,
      0xf8,
      0xf9,
      0xfa,
      // Huffman table DC (chrominance)
      0xff,
      0xc4,
      0x00,
      0x1f,
      0x01,
      0x00,
      0x03,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x01,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x01,
      0x02,
      0x03,
      0x04,
      0x05,
      0x06,
      0x07,
      0x08,
      0x09,
      0x0a,
      0x0b,
      // Huffman table AC (chrominance)
      0xff,
      0xc4,
      0x00,
      0xb5,
      0x11,
      0x00,
      0x02,
      0x01,
      0x02,
      0x04,
      0x04,
      0x03,
      0x04,
      0x07,
      0x05,
      0x04,
      0x04,
      0x00,
      0x01,
      0x02,
      0x77,
      0x00,
      0x01,
      0x02,
      0x03,
      0x11,
      0x04,
      0x05,
      0x21,
      0x31,
      0x06,
      0x12,
      0x41,
      0x51,
      0x07,
      0x61,
      0x71,
      0x13,
      0x22,
      0x32,
      0x81,
      0x08,
      0x14,
      0x42,
      0x91,
      0xa1,
      0xb1,
      0xc1,
      0x09,
      0x23,
      0x33,
      0x52,
      0xf0,
      0x15,
      0x62,
      0x72,
      0xd1,
      0x0a,
      0x16,
      0x24,
      0x34,
      0xe1,
      0x25,
      0xf1,
      0x17,
      0x18,
      0x19,
      0x1a,
      0x26,
      0x27,
      0x28,
      0x29,
      0x2a,
      0x35,
      0x36,
      0x37,
      0x38,
      0x39,
      0x3a,
      0x43,
      0x44,
      0x45,
      0x46,
      0x47,
      0x48,
      0x49,
      0x4a,
      0x53,
      0x54,
      0x55,
      0x56,
      0x57,
      0x58,
      0x59,
      0x5a,
      0x63,
      0x64,
      0x65,
      0x66,
      0x67,
      0x68,
      0x69,
      0x6a,
      0x73,
      0x74,
      0x75,
      0x76,
      0x77,
      0x78,
      0x79,
      0x7a,
      0x82,
      0x83,
      0x84,
      0x85,
      0x86,
      0x87,
      0x88,
      0x89,
      0x8a,
      0x92,
      0x93,
      0x94,
      0x95,
      0x96,
      0x97,
      0x98,
      0x99,
      0x9a,
      0xa2,
      0xa3,
      0xa4,
      0xa5,
      0xa6,
      0xa7,
      0xa8,
      0xa9,
      0xaa,
      0xb2,
      0xb3,
      0xb4,
      0xb5,
      0xb6,
      0xb7,
      0xb8,
      0xb9,
      0xba,
      0xc2,
      0xc3,
      0xc4,
      0xc5,
      0xc6,
      0xc7,
      0xc8,
      0xc9,
      0xca,
      0xd2,
      0xd3,
      0xd4,
      0xd5,
      0xd6,
      0xd7,
      0xd8,
      0xd9,
      0xda,
      0xe2,
      0xe3,
      0xe4,
      0xe5,
      0xe6,
      0xe7,
      0xe8,
      0xe9,
      0xea,
      0xf2,
      0xf3,
      0xf4,
      0xf5,
      0xf6,
      0xf7,
      0xf8,
      0xf9,
      0xfa,
  };

  // Scan header (SOS)
  static constexpr uint8_t SOS[] = {
      0xFF, 0xDA,      // SOS marker
      0x00, 0x0C,      // length
      0x03,            // number of components
      0x01, 0x00,      // component IDs and Huffman tables
      0x02, 0x11,      // component IDs and Huffman tables
      0x03, 0x11,      // component IDs and Huffman tables
      0x00, 0x3F, 0x00 // Ss, Se, Ah/Al
  };

  int add_sof0(int offset) {
    // add the SOF0 marker
    data_[offset++] = 0xFF;
    data_[offset++] = 0xC0;
    // add the length of the marker
    data_[offset++] = 0x00;
    data_[offset++] = 0x11;
    // add the precision
    data_[offset++] = 0x08;
    // add the height
    data_[offset++] = (height_ >> 8) & 0xFF;
    data_[offset++] = height_ & 0xFF;
    // add the width
    data_[offset++] = (width_ >> 8) & 0xFF;
    data_[offset++] = width_ & 0xFF;
    // add the number of components
    data_[offset++] = 0x03;
    // add the Y component
    data_[offset++] = 0x01;
    data_[offset++] = 0x21;
    data_[offset++] = 0x00;
    // add the Cb component
    data_[offset++] = 0x02;
    data_[offset++] = 0x11;
    data_[offset++] = 0x01;
    // add the Cr component
    data_[offset++] = 0x03;
    data_[offset++] = 0x11;
    data_[offset++] = 0x01;
    return offset;
  }

  void serialize() {
    int header_size = 2 + sizeof(JFIF_APP0_DATA) + DQT_HEADER_SIZE + q0_table_.size() +
                      DQT_HEADER_SIZE + q1_table_.size() + sizeof(HUFFMAN_TABLES) + SOF0_SIZE +
                      sizeof(SOS);
    // serialize the jpeg header to the data_ vector
    data_.resize(header_size);
    int offset = 0;

    // add the SOI marker
    data_[offset++] = 0xFF;
    data_[offset++] = 0xD8;

    // add the JFIF APP0 marker
    memcpy(data_.data() + offset, JFIF_APP0_DATA, sizeof(JFIF_APP0_DATA));
    offset += sizeof(JFIF_APP0_DATA);

    // add the DQT marker for luminance
    data_[offset++] = 0xFF;
    data_[offset++] = 0xDB;
    data_[offset++] = 0x00;
    data_[offset++] = 0x43;
    data_[offset++] = 0x00;
    memcpy(data_.data() + offset, q0_table_.data(), q0_table_.size());
    offset += q0_table_.size();

    // add the DQT marker for chrominance
    data_[offset++] = 0xFF;
    data_[offset++] = 0xDB;
    data_[offset++] = 0x00;
    data_[offset++] = 0x43;
    data_[offset++] = 0x01;
    memcpy(data_.data() + offset, q1_table_.data(), q1_table_.size());
    offset += q1_table_.size();

    // add huffman tables
    memcpy(data_.data() + offset, HUFFMAN_TABLES, sizeof(HUFFMAN_TABLES));
    offset += sizeof(HUFFMAN_TABLES);

    // add the SOF0
    offset = add_sof0(offset);

    // add the SOS marker
    memcpy(data_.data() + offset, SOS, sizeof(SOS));
    // offset += sizeof(SOS);
  }

  uint16_t get_marker(size_t offset) const { return (data_[offset] << 8) | data_[offset + 1]; }

  static constexpr uint16_t SOI_MARKER = 0xFFD8;
  static constexpr uint16_t APP0_MARKER = 0xFFE0;
  static constexpr uint16_t DQT_MARKER = 0xFFDB;
  static constexpr uint16_t SOF0_MARKER = 0xFFC0;
  static constexpr uint16_t DHT_MARKER = 0xFFC4;
  static constexpr uint16_t SOS_MARKER = 0xFFDA;
  static constexpr uint16_t EOI_MARKER = 0xFFD9;

  void parse() {
    constexpr size_t debug_data_size = 50;
    // parse the jpeg header from the data_ vector
    int offset = 0;
    // check the SOI marker
    if (get_marker(offset) != SOI_MARKER) {
      std::span<const uint8_t> debug_data(data_.data() + offset,
                                          std::min<size_t>(data_.size(), debug_data_size));
      fmt::print("Invalid SOI marker: {::02x}\n", debug_data);
      return;
    }
    offset += 2;

    // check the JFIF APP0 marker
    if (get_marker(offset) == APP0_MARKER) {
      if (memcmp(data_.data() + offset, JFIF_APP0_DATA, JFIF_APP0_CMP_LEN) != 0) {
        std::span<const uint8_t> debug_data(data_.data() + offset,
                                            std::min<size_t>(data_.size(), debug_data_size));
        fmt::print("Invalid JFIF APP0 marker: {::02x}\n", debug_data);
        return;
      }
      offset += sizeof(JFIF_APP0_DATA);
    }

    size_t num_huffman_tables = 0;
    size_t num_dqt_tables = 0;
    bool found_sos = false;
    while (!found_sos && offset < data_.size()) {
      uint16_t marker = (data_[offset] << 8) | data_[offset + 1];
      offset += 2;
      if (marker == DQT_MARKER) {
        // next two bytes are length, so read them out and then skip ahead
        size_t dqt_length = (data_[offset] << 8) | data_[offset + 1];
        size_t dqt_table_id = data_[offset + 2];
        size_t dqt_table_length = dqt_length - 3; // length includes the marker and table ID
        const char *table_ptr = (const char *)data_.data() + offset + 3;
        auto dqt_table = std::string_view(table_ptr, dqt_table_length);
        switch (dqt_table_id) {
        case 0:
          q0_table_ = dqt_table;
          break;
        case 1:
          q1_table_ = dqt_table;
          break;
        default:
          fmt::print("Unknown DQT table ID: {}\n", dqt_table_id);
          break;
        }
        offset += dqt_length;
        num_dqt_tables++;
      } else if (marker == DHT_MARKER) {
        size_t huffman_tables_length = (data_[offset] << 8) | data_[offset + 1];
        offset += huffman_tables_length; // skip the length and the table
        num_huffman_tables++;
      } else if (marker == SOF0_MARKER) {
        size_t sof0_length = (data_[offset] << 8) | data_[offset + 1];
        [[maybe_unused]] uint8_t precision = data_[offset + 2];
        height_ = (data_[offset + 3] << 8) | data_[offset + 4];
        width_ = (data_[offset + 5] << 8) | data_[offset + 6];
        offset += sof0_length;
      } else if (marker == SOS_MARKER) {
        size_t sos_length = (data_[offset] << 8) | data_[offset + 1];
        offset += sos_length;
        found_sos = true;
      } else {
        fmt::print("Unknown marker: {:04x}\n", marker);
      }
    }

    // fmt::print("Found {} DQT tables\n", num_dqt_tables);
    // fmt::print("Found {} Huffman tables\n", num_huffman_tables);
    // fmt::print("Image size: {}x{} pixels\n", width_, height_);

    if (!found_sos) {
      fmt::print("Did not find SOS marker\n");
      return;
    }

    data_.resize(offset);
  }

  int width_{0};
  int height_{0};
  std::string_view q0_table_;
  std::string_view q1_table_;

  std::vector<uint8_t> data_;
};
} // namespace espp
