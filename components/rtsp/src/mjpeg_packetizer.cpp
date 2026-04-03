#include "mjpeg_packetizer.hpp"

#include <algorithm>
#include <cstring>

namespace espp {

static constexpr size_t MJPEG_HEADER_SIZE = 8;
static constexpr size_t QUANT_HEADER_SIZE = 4;
static constexpr size_t NUM_Q_TABLES = 2;
static constexpr size_t Q_TABLE_SIZE = 64;
static constexpr size_t FIRST_PACKET_OVERHEAD =
    MJPEG_HEADER_SIZE + QUANT_HEADER_SIZE + NUM_Q_TABLES * Q_TABLE_SIZE;
static constexpr size_t OTHER_PACKET_OVERHEAD = MJPEG_HEADER_SIZE;

std::vector<RtpPayloadChunk> MjpegPacketizer::packetize(std::span<const uint8_t> frame_data) {
  // Parse the JPEG header to extract width, height, and quantization tables
  JpegHeader header(frame_data);
  int width = header.get_width();
  int height = header.get_height();
  auto q0 = header.get_quantization_table(0);
  auto q1 = header.get_quantization_table(1);

  // Get scan data (everything after the JPEG header)
  size_t header_size = header.size();
  auto scan_data = frame_data.subspan(header_size);
  size_t data_size = scan_data.size();

  logger_.debug("Packetizing JPEG frame: {}x{}, header={} bytes, scan={} bytes", width, height,
                header_size, data_size);

  // Calculate how much scan data fits in each packet
  size_t first_data_capacity =
      max_payload_size_ > FIRST_PACKET_OVERHEAD ? max_payload_size_ - FIRST_PACKET_OVERHEAD : 0;
  size_t other_data_capacity =
      max_payload_size_ > OTHER_PACKET_OVERHEAD ? max_payload_size_ - OTHER_PACKET_OVERHEAD : 0;

  size_t first_chunk_size = std::min(data_size, first_data_capacity);
  size_t remaining = data_size - first_chunk_size;
  size_t num_additional = (other_data_capacity > 0 && remaining > 0)
                              ? (remaining + other_data_capacity - 1) / other_data_capacity
                              : 0;
  size_t total_packets = 1 + num_additional;

  std::vector<RtpPayloadChunk> chunks;
  chunks.reserve(total_packets);

  size_t data_offset = 0;

  for (size_t i = 0; i < total_packets; i++) {
    RtpPayloadChunk chunk;

    if (i == 0) {
      // First packet: MJPEG header + Q table header + Q tables + scan data
      chunk.data.resize(FIRST_PACKET_OVERHEAD + first_chunk_size);
      size_t pos = 0;

      // MJPEG header (8 bytes per RFC 2435)
      chunk.data[pos++] = 0;                                // type_specific
      chunk.data[pos++] = 0;                                // fragment offset [23:16]
      chunk.data[pos++] = 0;                                // fragment offset [15:8]
      chunk.data[pos++] = 0;                                // fragment offset [7:0]
      chunk.data[pos++] = 0;                                // fragment type
      chunk.data[pos++] = 128;                              // q (128 = includes Q tables)
      chunk.data[pos++] = static_cast<uint8_t>(width / 8);  // width / 8
      chunk.data[pos++] = static_cast<uint8_t>(height / 8); // height / 8

      // Quantization table header (4 bytes)
      chunk.data[pos++] = 0;                                                 // MBZ
      chunk.data[pos++] = 0;                                                 // precision
      chunk.data[pos++] = 0;                                                 // length (high byte)
      chunk.data[pos++] = static_cast<uint8_t>(NUM_Q_TABLES * Q_TABLE_SIZE); // length (low byte)

      // Quantization tables (2 x 64 bytes)
      std::memcpy(chunk.data.data() + pos, q0.data(), Q_TABLE_SIZE);
      pos += Q_TABLE_SIZE;
      std::memcpy(chunk.data.data() + pos, q1.data(), Q_TABLE_SIZE);
      pos += Q_TABLE_SIZE;

      // Scan data
      std::memcpy(chunk.data.data() + pos, scan_data.data() + data_offset, first_chunk_size);
      data_offset += first_chunk_size;
    } else {
      // Subsequent packets: MJPEG header + scan data (no Q tables)
      size_t chunk_data_size = std::min(remaining, other_data_capacity);
      remaining -= chunk_data_size;

      chunk.data.resize(OTHER_PACKET_OVERHEAD + chunk_data_size);
      size_t pos = 0;

      // MJPEG header (8 bytes)
      chunk.data[pos++] = 0;                                                // type_specific
      chunk.data[pos++] = static_cast<uint8_t>((data_offset >> 16) & 0xff); // offset [23:16]
      chunk.data[pos++] = static_cast<uint8_t>((data_offset >> 8) & 0xff);  // offset [15:8]
      chunk.data[pos++] = static_cast<uint8_t>(data_offset & 0xff);         // offset [7:0]
      chunk.data[pos++] = 0;                                                // fragment type
      chunk.data[pos++] = 96;                                               // q (no Q tables)
      chunk.data[pos++] = static_cast<uint8_t>(width / 8);                  // width / 8
      chunk.data[pos++] = static_cast<uint8_t>(height / 8);                 // height / 8

      // Scan data
      std::memcpy(chunk.data.data() + pos, scan_data.data() + data_offset, chunk_data_size);
      data_offset += chunk_data_size;
    }

    // Last packet gets the marker bit
    if (i == total_packets - 1) {
      chunk.marker = true;
    }

    chunks.push_back(std::move(chunk));
  }

  logger_.debug("Produced {} payload chunks", chunks.size());
  return chunks;
}

int MjpegPacketizer::get_payload_type() const { return 26; }

uint32_t MjpegPacketizer::get_clock_rate() const { return 90000; }

std::string MjpegPacketizer::get_sdp_media_line() const { return "m=video 0 RTP/AVP 26"; }

std::string MjpegPacketizer::get_sdp_media_attributes() const {
  return "a=rtpmap:26 JPEG/90000\r\n";
}

} // namespace espp
