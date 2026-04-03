#include "h264_packetizer.hpp"

#include <algorithm>
#include <array>

namespace {

/// Base64 encoding table.
static constexpr char kBase64Table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encode binary data to base64 string.
/// @param data The binary data to encode.
/// @return The base64-encoded string.
std::string base64_encode(std::span<const uint8_t> data) {
  std::string result;
  result.reserve(((data.size() + 2) / 3) * 4);

  size_t i = 0;
  while (i + 2 < data.size()) {
    uint32_t triple = (static_cast<uint32_t>(data[i]) << 16) |
                      (static_cast<uint32_t>(data[i + 1]) << 8) |
                      static_cast<uint32_t>(data[i + 2]);
    result.push_back(kBase64Table[(triple >> 18) & 0x3F]);
    result.push_back(kBase64Table[(triple >> 12) & 0x3F]);
    result.push_back(kBase64Table[(triple >> 6) & 0x3F]);
    result.push_back(kBase64Table[triple & 0x3F]);
    i += 3;
  }

  if (i + 1 == data.size()) {
    uint32_t val = static_cast<uint32_t>(data[i]) << 16;
    result.push_back(kBase64Table[(val >> 18) & 0x3F]);
    result.push_back(kBase64Table[(val >> 12) & 0x3F]);
    result.push_back('=');
    result.push_back('=');
  } else if (i + 2 == data.size()) {
    uint32_t val =
        (static_cast<uint32_t>(data[i]) << 16) | (static_cast<uint32_t>(data[i + 1]) << 8);
    result.push_back(kBase64Table[(val >> 18) & 0x3F]);
    result.push_back(kBase64Table[(val >> 12) & 0x3F]);
    result.push_back(kBase64Table[(val >> 6) & 0x3F]);
    result.push_back('=');
  }

  return result;
}

} // namespace

using namespace espp;

H264Packetizer::H264Packetizer(const Config &config)
    : RtpPacketizer({.max_payload_size = config.max_payload_size, .log_level = config.log_level},
                    "H264Packetizer")
    , payload_type_(config.payload_type)
    , profile_level_id_(config.profile_level_id)
    , packetization_mode_(config.packetization_mode)
    , sps_(config.sps)
    , pps_(config.pps) {}

std::vector<RtpPayloadChunk> H264Packetizer::packetize(std::span<const uint8_t> frame_data) {
  auto nals = parse_annex_b(frame_data);
  if (nals.empty()) {
    logger_.warn("No NAL units found in frame data ({} bytes)", frame_data.size());
    return {};
  }

  std::vector<RtpPayloadChunk> chunks;
  for (size_t i = 0; i < nals.size(); ++i) {
    bool is_last = (i == nals.size() - 1);
    auto nal_chunks = packetize_nal(nals[i], is_last);
    chunks.insert(chunks.end(), std::make_move_iterator(nal_chunks.begin()),
                  std::make_move_iterator(nal_chunks.end()));
  }

  logger_.debug("Packetized {} NAL unit(s) into {} chunk(s)", nals.size(), chunks.size());
  return chunks;
}

std::vector<RtpPayloadChunk> H264Packetizer::packetize_nal(std::span<const uint8_t> nal_data,
                                                           bool is_last_nal) {
  std::vector<RtpPayloadChunk> chunks;

  if (nal_data.empty()) {
    return chunks;
  }

  if (nal_data.size() <= max_payload_size_) {
    // Single NAL unit mode — payload is the raw NAL bytes.
    RtpPayloadChunk chunk;
    chunk.data.assign(nal_data.begin(), nal_data.end());
    chunk.marker = is_last_nal;
    chunks.push_back(std::move(chunk));
    logger_.debug("Single NAL: type={}, size={}", nal_data[0] & 0x1F, nal_data.size());
  } else {
    // FU-A fragmentation (only if packetization_mode >= 1).
    if (packetization_mode_ < 1) {
      logger_.warn("NAL unit ({} bytes) exceeds max_payload_size ({}) but "
                   "packetization_mode is 0; dropping",
                   nal_data.size(), max_payload_size_);
      return chunks;
    }

    uint8_t nal_header = nal_data[0];
    uint8_t nri = nal_header & 0x60;      // NRI bits (bits 5-6)
    uint8_t nal_type = nal_header & 0x1F; // NAL unit type (bits 0-4)
    uint8_t fu_indicator = nri | 28;      // FU-A type = 28

    // Fragment the NAL body (everything after the NAL header byte).
    // Each FU-A packet has: FU indicator (1) + FU header (1) + fragment data.
    size_t max_fragment_size = max_payload_size_ - 2; // 2 bytes for FU indicator + FU header
    const uint8_t *body = nal_data.data() + 1;
    size_t body_size = nal_data.size() - 1;
    size_t offset = 0;

    while (offset < body_size) {
      size_t fragment_size = std::min(max_fragment_size, body_size - offset);
      bool is_start = (offset == 0);
      bool is_end = (offset + fragment_size >= body_size);

      uint8_t fu_header = nal_type;
      if (is_start) {
        fu_header |= 0x80; // S bit
      }
      if (is_end) {
        fu_header |= 0x40; // E bit
      }

      RtpPayloadChunk chunk;
      chunk.data.reserve(2 + fragment_size);
      chunk.data.push_back(fu_indicator);
      chunk.data.push_back(fu_header);
      chunk.data.insert(chunk.data.end(), body + offset, body + offset + fragment_size);
      chunk.marker = is_last_nal && is_end;
      chunks.push_back(std::move(chunk));

      offset += fragment_size;
    }

    logger_.debug("FU-A: type={}, size={}, fragments={}", nal_type, nal_data.size(), chunks.size());
  }

  return chunks;
}

void H264Packetizer::set_sps_pps(std::span<const uint8_t> sps, std::span<const uint8_t> pps) {
  sps_.assign(sps.begin(), sps.end());
  pps_.assign(pps.begin(), pps.end());
}

int H264Packetizer::get_payload_type() const { return payload_type_; }

uint32_t H264Packetizer::get_clock_rate() const { return 90000; }

std::string H264Packetizer::get_sdp_media_attributes() const {
  // a=rtpmap:{pt} H264/90000
  std::string attrs = "a=rtpmap:" + std::to_string(payload_type_) + " H264/90000";

  // a=fmtp:{pt} packetization-mode={mode};profile-level-id={profile}
  attrs += "\r\na=fmtp:" + std::to_string(payload_type_) +
           " packetization-mode=" + std::to_string(packetization_mode_);

  if (!profile_level_id_.empty()) {
    attrs += ";profile-level-id=" + profile_level_id_;
  }

  if (!sps_.empty() && !pps_.empty()) {
    std::string sps_b64 = base64_encode(std::span<const uint8_t>(sps_));
    std::string pps_b64 = base64_encode(std::span<const uint8_t>(pps_));
    attrs += ";sprop-parameter-sets=" + sps_b64 + "," + pps_b64;
  }

  return attrs;
}

std::string H264Packetizer::get_sdp_media_line() const {
  return "m=video 0 RTP/AVP " + std::to_string(payload_type_);
}

std::vector<std::span<const uint8_t>> H264Packetizer::parse_annex_b(std::span<const uint8_t> data) {
  std::vector<std::span<const uint8_t>> nals;

  if (data.size() < 4) {
    return nals;
  }

  // Find all start code positions. Start codes are 0x000001 (3-byte) or
  // 0x00000001 (4-byte).
  std::vector<size_t> nal_starts;
  size_t i = 0;
  while (i + 2 < data.size()) {
    if (data[i] == 0x00 && data[i + 1] == 0x00) {
      if (i + 3 < data.size() && data[i + 2] == 0x00 && data[i + 3] == 0x01) {
        // 4-byte start code
        nal_starts.push_back(i + 4);
        i += 4;
        continue;
      } else if (data[i + 2] == 0x01) {
        // 3-byte start code
        nal_starts.push_back(i + 3);
        i += 3;
        continue;
      }
    }
    ++i;
  }

  // Extract NAL units between start codes.
  for (size_t n = 0; n < nal_starts.size(); ++n) {
    size_t start = nal_starts[n];
    size_t end;
    if (n + 1 < nal_starts.size()) {
      // Find the beginning of the next start code (rewind past the 0x00 bytes).
      end = nal_starts[n + 1];
      // The start code for the next NAL begins with 0x00 0x00 ..., so rewind.
      while (end > start && data[end - 1] == 0x00) {
        // We need to be careful: the 0x00 bytes before the next start code
        // belong to the start code, not the NAL. But we stored nal_starts as
        // the position AFTER the start code. So the start code for nal n+1
        // starts at some position before nal_starts[n+1]. We need to find
        // where the 0x00 0x00 [0x00] 0x01 pattern begins.
        break;
      }
      // Recompute: the start code at nal_starts[n+1] was either 3-byte or 4-byte.
      // We need to find where the 0x00 bytes begin before nal_starts[n+1].
      size_t sc_start = nal_starts[n + 1];
      // sc_start points right after the start code. Determine start code length.
      if (sc_start >= 4 && data[sc_start - 4] == 0x00 && data[sc_start - 3] == 0x00 &&
          data[sc_start - 2] == 0x00 && data[sc_start - 1] == 0x01) {
        end = sc_start - 4;
      } else if (sc_start >= 3 && data[sc_start - 3] == 0x00 && data[sc_start - 2] == 0x00 &&
                 data[sc_start - 1] == 0x01) {
        end = sc_start - 3;
      } else {
        end = sc_start;
      }
    } else {
      end = data.size();
    }

    if (end > start) {
      nals.push_back(data.subspan(start, end - start));
    }
  }

  return nals;
}
