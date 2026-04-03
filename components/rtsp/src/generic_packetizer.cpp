#include "generic_packetizer.hpp"

#include <algorithm>

#include "format.hpp"

using namespace espp;

GenericPacketizer::GenericPacketizer(const Config &config)
    : RtpPacketizer({.max_payload_size = config.max_payload_size, .log_level = config.log_level},
                    "GenericPacketizer")
    , payload_type_(config.payload_type)
    , clock_rate_(config.clock_rate)
    , encoding_name_(config.encoding_name)
    , channels_(config.channels)
    , fmtp_(config.fmtp)
    , media_type_(config.media_type) {}

std::vector<RtpPayloadChunk> GenericPacketizer::packetize(std::span<const uint8_t> frame_data) {
  std::vector<RtpPayloadChunk> chunks;
  if (frame_data.empty()) {
    return chunks;
  }

  size_t offset = 0;
  size_t remaining = frame_data.size();

  while (remaining > 0) {
    size_t chunk_size = std::min(remaining, max_payload_size_);
    bool is_last = (chunk_size == remaining);

    RtpPayloadChunk chunk;
    chunk.data.assign(frame_data.data() + offset, frame_data.data() + offset + chunk_size);
    chunk.marker = is_last;
    chunks.push_back(std::move(chunk));

    offset += chunk_size;
    remaining -= chunk_size;
  }

  logger_.debug("Packetized {} bytes into {} chunk(s)", frame_data.size(), chunks.size());
  return chunks;
}

int GenericPacketizer::get_payload_type() const { return payload_type_; }

uint32_t GenericPacketizer::get_clock_rate() const { return clock_rate_; }

std::string GenericPacketizer::get_sdp_media_attributes() const {
  std::string attrs;

  if (media_type_ == MediaType::VIDEO || channels_ <= 1) {
    attrs += fmt::format("a=rtpmap:{} {}/{}\r\n", payload_type_, encoding_name_, clock_rate_);
  } else {
    attrs += fmt::format("a=rtpmap:{} {}/{}/{}\r\n", payload_type_, encoding_name_, clock_rate_,
                         channels_);
  }

  if (!fmtp_.empty()) {
    attrs += fmt::format("a=fmtp:{} {}\r\n", payload_type_, fmtp_);
  }

  return attrs;
}

std::string GenericPacketizer::get_sdp_media_line() const {
  const char *media = (media_type_ == MediaType::AUDIO) ? "audio" : "video";
  return fmt::format("m={} 0 RTP/AVP {}", media, payload_type_);
}
