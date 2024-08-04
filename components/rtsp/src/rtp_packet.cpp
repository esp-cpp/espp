#include "rtp_packet.hpp"

using namespace espp;

RtpPacket::RtpPacket() {
  // ensure that the packet_ vector is at least RTP_HEADER_SIZE bytes long
  packet_.resize(RTP_HEADER_SIZE);
}

RtpPacket::RtpPacket(size_t payload_size)
    : payload_size_(payload_size) {
  // ensure that the packet_ vector is at least RTP_HEADER_SIZE + payload_size bytes long
  packet_.resize(RTP_HEADER_SIZE + payload_size);
}

RtpPacket::RtpPacket(std::string_view data) {
  packet_.assign(data.begin(), data.end());
  payload_size_ = packet_.size() - RTP_HEADER_SIZE;
  if (packet_.size() >= RTP_HEADER_SIZE)
    parse_rtp_header();
}

RtpPacket::~RtpPacket() {}

/// Getters for the RTP header fields.
int RtpPacket::get_version() const { return version_; }
bool RtpPacket::get_padding() const { return padding_; }
bool RtpPacket::get_extension() const { return extension_; }
int RtpPacket::get_csrc_count() const { return csrc_count_; }
bool RtpPacket::get_marker() const { return marker_; }
int RtpPacket::get_payload_type() const { return payload_type_; }
int RtpPacket::get_sequence_number() const { return sequence_number_; }
int RtpPacket::get_timestamp() const { return timestamp_; }
int RtpPacket::get_ssrc() const { return ssrc_; }

/// Setters for the RTP header fields.
void RtpPacket::set_version(int version) { version_ = version; }
void RtpPacket::set_padding(bool padding) { padding_ = padding; }
void RtpPacket::set_extension(bool extension) { extension_ = extension; }
void RtpPacket::set_csrc_count(int csrc_count) { csrc_count_ = csrc_count; }
void RtpPacket::set_marker(bool marker) { marker_ = marker; }
void RtpPacket::set_payload_type(int payload_type) { payload_type_ = payload_type; }
void RtpPacket::set_sequence_number(int sequence_number) { sequence_number_ = sequence_number; }
void RtpPacket::set_timestamp(int timestamp) { timestamp_ = timestamp; }
void RtpPacket::set_ssrc(int ssrc) { ssrc_ = ssrc; }

void RtpPacket::serialize() { serialize_rtp_header(); }

std::string_view RtpPacket::get_data() const {
  return std::string_view((char *)packet_.data(), packet_.size());
}

size_t RtpPacket::get_rtp_header_size() const { return RTP_HEADER_SIZE; }

std::string_view RtpPacket::get_rpt_header() const {
  return std::string_view((char *)packet_.data(), RTP_HEADER_SIZE);
}

std::vector<uint8_t> &RtpPacket::get_packet() { return packet_; }

std::string_view RtpPacket::get_payload() const {
  return std::string_view((char *)packet_.data() + RTP_HEADER_SIZE, payload_size_);
}

void RtpPacket::set_payload(std::string_view payload) {
  packet_.resize(RTP_HEADER_SIZE + payload.size());
  std::copy(payload.begin(), payload.end(), packet_.begin() + RTP_HEADER_SIZE);
  payload_size_ = payload.size();
}

void RtpPacket::parse_rtp_header() {
  version_ = (packet_[0] & 0xC0) >> 6;
  padding_ = (packet_[0] & 0x20) >> 5;
  extension_ = (packet_[0] & 0x10) >> 4;
  csrc_count_ = packet_[0] & 0x0F;
  marker_ = (packet_[1] & 0x80) >> 7;
  payload_type_ = packet_[1] & 0x7F;
  sequence_number_ = (packet_[2] << 8) | packet_[3];
  timestamp_ = (packet_[4] << 24) | (packet_[5] << 16) | (packet_[6] << 8) | packet_[7];
  ssrc_ = (packet_[8] << 24) | (packet_[9] << 16) | (packet_[10] << 8) | packet_[11];
}

void RtpPacket::serialize_rtp_header() {
  packet_[0] = (version_ << 6) | (padding_ << 5) | (extension_ << 4) | csrc_count_;
  packet_[1] = (marker_ << 7) | payload_type_;
  packet_[2] = sequence_number_ >> 8;
  packet_[3] = sequence_number_ & 0xFF;
  packet_[4] = timestamp_ >> 24;
  packet_[5] = (timestamp_ >> 16) & 0xFF;
  packet_[6] = (timestamp_ >> 8) & 0xFF;
  packet_[7] = timestamp_ & 0xFF;
  packet_[8] = ssrc_ >> 24;
  packet_[9] = (ssrc_ >> 16) & 0xFF;
  packet_[10] = (ssrc_ >> 8) & 0xFF;
  packet_[11] = ssrc_ & 0xFF;
}
