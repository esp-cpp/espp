#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace espp {
  /// RtpPacket is a class to parse RTP packet.
  class RtpPacket {
  public:
    /// Construct an empty RtpPacket.
    /// The packet_ vector is empty and the header fields are set to 0.
    RtpPacket() : version_(2), padding_(false), extension_(false), csrc_count_(0),
                  marker_(false), payload_type_(0), sequence_number_(0), timestamp_(0),
                  ssrc_(0), payload_size_(0) {
      // ensure that the packet_ vector is at least RTP_HEADER_SIZE bytes long
      packet_.resize(RTP_HEADER_SIZE);
    }

    /// Construct an RtpPacket with a payload of size payload_size.
    explicit RtpPacket(size_t payload_size) : version_(2), padding_(false), extension_(false), csrc_count_(0),
                                              marker_(false), payload_type_(0), sequence_number_(0), timestamp_(0),
                                              ssrc_(0), payload_size_(payload_size) {
      // ensure that the packet_ vector is at least RTP_HEADER_SIZE + payload_size bytes long
      packet_.resize(RTP_HEADER_SIZE + payload_size);
    }

    /// Construct an RtpPacket from a string_view.
    /// Store the string_view in the packet_ vector and parses the header.
    /// @param data The string_view to parse.
    explicit RtpPacket(std::string_view data) {
      packet_.assign(data.begin(), data.end());
      payload_size_ = packet_.size() - RTP_HEADER_SIZE;
      if (packet_.size() >= RTP_HEADER_SIZE)
        parse_rtp_header();
    }

    ~RtpPacket() {}

    /// Getters for the RTP header fields.
    int get_version() const { return version_; }
    bool get_padding() const { return padding_; }
    bool get_extension() const { return extension_; }
    int get_csrc_count() const { return csrc_count_; }
    bool get_marker() const { return marker_; }
    int get_payload_type() const { return payload_type_; }
    int get_sequence_number() const { return sequence_number_; }
    int get_timestamp() const { return timestamp_; }
    int get_ssrc() const { return ssrc_; }

    /// Setters for the RTP header fields.
    void set_version(int version) { version_ = version; }
    void set_padding(bool padding) { padding_ = padding; }
    void set_extension(bool extension) { extension_ = extension; }
    void set_csrc_count(int csrc_count) { csrc_count_ = csrc_count; }
    void set_marker(bool marker) { marker_ = marker; }
    void set_payload_type(int payload_type) { payload_type_ = payload_type; }
    void set_sequence_number(int sequence_number) { sequence_number_ = sequence_number; }
    void set_timestamp(int timestamp) { timestamp_ = timestamp; }
    void set_ssrc(int ssrc) { ssrc_ = ssrc; }

    /// Serialize the RTP header.
    /// @note This method should be called after modifying the RTP header fields.
    /// @note This method does not serialize the payload. To set the payload, use
    ///       set_payload().
    ///       To get the payload, use get_payload().
    void serialize() {
      serialize_rtp_header();
    }

    /// Get a string_view of the whole packet.
    /// @note The string_view is valid as long as the packet_ vector is not modified.
    /// @note If you manually build the packet_ vector, you should make sure that you
    ///       call serialize() before calling this method.
    /// @return A string_view of the whole packet.
    std::string_view get_data() const {
      return std::string_view(packet_.data(), packet_.size());
    }

    /// Get the size of the RTP header.
    /// @return The size of the RTP header.
    size_t get_rtp_header_size() const {
      return RTP_HEADER_SIZE;
    }

    /// Get a string_view of the RTP header.
    /// @return A string_view of the RTP header.
    std::string_view get_rpt_header() const {
      return std::string_view(packet_.data(), RTP_HEADER_SIZE);
    }

    /// Get a reference to the packet_ vector.
    /// @return A reference to the packet_ vector.
    std::vector<char>& get_packet() {
      return packet_;
    }

    /// Get a string_view of the payload.
    /// @return A string_view of the payload.
    std::string_view get_payload() const {
      return std::string_view(packet_.data() + RTP_HEADER_SIZE, payload_size_);
    }

    /// Set the payload.
    /// @param payload The payload to set.
    void set_payload(std::string_view payload) {
      packet_.resize(RTP_HEADER_SIZE + payload.size());
      std::copy(payload.begin(), payload.end(), packet_.begin() + RTP_HEADER_SIZE);
      payload_size_ = payload.size();
    }

  protected:
    static constexpr int RTP_HEADER_SIZE = 12;

    void parse_rtp_header() {
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

    void serialize_rtp_header() {
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

    std::vector<char> packet_;
    int version_;
    bool padding_;
    bool extension_;
    int csrc_count_;
    bool marker_;
    int payload_type_;
    int sequence_number_;
    int timestamp_;
    int ssrc_;
    int payload_size_;
  };
}  // namespace espp
