#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace espp {
/// RtpPacket is a class to parse RTP packet.
/// It can be used to parse and serialize RTP packets.
/// The RTP header fields are stored in the class and can be modified.
/// The payload is stored in the packet_ vector and can be modified.
class RtpPacket {
public:
  /// Construct an empty RtpPacket.
  /// The packet_ vector is empty and the header fields are set to 0.
  RtpPacket();

  /// Construct an RtpPacket with a payload of size payload_size.
  /// The packet_ vector is resized to RTP_HEADER_SIZE + payload_size.
  explicit RtpPacket(size_t payload_size);

  /// Construct an RtpPacket from a span of bytes.
  /// Stores the bytes in the packet_ vector and parses the header.
  /// @param data The span of bytes to parse.
  explicit RtpPacket(std::span<const uint8_t> data);

  /// Destructor.
  ~RtpPacket();

  // -----------------------------------------------------------------
  // Getters for the RTP header fields.
  // -----------------------------------------------------------------

  /// Get the RTP version.
  /// @return The RTP version.
  int get_version() const;

  /// Get the padding flag.
  /// @return The padding flag.
  bool get_padding() const;

  /// Get the extension flag.
  /// @return The extension flag.
  bool get_extension() const;

  /// Get the CSRC count.
  /// @return The CSRC count.
  int get_csrc_count() const;

  /// Get the marker flag.
  /// @return The marker flag.
  bool get_marker() const;

  /// Get the payload type.
  /// @return The payload type.
  int get_payload_type() const;

  /// Get the sequence number.
  /// @return The sequence number.
  int get_sequence_number() const;

  /// Get the timestamp.
  /// @return The timestamp.
  int get_timestamp() const;

  /// Get the SSRC.
  /// @return The SSRC.
  int get_ssrc() const;

  // -----------------------------------------------------------------
  // Setters for the RTP header fields.
  // -----------------------------------------------------------------

  /// Set the RTP version.
  /// @param version The RTP version to set.
  void set_version(int version);

  /// Set the padding flag.
  /// @param padding The padding flag to set.
  void set_padding(bool padding);

  /// Set the extension flag.
  /// @param extension The extension flag to set.
  void set_extension(bool extension);

  /// Set the CSRC count.
  /// @param csrc_count The CSRC count to set.
  void set_csrc_count(int csrc_count);

  /// Set the marker flag.
  /// @param marker The marker flag to set.
  void set_marker(bool marker);

  /// Set the payload type.
  /// @param payload_type The payload type to set.
  void set_payload_type(int payload_type);

  /// Set the sequence number.
  /// @param sequence_number The sequence number to set.
  void set_sequence_number(int sequence_number);

  /// Set the timestamp.
  /// @param timestamp The timestamp to set.
  void set_timestamp(int timestamp);

  /// Set the SSRC.
  /// @param ssrc The SSRC to set.
  void set_ssrc(int ssrc);

  // -----------------------------------------------------------------
  // Utility methods.
  // -----------------------------------------------------------------

  /// Serialize the RTP header.
  /// @note This method should be called after modifying the RTP header fields.
  /// @note This method does not serialize the payload. To set the payload, use
  ///       set_payload().
  ///       To get the payload, use get_payload().
  void serialize();

  /// Get a span view of the whole packet.
  /// @note The span is valid as long as the packet_ vector is not modified.
  /// @note If you manually build the packet_ vector, you should make sure that you
  ///       call serialize() before calling this method.
  /// @return A span of the whole packet.
  std::span<const uint8_t> get_data() const;

  /// Get the size of the RTP header.
  /// @return The size of the RTP header.
  size_t get_rtp_header_size() const;

  /// Get a span of bytes of the RTP header.
  /// @return A span of bytes of the RTP header.
  std::span<const uint8_t> get_rtp_header() const;

  /// Get a reference to the packet_ vector.
  /// @return A reference to the packet_ vector.
  std::vector<uint8_t> &get_packet();

  /// Get a span of bytes of the payload.
  /// @return A span of bytes of the payload.
  std::span<const uint8_t> get_payload() const;

  /// Set the payload.
  /// @param payload The payload to set.
  void set_payload(std::span<const uint8_t> payload);

protected:
  static constexpr int RTP_HEADER_SIZE = 12;

  void parse_rtp_header();
  void serialize_rtp_header();

  std::vector<uint8_t> packet_;
  int version_{2};
  bool padding_{false};
  bool extension_{false};
  int csrc_count_{0};
  bool marker_{false};
  int payload_type_{0};
  int sequence_number_{0};
  int timestamp_{0};
  int ssrc_{0};
  int payload_size_{0};
};
} // namespace espp
