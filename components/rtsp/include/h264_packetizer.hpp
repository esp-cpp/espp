#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "rtp_packetizer.hpp"

namespace espp {

/// @brief RTP packetizer for H.264 video per RFC 6184.
///
/// Accepts H.264 access units in Annex B byte-stream format (NAL units
/// separated by 0x00000001 or 0x000001 start codes) and produces a sequence
/// of RTP payload chunks suitable for transmission.
///
/// Supports two NAL-unit packetization strategies:
///   - **Single NAL unit mode** — NAL fits within max_payload_size.
///   - **FU-A fragmentation** — NAL exceeds max_payload_size (packetization_mode >= 1).
///
/// @note This class does not manage RTP headers (sequence numbers, timestamps,
///       SSRC). The caller wraps each returned chunk into an RtpPacket.
///
/// \section h264_packetizer_ex1 Example
/// \snippet h264_packetizer_example.cpp h264_packetizer example
class H264Packetizer : public RtpPacketizer {
public:
  /// Configuration for the H264Packetizer.
  struct Config {
    size_t max_payload_size{1400}; ///< Maximum payload bytes per RTP packet
    int payload_type{96};          ///< Dynamic RTP payload type (typically 96–127).
    std::string profile_level_id;  ///< H.264 profile-level-id hex string, e.g. "42C01E".
    int packetization_mode{1};     ///< 0 = single NAL only, 1 = non-interleaved (FU-A allowed).
    std::vector<uint8_t> sps;      ///< Sequence Parameter Set raw bytes (without start code).
    std::vector<uint8_t> pps;      ///< Picture Parameter Set raw bytes (without start code).
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity level
  };

  /// Construct an H264Packetizer.
  /// @param config The configuration for the packetizer.
  explicit H264Packetizer(const Config &config);

  /// Destructor.
  ~H264Packetizer() override = default;

  /// Packetize a complete H.264 access unit (Annex B format).
  ///
  /// The input may contain multiple NAL units separated by 3-byte or 4-byte
  /// start codes. Each NAL is individually packetized (single NAL or FU-A).
  /// The marker bit is set on the last chunk of the last NAL unit in the
  /// access unit.
  ///
  /// @param frame_data Raw Annex B byte-stream of one access unit.
  /// @return Vector of RTP payload chunks ready for transmission.
  std::vector<RtpPayloadChunk> packetize(std::span<const uint8_t> frame_data) override;

  /// Packetize a single pre-parsed NAL unit (no start code prefix).
  ///
  /// @param nal_data The raw NAL unit bytes (including NAL header byte).
  /// @param is_last_nal If true, the marker bit is set on the last chunk.
  /// @return Vector of RTP payload chunks for this NAL.
  std::vector<RtpPayloadChunk> packetize_nal(std::span<const uint8_t> nal_data,
                                             bool is_last_nal = true);

  /// Update the SPS and PPS used for SDP generation.
  /// @param sps Sequence Parameter Set raw bytes.
  /// @param pps Picture Parameter Set raw bytes.
  void set_sps_pps(std::span<const uint8_t> sps, std::span<const uint8_t> pps);

  /// Get the RTP payload type.
  /// @return The dynamic payload type configured for H.264.
  int get_payload_type() const override;

  /// Get the RTP clock rate for H.264 video.
  /// @return 90000 (fixed for H.264).
  uint32_t get_clock_rate() const override;

  /// Get the SDP attribute lines for H.264.
  /// @return SDP a= lines (rtpmap and fmtp) without trailing CRLF.
  std::string get_sdp_media_attributes() const override;

  /// Get the SDP m= media line for H.264.
  /// @return SDP m= line without trailing CRLF.
  std::string get_sdp_media_line() const override;

protected:
  /// Parse Annex B byte-stream into individual NAL units.
  /// @param data The Annex B data to parse.
  /// @return Vector of spans, each pointing to a NAL unit (without start code).
  static std::vector<std::span<const uint8_t>> parse_annex_b(std::span<const uint8_t> data);

  int payload_type_;
  std::string profile_level_id_;
  int packetization_mode_;
  std::vector<uint8_t> sps_;
  std::vector<uint8_t> pps_;
};

} // namespace espp
