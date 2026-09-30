#pragma once

// Wire protocol of espp::SystemService: device identity / status plus reboot
// control over the espp stream_frame codec, routed by an espp::Dispatcher on
// module 7 by default (`espp.system` v1 through discovery).
//
// This header is deliberately host-buildable (stream_frame.hpp + the standard
// library only) so the codec is unit-tested on the host
// (components/system/test/system_host_test.cpp) and so host tools can reuse
// it. All multi-byte fields are little-endian.
//
// Requests (host -> device, reply flag clear):
//   0x01 GET_INFO              (no payload)
//   0x02 REBOOT                [delay_ms u16]
//   0x03 REBOOT_TO_BOOTLOADER  [delay_ms u16]
// Replies (device -> host, high bit set = frame reply flag):
//   0x81 INFO   a list of tagged records [tag u8][len u8][value...]; a host
//               skips tags it does not know, so fields can be added without a
//               version bump (see InfoTag for the values).
//   0x83 OK     [request_type u8]
//   0x84 ERROR  [request_type u8][code u32][utf8 message]
// A reboot request is acknowledged with OK first; the device restarts after
// the requested delay (clamped to at least Config::min_restart_delay).

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "stream_frame.hpp"

namespace espp::detail::system_protocol {

/// Default dispatcher module id (a routing key only; see SystemService::Config::module).
inline constexpr uint8_t kModule = 7;
/// Stable protocol identifier + version advertised through discovery.
inline constexpr const char *kProtocol = "espp.system";
inline constexpr uint16_t kProtocolVersion = 1;

/// Frame `type` values within the system module.
enum class Type : uint8_t {
  // host -> device
  GetInfo = 0x01,
  Reboot = 0x02,
  RebootToBootloader = 0x03,
  // device -> host (high bit set)
  Info = 0x81,
  Ok = 0x83,
  Error = 0x84,
};

/// Tags of the INFO records. Values are little-endian; `str` is raw UTF-8
/// (the record's len is the string length).
enum class InfoTag : uint8_t {
  ChipModel = 1,         ///< str, e.g. "ESP32-S3"
  ChipRevision = 2,      ///< u16 (MXX: major * 100 + minor)
  Cores = 3,             ///< u8
  ChipFeatures = 4,      ///< u32 (CHIP_FEATURE_* bitmask)
  IdfVersion = 5,        ///< str
  ProjectName = 6,       ///< str
  AppVersion = 7,        ///< str
  BuildDate = 8,         ///< str
  BuildTime = 9,         ///< str
  ElfSha256 = 10,        ///< 32 raw bytes
  RunningPartition = 11, ///< str (partition label)
  BootPartition = 12,    ///< str (partition label)
  OtaState = 13,         ///< u8 (esp_ota_img_states_t; 0xFF = undefined / not an OTA partition)
  ResetReason = 14,      ///< u8 (esp_reset_reason_t)
  UptimeMs = 15,         ///< u64
  Mac = 16,              ///< 6 raw bytes (base MAC)
  FlashSize = 17,        ///< u32 bytes
  PsramSize = 18,        ///< u32 bytes (0 = none)
  CpuMhz = 19,           ///< u32
  FreeHeap = 20,         ///< u32 bytes
  MinFreeHeap = 21,      ///< u32 bytes
  Capabilities = 22,     ///< u32 (kCapReboot | kCapBootloader)
};

/// Capabilities bits (InfoTag::Capabilities).
inline constexpr uint32_t kCapReboot = 0x01;     ///< REBOOT is allowed by the service.
inline constexpr uint32_t kCapBootloader = 0x02; ///< REBOOT_TO_BOOTLOADER is allowed AND supported.

/// Whether a type value is a device->host reply.
inline constexpr bool is_reply(Type type) { return (static_cast<uint8_t>(type) & 0x80) != 0; }

/// Build an encoded frame for a system message (device->host types map to the
/// frame reply flag).
inline std::vector<uint8_t> build_frame(Type type, std::span<const uint8_t> payload = {},
                                        uint8_t module = kModule) {
  return espp::stream_frame::build_frame(is_reply(type), module, static_cast<uint8_t>(type),
                                         payload);
}

// ---- INFO record encoding ------------------------------------------------------

/// Builds an INFO payload one tagged record at a time. Values longer than 255
/// bytes are truncated (strings) -- every value defined today is far shorter.
class InfoBuilder {
public:
  InfoBuilder &str(InfoTag tag, std::string_view s) {
    const size_t n = s.size() > 255 ? 255 : s.size();
    header(tag, n);
    out_.insert(out_.end(), s.begin(), s.begin() + static_cast<std::ptrdiff_t>(n));
    return *this;
  }
  InfoBuilder &u8(InfoTag tag, uint8_t v) {
    header(tag, 1);
    out_.push_back(v);
    return *this;
  }
  InfoBuilder &u16(InfoTag tag, uint16_t v) {
    header(tag, 2);
    espp::stream_frame::put_u16(out_, v);
    return *this;
  }
  InfoBuilder &u32(InfoTag tag, uint32_t v) {
    header(tag, 4);
    espp::stream_frame::put_u32(out_, v);
    return *this;
  }
  InfoBuilder &u64(InfoTag tag, uint64_t v) {
    header(tag, 8);
    espp::stream_frame::put_u32(out_, static_cast<uint32_t>(v));
    espp::stream_frame::put_u32(out_, static_cast<uint32_t>(v >> 32));
    return *this;
  }
  InfoBuilder &bytes(InfoTag tag, std::span<const uint8_t> b) {
    const size_t n = b.size() > 255 ? 255 : b.size();
    header(tag, n);
    out_.insert(out_.end(), b.begin(), b.begin() + static_cast<std::ptrdiff_t>(n));
    return *this;
  }
  const std::vector<uint8_t> &payload() const { return out_; }
  std::vector<uint8_t> take() { return std::move(out_); }

private:
  void header(InfoTag tag, size_t len) {
    out_.push_back(static_cast<uint8_t>(tag));
    out_.push_back(static_cast<uint8_t>(len));
  }
  std::vector<uint8_t> out_;
};

/// A decoded INFO payload: every field is optional (absent when the device did
/// not send the tag). Unknown tags are skipped, so a newer device decodes fine.
struct Info {
  std::optional<std::string> chip_model;
  std::optional<uint16_t> chip_revision;
  std::optional<uint8_t> cores;
  std::optional<uint32_t> chip_features;
  std::optional<std::string> idf_version;
  std::optional<std::string> project_name;
  std::optional<std::string> app_version;
  std::optional<std::string> build_date;
  std::optional<std::string> build_time;
  std::optional<std::array<uint8_t, 32>> elf_sha256;
  std::optional<std::string> running_partition;
  std::optional<std::string> boot_partition;
  std::optional<uint8_t> ota_state;
  std::optional<uint8_t> reset_reason;
  std::optional<uint64_t> uptime_ms;
  std::optional<std::array<uint8_t, 6>> mac;
  std::optional<uint32_t> flash_size;
  std::optional<uint32_t> psram_size;
  std::optional<uint32_t> cpu_mhz;
  std::optional<uint32_t> free_heap;
  std::optional<uint32_t> min_free_heap;
  std::optional<uint32_t> capabilities;
  size_t unknown_tags{0}; ///< how many records carried a tag this decoder does not know
};

/// Decode an INFO payload. Returns nullopt only on a truncated record (a record
/// whose declared length runs past the payload); a record with an unexpected
/// length for a known fixed-size tag is skipped (counted as unknown).
inline std::optional<Info> decode_info(std::span<const uint8_t> p) {
  Info info;
  size_t i = 0;
  while (i < p.size()) {
    if (i + 2 > p.size())
      return std::nullopt;
    const uint8_t tag = p[i];
    const size_t len = p[i + 1];
    i += 2;
    if (i + len > p.size())
      return std::nullopt;
    const std::span<const uint8_t> v = p.subspan(i, len);
    i += len;
    auto as_str = [&]() { return std::string(reinterpret_cast<const char *>(v.data()), v.size()); };
    auto u8 = [&](std::optional<uint8_t> &dst) {
      if (len == 1)
        dst = v[0];
      else
        ++info.unknown_tags;
    };
    auto u16 = [&](std::optional<uint16_t> &dst) {
      if (len == 2)
        dst = espp::stream_frame::get_u16(v);
      else
        ++info.unknown_tags;
    };
    auto u32 = [&](std::optional<uint32_t> &dst) {
      if (len == 4)
        dst = espp::stream_frame::get_u32(v);
      else
        ++info.unknown_tags;
    };
    switch (static_cast<InfoTag>(tag)) {
    case InfoTag::ChipModel:
      info.chip_model = as_str();
      break;
    case InfoTag::ChipRevision:
      u16(info.chip_revision);
      break;
    case InfoTag::Cores:
      u8(info.cores);
      break;
    case InfoTag::ChipFeatures:
      u32(info.chip_features);
      break;
    case InfoTag::IdfVersion:
      info.idf_version = as_str();
      break;
    case InfoTag::ProjectName:
      info.project_name = as_str();
      break;
    case InfoTag::AppVersion:
      info.app_version = as_str();
      break;
    case InfoTag::BuildDate:
      info.build_date = as_str();
      break;
    case InfoTag::BuildTime:
      info.build_time = as_str();
      break;
    case InfoTag::ElfSha256:
      if (len == 32) {
        std::array<uint8_t, 32> sha{};
        std::copy(v.begin(), v.end(), sha.begin());
        info.elf_sha256 = sha;
      } else {
        ++info.unknown_tags;
      }
      break;
    case InfoTag::RunningPartition:
      info.running_partition = as_str();
      break;
    case InfoTag::BootPartition:
      info.boot_partition = as_str();
      break;
    case InfoTag::OtaState:
      u8(info.ota_state);
      break;
    case InfoTag::ResetReason:
      u8(info.reset_reason);
      break;
    case InfoTag::UptimeMs:
      if (len == 8) {
        info.uptime_ms = static_cast<uint64_t>(espp::stream_frame::get_u32(v)) |
                         (static_cast<uint64_t>(espp::stream_frame::get_u32(v.subspan(4))) << 32);
      } else {
        ++info.unknown_tags;
      }
      break;
    case InfoTag::Mac:
      if (len == 6) {
        std::array<uint8_t, 6> mac{};
        std::copy(v.begin(), v.end(), mac.begin());
        info.mac = mac;
      } else {
        ++info.unknown_tags;
      }
      break;
    case InfoTag::FlashSize:
      u32(info.flash_size);
      break;
    case InfoTag::PsramSize:
      u32(info.psram_size);
      break;
    case InfoTag::CpuMhz:
      u32(info.cpu_mhz);
      break;
    case InfoTag::FreeHeap:
      u32(info.free_heap);
      break;
    case InfoTag::MinFreeHeap:
      u32(info.min_free_heap);
      break;
    case InfoTag::Capabilities:
      u32(info.capabilities);
      break;
    default:
      ++info.unknown_tags;
      break;
    }
  }
  return info;
}

// ---- request / reply helpers ---------------------------------------------------

/// Encode a REBOOT / REBOOT_TO_BOOTLOADER payload.
inline std::vector<uint8_t> encode_delay(uint16_t delay_ms) {
  std::vector<uint8_t> p;
  espp::stream_frame::put_u16(p, delay_ms);
  return p;
}

/// Decode a REBOOT / REBOOT_TO_BOOTLOADER payload (an empty payload means 0 ms).
inline std::optional<uint16_t> decode_delay(std::span<const uint8_t> p) {
  if (p.empty())
    return 0;
  if (p.size() < 2)
    return std::nullopt;
  return espp::stream_frame::get_u16(p);
}

/// Encode an OK payload.
inline std::vector<uint8_t> encode_ok(uint8_t request_type) { return {request_type}; }

/// Encode an ERROR payload.
inline std::vector<uint8_t> encode_error(uint8_t request_type, uint32_t code,
                                         std::string_view message) {
  std::vector<uint8_t> p;
  p.push_back(request_type);
  espp::stream_frame::put_u32(p, code);
  p.insert(p.end(), message.begin(), message.end());
  return p;
}

/// Decoded ERROR payload.
struct Error {
  uint8_t request_type{0};
  uint32_t code{0};
  std::string message;
};

inline std::optional<Error> decode_error(std::span<const uint8_t> p) {
  if (p.size() < 5)
    return std::nullopt;
  Error e;
  e.request_type = p[0];
  e.code = espp::stream_frame::get_u32(p.subspan(1));
  e.message.assign(reinterpret_cast<const char *>(p.data() + 5), p.size() - 5);
  return e;
}

} // namespace espp::detail::system_protocol
