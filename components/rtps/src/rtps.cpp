#include "rtps.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <numeric>
#include <optional>
#include <sstream>

#include "cdr.hpp"

namespace {
constexpr std::array<char, 4> kRtpsMagic{'R', 'T', 'P', 'S'};

constexpr uint16_t kPortBase = 7400;
constexpr uint16_t kDomainGain = 250;
constexpr uint16_t kParticipantGain = 2;
constexpr uint16_t kMetatrafficMulticastOffset = 0;
constexpr uint16_t kMetatrafficUnicastOffset = 10;
constexpr uint16_t kUserMulticastOffset = 1;
constexpr uint16_t kUserUnicastOffset = 11;

constexpr uint8_t kSubmessageFlagLittleEndian = 0x01;
constexpr uint8_t kSubmessageFlagInlineQos = 0x02;
constexpr uint8_t kSubmessageFlagData = 0x04;
// For HEARTBEAT and ACKNACK submessages, bit 1 is the Final flag and bit 2 is
// the Liveliness flag (the InlineQos/Data bits above are DATA-specific).
constexpr uint8_t kSubmessageFlagFinal = 0x02;
constexpr uint8_t kSubmessageFlagLiveliness = 0x04;
constexpr uint16_t kDataSubmessageOctetsToInlineQos = 16;

constexpr uint32_t kBuiltinEndpointParticipantAnnouncer = 1u << 0;
constexpr uint32_t kBuiltinEndpointParticipantDetector = 1u << 1;
constexpr uint32_t kBuiltinEndpointPublicationAnnouncer = 1u << 2;
constexpr uint32_t kBuiltinEndpointPublicationDetector = 1u << 3;
constexpr uint32_t kBuiltinEndpointSubscriptionAnnouncer = 1u << 4;
constexpr uint32_t kBuiltinEndpointSubscriptionDetector = 1u << 5;
constexpr uint32_t kBuiltinEndpointParticipantMessageWriter = 1u << 10;
constexpr uint32_t kBuiltinEndpointParticipantMessageReader = 1u << 11;
constexpr uint32_t kBuiltinEndpointSet =
    kBuiltinEndpointParticipantAnnouncer | kBuiltinEndpointParticipantDetector |
    kBuiltinEndpointPublicationAnnouncer | kBuiltinEndpointPublicationDetector |
    kBuiltinEndpointSubscriptionAnnouncer | kBuiltinEndpointSubscriptionDetector;

constexpr std::array<uint8_t, 4> kEntityIdUnknown{{0x00, 0x00, 0x00, 0x00}};
constexpr std::array<uint8_t, 4> kParticipantEntityId{{0x00, 0x00, 0x01, 0xc1}};
constexpr std::array<uint8_t, 4> kSpdpWriterEntityId{{0x00, 0x01, 0x00, 0xc2}};
constexpr std::array<uint8_t, 4> kSpdpReaderEntityId{{0x00, 0x01, 0x00, 0xc7}};
constexpr std::array<uint8_t, 4> kSedpPublicationsWriterEntityId{{0x00, 0x00, 0x03, 0xc2}};
constexpr std::array<uint8_t, 4> kSedpPublicationsReaderEntityId{{0x00, 0x00, 0x03, 0xc7}};
constexpr std::array<uint8_t, 4> kSedpSubscriptionsWriterEntityId{{0x00, 0x00, 0x04, 0xc2}};
constexpr std::array<uint8_t, 4> kSedpSubscriptionsReaderEntityId{{0x00, 0x00, 0x04, 0xc7}};
constexpr uint8_t kUserWriterNoKeyKind = 0x03;
constexpr uint8_t kUserReaderNoKeyKind = 0x04;

constexpr uint32_t kHistoryKeepLast = 0;
constexpr uint32_t kReliabilityBestEffort = 1;
constexpr uint32_t kReliabilityReliable = 2;
constexpr uint32_t kDurabilityVolatile = 0;
constexpr uint32_t kLivelinessAutomatic = 0;
constexpr int32_t kDefaultLeaseDurationSeconds = 20;
constexpr uint32_t kDefaultLeaseDurationNanoseconds = 0;
constexpr int32_t kDefaultMaxBlockingSeconds = 0;
constexpr uint32_t kDefaultMaxBlockingNanoseconds = 100000000;
// PID_TYPE_MAX_SIZE_SERIALIZED carries the max CDR-serialized size of the type *including* the
// 4-byte encapsulation header (matching FastDDS: getMaxCdrSerializedSize() + 4). A UInt32 body is
// 4 bytes, so the spec-exact advertised value is 4 + 4 = 8.
constexpr uint32_t kUInt32SerializedSize = 8;

enum class ParameterId : uint16_t {
  PID_SENTINEL = 0x0001,
  PID_PARTICIPANT_LEASE_DURATION = 0x0002,
  PID_TOPIC_NAME = 0x0005,
  PID_TYPE_NAME = 0x0007,
  PID_DOMAIN_ID = 0x000f,
  PID_PROTOCOL_VERSION = 0x0015,
  PID_VENDORID = 0x0016,
  PID_DURABILITY = 0x001d,
  PID_RELIABILITY = 0x001a,
  PID_LIVELINESS = 0x001b,
  PID_USER_DATA = 0x002c,
  PID_UNICAST_LOCATOR = 0x002f,
  PID_DEFAULT_UNICAST_LOCATOR = 0x0031,
  PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032,
  PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033,
  PID_MULTICAST_LOCATOR = 0x0030,
  PID_EXPECTS_INLINE_QOS = 0x0043,
  PID_DEFAULT_MULTICAST_LOCATOR = 0x0048,
  PID_PARTICIPANT_GUID = 0x0050,
  PID_BUILTIN_ENDPOINT_SET = 0x0058,
  PID_ENDPOINT_GUID = 0x005a,
  PID_TYPE_MAX_SIZE_SERIALIZED = 0x0060,
  PID_ENTITY_NAME = 0x0062,
  PID_KEY_HASH = 0x0070,
  PID_HISTORY = 0x0040,
};

// 64-bit FNV-1a hash. Used to derive the node-name portion of the GUID prefix with a full 64 bits
// of entropy regardless of the platform's size_t width. std::hash<std::string> is only 32-bit on
// the 32-bit ESP32, which made bytes 8..11 of the prefix a repeated copy of bytes 4..7 (the shift
// `hash >> (8 * i)` for i >= 4 was undefined behavior on a 32-bit value).
uint64_t fnv1a_64(std::string_view text) {
  uint64_t hash = 1469598103934665603ull; // FNV-1a 64-bit offset basis
  for (unsigned char c : text) {
    hash ^= c;
    hash *= 1099511628211ull; // FNV-1a 64-bit prime
  }
  return hash;
}

class ByteWriter {
public:
  void append_bytes(std::span<const uint8_t> bytes) {
    data_.insert(data_.end(), bytes.begin(), bytes.end());
  }

  template <size_t N> void append_bytes(const std::array<uint8_t, N> &bytes) {
    data_.insert(data_.end(), bytes.begin(), bytes.end());
  }

  template <size_t N> void append_chars(const std::array<char, N> &bytes) {
    data_.insert(data_.end(), bytes.begin(), bytes.end());
  }

  void append_u8(uint8_t value) { data_.push_back(value); }

  void append_u16_le(uint16_t value) {
    data_.push_back(static_cast<uint8_t>(value & 0xff));
    data_.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  }

  void append_u32_le(uint32_t value) {
    for (int i = 0; i < 4; i++) {
      data_.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xff));
    }
  }

  void append_i32_le(int32_t value) { append_u32_le(static_cast<uint32_t>(value)); }

  void append_sequence_number_le(int64_t value) {
    auto high = static_cast<int32_t>(value >> 32);
    auto low = static_cast<uint32_t>(value & 0xffffffffu);
    append_i32_le(high);
    append_u32_le(low);
  }

  size_t size() const { return data_.size(); }

  void align(size_t alignment) {
    while (data_.size() % alignment != 0) {
      data_.push_back(0);
    }
  }

  std::vector<uint8_t> take() { return std::move(data_); }

private:
  std::vector<uint8_t> data_;
};

class ByteReader {
public:
  explicit ByteReader(std::span<const uint8_t> data)
      : data_(data) {}

  bool read_u8(uint8_t &value) {
    if (remaining() < 1) {
      return false;
    }
    value = data_[offset_++];
    return true;
  }

  bool read_u16_le(uint16_t &value) {
    if (remaining() < 2) {
      return false;
    }
    value = static_cast<uint16_t>(data_[offset_]) |
            static_cast<uint16_t>(static_cast<uint16_t>(data_[offset_ + 1]) << 8);
    offset_ += 2;
    return true;
  }

  bool read_u16_be(uint16_t &value) {
    if (remaining() < 2) {
      return false;
    }
    value = static_cast<uint16_t>(static_cast<uint16_t>(data_[offset_]) << 8) |
            static_cast<uint16_t>(data_[offset_ + 1]);
    offset_ += 2;
    return true;
  }

  bool read_u16(uint16_t &value, bool little_endian) {
    return little_endian ? read_u16_le(value) : read_u16_be(value);
  }

  bool read_u32_le(uint32_t &value) {
    if (remaining() < 4) {
      return false;
    }
    value = static_cast<uint32_t>(data_[offset_]) |
            (static_cast<uint32_t>(data_[offset_ + 1]) << 8) |
            (static_cast<uint32_t>(data_[offset_ + 2]) << 16) |
            (static_cast<uint32_t>(data_[offset_ + 3]) << 24);
    offset_ += 4;
    return true;
  }

  bool read_i32_le(int32_t &value) {
    uint32_t unsigned_value = 0;
    if (!read_u32_le(unsigned_value)) {
      return false;
    }
    value = static_cast<int32_t>(unsigned_value);
    return true;
  }

  bool read_u32_be(uint32_t &value) {
    if (remaining() < 4) {
      return false;
    }
    value = (static_cast<uint32_t>(data_[offset_]) << 24) |
            (static_cast<uint32_t>(data_[offset_ + 1]) << 16) |
            (static_cast<uint32_t>(data_[offset_ + 2]) << 8) |
            static_cast<uint32_t>(data_[offset_ + 3]);
    offset_ += 4;
    return true;
  }

  bool read_u32(uint32_t &value, bool little_endian) {
    return little_endian ? read_u32_le(value) : read_u32_be(value);
  }

  bool read_sequence_number(int64_t &value, bool little_endian) {
    uint32_t high = 0;
    uint32_t low = 0;
    if (little_endian) {
      if (!read_u32_le(high) || !read_u32_le(low)) {
        return false;
      }
    } else {
      if (!read_u32_be(high) || !read_u32_be(low)) {
        return false;
      }
    }
    value = (static_cast<int64_t>(static_cast<int32_t>(high)) << 32) | low;
    return true;
  }

  bool skip(size_t length) {
    if (remaining() < length) {
      return false;
    }
    offset_ += length;
    return true;
  }

  bool read_bytes(std::span<uint8_t> destination) {
    if (remaining() < destination.size()) {
      return false;
    }
    std::memcpy(destination.data(), data_.data() + offset_, destination.size());
    offset_ += destination.size();
    return true;
  }

  std::span<const uint8_t> read_span(size_t length) {
    if (remaining() < length) {
      return {};
    }
    auto span = data_.subspan(offset_, length);
    offset_ += length;
    return span;
  }

  size_t remaining() const { return data_.size() - offset_; }

private:
  std::span<const uint8_t> data_;
  size_t offset_{0};
};

struct ParameterView {
  ParameterId id{ParameterId::PID_SENTINEL};
  std::span<const uint8_t> value{};
};

struct DataSubmessageView {
  espp::RtpsParticipant::EntityId reader_id{};
  espp::RtpsParticipant::EntityId writer_id{};
  int64_t writer_sn{0};
  std::span<const uint8_t> serialized_payload{};
  bool inline_qos_present{false};
  bool data_present{false};
};

std::string hex_string(std::span<const uint8_t> bytes) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (size_t i = 0; i < bytes.size(); i++) {
    if (i != 0) {
      stream << ':';
    }
    stream << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return stream.str();
}

bool parse_ipv4(std::string_view address, std::array<uint8_t, 4> &octets) {
  std::array<uint8_t, 4> parsed{};
  size_t part_index = 0;
  size_t cursor = 0;
  while (cursor < address.size() && part_index < parsed.size()) {
    auto next = address.find('.', cursor);
    if (next == std::string_view::npos) {
      next = address.size();
    }
    if (next == cursor) {
      return false;
    }
    unsigned value = 0;
    for (size_t i = cursor; i < next; i++) {
      if (!std::isdigit(static_cast<unsigned char>(address[i]))) {
        return false;
      }
      value = value * 10 + static_cast<unsigned>(address[i] - '0');
      if (value > 255) {
        return false;
      }
    }
    parsed[part_index++] = static_cast<uint8_t>(value);
    cursor = next + 1;
  }
  if (part_index != parsed.size() || cursor < address.size()) {
    return false;
  }
  octets = parsed;
  return true;
}

void append_parameter_header(ByteWriter &writer, ParameterId id, uint16_t length) {
  writer.append_u16_le(static_cast<uint16_t>(id));
  writer.append_u16_le(length);
}

void append_parameter_guid(ByteWriter &writer, ParameterId id,
                           const espp::RtpsParticipant::Guid &guid) {
  append_parameter_header(writer, id, 16);
  writer.append_bytes(guid.prefix.value);
  writer.append_bytes(guid.entity_id.value);
}

void append_parameter_protocol_version(ByteWriter &writer,
                                       const espp::RtpsParticipant::ProtocolVersion &version) {
  append_parameter_header(writer, ParameterId::PID_PROTOCOL_VERSION, 4);
  writer.append_u8(version.major);
  writer.append_u8(version.minor);
  writer.append_u8(0);
  writer.append_u8(0);
}

void append_parameter_vendor_id(ByteWriter &writer,
                                const espp::RtpsParticipant::VendorId &vendor_id) {
  append_parameter_header(writer, ParameterId::PID_VENDORID, 4);
  writer.append_bytes(vendor_id.value);
  writer.append_u8(0);
  writer.append_u8(0);
}

void append_parameter_u32(ByteWriter &writer, ParameterId id, uint32_t value) {
  append_parameter_header(writer, id, 4);
  writer.append_u32_le(value);
}

void append_parameter_bool(ByteWriter &writer, ParameterId id, bool value) {
  append_parameter_header(writer, id, 4);
  writer.append_u8(value ? 1 : 0);
  writer.append_u8(0);
  writer.append_u8(0);
  writer.append_u8(0);
}

// RTPS Duration_t/Time_t use the NTP representation {int32 seconds, uint32 fraction} where the
// fraction is in units of 1/2^32 of a second (see DDSI-RTPS; OpenDDS RtpsCore.idl references RFC
// 1305). Convert a nanosecond count to that fraction so durations are encoded spec-exactly.
constexpr uint32_t ntp_fraction_from_nanoseconds(uint32_t nanoseconds) {
  return static_cast<uint32_t>((static_cast<uint64_t>(nanoseconds) << 32) / 1000000000ULL);
}

void append_parameter_duration(ByteWriter &writer, ParameterId id, int32_t seconds,
                               uint32_t nanoseconds) {
  append_parameter_header(writer, id, 8);
  writer.append_i32_le(seconds);
  writer.append_u32_le(ntp_fraction_from_nanoseconds(nanoseconds));
}

void append_parameter_locator(ByteWriter &writer, ParameterId id,
                              const espp::RtpsParticipant::Locator &locator) {
  append_parameter_header(writer, id, 24);
  // Locator_t.kind and .port are CDR long/unsigned long encoded in the parameter list endianness
  // (little-endian for PL_CDR_LE); only the 16-byte address is a raw per-byte (network-order)
  // field.
  writer.append_u32_le(static_cast<uint32_t>(locator.kind));
  writer.append_u32_le(locator.port);
  writer.append_bytes(locator.address);
}

void append_parameter_string_cdr(ByteWriter &writer, ParameterId id, std::string_view text) {
  auto cdr_writer = espp::CdrWriter::make_body_writer(espp::CdrEncapsulation::CDR_LE);
  cdr_writer.write_string(text);
  auto cdr_payload = cdr_writer.payload();
  // The RTPS PL_CDR encoding requires parameterLength to be a multiple of 4 so the next
  // parameter starts 4-byte aligned. write_string() already trailing-aligns the body to 4, so the
  // payload length is the padded length we must declare.
  append_parameter_header(writer, id, static_cast<uint16_t>(cdr_payload.size()));
  writer.append_bytes(cdr_payload);
}

void append_parameter_octet_sequence(ByteWriter &writer, ParameterId id,
                                     std::span<const uint8_t> bytes) {
  auto cdr_writer = espp::CdrWriter::make_body_writer(espp::CdrEncapsulation::CDR_LE);
  cdr_writer.write<uint32_t>(static_cast<uint32_t>(bytes.size()));
  cdr_writer.write_bytes(bytes);
  cdr_writer.align(4);
  auto cdr_payload = cdr_writer.payload();
  // parameterLength must be a multiple of 4 (see append_parameter_string_cdr); the align(4) above
  // padded the payload to that length, so declare the padded length.
  append_parameter_header(writer, id, static_cast<uint16_t>(cdr_payload.size()));
  writer.append_bytes(cdr_payload);
}

void append_parameter_reliability(ByteWriter &writer,
                                  espp::RtpsParticipant::ReliabilityKind reliability) {
  append_parameter_header(writer, ParameterId::PID_RELIABILITY, 12);
  writer.append_u32_le(reliability == espp::RtpsParticipant::ReliabilityKind::RELIABLE
                           ? kReliabilityReliable
                           : kReliabilityBestEffort);
  writer.append_i32_le(kDefaultMaxBlockingSeconds);
  writer.append_u32_le(ntp_fraction_from_nanoseconds(kDefaultMaxBlockingNanoseconds));
}

void append_parameter_durability(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_DURABILITY, 4);
  writer.append_u32_le(kDurabilityVolatile);
}

void append_parameter_liveliness(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_LIVELINESS, 12);
  writer.append_u32_le(kLivelinessAutomatic);
  writer.append_i32_le(kDefaultLeaseDurationSeconds);
  writer.append_u32_le(ntp_fraction_from_nanoseconds(kDefaultLeaseDurationNanoseconds));
}

void append_parameter_history(ByteWriter &writer, uint32_t depth = 1) {
  append_parameter_header(writer, ParameterId::PID_HISTORY, 8);
  writer.append_u32_le(kHistoryKeepLast); // history kind: KEEP_LAST
  writer.append_u32_le(depth);            // history depth
}

void append_parameter_key_hash(ByteWriter &writer, const espp::RtpsParticipant::Guid &guid) {
  append_parameter_header(writer, ParameterId::PID_KEY_HASH, 16);
  writer.append_bytes(guid.prefix.value);
  writer.append_bytes(guid.entity_id.value);
}

void append_parameter_sentinel(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_SENTINEL, 0);
}

std::vector<ParameterView> parse_parameter_list(std::span<const uint8_t> payload) {
  std::vector<ParameterView> parameters;
  espp::CdrReader cdr_reader(payload);
  // Limitation: only little-endian parameter lists (PL_CDR_LE) are decoded. The parameter value
  // parsers below (parse_u32_le, parse_locator, parse_guid, ...) assume little-endian contents, so
  // a big-endian (PL_CDR_BE) list is intentionally rejected rather than misparsed. In practice DDS
  // and ROS 2 implementations emit PL_CDR_LE for SPDP/SEDP discovery, so this is a discovery-only
  // gap.
  if (!cdr_reader.valid() || cdr_reader.encapsulation() != espp::CdrEncapsulation::PL_CDR_LE) {
    return parameters;
  }

  ByteReader reader(cdr_reader.payload());
  while (reader.remaining() >= 4) {
    uint16_t pid = 0;
    uint16_t length = 0;
    if (!reader.read_u16_le(pid) || !reader.read_u16_le(length)) {
      return {};
    }
    if (pid == static_cast<uint16_t>(ParameterId::PID_SENTINEL)) {
      break;
    }
    auto value = reader.read_span(length);
    if (value.size() != length) {
      return {};
    }
    parameters.push_back({.id = static_cast<ParameterId>(pid), .value = value});
    auto padding = (4 - (length % 4)) & 0x3;
    if (padding > 0 && reader.read_span(padding).size() != padding) {
      return {};
    }
  }
  return parameters;
}

std::optional<ParameterView> find_parameter(std::span<const ParameterView> parameters,
                                            ParameterId id) {
  auto iterator = std::find_if(parameters.begin(), parameters.end(),
                               [id](const auto &parameter) { return parameter.id == id; });
  if (iterator == parameters.end()) {
    return std::nullopt;
  }
  return *iterator;
}

std::vector<ParameterView> find_parameters(std::span<const ParameterView> parameters,
                                           ParameterId id) {
  std::vector<ParameterView> matches;
  std::copy_if(parameters.begin(), parameters.end(), std::back_inserter(matches),
               [id](const auto &parameter) { return parameter.id == id; });
  return matches;
}

std::optional<espp::RtpsParticipant::Guid> parse_guid(std::span<const uint8_t> value) {
  if (value.size() != 16) {
    return std::nullopt;
  }
  espp::RtpsParticipant::Guid guid;
  std::memcpy(guid.prefix.value.data(), value.data(), guid.prefix.value.size());
  std::memcpy(guid.entity_id.value.data(), value.data() + guid.prefix.value.size(),
              guid.entity_id.value.size());
  return guid;
}

std::optional<uint32_t> parse_u32_le(std::span<const uint8_t> value) {
  ByteReader reader(value);
  uint32_t parsed = 0;
  if (!reader.read_u32_le(parsed)) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<bool> parse_bool(std::span<const uint8_t> value) {
  if (value.size() < 1) {
    return std::nullopt;
  }
  return value[0] != 0;
}

std::optional<std::string> parse_cdr_string(std::span<const uint8_t> value) {
  auto reader = espp::CdrReader::make_body_reader(value, espp::CdrEncapsulation::CDR_LE);
  if (!reader.valid()) {
    return std::nullopt;
  }
  uint32_t length = 0;
  if (!reader.read<uint32_t>(length) || length == 0) {
    return std::nullopt;
  }
  auto text_bytes = reader.read_span(length);
  if (text_bytes.size() != length || text_bytes.back() != 0) {
    return std::nullopt;
  }
  return std::string(reinterpret_cast<const char *>(text_bytes.data()), text_bytes.size() - 1);
}

std::optional<std::vector<uint8_t>> parse_octet_sequence(std::span<const uint8_t> value) {
  auto reader = espp::CdrReader::make_body_reader(value, espp::CdrEncapsulation::CDR_LE);
  if (!reader.valid()) {
    return std::nullopt;
  }
  uint32_t length = 0;
  if (!reader.read<uint32_t>(length)) {
    return std::nullopt;
  }
  std::vector<uint8_t> bytes;
  if (!reader.read_bytes(bytes, length)) {
    return std::nullopt;
  }
  return bytes;
}

std::optional<espp::RtpsParticipant::Locator> parse_locator(std::span<const uint8_t> value) {
  if (value.size() != 24) {
    return std::nullopt;
  }
  ByteReader reader(value);
  uint32_t kind = 0;
  uint32_t port = 0;
  espp::RtpsParticipant::Locator locator;
  // kind and port are little-endian in PL_CDR_LE (see append_parameter_locator); the address is a
  // raw 16-byte field read verbatim.
  if (!reader.read_u32_le(kind) || !reader.read_u32_le(port) ||
      !reader.read_bytes(std::span<uint8_t>{locator.address.data(), locator.address.size()})) {
    return std::nullopt;
  }
  locator.kind = static_cast<espp::RtpsParticipant::Locator::Kind>(static_cast<int32_t>(kind));
  locator.port = port;
  return locator;
}

bool has_valid_locator(const espp::RtpsParticipant::Locator &locator) {
  return locator.kind == espp::RtpsParticipant::Locator::Kind::UDP_V4 && locator.port != 0 &&
         std::any_of(locator.address.begin() + 12, locator.address.end(),
                     [](uint8_t octet) { return octet != 0; });
}

// Human-readable "kind/address:port" for a locator, for discovery logging.
std::string locator_to_string(const espp::RtpsParticipant::Locator &locator) {
  if (locator.kind == espp::RtpsParticipant::Locator::Kind::INVALID) {
    return "<invalid>";
  }
  return fmt::format("udpv4/{}:{}", locator.address_string(), locator.port);
}

// Same as above for an optional locator that may not have been present in a message.
std::string locator_to_string(const std::optional<espp::RtpsParticipant::Locator> &locator) {
  return locator ? locator_to_string(*locator) : "<absent>";
}

std::optional<espp::RtpsParticipant::ReliabilityKind>
parse_reliability(std::span<const uint8_t> value) {
  auto maybe_kind = parse_u32_le(value);
  if (!maybe_kind) {
    return std::nullopt;
  }
  if (*maybe_kind == kReliabilityReliable) {
    return espp::RtpsParticipant::ReliabilityKind::RELIABLE;
  }
  return espp::RtpsParticipant::ReliabilityKind::BEST_EFFORT;
}

std::string extract_enclave(std::span<const uint8_t> user_data_bytes) {
  std::string text(reinterpret_cast<const char *>(user_data_bytes.data()), user_data_bytes.size());
  std::string key = "enclave=";
  auto position = text.find(key);
  if (position == std::string::npos) {
    return "/";
  }
  position += key.size();
  auto end = text.find(';', position);
  if (end == std::string::npos) {
    end = text.size();
  }
  // Normalize an empty enclave (e.g. "enclave=;") to the default "/" rather than returning "".
  if (end == position) {
    return "/";
  }
  return text.substr(position, end - position);
}

std::array<uint8_t, 4> entity_id_for_index(uint32_t entity_index, uint8_t kind) {
  return {0x00, 0x00, static_cast<uint8_t>(0x10 + entity_index), kind};
}

bool is_same_guid_prefix(const espp::RtpsParticipant::Guid &guid,
                         const espp::RtpsParticipant::GuidPrefix &prefix) {
  return guid.prefix == prefix;
}

// Skip an inline-QoS parameter list (a raw ParameterList without an encapsulation header) up to and
// including its PID_SENTINEL terminator. Returns false if the list is malformed/truncated.
bool skip_inline_qos(ByteReader &reader, bool little_endian) {
  while (reader.remaining() >= 4) {
    uint16_t pid = 0;
    uint16_t length = 0;
    if (!reader.read_u16(pid, little_endian) || !reader.read_u16(length, little_endian)) {
      return false;
    }
    if (pid == static_cast<uint16_t>(ParameterId::PID_SENTINEL)) {
      return true;
    }
    if (!reader.skip(length)) {
      return false;
    }
  }
  return false;
}

DataSubmessageView parse_data_submessage(const espp::RtpsParticipant::Submessage &submessage,
                                         bool &ok) {
  DataSubmessageView view;
  ok = false;
  if (submessage.kind != espp::RtpsParticipant::SubmessageKind::DATA ||
      (submessage.flags & kSubmessageFlagData) == 0) {
    return view;
  }

  const bool little_endian = (submessage.flags & kSubmessageFlagLittleEndian) != 0;
  ByteReader reader(std::span<const uint8_t>{submessage.payload.data(), submessage.payload.size()});
  uint16_t extra_flags = 0;
  uint16_t octets_to_inline_qos = 0;
  if (!reader.read_u16(extra_flags, little_endian) ||
      !reader.read_u16(octets_to_inline_qos, little_endian) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.reader_id.value.data(), view.reader_id.value.size()}) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.writer_id.value.data(), view.writer_id.value.size()}) ||
      !reader.read_sequence_number(view.writer_sn, little_endian)) {
    return view;
  }

  view.inline_qos_present = (submessage.flags & kSubmessageFlagInlineQos) != 0;
  view.data_present = true;

  // octetsToInlineQos counts from the byte after the octetsToInlineQos field to the start of the
  // inline QoS (or the serialized payload when no inline QoS is present). We have already consumed
  // the standard 16-byte readerId+writerId+writerSN block; honor any additional header octets a
  // sender may have included instead of assuming the fixed layout.
  if (octets_to_inline_qos < kDataSubmessageOctetsToInlineQos ||
      !reader.skip(octets_to_inline_qos - kDataSubmessageOctetsToInlineQos)) {
    return view;
  }

  // When inline QoS is present, skip past the inline QoS parameter list to reach the serialized
  // payload rather than dropping the sample.
  if (view.inline_qos_present && !skip_inline_qos(reader, little_endian)) {
    return view;
  }

  view.serialized_payload = reader.read_span(reader.remaining());
  ok = true;
  return view;
}

std::vector<uint8_t> build_parameter_list_payload(ByteWriter &parameter_writer) {
  auto parameter_bytes = parameter_writer.take();
  return espp::CdrWriter::encapsulate(parameter_bytes, espp::CdrEncapsulation::PL_CDR_LE);
}

std::vector<uint8_t> build_data_submessage_payload(const espp::RtpsParticipant::EntityId &reader_id,
                                                   const espp::RtpsParticipant::EntityId &writer_id,
                                                   int64_t sequence_number,
                                                   std::span<const uint8_t> serialized_payload) {
  ByteWriter writer;
  writer.append_u16_le(0);
  writer.append_u16_le(kDataSubmessageOctetsToInlineQos);
  writer.append_bytes(reader_id.value);
  writer.append_bytes(writer_id.value);
  writer.append_sequence_number_le(sequence_number);
  writer.append_bytes(serialized_payload);
  writer.align(4);
  return writer.take();
}

espp::RtpsParticipant::Message build_message(const espp::RtpsParticipant::GuidPrefix &guid_prefix,
                                             const espp::RtpsParticipant::EntityId &reader_id,
                                             const espp::RtpsParticipant::EntityId &writer_id,
                                             int64_t sequence_number,
                                             std::span<const uint8_t> serialized_payload) {
  return {.header = {.guid_prefix = guid_prefix},
          .submessages = {{
              .kind = espp::RtpsParticipant::SubmessageKind::DATA,
              .flags = static_cast<uint8_t>(kSubmessageFlagLittleEndian | kSubmessageFlagData),
              .payload = build_data_submessage_payload(reader_id, writer_id, sequence_number,
                                                       serialized_payload),
          }}};
}

// ---------------------------------------------------------------------------
// Reliable QoS submessage codecs (HEARTBEAT / ACKNACK / INFO_DST). See
// RELIABLE_RTPS_PLAN.md. These are the Phase 0 wire-format primitives; they are
// wired into the writer/reader state machines in later phases.
// ---------------------------------------------------------------------------

// RTPS SequenceNumberSet: a bitmapBase plus up to 256 bits marking which of
// [bitmapBase, bitmapBase + numBits) sequence numbers are present in the set
// (used by ACKNACK to mark the missing/requested sequence numbers). Bit i is the
// (31 - i % 32)-th bit (MSB-first) of word (i / 32).
struct SequenceNumberSet {
  static constexpr uint32_t kMaxBits = 256;
  int64_t base{1};                  ///< bitmapBase: lowest sequence number the set can represent.
  uint32_t num_bits{0};             ///< Number of valid bits (0..256).
  std::array<uint32_t, 8> bitmap{}; ///< Up to 256 bits (8 x uint32), MSB-first within each word.

  uint32_t num_words() const { return (num_bits + 31) / 32; }

  // Mark a sequence number as present in the set (no-op if out of [base, base+256)).
  void set(int64_t sequence_number) {
    if (sequence_number < base) {
      return;
    }
    int64_t delta = sequence_number - base;
    if (delta >= static_cast<int64_t>(kMaxBits)) {
      return;
    }
    auto index = static_cast<uint32_t>(delta);
    num_bits = std::max(num_bits, index + 1);
    bitmap[index / 32] |= (1u << (31 - (index % 32)));
  }

  bool contains(int64_t sequence_number) const {
    if (sequence_number < base) {
      return false;
    }
    int64_t delta = sequence_number - base;
    if (delta >= static_cast<int64_t>(num_bits)) {
      return false;
    }
    auto index = static_cast<uint32_t>(delta);
    return (bitmap[index / 32] >> (31 - (index % 32))) & 1u;
  }
};

void append_sequence_number_set(ByteWriter &writer, const SequenceNumberSet &set) {
  writer.append_sequence_number_le(set.base);
  writer.append_u32_le(set.num_bits);
  for (uint32_t word = 0; word < set.num_words(); word++) {
    writer.append_u32_le(set.bitmap[word]);
  }
}

bool read_sequence_number_set(ByteReader &reader, bool little_endian, SequenceNumberSet &set) {
  set = SequenceNumberSet{};
  if (!reader.read_sequence_number(set.base, little_endian) ||
      !reader.read_u32(set.num_bits, little_endian)) {
    return false;
  }
  if (set.num_bits > SequenceNumberSet::kMaxBits) {
    return false;
  }
  for (uint32_t word = 0; word < set.num_words(); word++) {
    if (!reader.read_u32(set.bitmap[word], little_endian)) {
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> build_info_dst_payload(const espp::RtpsParticipant::GuidPrefix &dest_prefix) {
  ByteWriter writer;
  writer.append_bytes(dest_prefix.value);
  return writer.take();
}

espp::RtpsParticipant::Submessage
build_info_dst_submessage(const espp::RtpsParticipant::GuidPrefix &dest_prefix) {
  return {.kind = espp::RtpsParticipant::SubmessageKind::INFO_DST,
          .flags = kSubmessageFlagLittleEndian,
          .payload = build_info_dst_payload(dest_prefix)};
}

std::vector<uint8_t> build_heartbeat_payload(const espp::RtpsParticipant::EntityId &reader_id,
                                             const espp::RtpsParticipant::EntityId &writer_id,
                                             int64_t first_sn, int64_t last_sn, uint32_t count) {
  ByteWriter writer;
  writer.append_bytes(reader_id.value);
  writer.append_bytes(writer_id.value);
  writer.append_sequence_number_le(first_sn);
  writer.append_sequence_number_le(last_sn);
  writer.append_u32_le(count);
  return writer.take();
}

espp::RtpsParticipant::Submessage
build_heartbeat_submessage(const espp::RtpsParticipant::EntityId &reader_id,
                           const espp::RtpsParticipant::EntityId &writer_id, int64_t first_sn,
                           int64_t last_sn, uint32_t count, bool final) {
  uint8_t flags = kSubmessageFlagLittleEndian | (final ? kSubmessageFlagFinal : 0);
  return {.kind = espp::RtpsParticipant::SubmessageKind::HEARTBEAT,
          .flags = flags,
          .payload = build_heartbeat_payload(reader_id, writer_id, first_sn, last_sn, count)};
}

std::vector<uint8_t> build_acknack_payload(const espp::RtpsParticipant::EntityId &reader_id,
                                           const espp::RtpsParticipant::EntityId &writer_id,
                                           const SequenceNumberSet &reader_sn_state,
                                           uint32_t count) {
  ByteWriter writer;
  writer.append_bytes(reader_id.value);
  writer.append_bytes(writer_id.value);
  append_sequence_number_set(writer, reader_sn_state);
  writer.append_u32_le(count);
  return writer.take();
}

espp::RtpsParticipant::Submessage
build_acknack_submessage(const espp::RtpsParticipant::EntityId &reader_id,
                         const espp::RtpsParticipant::EntityId &writer_id,
                         const SequenceNumberSet &reader_sn_state, uint32_t count, bool final) {
  uint8_t flags = kSubmessageFlagLittleEndian | (final ? kSubmessageFlagFinal : 0);
  return {.kind = espp::RtpsParticipant::SubmessageKind::ACKNACK,
          .flags = flags,
          .payload = build_acknack_payload(reader_id, writer_id, reader_sn_state, count)};
}

struct HeartbeatView {
  espp::RtpsParticipant::EntityId reader_id{};
  espp::RtpsParticipant::EntityId writer_id{};
  int64_t first_sn{0};
  int64_t last_sn{0};
  uint32_t count{0};
  bool final{false};
  bool liveliness{false};
  bool valid{false};
};

HeartbeatView parse_heartbeat_submessage(const espp::RtpsParticipant::Submessage &submessage) {
  HeartbeatView view;
  if (submessage.kind != espp::RtpsParticipant::SubmessageKind::HEARTBEAT) {
    return view;
  }
  const bool little_endian = (submessage.flags & kSubmessageFlagLittleEndian) != 0;
  ByteReader reader(std::span<const uint8_t>{submessage.payload.data(), submessage.payload.size()});
  if (!reader.read_bytes(
          std::span<uint8_t>{view.reader_id.value.data(), view.reader_id.value.size()}) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.writer_id.value.data(), view.writer_id.value.size()}) ||
      !reader.read_sequence_number(view.first_sn, little_endian) ||
      !reader.read_sequence_number(view.last_sn, little_endian) ||
      !reader.read_u32(view.count, little_endian)) {
    return view;
  }
  view.final = (submessage.flags & kSubmessageFlagFinal) != 0;
  view.liveliness = (submessage.flags & kSubmessageFlagLiveliness) != 0;
  view.valid = true;
  return view;
}

struct AckNackView {
  espp::RtpsParticipant::EntityId reader_id{};
  espp::RtpsParticipant::EntityId writer_id{};
  SequenceNumberSet reader_sn_state{};
  uint32_t count{0};
  bool final{false};
  bool valid{false};
};

AckNackView parse_acknack_submessage(const espp::RtpsParticipant::Submessage &submessage) {
  AckNackView view;
  if (submessage.kind != espp::RtpsParticipant::SubmessageKind::ACKNACK) {
    return view;
  }
  const bool little_endian = (submessage.flags & kSubmessageFlagLittleEndian) != 0;
  ByteReader reader(std::span<const uint8_t>{submessage.payload.data(), submessage.payload.size()});
  if (!reader.read_bytes(
          std::span<uint8_t>{view.reader_id.value.data(), view.reader_id.value.size()}) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.writer_id.value.data(), view.writer_id.value.size()}) ||
      !read_sequence_number_set(reader, little_endian, view.reader_sn_state) ||
      !reader.read_u32(view.count, little_endian)) {
    return view;
  }
  view.final = (submessage.flags & kSubmessageFlagFinal) != 0;
  view.valid = true;
  return view;
}

// Return the sequence numbers a reader is requesting (the set bits) from an ACKNACK's reader SN
// state. A positive ack (empty set) yields an empty list.
std::vector<int64_t> requested_sequence_numbers(const SequenceNumberSet &set) {
  std::vector<int64_t> sequence_numbers;
  for (uint32_t bit = 0; bit < set.num_bits; bit++) {
    const int64_t sequence_number = set.base + static_cast<int64_t>(bit);
    if (set.contains(sequence_number)) {
      sequence_numbers.push_back(sequence_number);
    }
  }
  return sequence_numbers;
}
} // namespace

namespace espp {
std::string RtpsParticipant::GuidPrefix::to_string() const { return hex_string(value); }

std::string RtpsParticipant::EntityId::to_string() const { return hex_string(value); }

std::string RtpsParticipant::Guid::to_string() const {
  return prefix.to_string() + '|' + entity_id.to_string();
}

RtpsParticipant::Locator RtpsParticipant::Locator::udp_v4(std::string_view ipv4_address,
                                                          uint16_t port) {
  Locator locator;
  locator.kind = Kind::UDP_V4;
  locator.port = port;
  std::array<uint8_t, 4> octets{};
  if (parse_ipv4(ipv4_address, octets)) {
    locator.address[12] = octets[0];
    locator.address[13] = octets[1];
    locator.address[14] = octets[2];
    locator.address[15] = octets[3];
  }
  return locator;
}

std::string RtpsParticipant::Locator::address_string() const {
  if (kind != Kind::UDP_V4) {
    return "0.0.0.0";
  }
  std::ostringstream stream;
  stream << static_cast<int>(address[12]) << '.' << static_cast<int>(address[13]) << '.'
         << static_cast<int>(address[14]) << '.' << static_cast<int>(address[15]);
  return stream.str();
}

std::vector<uint8_t> RtpsParticipant::Message::serialize() const {
  ByteWriter writer;
  writer.append_chars(kRtpsMagic);
  writer.append_u8(header.protocol_version.major);
  writer.append_u8(header.protocol_version.minor);
  writer.append_bytes(header.vendor_id.value);
  writer.append_bytes(header.guid_prefix.value);
  for (const auto &submessage : submessages) {
    writer.append_u8(static_cast<uint8_t>(submessage.kind));
    writer.append_u8(submessage.flags);
    writer.append_u16_le(static_cast<uint16_t>(submessage.payload.size()));
    writer.append_bytes(submessage.payload);
  }
  return writer.take();
}

std::optional<RtpsParticipant::Message>
RtpsParticipant::Message::parse(std::span<const uint8_t> data) {
  if (data.size() < 20 || !std::equal(kRtpsMagic.begin(), kRtpsMagic.end(), data.begin())) {
    return std::nullopt;
  }

  ByteReader reader(data.subspan(4));
  Message message;
  if (!reader.read_u8(message.header.protocol_version.major) ||
      !reader.read_u8(message.header.protocol_version.minor) ||
      !reader.read_bytes(std::span<uint8_t>{message.header.vendor_id.value.data(),
                                            message.header.vendor_id.value.size()}) ||
      !reader.read_bytes(std::span<uint8_t>{message.header.guid_prefix.value.data(),
                                            message.header.guid_prefix.value.size()})) {
    return std::nullopt;
  }

  while (reader.remaining() > 0) {
    Submessage submessage;
    uint8_t kind = 0;
    uint16_t length = 0;
    // Limitation: submessageLength is read little-endian regardless of the submessage E-flag (bit 0
    // of flags). Big-endian submessages are not supported; in practice DDS/ROS 2 peers emit
    // little-endian framing. Endianness of the DATA submessage body itself is honored separately in
    // parse_data_submessage().
    if (!reader.read_u8(kind) || !reader.read_u8(submessage.flags) || !reader.read_u16_le(length)) {
      return std::nullopt;
    }
    auto payload = reader.read_span(length);
    if (payload.size() != length) {
      return std::nullopt;
    }
    submessage.kind = static_cast<SubmessageKind>(kind);
    submessage.payload.assign(payload.begin(), payload.end());
    message.submessages.push_back(std::move(submessage));
  }
  return message;
}

RtpsParticipant::RtpsParticipant(const Config &config)
    : BaseComponent({.tag = "RtpsParticipant", .level = config.log_level})
    , config_(config) {
  // GUID prefix layout: bytes 0..1 = participant_id, 2..3 = domain_id, 4..11 = 64-bit node-name
  // hash. Uniqueness across participants on one host relies on distinct participant_ids; the
  // node-name hash distinguishes different nodes/applications.
  uint64_t hash = fnv1a_64(config_.node_name);
  guid_prefix_.value[0] = config_.participant_id & 0xff;
  guid_prefix_.value[1] = (config_.participant_id >> 8) & 0xff;
  guid_prefix_.value[2] = config_.domain_id & 0xff;
  guid_prefix_.value[3] = (config_.domain_id >> 8) & 0xff;
  for (size_t i = 0; i < 8; i++) {
    guid_prefix_.value[4 + i] = static_cast<uint8_t>((hash >> (8 * i)) & 0xff);
  }
}

RtpsParticipant::~RtpsParticipant() { stop(); }

bool RtpsParticipant::start() {
  if (started_.exchange(true)) {
    return false;
  }

  auto port_mapping = ports();
  logger_.info("RTPS participant {} starting: node '{}', domain {}, pid {}, bind {}, advertised {} "
               "| ports meta_mc={} meta_uc={} user_mc={} user_uc={} | meta_mc_group {}",
               guid_prefix_.to_string(), config_.node_name, config_.domain_id,
               config_.participant_id, config_.bind_address, config_.advertised_address,
               port_mapping.metatraffic_multicast, port_mapping.metatraffic_unicast,
               port_mapping.user_multicast, port_mapping.user_unicast,
               config_.metatraffic_multicast_group);
  metatraffic_multicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = config_.socket_log_level});
  metatraffic_unicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = config_.socket_log_level});
  user_unicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = config_.socket_log_level});

  auto multicast_task_config = config_.receive_task_config;
  multicast_task_config.name = config_.receive_task_config.name + "_spdp_mc";
  auto multicast_receive_config = UdpSocket::ReceiveConfig{
      .port = port_mapping.metatraffic_multicast,
      .buffer_size = 4096,
      .is_multicast_endpoint = true,
      .multicast_group = config_.metatraffic_multicast_group,
      .multicast_interface = config_.bind_address,
      .on_receive_callback = [this](auto &data,
                                    const auto &sender) -> std::optional<std::vector<uint8_t>> {
        handle_metatraffic_message(data, sender);
        return std::nullopt;
      },
  };
  if (!metatraffic_multicast_receiver_->start_receiving(multicast_task_config,
                                                        multicast_receive_config)) {
    logger_.error("Failed to start metatraffic multicast receiver");
    stop();
    return false;
  }

  auto unicast_meta_task_config = config_.receive_task_config;
  unicast_meta_task_config.name = config_.receive_task_config.name + "_meta_uc";
  auto unicast_meta_receive_config = UdpSocket::ReceiveConfig{
      .port = port_mapping.metatraffic_unicast,
      .buffer_size = 4096,
      .on_receive_callback = [this](auto &data,
                                    const auto &sender) -> std::optional<std::vector<uint8_t>> {
        handle_metatraffic_message(data, sender);
        return std::nullopt;
      },
  };
  if (!metatraffic_unicast_receiver_->start_receiving(unicast_meta_task_config,
                                                      unicast_meta_receive_config)) {
    logger_.error("Failed to start metatraffic unicast receiver");
    stop();
    return false;
  }

  auto user_task_config = config_.receive_task_config;
  user_task_config.name = config_.receive_task_config.name + "_user_uc";
  auto user_receive_config = UdpSocket::ReceiveConfig{
      .port = port_mapping.user_unicast,
      .buffer_size = 4096,
      .on_receive_callback = [this](auto &data,
                                    const auto &sender) -> std::optional<std::vector<uint8_t>> {
        handle_user_message(data, sender);
        return std::nullopt;
      },
  };
  if (!user_unicast_receiver_->start_receiving(user_task_config, user_receive_config)) {
    logger_.error("Failed to start user unicast receiver");
    stop();
    return false;
  }

  if (!ensure_user_multicast_receivers_started()) {
    stop();
    return false;
  }

  announce_task_ = Task::make_unique({
      .callback = [this](std::mutex &mutex, std::condition_variable &cv, bool &notified) -> bool {
        send_discovery_now();
        std::unique_lock<std::mutex> lock(mutex);
        auto stop_requested =
            cv.wait_for(lock, config_.announce_period, [&notified] { return notified; });
        notified = false;
        return stop_requested;
      },
      .task_config = config_.announce_task_config,
      .log_level = get_log_level(),
  });
  announce_task_->start();
  send_discovery_now();

  // Periodic HEARTBEAT for reliable writers so matched reliable readers can detect gaps and catch
  // up (e.g. late joiners). Does nothing while there are no reliable writers / cached samples.
  heartbeat_task_ = Task::make_unique({
      .callback = [this](std::mutex &mutex, std::condition_variable &cv, bool &notified) -> bool {
        send_heartbeats_now();
        std::unique_lock<std::mutex> lock(mutex);
        auto stop_requested =
            cv.wait_for(lock, config_.heartbeat_period, [&notified] { return notified; });
        notified = false;
        return stop_requested;
      },
      .task_config = config_.heartbeat_task_config,
      .log_level = get_log_level(),
  });
  heartbeat_task_->start();
  return true;
}

void RtpsParticipant::stop() {
  started_ = false;
  if (announce_task_) {
    announce_task_->stop();
    announce_task_.reset();
  }
  if (heartbeat_task_) {
    heartbeat_task_->stop();
    heartbeat_task_.reset();
  }
  if (metatraffic_multicast_receiver_) {
    metatraffic_multicast_receiver_->stop_receiving();
    metatraffic_multicast_receiver_.reset();
  }
  if (metatraffic_unicast_receiver_) {
    metatraffic_unicast_receiver_->stop_receiving();
    metatraffic_unicast_receiver_.reset();
  }
  {
    std::lock_guard<std::mutex> receivers_lock(receivers_mutex_);
    for (auto &receiver : user_multicast_receivers_) {
      if (receiver.socket) {
        receiver.socket->stop_receiving();
      }
    }
    user_multicast_receivers_.clear();
  }
  if (user_unicast_receiver_) {
    user_unicast_receiver_->stop_receiving();
    user_unicast_receiver_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(reliable_mutex_);
    writer_reliable_states_.clear();
    reader_reliable_states_.clear();
    builtin_reader_states_.clear();
  }
}

bool RtpsParticipant::is_started() const { return started_.load(); }

bool RtpsParticipant::add_writer(const WriterConfig &writer_config) {
  std::lock_guard<std::mutex> lock(mutex_);
  writers_.push_back(writer_config);
  return true;
}

bool RtpsParticipant::add_reader(const ReaderConfig &reader_config) {
  // Bring up the reader's multicast receiver (if any) before persisting the reader, so a failure
  // does not leave the participant with a registered reader that has no working receiver.
  if (started_.load() && !reader_config.multicast_group.empty() &&
      !ensure_user_multicast_receivers_started(reader_config.multicast_group)) {
    logger_.error("Failed to start multicast receiver for topic '{}'", reader_config.topic_name);
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  readers_.push_back(reader_config);
  return true;
}

size_t RtpsParticipant::GuidHash::operator()(const Guid &guid) const {
  // FNV-1a over the 12-byte prefix + 4-byte entity id.
  uint64_t hash = 1469598103934665603ull;
  auto mix = [&hash](uint8_t byte) {
    hash ^= byte;
    hash *= 1099511628211ull;
  };
  for (auto byte : guid.prefix.value) {
    mix(byte);
  }
  for (auto byte : guid.entity_id.value) {
    mix(byte);
  }
  return static_cast<size_t>(hash);
}

RtpsParticipant::DiscoveryDb::UpsertResult<RtpsParticipant::ParticipantProxy>
RtpsParticipant::DiscoveryDb::upsert_participant(
    const Guid &participant_guid, const std::function<void(ParticipantProxy &)> &apply) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto [iterator, inserted] = participants_.try_emplace(participant_guid);
  apply(iterator->second); // merge only the fields present in this announcement
  return {.is_new = inserted, .value = iterator->second};
}

RtpsParticipant::DiscoveryDb::UpsertResult<RtpsParticipant::EndpointProxy>
RtpsParticipant::DiscoveryDb::upsert_endpoint(bool is_reader, const Guid &endpoint_guid,
                                              const std::function<void(EndpointProxy &)> &apply) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &endpoints = is_reader ? readers_ : writers_;
  auto [iterator, inserted] = endpoints.try_emplace(endpoint_guid);
  apply(iterator->second); // merge only the fields present in this announcement
  return {.is_new = inserted, .value = iterator->second};
}

std::optional<RtpsParticipant::ParticipantProxy>
RtpsParticipant::DiscoveryDb::find_participant_by_prefix(const GuidPrefix &prefix) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &[guid, participant] : participants_) {
    if (participant.guid_prefix == prefix) {
      return participant;
    }
  }
  return std::nullopt;
}

std::optional<RtpsParticipant::EndpointProxy>
RtpsParticipant::DiscoveryDb::find_writer(const Guid &guid) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto iterator = writers_.find(guid);
  if (iterator == writers_.end()) {
    return std::nullopt;
  }
  return iterator->second;
}

std::vector<RtpsParticipant::ParticipantProxy> RtpsParticipant::DiscoveryDb::participants() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ParticipantProxy> result;
  result.reserve(participants_.size());
  for (const auto &[guid, participant] : participants_) {
    result.push_back(participant);
  }
  return result;
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::DiscoveryDb::writers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<EndpointProxy> result;
  result.reserve(writers_.size());
  for (const auto &[guid, writer] : writers_) {
    result.push_back(writer);
  }
  return result;
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::DiscoveryDb::readers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<EndpointProxy> result;
  result.reserve(readers_.size());
  for (const auto &[guid, reader] : readers_) {
    result.push_back(reader);
  }
  return result;
}

void RtpsParticipant::DiscoveryDb::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  participants_.clear();
  writers_.clear();
  readers_.clear();
}

std::vector<RtpsParticipant::ParticipantProxy> RtpsParticipant::discovered_participants() const {
  return discovery_.participants();
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::discovered_writers() const {
  return discovery_.writers();
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::discovered_readers() const {
  return discovery_.readers();
}

std::vector<RtpsParticipant::WriterConfig> RtpsParticipant::writers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return writers_;
}

std::vector<RtpsParticipant::ReaderConfig> RtpsParticipant::readers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return readers_;
}

RtpsParticipant::PortMapping RtpsParticipant::ports() const {
  return compute_port_mapping(config_.domain_id, config_.participant_id);
}

RtpsParticipant::Guid RtpsParticipant::participant_guid() const {
  return {.prefix = guid_prefix_, .entity_id = {.value = kParticipantEntityId}};
}

RtpsParticipant::Guid RtpsParticipant::writer_guid(size_t index) const {
  return {.prefix = guid_prefix_,
          .entity_id = {
              .value = entity_id_for_index(static_cast<uint32_t>(index), kUserWriterNoKeyKind)}};
}

RtpsParticipant::Guid RtpsParticipant::reader_guid(size_t index) const {
  return {.prefix = guid_prefix_,
          .entity_id = {
              .value = entity_id_for_index(static_cast<uint32_t>(index), kUserReaderNoKeyKind)}};
}

std::vector<uint8_t> RtpsParticipant::build_announce_message() const {
  return build_spdp_announce_message();
}

std::vector<uint8_t> RtpsParticipant::build_spdp_announce_message() const {
  ByteWriter parameters;
  append_parameter_protocol_version(parameters, ProtocolVersion{});
  append_parameter_vendor_id(parameters, VendorId{});
  append_parameter_u32(parameters, ParameterId::PID_DOMAIN_ID, config_.domain_id);
  append_parameter_guid(parameters, ParameterId::PID_PARTICIPANT_GUID, participant_guid());
  append_parameter_locator(
      parameters, ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR,
      Locator::udp_v4(config_.metatraffic_multicast_group, ports().metatraffic_multicast));
  append_parameter_locator(
      parameters, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR,
      Locator::udp_v4(config_.advertised_address, ports().metatraffic_unicast));
  append_parameter_locator(parameters, ParameterId::PID_DEFAULT_UNICAST_LOCATOR,
                           Locator::udp_v4(config_.advertised_address, ports().user_unicast));
  if (config_.use_multicast_for_user_data) {
    append_parameter_locator(parameters, ParameterId::PID_DEFAULT_MULTICAST_LOCATOR,
                             Locator::udp_v4(config_.user_multicast_group, ports().user_multicast));
  }
  append_parameter_duration(parameters, ParameterId::PID_PARTICIPANT_LEASE_DURATION,
                            kDefaultLeaseDurationSeconds, kDefaultLeaseDurationNanoseconds);
  append_parameter_u32(parameters, ParameterId::PID_BUILTIN_ENDPOINT_SET, kBuiltinEndpointSet);
  std::string enclave_text = "enclave=" + config_.enclave + ";";
  append_parameter_octet_sequence(
      parameters, ParameterId::PID_USER_DATA,
      std::span<const uint8_t>{reinterpret_cast<const uint8_t *>(enclave_text.data()),
                               enclave_text.size()});
  append_parameter_string_cdr(parameters, ParameterId::PID_ENTITY_NAME, config_.node_name);
  append_parameter_sentinel(parameters);

  auto payload = build_parameter_list_payload(parameters);
  return build_message(guid_prefix_, {.value = kEntityIdUnknown}, {.value = kSpdpWriterEntityId},
                       next_spdp_sequence_number(), payload)
      .serialize();
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_publication_payload(const WriterConfig &writer_config) const {
  ByteWriter parameters;
  auto guid = writer_guid(writer_config.entity_index);
  append_parameter_guid(parameters, ParameterId::PID_ENDPOINT_GUID, guid);
  append_parameter_locator(parameters, ParameterId::PID_UNICAST_LOCATOR,
                           Locator::udp_v4(config_.advertised_address, ports().user_unicast));
  if (!writer_config.multicast_group.empty()) {
    append_parameter_locator(
        parameters, ParameterId::PID_MULTICAST_LOCATOR,
        Locator::udp_v4(writer_config.multicast_group, ports().user_multicast));
  }
  append_parameter_guid(parameters, ParameterId::PID_PARTICIPANT_GUID, participant_guid());
  append_parameter_string_cdr(parameters, ParameterId::PID_TOPIC_NAME, writer_config.topic_name);
  append_parameter_string_cdr(parameters, ParameterId::PID_TYPE_NAME, writer_config.type_name);
  append_parameter_key_hash(parameters, guid);
  append_parameter_u32(parameters, ParameterId::PID_TYPE_MAX_SIZE_SERIALIZED,
                       kUInt32SerializedSize);
  append_parameter_protocol_version(parameters, ProtocolVersion{});
  append_parameter_vendor_id(parameters, VendorId{});
  append_parameter_durability(parameters);
  append_parameter_liveliness(parameters);
  append_parameter_reliability(parameters, writer_config.reliability);
  append_parameter_history(parameters, writer_config.history_depth);
  append_parameter_sentinel(parameters);
  return build_parameter_list_payload(parameters);
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_publication_message(const WriterConfig &writer_config) const {
  return build_message(guid_prefix_, {.value = kSedpPublicationsReaderEntityId},
                       {.value = kSedpPublicationsWriterEntityId},
                       next_sedp_publication_sequence_number(),
                       build_sedp_publication_payload(writer_config))
      .serialize();
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_subscription_payload(const ReaderConfig &reader_config) const {
  ByteWriter parameters;
  auto guid = reader_guid(reader_config.entity_index);
  append_parameter_guid(parameters, ParameterId::PID_ENDPOINT_GUID, guid);
  append_parameter_locator(parameters, ParameterId::PID_UNICAST_LOCATOR,
                           Locator::udp_v4(config_.advertised_address, ports().user_unicast));
  if (!reader_config.multicast_group.empty()) {
    append_parameter_locator(
        parameters, ParameterId::PID_MULTICAST_LOCATOR,
        Locator::udp_v4(reader_config.multicast_group, ports().user_multicast));
  }
  append_parameter_bool(parameters, ParameterId::PID_EXPECTS_INLINE_QOS, false);
  append_parameter_guid(parameters, ParameterId::PID_PARTICIPANT_GUID, participant_guid());
  append_parameter_string_cdr(parameters, ParameterId::PID_TOPIC_NAME, reader_config.topic_name);
  append_parameter_string_cdr(parameters, ParameterId::PID_TYPE_NAME, reader_config.type_name);
  append_parameter_key_hash(parameters, guid);
  append_parameter_protocol_version(parameters, ProtocolVersion{});
  append_parameter_vendor_id(parameters, VendorId{});
  append_parameter_durability(parameters);
  append_parameter_liveliness(parameters);
  append_parameter_reliability(parameters, reader_config.reliability);
  append_parameter_history(parameters);
  append_parameter_sentinel(parameters);
  return build_parameter_list_payload(parameters);
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_subscription_message(const ReaderConfig &reader_config) const {
  return build_message(guid_prefix_, {.value = kSedpSubscriptionsReaderEntityId},
                       {.value = kSedpSubscriptionsWriterEntityId},
                       next_sedp_subscription_sequence_number(),
                       build_sedp_subscription_payload(reader_config))
      .serialize();
}

std::vector<uint8_t>
RtpsParticipant::build_data_message(const WriterConfig &writer_config,
                                    std::span<const uint8_t> cdr_payload) const {
  return build_data_message_with_sequence_number(
      writer_config, cdr_payload, next_user_data_sequence_number(writer_config.entity_index),
      EntityId{});
}

std::vector<uint8_t> RtpsParticipant::build_data_message_with_sequence_number(
    const WriterConfig &writer_config, std::span<const uint8_t> cdr_payload,
    int64_t sequence_number, EntityId reader_id) const {
  // Standard RTPS: the DATA submessage serializedPayload is exactly the CDR-encapsulated sample.
  // The topic is identified by the writer GUID (resolved by the receiver via SEDP discovery), so no
  // topic name or other framing is embedded in the payload. readerId addresses a specific matched
  // reader for unicast sends (ENTITYID_UNKNOWN for multicast, which targets all matched readers).
  auto guid = writer_guid(writer_config.entity_index);
  return build_message(guid_prefix_, reader_id, guid.entity_id, sequence_number, cdr_payload)
      .serialize();
}

void RtpsParticipant::store_reliable_sample(const WriterConfig &writer_config,
                                            int64_t sequence_number,
                                            std::span<const uint8_t> cdr_payload) {
  std::lock_guard<std::mutex> lock(reliable_mutex_);
  auto &state = writer_reliable_states_[writer_config.entity_index];
  state.history[sequence_number].assign(cdr_payload.begin(), cdr_payload.end());
  state.last_sequence_number = std::max(state.last_sequence_number, sequence_number);
  // Bound the cache to the configured KEEP_LAST depth, dropping the oldest samples.
  uint32_t depth = std::max<uint32_t>(writer_config.history_depth, 1);
  while (state.history.size() > depth) {
    state.history.erase(state.history.begin());
  }
}

bool RtpsParticipant::publish(std::string_view topic_name, std::span<const uint8_t> cdr_payload) {
  WriterConfig writer_config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iterator =
        std::find_if(writers_.begin(), writers_.end(),
                     [topic_name](const auto &writer) { return writer.topic_name == topic_name; });
    if (iterator == writers_.end()) {
      logger_.warn("No writer registered for topic '{}'", topic_name);
      return false;
    }
    writer_config = *iterator;
  }

  if (!user_unicast_receiver_) {
    return false;
  }

  // Allocate the sequence number once so a reliable writer can cache the sample under the same
  // sequence number it sends on the wire.
  int64_t sequence_number = next_user_data_sequence_number(writer_config.entity_index);
  const bool reliable = writer_config.reliability == ReliabilityKind::RELIABLE;
  if (reliable) {
    store_reliable_sample(writer_config, sequence_number, cdr_payload);
  }

  auto destinations = build_user_send_configs(topic_name, writer_config);
  if (destinations.empty()) {
    logger_.warn("No send destinations available for topic '{}'", topic_name);
    return false;
  }

  // Build the DATA per destination so each unicast send addresses its target reader's entity id
  // (multicast destinations use ENTITYID_UNKNOWN). The sequence number and payload are identical.
  bool sent = false;
  for (const auto &destination : destinations) {
    auto payload = build_data_message_with_sequence_number(writer_config, cdr_payload,
                                                           sequence_number, destination.reader_id);
    sent = user_unicast_receiver_->send(payload, destination.send_config) || sent;
  }

  // Reliable writers announce the available sequence-number range so matched reliable readers can
  // detect gaps and request retransmission (Phase 3). The retransmission response to ACKNACK is not
  // wired up yet, but the HEARTBEAT exchange is interoperable with DDS/ROS 2 peers.
  if (reliable) {
    send_heartbeat_for_writer(writer_config);
  }
  return sent;
}

bool RtpsParticipant::send_heartbeat_for_writer(const WriterConfig &writer_config) {
  if (!user_unicast_receiver_) {
    return false;
  }

  // Snapshot the available sequence-number range and bump the heartbeat count under
  // reliable_mutex_. An empty history is advertised as firstSN=1, lastSN=0 (no samples available).
  int64_t first_sn = 1;
  int64_t last_sn = 0;
  uint32_t count = 0;
  {
    std::lock_guard<std::mutex> lock(reliable_mutex_);
    auto &state = writer_reliable_states_[writer_config.entity_index];
    if (!state.history.empty()) {
      first_sn = state.history.begin()->first;
      last_sn = state.last_sequence_number;
    }
    count = ++state.heartbeat_count;
  }

  auto writer_entity_id = writer_guid(writer_config.entity_index).entity_id;

  // Snapshot the matched reliable readers for this topic.
  std::vector<EndpointProxy> matched_readers;
  for (auto &reader : discovery_.readers()) {
    if (reader.is_reader && reader.topic_name == writer_config.topic_name &&
        reader.reliability == ReliabilityKind::RELIABLE) {
      matched_readers.push_back(std::move(reader));
    }
  }

  bool sent = false;
  for (const auto &reader : matched_readers) {
    if (!has_valid_locator(reader.unicast_locator)) {
      continue;
    }
    Message message;
    message.header.guid_prefix = guid_prefix_;
    // INFO_DST tells the destination participant which entity the HEARTBEAT is for, so DDS/ROS 2
    // peers route it to the right reader.
    message.submessages.push_back(build_info_dst_submessage(reader.guid.prefix));
    message.submessages.push_back(build_heartbeat_submessage(
        reader.guid.entity_id, writer_entity_id, first_sn, last_sn, count, /*final=*/false));
    auto bytes = message.serialize();
    UdpSocket::SendConfig send_config{
        .ip_address = reader.unicast_locator.address_string(),
        .port = static_cast<uint16_t>(reader.unicast_locator.port),
        .is_multicast_endpoint = false,
    };
    sent = user_unicast_receiver_->send(bytes, send_config) || sent;
  }
  return sent;
}

bool RtpsParticipant::send_heartbeats_now() {
  std::vector<WriterConfig> reliable_writers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &writer : writers_) {
      if (writer.reliability == ReliabilityKind::RELIABLE) {
        reliable_writers.push_back(writer);
      }
    }
  }
  bool sent = false;
  for (const auto &writer : reliable_writers) {
    sent = send_heartbeat_for_writer(writer) || sent;
  }
  return sent;
}

void RtpsParticipant::deliver_reliable_sample(
    uint32_t reader_entity_index, const Guid &writer_guid, int64_t sequence_number,
    std::span<const uint8_t> payload,
    const std::function<void(std::span<const uint8_t>)> &on_sample) {
  // Collect the in-order samples to deliver under the lock, then invoke the callback after
  // releasing it (the callback may re-enter the participant, e.g. publish a response).
  std::vector<std::vector<uint8_t>> to_deliver;
  {
    std::lock_guard<std::mutex> lock(reliable_mutex_);
    auto key = fmt::format("{}#{}", reader_entity_index, writer_guid.to_string());
    auto &state = reader_reliable_states_[key];
    if (sequence_number <= state.highest_delivered) {
      // Duplicate (e.g. a retransmission of an already-delivered sample); drop it.
      return;
    }
    if (sequence_number == state.highest_delivered + 1) {
      // Next expected sample: deliver it, then drain any now-contiguous buffered samples.
      to_deliver.emplace_back(payload.begin(), payload.end());
      state.highest_delivered = sequence_number;
      while (true) {
        auto iterator = state.reorder.find(state.highest_delivered + 1);
        if (iterator == state.reorder.end()) {
          break;
        }
        to_deliver.push_back(std::move(iterator->second));
        state.reorder.erase(iterator);
        state.highest_delivered++;
      }
    } else if (state.reorder.find(sequence_number) == state.reorder.end() &&
               state.reorder.size() < config_.reliable_reorder_depth) {
      // Out of order: buffer for in-order delivery once the gap is filled. If the buffer is full
      // the sample is dropped and will be re-requested via the next ACKNACK.
      state.reorder[sequence_number].assign(payload.begin(), payload.end());
    }
  }
  for (const auto &sample : to_deliver) {
    on_sample(sample);
  }
}

void RtpsParticipant::send_acknack_for_heartbeat(const GuidPrefix &writer_prefix,
                                                 const EntityId &writer_id, int64_t first_sn,
                                                 int64_t last_sn, uint32_t heartbeat_count,
                                                 bool heartbeat_final) {
  if (!user_unicast_receiver_) {
    return;
  }
  Guid remote_writer_guid{.prefix = writer_prefix, .entity_id = writer_id};

  // Resolve the writer's topic + where to send the ACKNACK from discovery, then the matched
  // reliable local readers.
  auto writer = discovery_.find_writer(remote_writer_guid);
  if (!writer || writer->reliability != ReliabilityKind::RELIABLE) {
    return;
  }
  const Locator writer_locator = writer->unicast_locator;
  std::vector<uint32_t> matched_reader_indices;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &reader_config : readers_) {
      if (reader_config.topic_name == writer->topic_name &&
          reader_config.reliability == ReliabilityKind::RELIABLE) {
        matched_reader_indices.push_back(reader_config.entity_index);
      }
    }
  }
  if (matched_reader_indices.empty()) {
    return;
  }
  std::string participant_address;
  uint16_t participant_user_unicast = 0;
  if (auto participant = discovery_.find_participant_by_prefix(writer_prefix)) {
    participant_address = participant->address;
    participant_user_unicast = participant->ports.user_unicast;
  }

  // Prefer the writer's advertised unicast locator; fall back to the participant's user-unicast
  // endpoint (the raw datagram source port is the writer's ephemeral send port, not its receiver).
  std::string dest_address;
  uint16_t dest_port = 0;
  if (has_valid_locator(writer_locator)) {
    dest_address = writer_locator.address_string();
    dest_port = static_cast<uint16_t>(writer_locator.port);
  } else if (!participant_address.empty() && participant_user_unicast != 0) {
    dest_address = participant_address;
    dest_port = participant_user_unicast;
  } else {
    return;
  }

  for (uint32_t reader_entity_index : matched_reader_indices) {
    auto reader_entity_id = reader_guid(reader_entity_index).entity_id;
    SequenceNumberSet reader_sn_state;
    uint32_t count = 0;
    {
      std::lock_guard<std::mutex> lock(reliable_mutex_);
      auto key = fmt::format("{}#{}", reader_entity_index, remote_writer_guid.to_string());
      auto &state = reader_reliable_states_[key];
      // Samples below firstSN are no longer available from the writer (history purged); treat the
      // gap as permanently lost and skip past it so we do not NACK samples that no longer exist.
      if (first_sn > state.highest_delivered + 1) {
        logger_.warn("Reliable reader missed samples [{}, {}] from writer {} (writer history "
                     "advanced past them)",
                     state.highest_delivered + 1, first_sn - 1, remote_writer_guid.to_string());
        state.highest_delivered = first_sn - 1;
        while (!state.reorder.empty() && state.reorder.begin()->first <= state.highest_delivered) {
          state.reorder.erase(state.reorder.begin());
        }
      }
      // Ignore a stale heartbeat that does not require a response.
      if (heartbeat_count <= state.last_heartbeat_count && heartbeat_final) {
        continue;
      }
      state.last_heartbeat_count = std::max(state.last_heartbeat_count, heartbeat_count);

      int64_t base = state.highest_delivered + 1;
      reader_sn_state.base = base;
      bool any_missing = false;
      for (int64_t sn = base;
           sn <= last_sn && (sn - base) < static_cast<int64_t>(SequenceNumberSet::kMaxBits); sn++) {
        if (state.reorder.find(sn) == state.reorder.end()) {
          reader_sn_state.set(sn);
          any_missing = true;
        }
      }
      if (!any_missing) {
        // Positive acknowledgement of everything up to lastSN (empty set, base = next expected).
        reader_sn_state = SequenceNumberSet{};
        reader_sn_state.base = (last_sn >= base) ? last_sn + 1 : base;
      }
      count = ++state.acknack_count;
    }

    Message message;
    message.header.guid_prefix = guid_prefix_;
    message.submessages.push_back(build_info_dst_submessage(writer_prefix));
    message.submessages.push_back(build_acknack_submessage(reader_entity_id, writer_id,
                                                           reader_sn_state, count, /*final=*/true));
    auto bytes = message.serialize();
    UdpSocket::SendConfig send_config{
        .ip_address = dest_address,
        .port = dest_port,
        .is_multicast_endpoint = false,
    };
    user_unicast_receiver_->send(bytes, send_config);
  }
}

int64_t RtpsParticipant::next_spdp_sequence_number() const {
  return spdp_sequence_number_.fetch_add(1, std::memory_order_relaxed);
}

int64_t RtpsParticipant::next_sedp_publication_sequence_number() const {
  return sedp_publications_sequence_number_.fetch_add(1, std::memory_order_relaxed);
}

int64_t RtpsParticipant::next_sedp_subscription_sequence_number() const {
  return sedp_subscriptions_sequence_number_.fetch_add(1, std::memory_order_relaxed);
}

int64_t RtpsParticipant::next_user_data_sequence_number(uint32_t entity_index) const {
  std::lock_guard<std::mutex> lock(sequence_mutex_);
  auto iterator = user_data_sequence_numbers_.try_emplace(entity_index, 1).first;
  auto &sequence_number = iterator->second;
  int64_t current = sequence_number;
  sequence_number++;
  return current;
}

RtpsParticipant::PortMapping RtpsParticipant::compute_port_mapping(uint16_t domain_id,
                                                                   uint16_t participant_id) {
  auto base = static_cast<uint32_t>(kPortBase) + static_cast<uint32_t>(kDomainGain) * domain_id;
  auto participant_offset = static_cast<uint32_t>(kParticipantGain) * participant_id;
  return {.metatraffic_multicast = static_cast<uint16_t>(base + kMetatrafficMulticastOffset),
          .metatraffic_unicast =
              static_cast<uint16_t>(base + kMetatrafficUnicastOffset + participant_offset),
          .user_multicast = static_cast<uint16_t>(base + kUserMulticastOffset),
          .user_unicast = static_cast<uint16_t>(base + kUserUnicastOffset + participant_offset)};
}

void RtpsParticipant::record_builtin_sample(const Guid &writer_guid, int64_t sequence_number) {
  std::lock_guard<std::mutex> lock(reliable_mutex_);
  auto &state = builtin_reader_states_[writer_guid.to_string()];
  if (sequence_number <= state.highest_delivered) {
    return; // already accounted for
  }
  if (sequence_number == state.highest_delivered + 1) {
    state.highest_delivered = sequence_number;
    while (true) {
      auto iterator = state.reorder.find(state.highest_delivered + 1);
      if (iterator == state.reorder.end()) {
        break;
      }
      state.reorder.erase(iterator);
      state.highest_delivered++;
    }
  } else {
    state.reorder[sequence_number]; // mark this out-of-order SEDP sample as received (key only)
  }
}

void RtpsParticipant::send_builtin_acknack(const GuidPrefix &writer_prefix,
                                           const EntityId &writer_id, int64_t first_sn,
                                           int64_t last_sn, uint32_t heartbeat_count,
                                           bool /*heartbeat_final*/) {
  if (!metatraffic_unicast_receiver_) {
    return;
  }
  // Only the reliable builtin SEDP writers need an ACKNACK; map each to our matching builtin
  // reader.
  EntityId reader_id;
  if (writer_id.value == kSedpPublicationsWriterEntityId) {
    reader_id.value = kSedpPublicationsReaderEntityId;
  } else if (writer_id.value == kSedpSubscriptionsWriterEntityId) {
    reader_id.value = kSedpSubscriptionsReaderEntityId;
  } else {
    return; // SPDP is best-effort; other builtin writers are not tracked.
  }

  // Resolve where the peer's builtin SEDP reader receives traffic: its metatraffic unicast
  // endpoint.
  std::string dest_address;
  uint16_t dest_port = 0;
  if (auto participant = discovery_.find_participant_by_prefix(writer_prefix)) {
    if (has_valid_locator(participant->metatraffic_unicast_locator)) {
      dest_address = participant->metatraffic_unicast_locator.address_string();
      dest_port = static_cast<uint16_t>(participant->metatraffic_unicast_locator.port);
    } else {
      dest_address = participant->address;
      dest_port = participant->ports.metatraffic_unicast;
    }
  }
  if (dest_address.empty() || dest_port == 0) {
    return;
  }

  Guid writer_guid{.prefix = writer_prefix, .entity_id = writer_id};
  SequenceNumberSet reader_sn_state;
  uint32_t count = 0;
  bool any_missing = false;
  {
    std::lock_guard<std::mutex> lock(reliable_mutex_);
    auto &state = builtin_reader_states_[writer_guid.to_string()];
    if (first_sn > state.highest_delivered + 1) {
      // Samples below firstSN are no longer available from the writer; skip past them.
      state.highest_delivered = first_sn - 1;
      while (!state.reorder.empty() && state.reorder.begin()->first <= state.highest_delivered) {
        state.reorder.erase(state.reorder.begin());
      }
    }
    state.last_heartbeat_count = std::max(state.last_heartbeat_count, heartbeat_count);
    int64_t base = state.highest_delivered + 1;
    reader_sn_state.base = base;
    for (int64_t sn = base;
         sn <= last_sn && (sn - base) < static_cast<int64_t>(SequenceNumberSet::kMaxBits); sn++) {
      if (state.reorder.find(sn) == state.reorder.end()) {
        reader_sn_state.set(sn);
        any_missing = true;
      }
    }
    if (!any_missing) {
      // Positive ack of everything announced (empty set, base = next expected).
      reader_sn_state = SequenceNumberSet{};
      reader_sn_state.base = (last_sn >= base) ? last_sn + 1 : base;
    }
    count = ++state.acknack_count;
  }

  Message ack_message;
  ack_message.header.guid_prefix = guid_prefix_;
  ack_message.submessages.push_back(build_info_dst_submessage(writer_prefix));
  // Leave the Final flag unset while samples are still missing so the writer responds promptly.
  ack_message.submessages.push_back(build_acknack_submessage(reader_id, writer_id, reader_sn_state,
                                                             count, /*final=*/!any_missing));
  auto bytes = ack_message.serialize();
  UdpSocket::SendConfig send_config{.ip_address = dest_address, .port = dest_port};
  metatraffic_unicast_receiver_->send(bytes, send_config);
  logger_.debug("Sent builtin ACKNACK to {}:{} for writer {} (base={}, missing={}, count={})",
                dest_address, dest_port, writer_guid.to_string(), reader_sn_state.base, any_missing,
                count);
}

bool RtpsParticipant::handle_metatraffic_message(std::vector<uint8_t> &data,
                                                 const Socket::Info &sender) {
  auto message = Message::parse(data);
  if (!message) {
    return false;
  }

  for (const auto &submessage : message->submessages) {
    // Reliable peers (e.g. Fast DDS) heartbeat their builtin SEDP writers and will not (re)send
    // discovery data until they get an ACKNACK. Reply so we receive their endpoint announcements.
    if (submessage.kind == SubmessageKind::HEARTBEAT) {
      if (message->header.guid_prefix != guid_prefix_) {
        auto heartbeat = parse_heartbeat_submessage(submessage);
        if (heartbeat.valid) {
          send_builtin_acknack(message->header.guid_prefix, heartbeat.writer_id, heartbeat.first_sn,
                               heartbeat.last_sn, heartbeat.count, heartbeat.final);
        }
      }
      continue;
    }
    // A reliable peer ACKNACKs our builtin SEDP writers; resend any NACKed discovery samples.
    if (submessage.kind == SubmessageKind::ACKNACK) {
      if (message->header.guid_prefix != guid_prefix_) {
        auto acknack = parse_acknack_submessage(submessage);
        if (acknack.valid) {
          retransmit_sedp(message->header.guid_prefix, acknack.reader_id, acknack.writer_id,
                          requested_sequence_numbers(acknack.reader_sn_state));
        }
      }
      continue;
    }

    bool valid_data = false;
    auto data_view = parse_data_submessage(submessage, valid_data);
    if (!valid_data) {
      continue;
    }

    auto parameters = parse_parameter_list(data_view.serialized_payload);
    if (parameters.empty()) {
      continue;
    }

    if (data_view.writer_id.value == kSpdpWriterEntityId) {
      auto maybe_participant_guid_parameter =
          find_parameter(parameters, ParameterId::PID_PARTICIPANT_GUID);
      if (!maybe_participant_guid_parameter) {
        continue;
      }
      auto maybe_participant_guid = parse_guid(maybe_participant_guid_parameter->value);
      if (!maybe_participant_guid || is_same_guid_prefix(*maybe_participant_guid, guid_prefix_)) {
        continue;
      }

      // Parse only the fields actually present in this announcement; the upsert below merges them
      // into the existing record, so a later (trimmed) announcement does not erase what we learned.
      std::optional<std::string> name;
      if (auto parameter = find_parameter(parameters, ParameterId::PID_ENTITY_NAME)) {
        name = parse_cdr_string(parameter->value);
      }
      std::optional<std::string> enclave;
      if (auto parameter = find_parameter(parameters, ParameterId::PID_USER_DATA)) {
        if (auto user_data = parse_octet_sequence(parameter->value)) {
          enclave = extract_enclave(*user_data);
        }
      }
      std::optional<uint32_t> builtin_endpoints;
      if (auto parameter = find_parameter(parameters, ParameterId::PID_BUILTIN_ENDPOINT_SET)) {
        builtin_endpoints = parse_u32_le(parameter->value);
      }
      std::optional<Locator> meta_unicast;
      if (auto parameter =
              find_parameter(parameters, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR)) {
        meta_unicast = parse_locator(parameter->value);
      }
      std::optional<Locator> meta_multicast;
      if (auto parameter =
              find_parameter(parameters, ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR)) {
        meta_multicast = parse_locator(parameter->value);
      }
      std::optional<Locator> default_unicast;
      if (auto parameter = find_parameter(parameters, ParameterId::PID_DEFAULT_UNICAST_LOCATOR)) {
        default_unicast = parse_locator(parameter->value);
      }
      std::optional<Locator> default_multicast;
      if (auto parameter = find_parameter(parameters, ParameterId::PID_DEFAULT_MULTICAST_LOCATOR)) {
        default_multicast = parse_locator(parameter->value);
      }
      const std::string sender_address = sender.address;

      auto result = discovery_.upsert_participant(
          *maybe_participant_guid, [&](ParticipantProxy &participant) {
            participant.participant_guid = *maybe_participant_guid;
            participant.guid_prefix = maybe_participant_guid->prefix;
            if (participant.address.empty()) {
              participant.address = sender_address;
            }
            if (name) {
              participant.name = *name;
            }
            if (enclave) {
              participant.enclave = *enclave;
            }
            if (builtin_endpoints) {
              participant.builtin_endpoints = *builtin_endpoints;
            }
            if (meta_unicast) {
              participant.ports.metatraffic_unicast = static_cast<uint16_t>(meta_unicast->port);
              if (has_valid_locator(*meta_unicast)) {
                participant.metatraffic_unicast_locator = *meta_unicast;
              }
            }
            if (meta_multicast) {
              participant.ports.metatraffic_multicast = static_cast<uint16_t>(meta_multicast->port);
              if (has_valid_locator(*meta_multicast)) {
                participant.metatraffic_multicast_locator = *meta_multicast;
              }
            }
            if (default_unicast) {
              participant.ports.user_unicast = static_cast<uint16_t>(default_unicast->port);
              if (has_valid_locator(*default_unicast)) {
                participant.default_unicast_locator = *default_unicast;
                participant.address = default_unicast->address_string();
              }
            }
            if (default_multicast) {
              participant.ports.user_multicast = static_cast<uint16_t>(default_multicast->port);
              if (has_valid_locator(*default_multicast)) {
                participant.default_multicast_locator = *default_multicast;
              }
            }
          });

      logger_.debug("SPDP parsed participant {} from src {}: meta_uc={} meta_mc={} default_uc={} "
                    "default_mc={} -> stored(address={}, ports.meta_uc={}, ports.user_uc={})",
                    maybe_participant_guid->prefix.to_string(), sender_address,
                    locator_to_string(meta_unicast), locator_to_string(meta_multicast),
                    locator_to_string(default_unicast), locator_to_string(default_multicast),
                    result.value.address, result.value.ports.metatraffic_unicast,
                    result.value.ports.user_unicast);

      if (result.is_new) {
        const auto &participant = result.value;
        logger_.info("SPDP discovered participant '{}' at {} (meta {}, user {})",
                     participant.name.empty() ? participant.guid_prefix.to_string()
                                              : participant.name,
                     participant.address, participant.ports.metatraffic_unicast,
                     participant.ports.user_unicast);
        send_sedp_announcements_to(participant);
        if (config_.on_participant_discovered) {
          config_.on_participant_discovered(participant);
        }
      }
      continue;
    }

    bool is_reader = false;
    if (data_view.writer_id.value == kSedpPublicationsWriterEntityId) {
      is_reader = false;
    } else if (data_view.writer_id.value == kSedpSubscriptionsWriterEntityId) {
      is_reader = true;
    } else {
      continue;
    }

    // Record the SEDP sample so we can correctly ACKNACK the peer's builtin SEDP heartbeats.
    record_builtin_sample({.prefix = message->header.guid_prefix, .entity_id = data_view.writer_id},
                          data_view.writer_sn);

    auto maybe_endpoint_guid_parameter = find_parameter(parameters, ParameterId::PID_ENDPOINT_GUID);
    if (!maybe_endpoint_guid_parameter) {
      continue;
    }
    auto maybe_endpoint_guid = parse_guid(maybe_endpoint_guid_parameter->value);
    if (!maybe_endpoint_guid || is_same_guid_prefix(*maybe_endpoint_guid, guid_prefix_)) {
      continue;
    }

    // Parse only the present fields; the upsert below merges them into the existing endpoint.
    const Guid endpoint_guid = *maybe_endpoint_guid;
    std::optional<Guid> endpoint_participant_guid;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_PARTICIPANT_GUID)) {
      endpoint_participant_guid = parse_guid(parameter->value);
    }
    std::optional<std::string> topic_name;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_TOPIC_NAME)) {
      topic_name = parse_cdr_string(parameter->value);
    }
    std::optional<std::string> type_name;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_TYPE_NAME)) {
      type_name = parse_cdr_string(parameter->value);
    }
    std::optional<Locator> unicast_locator;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_UNICAST_LOCATOR)) {
      unicast_locator = parse_locator(parameter->value);
    }
    std::vector<Locator> multicast_locators;
    for (const auto &locator_parameter :
         find_parameters(parameters, ParameterId::PID_MULTICAST_LOCATOR)) {
      if (auto parsed = parse_locator(locator_parameter.value)) {
        multicast_locators.push_back(*parsed);
      }
    }
    std::optional<ReliabilityKind> reliability;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_RELIABILITY)) {
      reliability = parse_reliability(parameter->value);
    }
    std::optional<bool> expects_inline_qos;
    if (auto parameter = find_parameter(parameters, ParameterId::PID_EXPECTS_INLINE_QOS)) {
      expects_inline_qos = parse_bool(parameter->value);
    }

    auto result =
        discovery_.upsert_endpoint(is_reader, endpoint_guid, [&](EndpointProxy &endpoint) {
          endpoint.guid = endpoint_guid;
          endpoint.is_reader = is_reader;
          if (endpoint_participant_guid) {
            endpoint.participant_guid = *endpoint_participant_guid;
          }
          if (endpoint.participant_guid.entity_id.value == std::array<uint8_t, 4>{}) {
            // Fall back to the implicit participant entity within the endpoint's prefix.
            endpoint.participant_guid = {.prefix = endpoint_guid.prefix,
                                         .entity_id = {.value = kParticipantEntityId}};
          }
          if (topic_name) {
            endpoint.topic_name = *topic_name;
          }
          if (type_name) {
            endpoint.type_name = *type_name;
          }
          if (unicast_locator && has_valid_locator(*unicast_locator)) {
            endpoint.unicast_locator = *unicast_locator;
          }
          if (!multicast_locators.empty()) {
            endpoint.multicast_locators = multicast_locators;
          }
          if (reliability) {
            endpoint.reliability = *reliability;
          }
          if (expects_inline_qos) {
            endpoint.expects_inline_qos = *expects_inline_qos;
          }
        });

    logger_.debug(
        "SEDP parsed {} {} topic '{}' [{}] reliability={} unicast={} multicast_count={} -> "
        "participant {}",
        is_reader ? "reader" : "writer", endpoint_guid.to_string(), result.value.topic_name,
        result.value.type_name,
        result.value.reliability == ReliabilityKind::RELIABLE ? "RELIABLE" : "BEST_EFFORT",
        locator_to_string(result.value.unicast_locator), result.value.multicast_locators.size(),
        result.value.participant_guid.to_string());

    if (result.is_new) {
      const auto &endpoint = result.value;
      logger_.info("SEDP discovered {} '{}' [{}] from participant {}",
                   endpoint.is_reader ? "reader" : "writer", endpoint.topic_name,
                   endpoint.type_name, endpoint.participant_guid.to_string());
      if (config_.on_endpoint_discovered) {
        config_.on_endpoint_discovered(endpoint);
      }
    }
  }
  return false;
}

bool RtpsParticipant::handle_user_message(std::vector<uint8_t> &data, const Socket::Info &sender) {
  auto message = Message::parse(data);
  if (!message) {
    return false;
  }
  if (message->header.guid_prefix == guid_prefix_) {
    return false;
  }

  for (const auto &submessage : message->submessages) {
    // Reliable-QoS submessages (Phase 0: parsed and logged; the writer/reader
    // state machines that act on them land in later phases — see
    // RELIABLE_RTPS_PLAN.md).
    if (submessage.kind == SubmessageKind::HEARTBEAT) {
      auto heartbeat = parse_heartbeat_submessage(submessage);
      if (heartbeat.valid) {
        logger_.debug("HEARTBEAT from {} (writer {}, reader {}): firstSN={} lastSN={} count={} "
                      "final={}",
                      message->header.guid_prefix.to_string(), heartbeat.writer_id.to_string(),
                      heartbeat.reader_id.to_string(), heartbeat.first_sn, heartbeat.last_sn,
                      heartbeat.count, heartbeat.final);
        // A matched reliable reader replies with an ACKNACK of the missing sequence numbers in
        // [firstSN, lastSN] so the writer can retransmit them.
        send_acknack_for_heartbeat(message->header.guid_prefix, heartbeat.writer_id,
                                   heartbeat.first_sn, heartbeat.last_sn, heartbeat.count,
                                   heartbeat.final);
      }
      continue;
    }
    if (submessage.kind == SubmessageKind::ACKNACK) {
      auto acknack = parse_acknack_submessage(submessage);
      if (acknack.valid) {
        logger_.debug(
            "ACKNACK from {} (writer {}, reader {}): base={} numBits={} count={} final={}",
            message->header.guid_prefix.to_string(), acknack.writer_id.to_string(),
            acknack.reader_id.to_string(), acknack.reader_sn_state.base,
            acknack.reader_sn_state.num_bits, acknack.count, acknack.final);
        // A matched reliable writer resends the NACKed sequence numbers from its history.
        retransmit_user_data(message->header.guid_prefix, acknack.reader_id, acknack.writer_id,
                             requested_sequence_numbers(acknack.reader_sn_state));
      }
      continue;
    }

    bool valid_data = false;
    auto data_view = parse_data_submessage(submessage, valid_data);
    if (!valid_data) {
      continue;
    }

    // Standard RTPS: the sample's topic is identified by the writer GUID, which we resolve through
    // SEDP discovery state. Build the remote writer GUID from the message prefix + DATA writerId,
    // then look up its topic and reliability among discovered writers, and collect the matching
    // local reader callbacks under a single lock.
    Guid remote_writer_guid{.prefix = message->header.guid_prefix,
                            .entity_id = data_view.writer_id};
    struct MatchedReader {
      uint32_t entity_index{0};
      bool reliable{false};
      std::function<void(std::span<const uint8_t>)> on_sample{};
    };
    auto writer = discovery_.find_writer(remote_writer_guid);
    if (!writer) {
      // Sample arrived before the writer was discovered via SEDP; drop it (best-effort).
      logger_.debug("Received DATA for unknown writer {} from {} (not discovered via SEDP yet)",
                    remote_writer_guid.to_string(), sender.address);
      continue;
    }
    const bool writer_is_reliable = writer->reliability == ReliabilityKind::RELIABLE;
    std::vector<MatchedReader> matched_readers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &reader_config : readers_) {
        if (reader_config.topic_name == writer->topic_name && reader_config.on_sample) {
          matched_readers.push_back(
              {.entity_index = reader_config.entity_index,
               .reliable = reader_config.reliability == ReliabilityKind::RELIABLE,
               .on_sample = reader_config.on_sample});
        }
      }
    }

    if (matched_readers.empty()) {
      continue;
    }
    // The reliable handshake (dedup + in-order delivery) runs only between endpoints that both
    // advertise RELIABLE; otherwise the sample is delivered best-effort (immediately, as received).
    for (const auto &reader : matched_readers) {
      if (writer_is_reliable && reader.reliable) {
        deliver_reliable_sample(reader.entity_index, remote_writer_guid, data_view.writer_sn,
                                data_view.serialized_payload, reader.on_sample);
      } else {
        reader.on_sample(data_view.serialized_payload);
      }
    }
  }
  return false;
}

bool RtpsParticipant::ensure_user_multicast_receivers_started(const std::string &extra_group) {
  if (!started_.load()) {
    return true;
  }

  std::vector<std::string> desired_groups;
  auto add_group = [&desired_groups](const std::string &group) {
    if (!group.empty() &&
        std::find(desired_groups.begin(), desired_groups.end(), group) == desired_groups.end()) {
      desired_groups.push_back(group);
    }
  };
  if (config_.use_multicast_for_user_data) {
    add_group(config_.user_multicast_group);
  }
  // Include the group of a reader being added before it is persisted in readers_, so the receiver
  // can be brought up (and any failure surfaced) without leaving the reader half-registered.
  add_group(extra_group);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &reader_config : readers_) {
      add_group(reader_config.multicast_group);
    }
  }

  auto port_mapping = ports();
  // receivers_mutex_ (not mutex_) guards user_multicast_receivers_; desired_groups was built above
  // under mutex_, which has already been released, so the two locks are never held nested.
  std::lock_guard<std::mutex> receivers_lock(receivers_mutex_);
  for (const auto &group : desired_groups) {
    auto existing =
        std::find_if(user_multicast_receivers_.begin(), user_multicast_receivers_.end(),
                     [&group](const auto &receiver) { return receiver.multicast_group == group; });
    if (existing != user_multicast_receivers_.end()) {
      continue;
    }

    auto socket =
        std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = config_.socket_log_level});
    auto task_config = config_.receive_task_config;
    task_config.name = fmt::format("{}_user_mc_{}", config_.receive_task_config.name,
                                   user_multicast_receivers_.size());
    auto receive_config = UdpSocket::ReceiveConfig{
        .port = port_mapping.user_multicast,
        .buffer_size = 4096,
        .is_multicast_endpoint = true,
        .multicast_group = group,
        .multicast_interface = config_.bind_address,
        .on_receive_callback = [this](auto &data,
                                      const auto &sender) -> std::optional<std::vector<uint8_t>> {
          handle_user_message(data, sender);
          return std::nullopt;
        },
    };
    if (!socket->start_receiving(task_config, receive_config)) {
      logger_.error("Failed to start user multicast receiver for group {}", group);
      return false;
    }
    user_multicast_receivers_.push_back({
        .multicast_group = group,
        .socket = std::move(socket),
    });
  }
  return true;
}

std::vector<RtpsParticipant::UserDataDestination>
RtpsParticipant::build_user_send_configs(std::string_view topic_name,
                                         const WriterConfig &writer_config) const {
  std::vector<UserDataDestination> destinations;
  const std::string &multicast_interface = config_.bind_address;
  auto add_destination = [&destinations, &multicast_interface](std::string ip_address,
                                                               uint16_t port, bool is_multicast,
                                                               const EntityId &reader_id) {
    if (ip_address.empty() || port == 0) {
      return;
    }
    auto existing = std::find_if(
        destinations.begin(), destinations.end(), [&](const UserDataDestination &destination) {
          return destination.send_config.ip_address == ip_address &&
                 destination.send_config.port == port &&
                 destination.send_config.is_multicast_endpoint == is_multicast &&
                 destination.reader_id == reader_id;
        });
    if (existing == destinations.end()) {
      destinations.push_back(
          {.send_config = {.ip_address = std::move(ip_address),
                           .port = port,
                           .is_multicast_endpoint = is_multicast,
                           .multicast_interface =
                               is_multicast ? multicast_interface : std::string{}},
           .reader_id = reader_id});
    }
  };

  // Multicast sends target all matched readers, so the DATA readerId is left as ENTITYID_UNKNOWN.
  if (!writer_config.multicast_group.empty()) {
    add_destination(writer_config.multicast_group, ports().user_multicast, true, EntityId{});
    return destinations;
  }

  if (config_.use_multicast_for_user_data) {
    add_destination(config_.user_multicast_group, ports().user_multicast, true, EntityId{});
    return destinations;
  }

  for (const auto &reader : discovery_.readers()) {
    if (!reader.is_reader || reader.topic_name != topic_name) {
      continue;
    }

    bool used_multicast = false;
    for (const auto &locator : reader.multicast_locators) {
      if (!has_valid_locator(locator)) {
        continue;
      }
      // Multicast to a reader's group: addressed to all readers in the group (readerId UNKNOWN).
      add_destination(locator.address_string(), static_cast<uint16_t>(locator.port), true,
                      EntityId{});
      used_multicast = true;
    }
    if (used_multicast) {
      continue;
    }

    // Unicast to a specific reader: address the DATA to that reader's entity id.
    if (has_valid_locator(reader.unicast_locator)) {
      add_destination(reader.unicast_locator.address_string(),
                      static_cast<uint16_t>(reader.unicast_locator.port), false,
                      reader.guid.entity_id);
      continue;
    }

    if (auto participant = discovery_.find_participant_by_prefix(reader.participant_guid.prefix)) {
      add_destination(participant->address, participant->ports.user_unicast, false,
                      reader.guid.entity_id);
    }
  }

  return destinations;
}

bool RtpsParticipant::send_spdp_announce_now() {
  if (!metatraffic_unicast_receiver_) {
    return false;
  }
  auto payload = build_spdp_announce_message();
  auto send_config = UdpSocket::SendConfig{
      .ip_address = config_.metatraffic_multicast_group,
      .port = ports().metatraffic_multicast,
      .is_multicast_endpoint = true,
      .multicast_interface = config_.bind_address,
  };
  return metatraffic_unicast_receiver_->send(payload, send_config);
}

bool RtpsParticipant::send_sedp_announcements_to(const ParticipantProxy &participant) {
  // Prefer the peer's advertised metatraffic unicast locator (its full address + port); fall back
  // to participant.address (the SPDP source / default-unicast address) + the derived metatraffic
  // port.
  std::string meta_address;
  uint16_t meta_port = 0;
  const char *address_source = "metatraffic_unicast_locator";
  if (has_valid_locator(participant.metatraffic_unicast_locator)) {
    meta_address = participant.metatraffic_unicast_locator.address_string();
    meta_port = static_cast<uint16_t>(participant.metatraffic_unicast_locator.port);
  } else {
    meta_address = participant.address;
    meta_port = participant.ports.metatraffic_unicast;
    address_source = "participant.address fallback";
  }
  logger_.debug(
      "send_sedp -> participant {} dest {}:{} (via {}) [meta_uc_loc={}, default_uc_loc={}, "
      "participant.address={}, ports.meta_uc={}]",
      participant.guid_prefix.to_string(), meta_address, meta_port, address_source,
      locator_to_string(participant.metatraffic_unicast_locator),
      locator_to_string(participant.default_unicast_locator), participant.address,
      participant.ports.metatraffic_unicast);
  if (!metatraffic_unicast_receiver_ || meta_port == 0 || meta_address.empty()) {
    logger_.warn("send_sedp: no usable metatraffic destination for participant {} "
                 "(address '{}', port {})",
                 participant.guid_prefix.to_string(), meta_address, meta_port);
    return false;
  }

  bool sent = false;
  std::vector<WriterConfig> local_writers;
  std::vector<ReaderConfig> local_readers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_writers = writers_;
    local_readers = readers_;
  }

  // Each local endpoint is a stable SEDP sample: writer/reader at index i has sequence number i+1.
  // Re-announcing resends the same sample under the same SN, so the SEDP HEARTBEAT range stays
  // meaningful and a reliable peer can detect/recover a missed announcement.
  const UdpSocket::SendConfig send_config{.ip_address = meta_address, .port = meta_port};
  for (size_t i = 0; i < local_writers.size(); i++) {
    auto payload =
        build_message(guid_prefix_, {.value = kSedpPublicationsReaderEntityId},
                      {.value = kSedpPublicationsWriterEntityId}, static_cast<int64_t>(i + 1),
                      build_sedp_publication_payload(local_writers[i]))
            .serialize();
    sent = metatraffic_unicast_receiver_->send(payload, send_config) || sent;
  }
  for (size_t i = 0; i < local_readers.size(); i++) {
    auto payload =
        build_message(guid_prefix_, {.value = kSedpSubscriptionsReaderEntityId},
                      {.value = kSedpSubscriptionsWriterEntityId}, static_cast<int64_t>(i + 1),
                      build_sedp_subscription_payload(local_readers[i]))
            .serialize();
    sent = metatraffic_unicast_receiver_->send(payload, send_config) || sent;
  }
  // HEARTBEAT our builtin SEDP writers so the peer's reliable SEDP readers ACKNACK (which lets us
  // detect and retransmit a missed announcement on a lossy link).
  send_sedp_heartbeats_to(meta_address, meta_port, participant.guid_prefix, local_writers.size(),
                          local_readers.size());
  return sent;
}

void RtpsParticipant::send_sedp_heartbeats_to(const std::string &dest_address, uint16_t dest_port,
                                              const GuidPrefix &dest_prefix, size_t writer_count,
                                              size_t reader_count) {
  if (!metatraffic_unicast_receiver_ || dest_address.empty() || dest_port == 0) {
    return;
  }
  const UdpSocket::SendConfig send_config{.ip_address = dest_address, .port = dest_port};
  auto emit = [&](const std::array<uint8_t, 4> &reader_eid,
                  const std::array<uint8_t, 4> &writer_eid, size_t count,
                  std::atomic<uint32_t> &heartbeat_count) {
    if (count == 0) {
      return; // nothing announced on this builtin writer yet
    }
    Message message;
    message.header.guid_prefix = guid_prefix_;
    message.submessages.push_back(build_info_dst_submessage(dest_prefix));
    message.submessages.push_back(build_heartbeat_submessage(
        {.value = reader_eid}, {.value = writer_eid}, /*first_sn=*/1,
        /*last_sn=*/static_cast<int64_t>(count), ++heartbeat_count, /*final=*/false));
    metatraffic_unicast_receiver_->send(message.serialize(), send_config);
  };
  emit(kSedpPublicationsReaderEntityId, kSedpPublicationsWriterEntityId, writer_count,
       sedp_pub_heartbeat_count_);
  emit(kSedpSubscriptionsReaderEntityId, kSedpSubscriptionsWriterEntityId, reader_count,
       sedp_sub_heartbeat_count_);
}

std::vector<uint8_t>
RtpsParticipant::build_directed_data_message(const GuidPrefix &dest_prefix, EntityId reader_id,
                                             EntityId writer_id, int64_t sequence_number,
                                             std::span<const uint8_t> serialized_payload) const {
  Message message;
  message.header.guid_prefix = guid_prefix_;
  message.submessages.push_back(build_info_dst_submessage(dest_prefix));
  message.submessages.push_back(
      {.kind = SubmessageKind::DATA,
       .flags = static_cast<uint8_t>(kSubmessageFlagLittleEndian | kSubmessageFlagData),
       .payload = build_data_submessage_payload(reader_id, writer_id, sequence_number,
                                                serialized_payload)});
  return message.serialize();
}

void RtpsParticipant::retransmit_user_data(const GuidPrefix &reader_prefix, EntityId reader_id,
                                           EntityId writer_id,
                                           const std::vector<int64_t> &requested_sequence_numbers) {
  if (!user_unicast_receiver_ || requested_sequence_numbers.empty()) {
    return;
  }
  // Find the local writer this ACKNACK targets (by its entity id) -> its reliable history slot.
  std::optional<uint32_t> writer_entity_index;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &writer_config : writers_) {
      if (writer_guid(writer_config.entity_index).entity_id == writer_id) {
        writer_entity_index = writer_config.entity_index;
        break;
      }
    }
  }
  if (!writer_entity_index) {
    return;
  }

  // Resolve where to send: the requesting reader's unicast locator, else its participant's user
  // unicast endpoint.
  const Guid requesting_reader_guid{.prefix = reader_prefix, .entity_id = reader_id};
  std::string dest_address;
  uint16_t dest_port = 0;
  for (const auto &reader : discovery_.readers()) {
    if (reader.guid == requesting_reader_guid && has_valid_locator(reader.unicast_locator)) {
      dest_address = reader.unicast_locator.address_string();
      dest_port = static_cast<uint16_t>(reader.unicast_locator.port);
      break;
    }
  }
  if (dest_address.empty()) {
    if (auto participant = discovery_.find_participant_by_prefix(reader_prefix)) {
      dest_address = participant->address;
      dest_port = participant->ports.user_unicast;
    }
  }
  if (dest_address.empty() || dest_port == 0) {
    return;
  }

  // Collect the requested samples still in history (copied under the lock; sent after releasing
  // it).
  std::vector<std::pair<int64_t, std::vector<uint8_t>>> samples;
  {
    std::lock_guard<std::mutex> lock(reliable_mutex_);
    auto state = writer_reliable_states_.find(*writer_entity_index);
    if (state == writer_reliable_states_.end()) {
      return;
    }
    for (int64_t sequence_number : requested_sequence_numbers) {
      auto entry = state->second.history.find(sequence_number);
      if (entry != state->second.history.end()) {
        samples.emplace_back(sequence_number, entry->second);
      }
    }
  }

  const UdpSocket::SendConfig send_config{.ip_address = dest_address, .port = dest_port};
  for (const auto &[sequence_number, payload] : samples) {
    auto bytes =
        build_directed_data_message(reader_prefix, reader_id, writer_id, sequence_number, payload);
    user_unicast_receiver_->send(bytes, send_config);
  }
  if (!samples.empty()) {
    logger_.debug("Retransmitted {} user-data sample(s) to {}:{} (writer {}, reader {})",
                  samples.size(), dest_address, dest_port, writer_id.to_string(),
                  reader_id.to_string());
  }
}

void RtpsParticipant::retransmit_sedp(const GuidPrefix &reader_prefix, EntityId reader_id,
                                      EntityId writer_id,
                                      const std::vector<int64_t> &requested_sequence_numbers) {
  if (!metatraffic_unicast_receiver_ || requested_sequence_numbers.empty()) {
    return;
  }
  const bool is_publications = writer_id.value == kSedpPublicationsWriterEntityId;
  const bool is_subscriptions = writer_id.value == kSedpSubscriptionsWriterEntityId;
  if (!is_publications && !is_subscriptions) {
    return; // not one of our builtin SEDP writers
  }

  std::string dest_address;
  uint16_t dest_port = 0;
  if (auto participant = discovery_.find_participant_by_prefix(reader_prefix)) {
    if (has_valid_locator(participant->metatraffic_unicast_locator)) {
      dest_address = participant->metatraffic_unicast_locator.address_string();
      dest_port = static_cast<uint16_t>(participant->metatraffic_unicast_locator.port);
    } else {
      dest_address = participant->address;
      dest_port = participant->ports.metatraffic_unicast;
    }
  }
  if (dest_address.empty() || dest_port == 0) {
    return;
  }

  std::vector<WriterConfig> local_writers;
  std::vector<ReaderConfig> local_readers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_writers = writers_;
    local_readers = readers_;
  }

  // Each SEDP sample's sequence number is the 1-based index of the local endpoint, so it is rebuilt
  // deterministically from writers_/readers_ (no separate SEDP history cache needed).
  const UdpSocket::SendConfig send_config{.ip_address = dest_address, .port = dest_port};
  size_t retransmitted = 0;
  for (int64_t sequence_number : requested_sequence_numbers) {
    if (sequence_number < 1) {
      continue;
    }
    const auto index = static_cast<size_t>(sequence_number - 1);
    std::vector<uint8_t> payload;
    if (is_publications) {
      if (index >= local_writers.size()) {
        continue;
      }
      payload = build_sedp_publication_payload(local_writers[index]);
    } else {
      if (index >= local_readers.size()) {
        continue;
      }
      payload = build_sedp_subscription_payload(local_readers[index]);
    }
    auto bytes =
        build_directed_data_message(reader_prefix, reader_id, writer_id, sequence_number, payload);
    metatraffic_unicast_receiver_->send(bytes, send_config);
    retransmitted++;
  }
  if (retransmitted > 0) {
    logger_.debug("Retransmitted {} SEDP sample(s) to {}:{} (writer {})", retransmitted,
                  dest_address, dest_port, writer_id.to_string());
  }
}

bool RtpsParticipant::send_discovery_now() {
  auto participants = discovered_participants();
  return std::accumulate(participants.begin(), participants.end(), send_spdp_announce_now(),
                         [this](bool sent, const auto &participant) {
                           return send_sedp_announcements_to(participant) || sent;
                         });
}

} // namespace espp
