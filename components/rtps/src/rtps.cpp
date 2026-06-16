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
constexpr std::array<char, 8> kUserDataMagic{'E', 'S', 'P', 'P', 'D', 'A', 'T', 'A'};
constexpr uint8_t kUserDataVersion = 1;

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
    kBuiltinEndpointSubscriptionAnnouncer | kBuiltinEndpointSubscriptionDetector |
    kBuiltinEndpointParticipantMessageWriter | kBuiltinEndpointParticipantMessageReader;

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

  void append_u16_be(uint16_t value) {
    data_.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    data_.push_back(static_cast<uint8_t>(value & 0xff));
  }

  void append_u32_le(uint32_t value) {
    for (int i = 0; i < 4; i++) {
      data_.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xff));
    }
  }

  void append_i32_le(int32_t value) { append_u32_le(static_cast<uint32_t>(value)); }

  void append_u32_be(uint32_t value) {
    for (int i = 3; i >= 0; i--) {
      data_.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xff));
    }
  }

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

  bool read_sequence_number_le(int64_t &value) {
    int32_t high = 0;
    uint32_t low = 0;
    if (!read_i32_le(high) || !read_u32_le(low)) {
      return false;
    }
    value = (static_cast<int64_t>(high) << 32) | low;
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

void append_string(ByteWriter &writer, std::string_view text) {
  writer.append_u16_le(static_cast<uint16_t>(text.size()));
  writer.append_bytes(
      std::span<const uint8_t>{reinterpret_cast<const uint8_t *>(text.data()), text.size()});
}

std::optional<std::string> read_string(ByteReader &reader) {
  uint16_t length = 0;
  if (!reader.read_u16_le(length)) {
    return std::nullopt;
  }
  auto span = reader.read_span(length);
  if (span.size() != length) {
    return std::nullopt;
  }
  return std::string(reinterpret_cast<const char *>(span.data()), span.size());
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

void append_parameter_duration(ByteWriter &writer, ParameterId id, int32_t seconds,
                               uint32_t nanoseconds) {
  append_parameter_header(writer, id, 8);
  writer.append_i32_le(seconds);
  writer.append_u32_le(nanoseconds);
}

void append_parameter_locator(ByteWriter &writer, ParameterId id,
                              const espp::RtpsParticipant::Locator &locator) {
  append_parameter_header(writer, id, 24);
  writer.append_u32_be(static_cast<uint32_t>(locator.kind));
  writer.append_u32_be(locator.port);
  writer.append_bytes(locator.address);
}

void append_parameter_string_cdr(ByteWriter &writer, ParameterId id, std::string_view text) {
  uint16_t raw_length = static_cast<uint16_t>(4 + text.size() + 1);
  append_parameter_header(writer, id, raw_length);
  auto cdr_writer = espp::CdrWriter::make_body_writer(espp::CdrEncapsulation::CDR_LE);
  cdr_writer.write_string(text);
  writer.append_bytes(cdr_writer.payload());
}

void append_parameter_octet_sequence(ByteWriter &writer, ParameterId id,
                                     std::span<const uint8_t> bytes) {
  uint16_t raw_length = static_cast<uint16_t>(4 + bytes.size());
  append_parameter_header(writer, id, raw_length);
  auto cdr_writer = espp::CdrWriter::make_body_writer(espp::CdrEncapsulation::CDR_LE);
  cdr_writer.write<uint32_t>(static_cast<uint32_t>(bytes.size()));
  cdr_writer.write_bytes(bytes);
  cdr_writer.align(4);
  writer.append_bytes(cdr_writer.payload());
}

void append_parameter_reliability(ByteWriter &writer,
                                  espp::RtpsParticipant::ReliabilityKind reliability) {
  append_parameter_header(writer, ParameterId::PID_RELIABILITY, 12);
  writer.append_u32_le(reliability == espp::RtpsParticipant::ReliabilityKind::RELIABLE
                           ? kReliabilityReliable
                           : kReliabilityBestEffort);
  writer.append_i32_le(kDefaultMaxBlockingSeconds);
  writer.append_u32_le(kDefaultMaxBlockingNanoseconds);
}

void append_parameter_durability(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_DURABILITY, 4);
  writer.append_u32_le(kDurabilityVolatile);
}

void append_parameter_liveliness(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_LIVELINESS, 12);
  writer.append_u32_le(kLivelinessAutomatic);
  writer.append_i32_le(kDefaultLeaseDurationSeconds);
  writer.append_u32_le(kDefaultLeaseDurationNanoseconds);
}

void append_parameter_history(ByteWriter &writer) {
  append_parameter_header(writer, ParameterId::PID_HISTORY, 8);
  writer.append_u32_le(kHistoryKeepLast);
  writer.append_u32_le(1);
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
  for (const auto &parameter : parameters) {
    if (parameter.id == id) {
      matches.push_back(parameter);
    }
  }
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
  if (!reader.read_u32_be(kind) || !reader.read_u32_be(port) ||
      !reader.read_bytes(std::span<uint8_t>{locator.address.data(), locator.address.size()})) {
    return std::nullopt;
  }
  locator.kind = static_cast<espp::RtpsParticipant::Locator::Kind>(static_cast<int32_t>(kind));
  locator.port = port;
  return locator;
}

bool has_valid_locator(const espp::RtpsParticipant::Locator &locator) {
  return locator.kind == espp::RtpsParticipant::Locator::Kind::UDP_V4 && locator.port != 0;
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
  return text.substr(position, end - position);
}

std::array<uint8_t, 4> entity_id_for_index(uint32_t entity_index, uint8_t kind) {
  return {0x00, 0x00, static_cast<uint8_t>(0x10 + entity_index), kind};
}

bool is_same_guid_prefix(const espp::RtpsParticipant::Guid &guid,
                         const espp::RtpsParticipant::GuidPrefix &prefix) {
  return guid.prefix == prefix;
}

DataSubmessageView parse_data_submessage(const espp::RtpsParticipant::Submessage &submessage,
                                         bool &ok) {
  DataSubmessageView view;
  ok = false;
  if (submessage.kind != espp::RtpsParticipant::SubmessageKind::DATA ||
      (submessage.flags & kSubmessageFlagData) == 0) {
    return view;
  }

  ByteReader reader(std::span<const uint8_t>{submessage.payload.data(), submessage.payload.size()});
  uint16_t extra_flags = 0;
  uint16_t octets_to_inline_qos = 0;
  if (!reader.read_u16_le(extra_flags) || !reader.read_u16_le(octets_to_inline_qos) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.reader_id.value.data(), view.reader_id.value.size()}) ||
      !reader.read_bytes(
          std::span<uint8_t>{view.writer_id.value.data(), view.writer_id.value.size()}) ||
      !reader.read_sequence_number_le(view.writer_sn)) {
    return view;
  }

  view.inline_qos_present = (submessage.flags & kSubmessageFlagInlineQos) != 0;
  view.data_present = true;
  if (view.inline_qos_present || octets_to_inline_qos != kDataSubmessageOctetsToInlineQos) {
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
  auto hash = std::hash<std::string>{}(config_.node_name);
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
  metatraffic_multicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = get_log_level()});
  metatraffic_unicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = get_log_level()});
  user_unicast_receiver_ =
      std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = get_log_level()});

  auto multicast_task_config = config_.receive_task_config;
  multicast_task_config.name = config_.receive_task_config.name + "_spdp_mc";
  auto multicast_receive_config = UdpSocket::ReceiveConfig{
      .port = port_mapping.metatraffic_multicast,
      .buffer_size = 4096,
      .is_multicast_endpoint = true,
      .multicast_group = config_.metatraffic_multicast_group,
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
  return true;
}

void RtpsParticipant::stop() {
  started_ = false;
  if (announce_task_) {
    announce_task_->stop();
    announce_task_.reset();
  }
  if (metatraffic_multicast_receiver_) {
    metatraffic_multicast_receiver_->stop_receiving();
    metatraffic_multicast_receiver_.reset();
  }
  if (metatraffic_unicast_receiver_) {
    metatraffic_unicast_receiver_->stop_receiving();
    metatraffic_unicast_receiver_.reset();
  }
  for (auto &receiver : user_multicast_receivers_) {
    if (receiver.socket) {
      receiver.socket->stop_receiving();
    }
  }
  user_multicast_receivers_.clear();
  if (user_unicast_receiver_) {
    user_unicast_receiver_->stop_receiving();
    user_unicast_receiver_.reset();
  }
}

bool RtpsParticipant::is_started() const { return started_.load(); }

bool RtpsParticipant::add_writer(const WriterConfig &writer_config) {
  std::lock_guard<std::mutex> lock(mutex_);
  writers_.push_back(writer_config);
  return true;
}

bool RtpsParticipant::add_reader(const ReaderConfig &reader_config) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    readers_.push_back(reader_config);
  }
  if (started_.load() && !reader_config.multicast_group.empty() &&
      !ensure_user_multicast_receivers_started()) {
    logger_.error("Failed to start multicast receiver for topic '{}'", reader_config.topic_name);
    return false;
  }
  return true;
}

std::vector<RtpsParticipant::ParticipantProxy> RtpsParticipant::discovered_participants() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_participants_;
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::discovered_writers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_writers_;
}

std::vector<RtpsParticipant::EndpointProxy> RtpsParticipant::discovered_readers() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_readers_;
}

const std::vector<RtpsParticipant::WriterConfig> &RtpsParticipant::writers() const {
  return writers_;
}

const std::vector<RtpsParticipant::ReaderConfig> &RtpsParticipant::readers() const {
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
  return build_message(guid_prefix_, {.value = kEntityIdUnknown}, {.value = kSpdpWriterEntityId}, 1,
                       payload)
      .serialize();
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_publication_message(const WriterConfig &writer_config) const {
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
  append_parameter_history(parameters);
  append_parameter_sentinel(parameters);

  auto payload = build_parameter_list_payload(parameters);
  return build_message(guid_prefix_, {.value = kSedpPublicationsReaderEntityId},
                       {.value = kSedpPublicationsWriterEntityId},
                       static_cast<int64_t>(writer_config.entity_index + 1), payload)
      .serialize();
}

std::vector<uint8_t>
RtpsParticipant::build_sedp_subscription_message(const ReaderConfig &reader_config) const {
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

  auto payload = build_parameter_list_payload(parameters);
  return build_message(guid_prefix_, {.value = kSedpSubscriptionsReaderEntityId},
                       {.value = kSedpSubscriptionsWriterEntityId},
                       static_cast<int64_t>(reader_config.entity_index + 1), payload)
      .serialize();
}

std::vector<uint8_t> RtpsParticipant::build_uint32_data_message(const WriterConfig &writer_config,
                                                                uint32_t value,
                                                                ReliabilityKind reliability) const {
  ByteWriter payload_writer;
  payload_writer.append_chars(kUserDataMagic);
  payload_writer.append_u8(kUserDataVersion);
  payload_writer.append_u8(static_cast<uint8_t>(reliability));
  append_string(payload_writer, writer_config.topic_name);
  auto cdr = serialize_uint32_cdr(value);
  payload_writer.append_u16_le(static_cast<uint16_t>(cdr.size()));
  payload_writer.append_bytes(cdr);

  auto guid = writer_guid(writer_config.entity_index);
  return build_message(guid_prefix_, {.value = kEntityIdUnknown}, guid.entity_id, 1,
                       payload_writer.take())
      .serialize();
}

bool RtpsParticipant::publish_uint32(std::string_view topic_name, uint32_t value) {
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

  if (writer_config.reliability == ReliabilityKind::RELIABLE) {
    logger_.warn("Reliable user-data retransmission is not implemented yet; sending best-effort");
  }

  auto encoded_reliability = writer_config.reliability == ReliabilityKind::RELIABLE
                                 ? ReliabilityKind::BEST_EFFORT
                                 : writer_config.reliability;
  auto payload = build_uint32_data_message(writer_config, value, encoded_reliability);

  if (!user_unicast_receiver_) {
    return false;
  }

  auto send_configs = build_user_send_configs(topic_name, writer_config);
  if (send_configs.empty()) {
    logger_.warn("No send destinations available for topic '{}'", topic_name);
    return false;
  }

  bool sent = false;
  for (const auto &send_config : send_configs) {
    sent = user_unicast_receiver_->send(payload, send_config) || sent;
  }
  return sent;
}

std::vector<uint8_t> RtpsParticipant::serialize_uint32_cdr(uint32_t value) {
  espp::CdrWriter writer({
      .encapsulation = espp::CdrEncapsulation::CDR_LE,
      .include_encapsulation = true,
  });
  writer.write<uint32_t>(value);
  return writer.take_buffer();
}

std::optional<uint32_t> RtpsParticipant::deserialize_uint32_cdr(std::span<const uint8_t> data) {
  espp::CdrReader reader(data);
  if (!reader.valid()) {
    return std::nullopt;
  }
  uint32_t value = 0;
  if (!reader.read<uint32_t>(value)) {
    return std::nullopt;
  }
  return value;
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

bool RtpsParticipant::handle_metatraffic_message(std::vector<uint8_t> &data,
                                                 const Socket::Info &sender) {
  auto message = Message::parse(data);
  if (!message) {
    return false;
  }

  for (const auto &submessage : message->submessages) {
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

      ParticipantProxy participant;
      participant.participant_guid = *maybe_participant_guid;
      participant.guid_prefix = maybe_participant_guid->prefix;
      participant.address = sender.address;

      if (auto maybe_name_parameter = find_parameter(parameters, ParameterId::PID_ENTITY_NAME)) {
        if (auto maybe_name = parse_cdr_string(maybe_name_parameter->value)) {
          participant.name = *maybe_name;
        }
      }
      if (auto maybe_user_data_parameter = find_parameter(parameters, ParameterId::PID_USER_DATA)) {
        if (auto maybe_user_data = parse_octet_sequence(maybe_user_data_parameter->value)) {
          participant.enclave = extract_enclave(*maybe_user_data);
        }
      }
      if (auto maybe_builtin_endpoint_parameter =
              find_parameter(parameters, ParameterId::PID_BUILTIN_ENDPOINT_SET)) {
        if (auto maybe_builtin_endpoints = parse_u32_le(maybe_builtin_endpoint_parameter->value)) {
          participant.builtin_endpoints = *maybe_builtin_endpoints;
        }
      }
      if (auto maybe_meta_unicast_parameter =
              find_parameter(parameters, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR)) {
        if (auto maybe_locator = parse_locator(maybe_meta_unicast_parameter->value)) {
          participant.ports.metatraffic_unicast = static_cast<uint16_t>(maybe_locator->port);
        }
      }
      if (auto maybe_meta_multicast_parameter =
              find_parameter(parameters, ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR)) {
        if (auto maybe_locator = parse_locator(maybe_meta_multicast_parameter->value)) {
          participant.ports.metatraffic_multicast = static_cast<uint16_t>(maybe_locator->port);
        }
      }
      if (auto maybe_default_unicast_parameter =
              find_parameter(parameters, ParameterId::PID_DEFAULT_UNICAST_LOCATOR)) {
        if (auto maybe_locator = parse_locator(maybe_default_unicast_parameter->value)) {
          participant.ports.user_unicast = static_cast<uint16_t>(maybe_locator->port);
          participant.address = maybe_locator->address_string();
        }
      }
      if (auto maybe_default_multicast_parameter =
              find_parameter(parameters, ParameterId::PID_DEFAULT_MULTICAST_LOCATOR)) {
        if (auto maybe_locator = parse_locator(maybe_default_multicast_parameter->value)) {
          participant.ports.user_multicast = static_cast<uint16_t>(maybe_locator->port);
        }
      }

      std::function<void(const ParticipantProxy &)> callback;
      bool is_new_participant = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iterator =
            std::find_if(discovered_participants_.begin(), discovered_participants_.end(),
                         [&participant](const auto &candidate) {
                           return candidate.participant_guid == participant.participant_guid;
                         });
        if (iterator == discovered_participants_.end()) {
          discovered_participants_.push_back(participant);
          is_new_participant = true;
        } else {
          *iterator = participant;
        }
        callback = config_.on_participant_discovered;
      }

      if (is_new_participant) {
        logger_.info("SPDP discovered participant '{}' at {} (meta {}, user {})",
                     participant.name.empty() ? participant.guid_prefix.to_string()
                                              : participant.name,
                     participant.address, participant.ports.metatraffic_unicast,
                     participant.ports.user_unicast);
        send_sedp_announcements_to(participant);
        if (callback) {
          callback(participant);
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

    auto maybe_endpoint_guid_parameter = find_parameter(parameters, ParameterId::PID_ENDPOINT_GUID);
    if (!maybe_endpoint_guid_parameter) {
      continue;
    }
    auto maybe_endpoint_guid = parse_guid(maybe_endpoint_guid_parameter->value);
    if (!maybe_endpoint_guid || is_same_guid_prefix(*maybe_endpoint_guid, guid_prefix_)) {
      continue;
    }

    EndpointProxy endpoint;
    endpoint.guid = *maybe_endpoint_guid;
    endpoint.is_reader = is_reader;

    if (auto maybe_participant_guid_parameter =
            find_parameter(parameters, ParameterId::PID_PARTICIPANT_GUID)) {
      if (auto maybe_participant_guid = parse_guid(maybe_participant_guid_parameter->value)) {
        endpoint.participant_guid = *maybe_participant_guid;
      }
    }
    if (endpoint.participant_guid.entity_id.value == std::array<uint8_t, 4>{}) {
      endpoint.participant_guid = {.prefix = endpoint.guid.prefix,
                                   .entity_id = {.value = kParticipantEntityId}};
    }
    if (auto maybe_topic_name_parameter = find_parameter(parameters, ParameterId::PID_TOPIC_NAME)) {
      if (auto maybe_topic_name = parse_cdr_string(maybe_topic_name_parameter->value)) {
        endpoint.topic_name = *maybe_topic_name;
      }
    }
    if (auto maybe_type_name_parameter = find_parameter(parameters, ParameterId::PID_TYPE_NAME)) {
      if (auto maybe_type_name = parse_cdr_string(maybe_type_name_parameter->value)) {
        endpoint.type_name = *maybe_type_name;
      }
    }
    if (auto maybe_locator_parameter =
            find_parameter(parameters, ParameterId::PID_UNICAST_LOCATOR)) {
      if (auto maybe_locator = parse_locator(maybe_locator_parameter->value)) {
        endpoint.unicast_locator = *maybe_locator;
      }
    }
    for (const auto &locator_parameter :
         find_parameters(parameters, ParameterId::PID_MULTICAST_LOCATOR)) {
      if (auto maybe_locator = parse_locator(locator_parameter.value)) {
        endpoint.multicast_locators.push_back(*maybe_locator);
      }
    }
    if (auto maybe_reliability_parameter =
            find_parameter(parameters, ParameterId::PID_RELIABILITY)) {
      if (auto maybe_reliability = parse_reliability(maybe_reliability_parameter->value)) {
        endpoint.reliability = *maybe_reliability;
      }
    }
    if (auto maybe_inline_qos_parameter =
            find_parameter(parameters, ParameterId::PID_EXPECTS_INLINE_QOS)) {
      if (auto maybe_inline_qos = parse_bool(maybe_inline_qos_parameter->value)) {
        endpoint.expects_inline_qos = *maybe_inline_qos;
      }
    }

    std::function<void(const EndpointProxy &)> endpoint_callback;
    bool is_new_endpoint = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto &endpoint_list = is_reader ? discovered_readers_ : discovered_writers_;
      auto iterator = std::find_if(
          endpoint_list.begin(), endpoint_list.end(),
          [&endpoint](const auto &candidate) { return candidate.guid == endpoint.guid; });
      if (iterator == endpoint_list.end()) {
        endpoint_list.push_back(endpoint);
        is_new_endpoint = true;
      } else {
        *iterator = endpoint;
      }
      endpoint_callback = config_.on_endpoint_discovered;
    }

    if (is_new_endpoint) {
      logger_.info("SEDP discovered {} '{}' [{}] from participant {}",
                   endpoint.is_reader ? "reader" : "writer", endpoint.topic_name,
                   endpoint.type_name, endpoint.participant_guid.to_string());
      if (endpoint_callback) {
        endpoint_callback(endpoint);
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
    bool valid_data = false;
    auto data_view = parse_data_submessage(submessage, valid_data);
    if (!valid_data || data_view.serialized_payload.size() < kUserDataMagic.size() + 2 ||
        !std::equal(kUserDataMagic.begin(), kUserDataMagic.end(),
                    data_view.serialized_payload.begin())) {
      continue;
    }

    ByteReader reader(data_view.serialized_payload);
    std::array<uint8_t, kUserDataMagic.size()> magic{};
    uint8_t version = 0;
    uint8_t reliability = 0;
    if (!reader.read_bytes(std::span<uint8_t>{magic.data(), magic.size()}) ||
        !reader.read_u8(version) || !reader.read_u8(reliability)) {
      continue;
    }
    auto topic_name = read_string(reader);
    uint16_t payload_length = 0;
    if (version != kUserDataVersion || !topic_name || !reader.read_u16_le(payload_length)) {
      continue;
    }
    auto payload = reader.read_span(payload_length);
    auto maybe_value = deserialize_uint32_cdr(payload);
    if (!maybe_value) {
      continue;
    }

    if (static_cast<ReliabilityKind>(reliability) == ReliabilityKind::RELIABLE) {
      logger_.warn(
          "Received reliable topic '{}' from {}, but ACKNACK/HEARTBEAT is not implemented yet",
          *topic_name, sender);
    }

    std::vector<std::function<void(uint32_t)>> callbacks;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &reader_config : readers_) {
        if (reader_config.topic_name == *topic_name && reader_config.on_uint32_sample) {
          callbacks.push_back(reader_config.on_uint32_sample);
        }
      }
    }
    for (const auto &callback : callbacks) {
      callback(*maybe_value);
    }
  }
  return false;
}

bool RtpsParticipant::ensure_user_multicast_receivers_started() {
  if (!started_.load()) {
    return true;
  }

  std::vector<std::string> desired_groups;
  if (config_.use_multicast_for_user_data && !config_.user_multicast_group.empty()) {
    desired_groups.push_back(config_.user_multicast_group);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &reader_config : readers_) {
      if (!reader_config.multicast_group.empty() &&
          std::find(desired_groups.begin(), desired_groups.end(), reader_config.multicast_group) ==
              desired_groups.end()) {
        desired_groups.push_back(reader_config.multicast_group);
      }
    }
  }

  auto port_mapping = ports();
  for (const auto &group : desired_groups) {
    auto existing =
        std::find_if(user_multicast_receivers_.begin(), user_multicast_receivers_.end(),
                     [&group](const auto &receiver) { return receiver.multicast_group == group; });
    if (existing != user_multicast_receivers_.end()) {
      continue;
    }

    auto socket = std::make_unique<UdpSocket>(UdpSocket::Config{.log_level = get_log_level()});
    auto task_config = config_.receive_task_config;
    task_config.name = fmt::format("{}_user_mc_{}", config_.receive_task_config.name,
                                   user_multicast_receivers_.size());
    auto receive_config = UdpSocket::ReceiveConfig{
        .port = port_mapping.user_multicast,
        .buffer_size = 4096,
        .is_multicast_endpoint = true,
        .multicast_group = group,
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

std::vector<UdpSocket::SendConfig>
RtpsParticipant::build_user_send_configs(std::string_view topic_name,
                                         const WriterConfig &writer_config) const {
  std::vector<UdpSocket::SendConfig> send_configs;
  auto add_send_config = [&send_configs](std::string ip_address, uint16_t port, bool is_multicast) {
    if (ip_address.empty() || port == 0) {
      return;
    }
    auto existing =
        std::find_if(send_configs.begin(), send_configs.end(), [&](const auto &send_config) {
          return send_config.ip_address == ip_address && send_config.port == port &&
                 send_config.is_multicast_endpoint == is_multicast;
        });
    if (existing == send_configs.end()) {
      send_configs.push_back({
          .ip_address = std::move(ip_address),
          .port = port,
          .is_multicast_endpoint = is_multicast,
      });
    }
  };

  if (!writer_config.multicast_group.empty()) {
    add_send_config(writer_config.multicast_group, ports().user_multicast, true);
    return send_configs;
  }

  if (config_.use_multicast_for_user_data) {
    add_send_config(config_.user_multicast_group, ports().user_multicast, true);
    return send_configs;
  }

  std::vector<EndpointProxy> remote_readers;
  std::vector<ParticipantProxy> participants;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    remote_readers = discovered_readers_;
    participants = discovered_participants_;
  }

  for (const auto &reader : remote_readers) {
    if (!reader.is_reader || reader.topic_name != topic_name) {
      continue;
    }

    bool used_multicast = false;
    for (const auto &locator : reader.multicast_locators) {
      if (!has_valid_locator(locator)) {
        continue;
      }
      add_send_config(locator.address_string(), static_cast<uint16_t>(locator.port), true);
      used_multicast = true;
    }
    if (used_multicast) {
      continue;
    }

    if (has_valid_locator(reader.unicast_locator)) {
      add_send_config(reader.unicast_locator.address_string(),
                      static_cast<uint16_t>(reader.unicast_locator.port), false);
      continue;
    }

    auto participant =
        std::find_if(participants.begin(), participants.end(), [&](const auto &proxy) {
          return proxy.guid_prefix == reader.participant_guid.prefix;
        });
    if (participant != participants.end()) {
      add_send_config(participant->address, participant->ports.user_unicast, false);
    }
  }

  return send_configs;
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
  };
  return metatraffic_unicast_receiver_->send(payload, send_config);
}

bool RtpsParticipant::send_sedp_announcements_to(const ParticipantProxy &participant) {
  if (!metatraffic_unicast_receiver_ || participant.ports.metatraffic_unicast == 0 ||
      participant.address.empty()) {
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

  for (const auto &writer_config : local_writers) {
    auto payload = build_sedp_publication_message(writer_config);
    auto send_config = UdpSocket::SendConfig{
        .ip_address = participant.address,
        .port = participant.ports.metatraffic_unicast,
    };
    sent = metatraffic_unicast_receiver_->send(payload, send_config) || sent;
  }
  for (const auto &reader_config : local_readers) {
    auto payload = build_sedp_subscription_message(reader_config);
    auto send_config = UdpSocket::SendConfig{
        .ip_address = participant.address,
        .port = participant.ports.metatraffic_unicast,
    };
    sent = metatraffic_unicast_receiver_->send(payload, send_config) || sent;
  }
  return sent;
}

bool RtpsParticipant::send_discovery_now() {
  auto participants = discovered_participants();
  return std::accumulate(participants.begin(), participants.end(), send_spdp_announce_now(),
                         [this](bool sent, const auto &participant) {
                           return send_sedp_announcements_to(participant) || sent;
                         });
}

RtpsParticipant::ParticipantProxy RtpsParticipant::make_local_participant_proxy() const {
  return {.participant_guid = participant_guid(),
          .guid_prefix = guid_prefix_,
          .name = config_.node_name,
          .enclave = config_.enclave,
          .address = config_.advertised_address,
          .ports = ports(),
          .builtin_endpoints = kBuiltinEndpointSet};
}
} // namespace espp
