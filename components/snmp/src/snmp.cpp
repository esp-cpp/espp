#include "snmp.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <span>
#include <sstream>
#include <unordered_map>

#include <arpa/inet.h>
#include <lwip/netdb.h>
#include <mbedtls/aes.h>
#include <mbedtls/md.h>
#include <mbedtls/sha1.h>

#include "udp_socket.hpp"

using namespace espp;

namespace {
using bytes = std::vector<uint8_t>;

constexpr uint8_t kTagInteger = 0x02;
constexpr uint8_t kTagOctetString = 0x04;
constexpr uint8_t kTagNull = 0x05;
constexpr uint8_t kTagObjectIdentifier = 0x06;
constexpr uint8_t kTagSequence = 0x30;

constexpr uint8_t kTagIpAddress = 0x40;
constexpr uint8_t kTagCounter32 = 0x41;
constexpr uint8_t kTagUnsigned32 = 0x42;
constexpr uint8_t kTagTimeTicks = 0x43;
constexpr uint8_t kTagOpaque = 0x44;
constexpr uint8_t kTagCounter64 = 0x46;

constexpr uint8_t kTagNoSuchObject = 0x80;
constexpr uint8_t kTagNoSuchInstance = 0x81;
constexpr uint8_t kTagEndOfMibView = 0x82;

constexpr uint8_t kPduGet = 0xA0;
constexpr uint8_t kPduGetNext = 0xA1;
constexpr uint8_t kPduResponse = 0xA2;
constexpr uint8_t kPduSet = 0xA3;
constexpr uint8_t kPduGetBulk = 0xA5;
constexpr uint8_t kPduReport = 0xA8;

constexpr uint8_t kMsgFlagsAuth = 0x01;
constexpr uint8_t kMsgFlagsPriv = 0x02;
constexpr uint8_t kMsgFlagsReportable = 0x04;
constexpr uint8_t kEncryptedPduTag = kTagOctetString;

constexpr int32_t kSnmpV2cVersion = 1;
constexpr int32_t kSnmpV3Version = 3;
constexpr int32_t kUsmSecurityModel = 3;
constexpr size_t kUsmAuthParamLength = 12;
constexpr size_t kSha1DigestLength = 20;
constexpr size_t kPasswordToKeyStretchLength = 1024 * 1024;

const Snmp::Oid kUsmStatsUnknownEngineIds = Snmp::Oid::from_string("1.3.6.1.6.3.15.1.1.4.0");
const Snmp::Oid kUsmStatsNotInTimeWindows = Snmp::Oid::from_string("1.3.6.1.6.3.15.1.1.2.0");

struct Slice {
  const uint8_t *data{nullptr};
  size_t size{0};
  size_t absolute_offset{0};
};

struct Decoder {
  Slice slice;
  size_t pos{0};

  explicit Decoder(const Slice &slice)
      : slice(slice) {}

  bool empty() const { return pos >= slice.size; }

  bool read_tlv(uint8_t &tag, Slice &value) {
    if (pos >= slice.size) {
      return false;
    }
    tag = slice.data[pos++];
    if (pos >= slice.size) {
      return false;
    }
    size_t length = 0;
    if ((slice.data[pos] & 0x80) == 0) {
      length = slice.data[pos++];
    } else {
      size_t count = slice.data[pos++] & 0x7F;
      if (count == 0 || count > sizeof(size_t) || pos + count > slice.size) {
        return false;
      }
      for (size_t i = 0; i < count; i++) {
        length = (length << 8) | slice.data[pos++];
      }
    }
    if (pos + length > slice.size) {
      return false;
    }
    value = Slice{slice.data + pos, length, slice.absolute_offset + pos};
    pos += length;
    return true;
  }
};

struct UsmSecurityParameters {
  bytes authoritative_engine_id;
  uint32_t authoritative_engine_boots{0};
  uint32_t authoritative_engine_time{0};
  std::string user_name;
  bytes authentication_parameters;
  bytes privacy_parameters;
  size_t auth_parameters_offset{0};
};

struct V3Message {
  int32_t version{0};
  int32_t message_id{0};
  int32_t max_message_size{0};
  uint8_t flags{0};
  int32_t security_model{0};
  UsmSecurityParameters usm;
  bytes scoped_pdu;
  bytes encrypted_pdu;
  bool encrypted{false};
  Snmp::Response response;
};

struct PduEncoding {
  bytes encoded;
  int32_t request_id{0};
};

struct EngineCache {
  bytes engine_id;
  uint32_t engine_boots{0};
  uint32_t engine_time{0};
  std::chrono::steady_clock::time_point observed_at{};
  std::array<uint8_t, kSha1DigestLength> auth_key{};
  std::array<uint8_t, kSha1DigestLength> priv_key{};
  bool discovered{false};
  bool localized{false};
};

std::mutex &snmp_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::unordered_map<const Snmp *, EngineCache> &engine_cache_map() {
  static std::unordered_map<const Snmp *, EngineCache> cache_map;
  return cache_map;
}

std::unordered_map<const Snmp *, uint32_t> &request_counter_map() {
  static std::unordered_map<const Snmp *, uint32_t> counters;
  return counters;
}

std::unordered_map<const Snmp *, uint32_t> &message_counter_map() {
  static std::unordered_map<const Snmp *, uint32_t> counters;
  return counters;
}

std::unordered_map<const Snmp *, uint64_t> &salt_counter_map() {
  static std::unordered_map<const Snmp *, uint64_t> counters;
  return counters;
}

bytes make_octet_string(std::string_view value) { return bytes(value.begin(), value.end()); }

std::optional<uint32_t> nonnegative_integer_to_u32(const Snmp::Value &value) {
  auto integer = value.as_integer();
  if (!integer.has_value() || *integer < 0) {
    return std::nullopt;
  }
  return static_cast<uint32_t>(*integer);
}

void encode_subidentifier(uint32_t value, bytes &out) {
  bytes encoded{static_cast<uint8_t>(value & 0x7F)};
  value >>= 7;
  while (value > 0) {
    encoded.insert(encoded.begin(), static_cast<uint8_t>(0x80 | (value & 0x7F)));
    value >>= 7;
  }
  out.insert(out.end(), encoded.begin(), encoded.end());
}

bool decode_subidentifier(const Slice &slice, size_t &offset, uint32_t &value) {
  if (offset >= slice.size) {
    return false;
  }
  value = 0;
  do {
    if (offset >= slice.size) {
      return false;
    }
    value = (value << 7) | (slice.data[offset] & 0x7F);
  } while (slice.data[offset++] & 0x80);
  return true;
}

void append_length(bytes &buffer, size_t length) {
  if (length < 0x80) {
    buffer.push_back(static_cast<uint8_t>(length));
    return;
  }
  bytes encoded_length;
  while (length > 0) {
    encoded_length.insert(encoded_length.begin(), static_cast<uint8_t>(length & 0xFF));
    length >>= 8;
  }
  buffer.push_back(static_cast<uint8_t>(0x80 | encoded_length.size()));
  buffer.insert(buffer.end(), encoded_length.begin(), encoded_length.end());
}

bytes make_tlv(uint8_t tag, std::span<const uint8_t> content) {
  bytes out;
  out.reserve(1 + 5 + content.size());
  out.push_back(tag);
  append_length(out, content.size());
  out.insert(out.end(), content.begin(), content.end());
  return out;
}

bytes encode_signed_integer(int64_t value) {
  bytes out(sizeof(int64_t));
  for (size_t i = 0; i < sizeof(int64_t); i++) {
    out[sizeof(int64_t) - i - 1] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
  }
  while (out.size() > 1) {
    auto first = out[0];
    auto second = out[1];
    if ((first == 0x00 && (second & 0x80) == 0) || (first == 0xFF && (second & 0x80) == 0x80)) {
      out.erase(out.begin());
    } else {
      break;
    }
  }
  return out;
}

bytes encode_unsigned_integer(uint64_t value) {
  bytes out;
  do {
    out.insert(out.begin(), static_cast<uint8_t>(value & 0xFF));
    value >>= 8;
  } while (value > 0);
  if ((out.front() & 0x80) != 0) {
    out.insert(out.begin(), 0x00);
  }
  return out;
}

int64_t decode_signed_integer(const Slice &slice, bool &ok) {
  ok = !slice.size || slice.size <= sizeof(int64_t);
  if (!ok) {
    return 0;
  }
  int64_t value = (slice.size > 0 && (slice.data[0] & 0x80)) ? -1 : 0;
  for (size_t i = 0; i < slice.size; i++) {
    value = (value << 8) | slice.data[i];
  }
  return value;
}

uint64_t decode_unsigned_integer(const Slice &slice, bool &ok) {
  ok = slice.size <= sizeof(uint64_t);
  if (!ok) {
    return 0;
  }
  uint64_t value = 0;
  for (size_t i = 0; i < slice.size; i++) {
    value = (value << 8) | slice.data[i];
  }
  return value;
}

bytes encode_oid(const Snmp::Oid &oid) {
  bytes out;
  if (oid.nodes.empty()) {
    return out;
  }
  uint32_t first = oid.nodes.size() > 0 ? oid.nodes[0] : 0;
  uint32_t second = oid.nodes.size() > 1 ? oid.nodes[1] : 0;
  first = std::min<uint32_t>(first, 2);
  encode_subidentifier(first * 40 + second, out);
  for (size_t i = 2; i < oid.nodes.size(); i++) {
    encode_subidentifier(oid.nodes[i], out);
  }
  return out;
}

Snmp::Oid decode_oid(const Slice &slice, bool &ok) {
  ok = slice.size > 0;
  if (!ok) {
    return {};
  }
  Snmp::Oid oid;
  size_t i = 0;
  uint32_t first_subidentifier = 0;
  if (!decode_subidentifier(slice, i, first_subidentifier)) {
    ok = false;
    return {};
  }
  oid.nodes.push_back(first_subidentifier < 80 ? first_subidentifier / 40 : 2);
  oid.nodes.push_back(first_subidentifier - oid.nodes[0] * 40);
  while (i < slice.size) {
    uint32_t value = 0;
    if (!decode_subidentifier(slice, i, value)) {
      ok = false;
      return {};
    }
    oid.nodes.push_back(value);
  }
  return oid;
}

bytes encode_value(const Snmp::Value &value) {
  switch (value.type) {
  case Snmp::ValueType::Integer:
    return make_tlv(kTagInteger, encode_signed_integer(std::get<int32_t>(value.data)));
  case Snmp::ValueType::OctetString:
    return make_tlv(kTagOctetString, std::get<Snmp::Value::octets_t>(value.data));
  case Snmp::ValueType::Null:
    return make_tlv(kTagNull, bytes{});
  case Snmp::ValueType::ObjectIdentifier:
    return make_tlv(kTagObjectIdentifier, encode_oid(std::get<Snmp::Oid>(value.data)));
  case Snmp::ValueType::IpAddress: {
    const auto &ip = std::get<Snmp::IpAddress>(value.data);
    return make_tlv(kTagIpAddress, ip.bytes);
  }
  case Snmp::ValueType::Counter32:
    return make_tlv(kTagCounter32, encode_unsigned_integer(std::get<uint32_t>(value.data)));
  case Snmp::ValueType::Unsigned32:
  case Snmp::ValueType::Gauge32:
    return make_tlv(kTagUnsigned32, encode_unsigned_integer(std::get<uint32_t>(value.data)));
  case Snmp::ValueType::TimeTicks:
    return make_tlv(kTagTimeTicks, encode_unsigned_integer(std::get<uint32_t>(value.data)));
  case Snmp::ValueType::Opaque:
    return make_tlv(kTagOpaque, std::get<Snmp::Value::octets_t>(value.data));
  case Snmp::ValueType::Counter64:
    return make_tlv(kTagCounter64, encode_unsigned_integer(std::get<uint64_t>(value.data)));
  case Snmp::ValueType::NoSuchObject:
    return make_tlv(kTagNoSuchObject, bytes{});
  case Snmp::ValueType::NoSuchInstance:
    return make_tlv(kTagNoSuchInstance, bytes{});
  case Snmp::ValueType::EndOfMibView:
    return make_tlv(kTagEndOfMibView, bytes{});
  default:
    return {};
  }
}

Snmp::Value decode_value(uint8_t tag, const Slice &slice, bool &ok) {
  ok = true;
  switch (tag) {
  case kTagInteger:
    return Snmp::Value::integer(static_cast<int32_t>(decode_signed_integer(slice, ok)));
  case kTagOctetString:
    return Snmp::Value::octet_string(Snmp::Value::octets_t(slice.data, slice.data + slice.size));
  case kTagNull:
    return Snmp::Value::null();
  case kTagObjectIdentifier:
    return Snmp::Value::object_identifier(decode_oid(slice, ok));
  case kTagIpAddress:
    if (slice.size != 4) {
      ok = false;
      return {};
    }
    return Snmp::Value::ip_address(
        Snmp::IpAddress{{slice.data[0], slice.data[1], slice.data[2], slice.data[3]}});
  case kTagCounter32:
    return Snmp::Value::counter32(static_cast<uint32_t>(decode_unsigned_integer(slice, ok)));
  case kTagUnsigned32:
    return Snmp::Value::unsigned32(static_cast<uint32_t>(decode_unsigned_integer(slice, ok)));
  case kTagTimeTicks:
    return Snmp::Value::time_ticks(static_cast<uint32_t>(decode_unsigned_integer(slice, ok)));
  case kTagOpaque:
    return Snmp::Value::opaque(Snmp::Value::octets_t(slice.data, slice.data + slice.size));
  case kTagCounter64:
    return Snmp::Value::counter64(decode_unsigned_integer(slice, ok));
  case kTagNoSuchObject:
    return Snmp::Value::no_such_object();
  case kTagNoSuchInstance:
    return Snmp::Value::no_such_instance();
  case kTagEndOfMibView:
    return Snmp::Value::end_of_mib_view();
  default:
    ok = false;
    return {};
  }
}

uint8_t pdu_tag_for_type(Snmp::RequestType type) {
  switch (type) {
  case Snmp::RequestType::Get:
    return kPduGet;
  case Snmp::RequestType::GetNext:
    return kPduGetNext;
  case Snmp::RequestType::GetBulk:
    return kPduGetBulk;
  case Snmp::RequestType::Set:
    return kPduSet;
  case Snmp::RequestType::Response:
    return kPduResponse;
  case Snmp::RequestType::Report:
    return kPduReport;
  }
  return 0;
}

Snmp::RequestType pdu_type_for_tag(uint8_t tag, bool &ok) {
  ok = true;
  switch (tag) {
  case kPduGet:
    return Snmp::RequestType::Get;
  case kPduGetNext:
    return Snmp::RequestType::GetNext;
  case kPduGetBulk:
    return Snmp::RequestType::GetBulk;
  case kPduSet:
    return Snmp::RequestType::Set;
  case kPduResponse:
    return Snmp::RequestType::Response;
  case kPduReport:
    return Snmp::RequestType::Report;
  default:
    ok = false;
    return Snmp::RequestType::Response;
  }
}

bytes encode_sequence(const std::vector<bytes> &items) {
  bytes content;
  for (const auto &item : items) {
    content.insert(content.end(), item.begin(), item.end());
  }
  return make_tlv(kTagSequence, content);
}

bytes encode_varbind(const Snmp::VarBind &varbind) {
  auto oid = make_tlv(kTagObjectIdentifier, encode_oid(varbind.oid));
  auto value = encode_value(varbind.value);
  return encode_sequence({oid, value});
}

PduEncoding encode_pdu(Snmp::RequestType type, int32_t request_id, int32_t first_parameter,
                       int32_t second_parameter, const std::vector<Snmp::VarBind> &varbinds) {
  bytes varbind_list_content;
  for (const auto &varbind : varbinds) {
    auto encoded_varbind = encode_varbind(varbind);
    varbind_list_content.insert(varbind_list_content.end(), encoded_varbind.begin(),
                                encoded_varbind.end());
  }
  auto varbind_list = make_tlv(kTagSequence, varbind_list_content);
  bytes content;
  auto append = [&content](const bytes &tlv) {
    content.insert(content.end(), tlv.begin(), tlv.end());
  };
  append(make_tlv(kTagInteger, encode_signed_integer(request_id)));
  append(make_tlv(kTagInteger, encode_signed_integer(first_parameter)));
  append(make_tlv(kTagInteger, encode_signed_integer(second_parameter)));
  append(varbind_list);
  return {make_tlv(pdu_tag_for_type(type), content), request_id};
}

bool parse_varbind(const Slice &slice, Snmp::VarBind &varbind) {
  Decoder decoder(slice);
  uint8_t tag = 0;
  Slice value{};
  if (!decoder.read_tlv(tag, value) || tag != kTagObjectIdentifier) {
    return false;
  }
  bool ok = true;
  varbind.oid = decode_oid(value, ok);
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value)) {
    return false;
  }
  varbind.value = decode_value(tag, value, ok);
  return ok;
}

bool parse_pdu(uint8_t pdu_tag, const Slice &slice, Snmp::Response &response) {
  bool ok = true;
  response.type = pdu_type_for_tag(pdu_tag, ok);
  if (!ok) {
    return false;
  }
  Decoder decoder(slice);
  uint8_t tag = 0;
  Slice value{};
  if (!decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  response.request_id = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  response.error_status = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  response.error_index = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder vb_decoder(value);
  while (!vb_decoder.empty()) {
    Slice vb_slice{};
    if (!vb_decoder.read_tlv(tag, vb_slice) || tag != kTagSequence) {
      return false;
    }
    Snmp::VarBind varbind;
    if (!parse_varbind(vb_slice, varbind)) {
      return false;
    }
    response.varbinds.push_back(std::move(varbind));
  }
  return true;
}

bool resolve_host_ipv4(std::string_view host, std::string &resolved_ip) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  addrinfo *result = nullptr;
  auto ret = getaddrinfo(std::string(host).c_str(), nullptr, &hints, &result);
  if (ret != 0 || result == nullptr) {
    return false;
  }
  char buffer[INET_ADDRSTRLEN] = {0};
  auto *addr = reinterpret_cast<sockaddr_in *>(result->ai_addr);
  auto ok = inet_ntop(AF_INET, &(addr->sin_addr), buffer, sizeof(buffer)) != nullptr;
  if (ok) {
    resolved_ip = buffer;
  }
  freeaddrinfo(result);
  return ok;
}

bool password_to_key_sha1(std::string_view password, std::array<uint8_t, kSha1DigestLength> &out) {
  if (password.empty()) {
    return false;
  }
  mbedtls_sha1_context ctx;
  mbedtls_sha1_init(&ctx);
  if (mbedtls_sha1_starts(&ctx) != 0) {
    mbedtls_sha1_free(&ctx);
    return false;
  }
  bytes chunk(64);
  size_t password_index = 0;
  size_t total = 0;
  while (total < kPasswordToKeyStretchLength) {
    for (size_t i = 0; i < chunk.size(); i++) {
      chunk[i] = static_cast<uint8_t>(password[password_index++ % password.size()]);
    }
    size_t remaining = std::min(chunk.size(), kPasswordToKeyStretchLength - total);
    if (mbedtls_sha1_update(&ctx, chunk.data(), remaining) != 0) {
      mbedtls_sha1_free(&ctx);
      return false;
    }
    total += remaining;
  }
  auto ret = mbedtls_sha1_finish(&ctx, out.data());
  mbedtls_sha1_free(&ctx);
  return ret == 0;
}

bool localize_key_sha1(const std::array<uint8_t, kSha1DigestLength> &ku,
                       std::span<const uint8_t> engine_id,
                       std::array<uint8_t, kSha1DigestLength> &kul) {
  mbedtls_sha1_context ctx;
  mbedtls_sha1_init(&ctx);
  if (mbedtls_sha1_starts(&ctx) != 0) {
    mbedtls_sha1_free(&ctx);
    return false;
  }
  auto fail = [&ctx]() {
    mbedtls_sha1_free(&ctx);
    return false;
  };
  if (mbedtls_sha1_update(&ctx, ku.data(), ku.size()) != 0) {
    return fail();
  }
  if (!engine_id.empty() && mbedtls_sha1_update(&ctx, engine_id.data(), engine_id.size()) != 0) {
    return fail();
  }
  if (mbedtls_sha1_update(&ctx, ku.data(), ku.size()) != 0) {
    return fail();
  }
  if (mbedtls_sha1_finish(&ctx, kul.data()) != 0) {
    return fail();
  }
  mbedtls_sha1_free(&ctx);
  return true;
}

bool hmac_sha1(std::span<const uint8_t> key, std::span<const uint8_t> input,
               std::array<uint8_t, kSha1DigestLength> &digest) {
  auto *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
  if (!md_info) {
    return false;
  }
  return mbedtls_md_hmac(md_info, key.data(), key.size(), input.data(), input.size(),
                         digest.data()) == 0;
}

bytes make_iv(uint32_t engine_boots, uint32_t engine_time, std::span<const uint8_t> salt) {
  bytes iv(16, 0);
  iv[0] = static_cast<uint8_t>((engine_boots >> 24) & 0xFF);
  iv[1] = static_cast<uint8_t>((engine_boots >> 16) & 0xFF);
  iv[2] = static_cast<uint8_t>((engine_boots >> 8) & 0xFF);
  iv[3] = static_cast<uint8_t>(engine_boots & 0xFF);
  iv[4] = static_cast<uint8_t>((engine_time >> 24) & 0xFF);
  iv[5] = static_cast<uint8_t>((engine_time >> 16) & 0xFF);
  iv[6] = static_cast<uint8_t>((engine_time >> 8) & 0xFF);
  iv[7] = static_cast<uint8_t>(engine_time & 0xFF);
  for (size_t i = 0; i < std::min<size_t>(8, salt.size()); i++) {
    iv[8 + i] = salt[i];
  }
  return iv;
}

bool aes_cfb128_crypt(const std::array<uint8_t, kSha1DigestLength> &localized_key,
                      std::span<const uint8_t> iv_input, std::span<const uint8_t> input,
                      bytes &output, int mode) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  output.resize(input.size());
  bytes iv(iv_input.begin(), iv_input.end());
  size_t iv_off = 0;
  auto ret = mbedtls_aes_setkey_enc(&aes, localized_key.data(), 128);
  if (ret == 0) {
    ret = mbedtls_aes_crypt_cfb128(&aes, mode, input.size(), &iv_off, iv.data(), input.data(),
                                   output.data());
  }
  mbedtls_aes_free(&aes);
  return ret == 0;
}

uint32_t seconds_since(const EngineCache &cache) {
  if (cache.observed_at.time_since_epoch().count() == 0) {
    return 0;
  }
  auto elapsed = std::chrono::steady_clock::now() - cache.observed_at;
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
}

bool parse_usm_security_params(const Slice &outer_octet_string, UsmSecurityParameters &usm) {
  Decoder outer_decoder(outer_octet_string);
  uint8_t tag = 0;
  Slice value{};
  if (!outer_decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder decoder(value);
  bool ok = true;
  if (!decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  usm.authoritative_engine_id.assign(value.data, value.data + value.size);
  if (!decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  usm.authoritative_engine_boots = static_cast<uint32_t>(decode_unsigned_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  usm.authoritative_engine_time = static_cast<uint32_t>(decode_unsigned_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  usm.user_name.assign(reinterpret_cast<const char *>(value.data), value.size);
  if (!decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  usm.authentication_parameters.assign(value.data, value.data + value.size);
  usm.auth_parameters_offset = value.absolute_offset;
  if (!decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  usm.privacy_parameters.assign(value.data, value.data + value.size);
  return true;
}

bool parse_v2c_response(std::span<const uint8_t> packet, Snmp::Response &response) {
  Slice root{packet.data(), packet.size(), 0};
  Decoder decoder(root);
  uint8_t tag = 0;
  Slice value{};
  if (!decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder message_decoder(value);
  bool ok = true;
  if (!message_decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  auto version = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok || version != kSnmpV2cVersion) {
    return false;
  }
  if (!message_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!message_decoder.read_tlv(tag, value)) {
    return false;
  }
  return parse_pdu(tag, value, response);
}

bool parse_v3_message(std::span<const uint8_t> packet, V3Message &message) {
  Slice root{packet.data(), packet.size(), 0};
  Decoder decoder(root);
  uint8_t tag = 0;
  Slice value{};
  if (!decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder message_decoder(value);
  bool ok = true;
  if (!message_decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  message.version = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok || message.version != kSnmpV3Version) {
    return false;
  }
  if (!message_decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder header_decoder(value);
  if (!header_decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  message.message_id = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!header_decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  message.max_message_size = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok) {
    return false;
  }
  if (!header_decoder.read_tlv(tag, value) || tag != kTagOctetString || value.size != 1) {
    return false;
  }
  message.flags = value.data[0];
  if (!header_decoder.read_tlv(tag, value) || tag != kTagInteger) {
    return false;
  }
  message.security_model = static_cast<int32_t>(decode_signed_integer(value, ok));
  if (!ok || message.security_model != kUsmSecurityModel) {
    return false;
  }
  if (!message_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!parse_usm_security_params(value, message.usm)) {
    return false;
  }
  if (!message_decoder.read_tlv(tag, value)) {
    return false;
  }
  if (tag == kEncryptedPduTag) {
    message.encrypted = true;
    message.encrypted_pdu.assign(value.data, value.data + value.size);
    return true;
  }
  if (tag != kTagSequence) {
    return false;
  }
  Decoder scoped_decoder(value);
  if (!scoped_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!scoped_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!scoped_decoder.read_tlv(tag, value)) {
    return false;
  }
  return parse_pdu(tag, value, message.response);
}

bool parse_scoped_pdu(std::span<const uint8_t> scoped_pdu, Snmp::Response &response) {
  Slice slice{scoped_pdu.data(), scoped_pdu.size(), 0};
  Decoder decoder(slice);
  uint8_t tag = 0;
  Slice value{};
  if (!decoder.read_tlv(tag, value) || tag != kTagSequence) {
    return false;
  }
  Decoder scoped_decoder(value);
  if (!scoped_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!scoped_decoder.read_tlv(tag, value) || tag != kTagOctetString) {
    return false;
  }
  if (!scoped_decoder.read_tlv(tag, value)) {
    return false;
  }
  return parse_pdu(tag, value, response);
}

std::string bytes_to_hex(std::span<const uint8_t> data) {
  std::ostringstream oss;
  for (size_t i = 0; i < data.size(); i++) {
    if (i != 0) {
      oss << ':';
    }
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
  }
  return oss.str();
}

bytes encode_usm_security_params(const bytes &engine_id, uint32_t engine_boots,
                                 uint32_t engine_time, std::string_view username,
                                 const bytes &auth_parameters, const bytes &privacy_parameters) {
  auto engine_id_tlv = make_tlv(kTagOctetString, engine_id);
  auto engine_boots_tlv = make_tlv(kTagInteger, encode_unsigned_integer(engine_boots));
  auto engine_time_tlv = make_tlv(kTagInteger, encode_unsigned_integer(engine_time));
  auto user_tlv = make_tlv(kTagOctetString, make_octet_string(username));
  auto auth_tlv = make_tlv(kTagOctetString, auth_parameters);
  auto priv_tlv = make_tlv(kTagOctetString, privacy_parameters);
  auto inner = encode_sequence(
      {engine_id_tlv, engine_boots_tlv, engine_time_tlv, user_tlv, auth_tlv, priv_tlv});
  return make_tlv(kTagOctetString, inner);
}

bytes encode_scoped_pdu(const bytes &context_engine_id, std::string_view context_name,
                        const bytes &pdu) {
  auto engine_id_tlv = make_tlv(kTagOctetString, context_engine_id);
  auto context_tlv = make_tlv(kTagOctetString, make_octet_string(context_name));
  return encode_sequence({engine_id_tlv, context_tlv, pdu});
}

bool udp_round_trip(const Snmp::Config &config, std::span<const uint8_t> request, bytes &response) {
  std::string resolved_ip;
  if (!resolve_host_ipv4(config.endpoint.host, resolved_ip)) {
    return false;
  }
  UdpSocket socket({.log_level = config.log_level});
  bool got_response = false;
  auto timeout_seconds = static_cast<float>(config.timeout.count()) / 1000.0f;
  UdpSocket::SendConfig send_config{
      .ip_address = resolved_ip,
      .port = config.endpoint.port,
      .wait_for_response = true,
      .response_size = config.max_message_size,
      .on_response_callback =
          [&](auto &data) {
            response = data;
            got_response = true;
          },
      .response_timeout = std::chrono::duration<float>(timeout_seconds),
  };
  if (!socket.send(request, send_config)) {
    return false;
  }
  return got_response;
}

Snmp::Oid append_index(const Snmp::Oid &base, uint32_t index) {
  auto oid = base;
  oid.nodes.push_back(index);
  return oid;
}

Snmp::VarBind make_null_varbind(const Snmp::Oid &oid) {
  return Snmp::VarBind{.oid = oid, .value = Snmp::Value::null()};
}

int32_t next_request_id(const Snmp *snmp) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  auto &counter = request_counter_map()[snmp];
  if (counter == 0) {
    counter = 1;
  }
  return static_cast<int32_t>(counter++);
}

int32_t next_message_id(const Snmp *snmp) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  auto &counter = message_counter_map()[snmp];
  if (counter == 0) {
    counter = 1;
  }
  return static_cast<int32_t>(counter++);
}

uint64_t next_salt(const Snmp *snmp) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  auto &counter = salt_counter_map()[snmp];
  if (counter == 0) {
    counter = 1;
  }
  return counter++;
}

void clear_instance_state(const Snmp *snmp) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  engine_cache_map().erase(snmp);
  request_counter_map().erase(snmp);
  message_counter_map().erase(snmp);
  salt_counter_map().erase(snmp);
}

EngineCache get_engine_cache(const Snmp *snmp) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  return engine_cache_map()[snmp];
}

void set_engine_cache(const Snmp *snmp, const EngineCache &cache) {
  std::lock_guard<std::mutex> lock(snmp_mutex());
  engine_cache_map()[snmp] = cache;
}

bool localize_engine_keys(const Snmp *snmp, const Snmp::Config &config, EngineCache &cache) {
  std::array<uint8_t, kSha1DigestLength> auth_ku{};
  std::array<uint8_t, kSha1DigestLength> priv_ku{};
  if (!password_to_key_sha1(config.v3.auth_password, auth_ku) ||
      !password_to_key_sha1(config.v3.privacy_password, priv_ku)) {
    return false;
  }
  if (!localize_key_sha1(auth_ku, cache.engine_id, cache.auth_key) ||
      !localize_key_sha1(priv_ku, cache.engine_id, cache.priv_key)) {
    return false;
  }
  cache.localized = true;
  set_engine_cache(snmp, cache);
  return true;
}

bool update_engine_cache_from_response(const Snmp *snmp, const V3Message &message,
                                       EngineCache &cache) {
  if (!message.usm.authoritative_engine_id.empty()) {
    cache.engine_id = message.usm.authoritative_engine_id;
    cache.engine_boots = message.usm.authoritative_engine_boots;
    cache.engine_time = message.usm.authoritative_engine_time;
    cache.observed_at = std::chrono::steady_clock::now();
    cache.discovered = true;
    set_engine_cache(snmp, cache);
  }
  return cache.discovered;
}

bool is_report_oid(const Snmp::Response &response, const Snmp::Oid &oid) {
  return !response.varbinds.empty() && response.varbinds.front().oid == oid;
}

bool build_v2c_packet(const Snmp::Config &config, const PduEncoding &pdu, bytes &packet) {
  auto version = make_tlv(kTagInteger, encode_signed_integer(kSnmpV2cVersion));
  auto community = make_tlv(kTagOctetString, make_octet_string(config.v2c.community));
  packet = encode_sequence({version, community, pdu.encoded});
  return true;
}

bool build_v3_discovery_packet(const Snmp::Config &config, const PduEncoding &pdu,
                               int32_t message_id, bytes &packet) {
  auto version = make_tlv(kTagInteger, encode_signed_integer(kSnmpV3Version));
  bytes flags{kMsgFlagsReportable};
  auto header = encode_sequence({
      make_tlv(kTagInteger, encode_signed_integer(message_id)),
      make_tlv(kTagInteger, encode_unsigned_integer(config.max_message_size)),
      make_tlv(kTagOctetString, flags),
      make_tlv(kTagInteger, encode_signed_integer(kUsmSecurityModel)),
  });
  bytes empty_engine_id;
  bytes empty_auth;
  bytes empty_priv;
  auto security = encode_usm_security_params(empty_engine_id, 0, 0, "", empty_auth, empty_priv);
  auto scoped = encode_scoped_pdu({}, "", pdu.encoded);
  packet = encode_sequence({version, header, security, scoped});
  return true;
}

bool build_v3_authpriv_packet(const Snmp *snmp, const Snmp::Config &config,
                              const EngineCache &cache, const PduEncoding &pdu, int32_t message_id,
                              bytes &packet) {
  auto engine_time = cache.engine_time + seconds_since(cache);
  uint64_t salt_value = next_salt(snmp);
  bytes salt(8, 0);
  for (size_t i = 0; i < salt.size(); i++) {
    salt[salt.size() - i - 1] = static_cast<uint8_t>((salt_value >> (i * 8)) & 0xFF);
  }
  auto scoped = encode_scoped_pdu(cache.engine_id, config.v3.context_name, pdu.encoded);
  bytes encrypted_scoped;
  auto iv = make_iv(cache.engine_boots, engine_time, salt);
  if (!aes_cfb128_crypt(cache.priv_key, iv, scoped, encrypted_scoped, MBEDTLS_AES_ENCRYPT)) {
    return false;
  }
  auto version = make_tlv(kTagInteger, encode_signed_integer(kSnmpV3Version));
  bytes flags{static_cast<uint8_t>(kMsgFlagsAuth | kMsgFlagsPriv | kMsgFlagsReportable)};
  auto header = encode_sequence({
      make_tlv(kTagInteger, encode_signed_integer(message_id)),
      make_tlv(kTagInteger, encode_unsigned_integer(config.max_message_size)),
      make_tlv(kTagOctetString, flags),
      make_tlv(kTagInteger, encode_signed_integer(kUsmSecurityModel)),
  });
  bytes auth_placeholder(kUsmAuthParamLength, 0xAB);
  auto security = encode_usm_security_params(cache.engine_id, cache.engine_boots, engine_time,
                                             config.v3.username, auth_placeholder, salt);
  auto encrypted = make_tlv(kEncryptedPduTag, encrypted_scoped);
  packet = encode_sequence({version, header, security, encrypted});

  auto auth_it =
      std::search(packet.begin(), packet.end(), auth_placeholder.begin(), auth_placeholder.end());
  if (auth_it == packet.end()) {
    return false;
  }
  std::fill(auth_it, auth_it + kUsmAuthParamLength, 0x00);
  std::array<uint8_t, kSha1DigestLength> digest{};
  if (!hmac_sha1(cache.auth_key, packet, digest)) {
    return false;
  }
  std::copy_n(digest.begin(), kUsmAuthParamLength, auth_it);
  return true;
}

bool verify_v3_authentication(const EngineCache &cache, const V3Message &message,
                              std::span<const uint8_t> raw_packet) {
  if (message.usm.authentication_parameters.size() != kUsmAuthParamLength ||
      message.usm.auth_parameters_offset + kUsmAuthParamLength > raw_packet.size()) {
    return false;
  }
  bytes copy(raw_packet.begin(), raw_packet.end());
  std::fill(copy.begin() + message.usm.auth_parameters_offset,
            copy.begin() + message.usm.auth_parameters_offset + kUsmAuthParamLength, 0x00);
  std::array<uint8_t, kSha1DigestLength> digest{};
  if (!hmac_sha1(cache.auth_key, copy, digest)) {
    return false;
  }
  return std::equal(message.usm.authentication_parameters.begin(),
                    message.usm.authentication_parameters.end(), digest.begin());
}

bool decrypt_v3_scoped_pdu(const EngineCache &cache, const V3Message &message, bytes &scoped_pdu) {
  auto iv = make_iv(message.usm.authoritative_engine_boots, message.usm.authoritative_engine_time,
                    message.usm.privacy_parameters);
  return aes_cfb128_crypt(cache.priv_key, iv, message.encrypted_pdu, scoped_pdu,
                          MBEDTLS_AES_DECRYPT);
}

bool exchange_v2c(const Snmp *snmp, const Snmp::Config &config, const PduEncoding &pdu,
                  Snmp::Response &response, std::error_code &ec) {
  (void)snmp;
  bytes packet;
  if (!build_v2c_packet(config, pdu, packet)) {
    ec = SnmpErrc::encode_failed;
    return false;
  }
  for (size_t attempt = 0; attempt <= config.retries; attempt++) {
    bytes raw_response;
    if (!udp_round_trip(config, packet, raw_response)) {
      ec = attempt < config.retries ? SnmpErrc::timeout : SnmpErrc::transport_error;
      continue;
    }
    if (!parse_v2c_response(raw_response, response)) {
      ec = SnmpErrc::decode_failed;
      continue;
    }
    if (response.request_id != pdu.request_id) {
      ec = SnmpErrc::request_id_mismatch;
      continue;
    }
    if (response.has_error()) {
      ec = SnmpErrc::response_error;
      return false;
    }
    ec.clear();
    return true;
  }
  return false;
}

bool discover_engine(const Snmp *snmp, const Snmp::Config &config, const PduEncoding &pdu,
                     EngineCache &cache, std::error_code &ec) {
  (void)pdu;
  auto discovery_pdu = encode_pdu(Snmp::RequestType::Get, next_request_id(snmp), 0, 0, {});
  if (discovery_pdu.encoded.empty()) {
    ec = SnmpErrc::encode_failed;
    return false;
  }
  bytes packet;
  if (!build_v3_discovery_packet(config, discovery_pdu, next_message_id(snmp), packet)) {
    ec = SnmpErrc::encode_failed;
    return false;
  }
  bytes raw_response;
  if (!udp_round_trip(config, packet, raw_response)) {
    ec = SnmpErrc::discovery_failed;
    return false;
  }
  V3Message message;
  if (!parse_v3_message(raw_response, message)) {
    ec = SnmpErrc::decode_failed;
    return false;
  }
  update_engine_cache_from_response(snmp, message, cache);
  if (!cache.discovered) {
    ec = SnmpErrc::discovery_failed;
    return false;
  }
  if (!localize_engine_keys(snmp, config, cache)) {
    ec = SnmpErrc::authentication_failed;
    return false;
  }
  ec.clear();
  return true;
}

bool exchange_v3(const Snmp *snmp, const Snmp::Config &config, const PduEncoding &pdu,
                 Snmp::Response &response, std::error_code &ec) {
  if (config.v3.security_level != Snmp::SecurityLevel::AuthPriv) {
    ec = SnmpErrc::unsupported_security_level;
    return false;
  }
  if (config.v3.username.empty() || config.v3.auth_password.empty() ||
      config.v3.privacy_password.empty()) {
    ec = SnmpErrc::invalid_configuration;
    return false;
  }

  auto cache = get_engine_cache(snmp);
  if (!cache.discovered && !discover_engine(snmp, config, pdu, cache, ec)) {
    return false;
  }

  for (size_t attempt = 0; attempt <= config.retries; attempt++) {
    bytes packet;
    auto message_id = next_message_id(snmp);
    if (!build_v3_authpriv_packet(snmp, config, cache, pdu, message_id, packet)) {
      ec = SnmpErrc::encode_failed;
      return false;
    }
    bytes raw_response;
    if (!udp_round_trip(config, packet, raw_response)) {
      ec = attempt < config.retries ? SnmpErrc::timeout : SnmpErrc::transport_error;
      continue;
    }
    V3Message message;
    if (!parse_v3_message(raw_response, message)) {
      ec = SnmpErrc::decode_failed;
      continue;
    }
    update_engine_cache_from_response(snmp, message, cache);
    if (!verify_v3_authentication(cache, message, raw_response)) {
      ec = SnmpErrc::authentication_failed;
      continue;
    }
    if (message.encrypted) {
      bytes scoped_pdu;
      if (!decrypt_v3_scoped_pdu(cache, message, scoped_pdu)) {
        ec = SnmpErrc::privacy_failed;
        continue;
      }
      if (!parse_scoped_pdu(scoped_pdu, response)) {
        ec = SnmpErrc::decode_failed;
        continue;
      }
    } else {
      response = message.response;
    }
    if (response.request_id != pdu.request_id) {
      ec = SnmpErrc::request_id_mismatch;
      continue;
    }
    if (response.is_report()) {
      if (is_report_oid(response, kUsmStatsUnknownEngineIds)) {
        cache.discovered = false;
        set_engine_cache(snmp, cache);
        if (!discover_engine(snmp, config, pdu, cache, ec)) {
          return false;
        }
        continue;
      }
      if (is_report_oid(response, kUsmStatsNotInTimeWindows)) {
        cache.engine_boots = message.usm.authoritative_engine_boots;
        cache.engine_time = message.usm.authoritative_engine_time;
        cache.observed_at = std::chrono::steady_clock::now();
        set_engine_cache(snmp, cache);
        ec = SnmpErrc::not_in_time_window;
        continue;
      }
      ec = SnmpErrc::report_received;
      return false;
    }
    if (response.has_error()) {
      ec = SnmpErrc::response_error;
      return false;
    }
    ec.clear();
    return true;
  }
  return false;
}

std::string response_error_to_string(int32_t error_status, int32_t error_index) {
  std::ostringstream oss;
  oss << "SNMP response error " << error_status;
  if (error_index > 0) {
    oss << " at varbind " << error_index;
  }
  return oss.str();
}

} // namespace

namespace espp {
class SnmpErrorCategory : public std::error_category {
public:
  const char *name() const noexcept override { return "snmp"; }

  std::string message(int ev) const override {
    switch (static_cast<SnmpErrc>(ev)) {
    case SnmpErrc::success:
      return "success";
    case SnmpErrc::invalid_configuration:
      return "invalid configuration";
    case SnmpErrc::invalid_argument:
      return "invalid argument";
    case SnmpErrc::unsupported_version:
      return "unsupported SNMP version";
    case SnmpErrc::unsupported_value_type:
      return "unsupported SNMP value type";
    case SnmpErrc::unsupported_security_level:
      return "unsupported SNMP security level";
    case SnmpErrc::name_resolution_failed:
      return "failed to resolve host";
    case SnmpErrc::encode_failed:
      return "failed to encode SNMP packet";
    case SnmpErrc::decode_failed:
      return "failed to decode SNMP packet";
    case SnmpErrc::timeout:
      return "SNMP request timed out";
    case SnmpErrc::transport_error:
      return "SNMP transport error";
    case SnmpErrc::response_error:
      return "SNMP response error";
    case SnmpErrc::request_id_mismatch:
      return "SNMP response request-id mismatch";
    case SnmpErrc::report_received:
      return "SNMPv3 report received";
    case SnmpErrc::authentication_failed:
      return "SNMPv3 authentication failed";
    case SnmpErrc::privacy_failed:
      return "SNMPv3 privacy failure";
    case SnmpErrc::discovery_failed:
      return "SNMPv3 engine discovery failed";
    case SnmpErrc::not_in_time_window:
      return "SNMPv3 not in time window";
    }
    return "unknown SNMP error";
  }
};

std::error_code make_error_code(SnmpErrc e) {
  static SnmpErrorCategory category;
  return {static_cast<int>(e), category};
}
} // namespace espp

Snmp::Oid::Oid(std::initializer_list<uint32_t> init)
    : nodes(init) {}

Snmp::Oid::Oid(const std::vector<uint32_t> &values)
    : nodes(values) {}

std::optional<Snmp::Oid> Snmp::Oid::try_from_string(std::string_view oid_string) {
  Oid oid;
  uint64_t value = 0;
  bool have_digit = false;
  for (char c : oid_string) {
    if (c == '.') {
      if (!have_digit || value > std::numeric_limits<uint32_t>::max()) {
        return std::nullopt;
      }
      oid.nodes.push_back(static_cast<uint32_t>(value));
      value = 0;
      have_digit = false;
      continue;
    }
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    have_digit = true;
    value = value * 10 + static_cast<uint64_t>(c - '0');
    if (value > std::numeric_limits<uint32_t>::max()) {
      return std::nullopt;
    }
  }
  if (!have_digit) {
    return std::nullopt;
  }
  oid.nodes.push_back(static_cast<uint32_t>(value));
  return oid;
}

Snmp::Oid Snmp::Oid::from_string(std::string_view oid_string) {
  auto maybe_oid = try_from_string(oid_string);
  return maybe_oid.value_or(Oid{});
}

std::string Snmp::Oid::to_string() const {
  std::ostringstream oss;
  for (size_t i = 0; i < nodes.size(); i++) {
    if (i != 0) {
      oss << '.';
    }
    oss << nodes[i];
  }
  return oss.str();
}

bool Snmp::Oid::starts_with(const Oid &prefix) const {
  return nodes.size() >= prefix.nodes.size() &&
         std::equal(prefix.nodes.begin(), prefix.nodes.end(), nodes.begin());
}

bool Snmp::Oid::operator<(const Oid &other) const {
  return std::lexicographical_compare(nodes.begin(), nodes.end(), other.nodes.begin(),
                                      other.nodes.end());
}

std::string Snmp::IpAddress::to_string() const {
  std::ostringstream oss;
  oss << static_cast<int>(bytes[0]) << '.' << static_cast<int>(bytes[1]) << '.'
      << static_cast<int>(bytes[2]) << '.' << static_cast<int>(bytes[3]);
  return oss.str();
}

Snmp::Value Snmp::Value::integer(int32_t value) {
  return {.type = ValueType::Integer, .data = value};
}

Snmp::Value Snmp::Value::octet_string(std::string_view value) {
  return octet_string(octets_t(value.begin(), value.end()));
}

Snmp::Value Snmp::Value::octet_string(const octets_t &value) {
  return {.type = ValueType::OctetString, .data = value};
}

Snmp::Value Snmp::Value::null() { return {.type = ValueType::Null, .data = std::monostate{}}; }

Snmp::Value Snmp::Value::object_identifier(const Oid &value) {
  return {.type = ValueType::ObjectIdentifier, .data = value};
}

Snmp::Value Snmp::Value::ip_address(const IpAddress &value) {
  return {.type = ValueType::IpAddress, .data = value};
}

Snmp::Value Snmp::Value::counter32(uint32_t value) {
  return {.type = ValueType::Counter32, .data = value};
}

Snmp::Value Snmp::Value::unsigned32(uint32_t value) {
  return {.type = ValueType::Unsigned32, .data = value};
}

Snmp::Value Snmp::Value::gauge32(uint32_t value) {
  return {.type = ValueType::Gauge32, .data = value};
}

Snmp::Value Snmp::Value::time_ticks(uint32_t value) {
  return {.type = ValueType::TimeTicks, .data = value};
}

Snmp::Value Snmp::Value::opaque(const octets_t &value) {
  return {.type = ValueType::Opaque, .data = value};
}

Snmp::Value Snmp::Value::counter64(uint64_t value) {
  return {.type = ValueType::Counter64, .data = value};
}

Snmp::Value Snmp::Value::no_such_object() {
  return {.type = ValueType::NoSuchObject, .data = std::monostate{}};
}

Snmp::Value Snmp::Value::no_such_instance() {
  return {.type = ValueType::NoSuchInstance, .data = std::monostate{}};
}

Snmp::Value Snmp::Value::end_of_mib_view() {
  return {.type = ValueType::EndOfMibView, .data = std::monostate{}};
}

bool Snmp::Value::is_exception() const {
  return type == ValueType::NoSuchObject || type == ValueType::NoSuchInstance ||
         type == ValueType::EndOfMibView;
}

bool Snmp::Value::is_end_of_mib_view() const { return type == ValueType::EndOfMibView; }

std::optional<int32_t> Snmp::Value::as_integer() const {
  if (type == ValueType::Integer) {
    return std::get<int32_t>(data);
  }
  return std::nullopt;
}

std::optional<uint32_t> Snmp::Value::as_unsigned32() const {
  if (type == ValueType::Counter32 || type == ValueType::Unsigned32 || type == ValueType::Gauge32 ||
      type == ValueType::TimeTicks) {
    return std::get<uint32_t>(data);
  }
  return std::nullopt;
}

std::optional<uint64_t> Snmp::Value::as_counter64() const {
  if (type == ValueType::Counter64) {
    return std::get<uint64_t>(data);
  }
  return std::nullopt;
}

std::optional<Snmp::Value::octets_t> Snmp::Value::as_octets() const {
  if (type == ValueType::OctetString || type == ValueType::Opaque) {
    return std::get<octets_t>(data);
  }
  return std::nullopt;
}

std::optional<std::string> Snmp::Value::as_string() const {
  auto octets = as_octets();
  if (!octets.has_value()) {
    return std::nullopt;
  }
  return std::string(octets->begin(), octets->end());
}

std::optional<Snmp::Oid> Snmp::Value::as_oid() const {
  if (type == ValueType::ObjectIdentifier) {
    return std::get<Oid>(data);
  }
  return std::nullopt;
}

std::optional<Snmp::IpAddress> Snmp::Value::as_ip_address() const {
  if (type == ValueType::IpAddress) {
    return std::get<IpAddress>(data);
  }
  return std::nullopt;
}

std::string Snmp::Value::to_string() const {
  switch (type) {
  case ValueType::Integer:
    return std::to_string(std::get<int32_t>(data));
  case ValueType::OctetString:
    return as_string().value_or("");
  case ValueType::Null:
    return "null";
  case ValueType::ObjectIdentifier:
    return std::get<Oid>(data).to_string();
  case ValueType::IpAddress:
    return std::get<IpAddress>(data).to_string();
  case ValueType::Counter32:
  case ValueType::Unsigned32:
  case ValueType::Gauge32:
  case ValueType::TimeTicks:
    return std::to_string(std::get<uint32_t>(data));
  case ValueType::Opaque:
    return bytes_to_hex(std::get<octets_t>(data));
  case ValueType::Counter64:
    return std::to_string(std::get<uint64_t>(data));
  case ValueType::NoSuchObject:
    return "noSuchObject";
  case ValueType::NoSuchInstance:
    return "noSuchInstance";
  case ValueType::EndOfMibView:
    return "endOfMibView";
  }
  return {};
}

Snmp::Snmp(const Config &config)
    : BaseComponent("Snmp", config.log_level) {
  set_config(config);
}

Snmp::~Snmp() { clear_instance_state(this); }

void Snmp::set_config(const Config &config) {
  set_log_level(config.log_level);
  clear_instance_state(this);
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
}

Snmp::Config Snmp::get_config() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_;
}

bool Snmp::request(RequestType type, const std::vector<VarBind> &request_varbinds,
                   Response &response, std::error_code &ec, const BulkOptions &bulk_options) const {
  auto config = get_config();
  if (config.endpoint.host.empty() || config.endpoint.port == 0 || config.max_message_size == 0) {
    ec = SnmpErrc::invalid_configuration;
    return false;
  }
  std::string resolved_ip;
  if (!resolve_host_ipv4(config.endpoint.host, resolved_ip)) {
    ec = SnmpErrc::name_resolution_failed;
    return false;
  }

  auto request_id = next_request_id(this);
  auto pdu =
      encode_pdu(type, request_id, type == RequestType::GetBulk ? bulk_options.non_repeaters : 0,
                 type == RequestType::GetBulk ? bulk_options.max_repetitions : 0, request_varbinds);
  if (pdu.encoded.empty()) {
    ec = SnmpErrc::encode_failed;
    return false;
  }

  response = {};
  switch (config.version) {
  case Version::V2C:
    if (!exchange_v2c(this, config, pdu, response, ec)) {
      if (response.has_error()) {
        logger_.warn("{}", response_error_to_string(response.error_status, response.error_index));
      }
      return false;
    }
    return true;
  case Version::V3:
    if (!exchange_v3(this, config, pdu, response, ec)) {
      if (response.is_report() && !response.varbinds.empty()) {
        logger_.warn("SNMPv3 report: {}", response.varbinds.front().oid.to_string());
      }
      return false;
    }
    return true;
  default:
    ec = SnmpErrc::unsupported_version;
    return false;
  }
}

bool Snmp::get(const Oid &oid, Value &value, std::error_code &ec) const {
  Response response;
  if (!get(std::vector<Oid>{oid}, response, ec) || response.varbinds.empty()) {
    return false;
  }
  value = response.varbinds.front().value;
  return true;
}

bool Snmp::get(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const {
  std::vector<VarBind> varbinds;
  varbinds.reserve(oids.size());
  for (const auto &oid : oids) {
    varbinds.push_back(make_null_varbind(oid));
  }
  return request(RequestType::Get, varbinds, response, ec, BulkOptions{});
}

bool Snmp::get_next(const Oid &oid, VarBind &varbind, std::error_code &ec) const {
  Response response;
  if (!get_next(std::vector<Oid>{oid}, response, ec) || response.varbinds.empty()) {
    return false;
  }
  varbind = response.varbinds.front();
  return true;
}

bool Snmp::get_next(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const {
  std::vector<VarBind> varbinds;
  varbinds.reserve(oids.size());
  for (const auto &oid : oids) {
    varbinds.push_back(make_null_varbind(oid));
  }
  return request(RequestType::GetNext, varbinds, response, ec, BulkOptions{});
}

bool Snmp::get_bulk(const Oid &oid, std::vector<VarBind> &varbinds, std::error_code &ec) const {
  return get_bulk(oid, varbinds, ec, BulkOptions{});
}

bool Snmp::get_bulk(const Oid &oid, std::vector<VarBind> &varbinds, std::error_code &ec,
                    const BulkOptions &options) const {
  Response response;
  if (!get_bulk(std::vector<Oid>{oid}, response, ec, options)) {
    return false;
  }
  varbinds = std::move(response.varbinds);
  return true;
}

bool Snmp::get_bulk(const std::vector<Oid> &oids, Response &response, std::error_code &ec) const {
  return get_bulk(oids, response, ec, BulkOptions{});
}

bool Snmp::get_bulk(const std::vector<Oid> &oids, Response &response, std::error_code &ec,
                    const BulkOptions &options) const {
  std::vector<VarBind> varbinds;
  varbinds.reserve(oids.size());
  for (const auto &oid : oids) {
    varbinds.push_back(make_null_varbind(oid));
  }
  return request(RequestType::GetBulk, varbinds, response, ec, options);
}

bool Snmp::set(const VarBind &varbind, Response &response, std::error_code &ec) const {
  return set(std::vector<VarBind>{varbind}, response, ec);
}

bool Snmp::set(const std::vector<VarBind> &varbinds, Response &response,
               std::error_code &ec) const {
  return request(RequestType::Set, varbinds, response, ec, BulkOptions{});
}

bool Snmp::walk(const Oid &root, std::vector<VarBind> &varbinds, std::error_code &ec) const {
  return walk(root, varbinds, ec, WalkOptions{});
}

bool Snmp::walk(const Oid &root, std::vector<VarBind> &varbinds, std::error_code &ec,
                const WalkOptions &options) const {
  varbinds.clear();
  Oid cursor = root;
  while (true) {
    Response response;
    bool ok = false;
    if (options.use_get_bulk) {
      ok = get_bulk(std::vector<Oid>{cursor}, response, ec,
                    {.non_repeaters = 0, .max_repetitions = options.max_repetitions});
    } else {
      ok = get_next(std::vector<Oid>{cursor}, response, ec);
    }
    if (!ok) {
      return false;
    }
    if (response.varbinds.empty()) {
      ec.clear();
      return true;
    }
    bool reached_end = false;
    for (const auto &varbind : response.varbinds) {
      if (varbind.value.is_end_of_mib_view() || !varbind.oid.starts_with(root) ||
          !(cursor < varbind.oid)) {
        reached_end = true;
        break;
      }
      varbinds.push_back(varbind);
      cursor = varbind.oid;
      if (options.max_rows > 0 && varbinds.size() >= options.max_rows) {
        ec.clear();
        return true;
      }
    }
    if (reached_end) {
      ec.clear();
      return true;
    }
  }
}

bool Snmp::get_system_group(SystemGroup &system, std::error_code &ec) const {
  Response response;
  if (!get({sys_descr_oid(), sys_object_id_oid(), sys_up_time_oid(), sys_contact_oid(),
            sys_name_oid(), sys_location_oid()},
           response, ec)) {
    return false;
  }
  if (response.varbinds.size() != 6) {
    ec = SnmpErrc::decode_failed;
    return false;
  }
  auto sys_descr = response.varbinds[0].value.as_string();
  auto sys_object_id = response.varbinds[1].value.as_oid();
  auto sys_up_time = response.varbinds[2].value.as_unsigned32();
  auto sys_contact = response.varbinds[3].value.as_string();
  auto sys_name = response.varbinds[4].value.as_string();
  auto sys_location = response.varbinds[5].value.as_string();
  if (response.varbinds[0].value.is_exception() || response.varbinds[1].value.is_exception() ||
      response.varbinds[2].value.is_exception() || response.varbinds[3].value.is_exception() ||
      response.varbinds[4].value.is_exception() || response.varbinds[5].value.is_exception() ||
      !sys_descr.has_value() || !sys_object_id.has_value() || !sys_up_time.has_value() ||
      !sys_contact.has_value() || !sys_name.has_value() || !sys_location.has_value()) {
    ec = SnmpErrc::decode_failed;
    return false;
  }
  system.sys_descr = *sys_descr;
  system.sys_object_id = *sys_object_id;
  system.sys_up_time = *sys_up_time;
  system.sys_contact = *sys_contact;
  system.sys_name = *sys_name;
  system.sys_location = *sys_location;
  ec.clear();
  return true;
}

bool Snmp::get_interface(uint32_t if_index, InterfaceInfo &info, std::error_code &ec) const {
  Response response;
  if (!get({if_index_oid(if_index), if_descr_oid(if_index), if_type_oid(if_index),
            if_mtu_oid(if_index), if_speed_oid(if_index), if_admin_status_oid(if_index),
            if_oper_status_oid(if_index), if_alias_oid(if_index)},
           response, ec)) {
    return false;
  }
  if (response.varbinds.size() != 8) {
    ec = SnmpErrc::decode_failed;
    return false;
  }
  auto parsed_index = nonnegative_integer_to_u32(response.varbinds[0].value);
  auto if_descr = response.varbinds[1].value.as_string();
  auto if_type = nonnegative_integer_to_u32(response.varbinds[2].value);
  auto if_mtu = nonnegative_integer_to_u32(response.varbinds[3].value);
  auto if_speed = response.varbinds[4].value.as_unsigned32();
  auto if_admin = response.varbinds[5].value.as_integer();
  auto if_oper = response.varbinds[6].value.as_integer();
  auto if_alias = response.varbinds[7].value.is_exception()
                      ? std::optional<std::string>("")
                      : response.varbinds[7].value.as_string();
  if (response.varbinds[0].value.is_exception() || response.varbinds[1].value.is_exception() ||
      response.varbinds[2].value.is_exception() || response.varbinds[3].value.is_exception() ||
      response.varbinds[4].value.is_exception() || response.varbinds[5].value.is_exception() ||
      response.varbinds[6].value.is_exception() || !parsed_index.has_value() ||
      !if_descr.has_value() || !if_type.has_value() || !if_mtu.has_value() ||
      !if_speed.has_value() || !if_admin.has_value() || !if_oper.has_value() ||
      !if_alias.has_value()) {
    ec = SnmpErrc::decode_failed;
    return false;
  }
  info.if_index = *parsed_index;
  info.if_descr = *if_descr;
  info.if_type = *if_type;
  info.if_mtu = *if_mtu;
  info.if_speed = *if_speed;
  info.if_admin_status = static_cast<InterfaceAdminStatus>(*if_admin);
  info.if_oper_status = static_cast<InterfaceOperStatus>(*if_oper);
  info.if_alias = *if_alias;
  ec.clear();
  return true;
}

bool Snmp::walk_interfaces(std::vector<InterfaceInfo> &interfaces, std::error_code &ec) const {
  std::vector<VarBind> index_varbinds;
  if (!walk(Oid::from_string("1.3.6.1.2.1.2.2.1.1"), index_varbinds, ec)) {
    return false;
  }
  interfaces.clear();
  for (const auto &varbind : index_varbinds) {
    auto index = nonnegative_integer_to_u32(varbind.value);
    if (!index.has_value()) {
      continue;
    }
    InterfaceInfo info;
    if (!get_interface(*index, info, ec)) {
      return false;
    }
    interfaces.push_back(std::move(info));
  }
  ec.clear();
  return true;
}

bool Snmp::set_system_name(std::string_view name, std::error_code &ec) const {
  Response response;
  return set({.oid = sys_name_oid(), .value = Value::octet_string(name)}, response, ec);
}

bool Snmp::set_system_contact(std::string_view contact, std::error_code &ec) const {
  Response response;
  return set({.oid = sys_contact_oid(), .value = Value::octet_string(contact)}, response, ec);
}

bool Snmp::set_system_location(std::string_view location, std::error_code &ec) const {
  Response response;
  return set({.oid = sys_location_oid(), .value = Value::octet_string(location)}, response, ec);
}

bool Snmp::set_interface_admin_status(uint32_t if_index, InterfaceAdminStatus status,
                                      std::error_code &ec) const {
  Response response;
  return set(
      {.oid = if_admin_status_oid(if_index), .value = Value::integer(static_cast<int32_t>(status))},
      response, ec);
}

bool Snmp::set_interface_alias(uint32_t if_index, std::string_view alias,
                               std::error_code &ec) const {
  Response response;
  return set({.oid = if_alias_oid(if_index), .value = Value::octet_string(alias)}, response, ec);
}

Snmp::Oid Snmp::sys_descr_oid() { return Oid::from_string("1.3.6.1.2.1.1.1.0"); }
Snmp::Oid Snmp::sys_object_id_oid() { return Oid::from_string("1.3.6.1.2.1.1.2.0"); }
Snmp::Oid Snmp::sys_up_time_oid() { return Oid::from_string("1.3.6.1.2.1.1.3.0"); }
Snmp::Oid Snmp::sys_contact_oid() { return Oid::from_string("1.3.6.1.2.1.1.4.0"); }
Snmp::Oid Snmp::sys_name_oid() { return Oid::from_string("1.3.6.1.2.1.1.5.0"); }
Snmp::Oid Snmp::sys_location_oid() { return Oid::from_string("1.3.6.1.2.1.1.6.0"); }
Snmp::Oid Snmp::if_index_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.1"), if_index);
}
Snmp::Oid Snmp::if_descr_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.2"), if_index);
}
Snmp::Oid Snmp::if_type_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.3"), if_index);
}
Snmp::Oid Snmp::if_mtu_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.4"), if_index);
}
Snmp::Oid Snmp::if_speed_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.5"), if_index);
}
Snmp::Oid Snmp::if_admin_status_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.7"), if_index);
}
Snmp::Oid Snmp::if_oper_status_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.2.2.1.8"), if_index);
}
Snmp::Oid Snmp::if_alias_oid(uint32_t if_index) {
  return append_index(Oid::from_string("1.3.6.1.2.1.31.1.1.1.18"), if_index);
}
