#include "android_tv_remote.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"
#include "esp_tls.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/error.h"
#include "mbedtls/pk.h"
#include "mbedtls/rsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mdns.h"
#include "nvs.hpp"

namespace {
constexpr uint32_t kPairingProtocolVersion = 2;
constexpr uint32_t kPairingStatusOk = 200;
constexpr uint32_t kFeaturePing = 1u << 0;
constexpr uint32_t kFeatureKey = 1u << 1;
constexpr uint32_t kFeatureIme = 1u << 2;
constexpr uint32_t kFeatureVoice = 1u << 3;
constexpr uint32_t kFeatureUnknown1 = 1u << 4;
constexpr uint32_t kFeaturePower = 1u << 5;
constexpr uint32_t kFeatureVolume = 1u << 6;
constexpr uint32_t kFeatureAppLink = 1u << 9;

enum class WireType : uint8_t { Varint = 0, LengthDelimited = 2 };

struct ProtoField {
  uint32_t number{0};
  WireType wire_type{WireType::Varint};
  uint64_t varint_value{0};
  const uint8_t *bytes{nullptr};
  size_t size{0};
};

template <typename Fn> bool for_each_proto_field(const uint8_t *data, size_t size, Fn &&fn) {
  size_t offset = 0;
  while (offset < size) {
    uint64_t tag = 0;
    int shift = 0;
    while (true) {
      if (offset >= size || shift > 63)
        return false;
      uint8_t byte = data[offset++];
      tag |= static_cast<uint64_t>(byte & 0x7f) << shift;
      if ((byte & 0x80) == 0)
        break;
      shift += 7;
    }
    ProtoField field;
    field.number = static_cast<uint32_t>(tag >> 3);
    field.wire_type = static_cast<WireType>(tag & 0x07);
    if (field.wire_type == WireType::Varint) {
      uint64_t value = 0;
      shift = 0;
      while (true) {
        if (offset >= size || shift > 63)
          return false;
        uint8_t byte = data[offset++];
        value |= static_cast<uint64_t>(byte & 0x7f) << shift;
        if ((byte & 0x80) == 0)
          break;
        shift += 7;
      }
      field.varint_value = value;
    } else if (field.wire_type == WireType::LengthDelimited) {
      uint64_t length = 0;
      shift = 0;
      while (true) {
        if (offset >= size || shift > 63)
          return false;
        uint8_t byte = data[offset++];
        length |= static_cast<uint64_t>(byte & 0x7f) << shift;
        if ((byte & 0x80) == 0)
          break;
        shift += 7;
      }
      if (offset + length > size)
        return false;
      field.bytes = data + offset;
      field.size = static_cast<size_t>(length);
      offset += length;
    } else {
      return false;
    }
    if (!fn(field))
      return false;
  }
  return true;
}

void append_varint(uint64_t value, std::vector<uint8_t> &buffer) {
  do {
    uint8_t byte = value & 0x7f;
    value >>= 7;
    if (value)
      byte |= 0x80;
    buffer.push_back(byte);
  } while (value);
}

void append_tag(uint32_t field_number, WireType wire_type, std::vector<uint8_t> &buffer) {
  append_varint((static_cast<uint64_t>(field_number) << 3) | static_cast<uint8_t>(wire_type),
                buffer);
}

void append_uint32(uint32_t field_number, uint32_t value, std::vector<uint8_t> &buffer) {
  append_tag(field_number, WireType::Varint, buffer);
  append_varint(value, buffer);
}

void append_bool(uint32_t field_number, bool value, std::vector<uint8_t> &buffer) {
  append_uint32(field_number, value ? 1u : 0u, buffer);
}

void append_bytes(uint32_t field_number, const uint8_t *data, size_t size,
                  std::vector<uint8_t> &buffer) {
  append_tag(field_number, WireType::LengthDelimited, buffer);
  append_varint(size, buffer);
  buffer.insert(buffer.end(), data, data + size);
}

void append_string(uint32_t field_number, std::string_view value, std::vector<uint8_t> &buffer) {
  append_bytes(field_number, reinterpret_cast<const uint8_t *>(value.data()), value.size(), buffer);
}

void append_message(uint32_t field_number, const std::vector<uint8_t> &message,
                    std::vector<uint8_t> &buffer) {
  append_bytes(field_number, message.data(), message.size(), buffer);
}

std::vector<uint8_t> make_pairing_request(std::string_view client_name) {
  std::vector<uint8_t> pairing_request;
  append_string(1, "atvremote", pairing_request);
  append_string(2, client_name, pairing_request);

  std::vector<uint8_t> message;
  append_uint32(1, kPairingProtocolVersion, message);
  append_uint32(2, kPairingStatusOk, message);
  append_message(10, pairing_request, message);
  return message;
}

std::vector<uint8_t> make_pairing_options() {
  std::vector<uint8_t> encoding;
  append_uint32(1, 3, encoding);
  append_uint32(2, 6, encoding);

  std::vector<uint8_t> options;
  append_message(1, encoding, options);
  append_uint32(3, 1, options);

  std::vector<uint8_t> message;
  append_uint32(1, kPairingProtocolVersion, message);
  append_uint32(2, kPairingStatusOk, message);
  append_message(20, options, message);
  return message;
}

std::vector<uint8_t> make_pairing_configuration() {
  std::vector<uint8_t> encoding;
  append_uint32(1, 3, encoding);
  append_uint32(2, 6, encoding);

  std::vector<uint8_t> configuration;
  append_message(1, encoding, configuration);
  append_uint32(2, 1, configuration);

  std::vector<uint8_t> message;
  append_uint32(1, kPairingProtocolVersion, message);
  append_uint32(2, kPairingStatusOk, message);
  append_message(30, configuration, message);
  return message;
}

std::vector<uint8_t> make_pairing_secret(const std::vector<uint8_t> &secret) {
  std::vector<uint8_t> inner;
  append_bytes(1, secret.data(), secret.size(), inner);

  std::vector<uint8_t> message;
  append_uint32(1, kPairingProtocolVersion, message);
  append_uint32(2, kPairingStatusOk, message);
  append_message(40, inner, message);
  return message;
}

struct ParsedPairingMessage {
  uint32_t status{kPairingStatusOk};
  bool has_pairing_request_ack{false};
  bool has_options{false};
  bool has_configuration_ack{false};
  bool has_secret_ack{false};
};

bool parse_pairing_message(const std::vector<uint8_t> &data, ParsedPairingMessage &message) {
  return for_each_proto_field(data.data(), data.size(), [&](const ProtoField &field) {
    if (field.number == 2 && field.wire_type == WireType::Varint) {
      message.status = static_cast<uint32_t>(field.varint_value);
    } else if (field.number == 11 && field.wire_type == WireType::LengthDelimited) {
      message.has_pairing_request_ack = true;
    } else if (field.number == 20 && field.wire_type == WireType::LengthDelimited) {
      message.has_options = true;
    } else if (field.number == 31 && field.wire_type == WireType::LengthDelimited) {
      message.has_configuration_ack = true;
    } else if (field.number == 41 && field.wire_type == WireType::LengthDelimited) {
      message.has_secret_ack = true;
    }
    return true;
  });
}

struct ParsedRemoteMessage {
  enum class Type {
    Unknown,
    Configure,
    SetActive,
    Error,
    PingRequest,
    KeyInject,
    ImeKeyInject,
    ImeBatchEdit,
    Start,
    SetVolumeLevel,
  };

  Type type{Type::Unknown};
  uint32_t code1{0};
  bool started{false};
  int32_t ping_value{0};
  int32_t ime_counter{0};
  int32_t field_counter{0};
  std::string manufacturer;
  std::string model;
  std::string app_version;
  std::string app_package;
  uint32_t volume_max{0};
  uint32_t volume_level{0};
  bool volume_muted{false};
};

bool parse_remote_configure(const uint8_t *data, size_t size, ParsedRemoteMessage &message) {
  return for_each_proto_field(data, size, [&](const ProtoField &field) {
    if (field.number == 1 && field.wire_type == WireType::Varint) {
      message.code1 = static_cast<uint32_t>(field.varint_value);
    } else if (field.number == 2 && field.wire_type == WireType::LengthDelimited) {
      return for_each_proto_field(field.bytes, field.size, [&](const ProtoField &device_field) {
        if (device_field.wire_type != WireType::LengthDelimited)
          return true;
        if (device_field.number == 1) {
          message.model.assign(reinterpret_cast<const char *>(device_field.bytes),
                               device_field.size);
        } else if (device_field.number == 2) {
          message.manufacturer.assign(reinterpret_cast<const char *>(device_field.bytes),
                                      device_field.size);
        } else if (device_field.number == 6) {
          message.app_version.assign(reinterpret_cast<const char *>(device_field.bytes),
                                     device_field.size);
        }
        return true;
      });
    }
    return true;
  });
}

bool parse_remote_ime_key_inject(const uint8_t *data, size_t size, ParsedRemoteMessage &message) {
  return for_each_proto_field(data, size, [&](const ProtoField &field) {
    if (field.number == 1 && field.wire_type == WireType::LengthDelimited) {
      return for_each_proto_field(field.bytes, field.size, [&](const ProtoField &app_field) {
        if (app_field.number == 12 && app_field.wire_type == WireType::LengthDelimited) {
          message.app_package.assign(reinterpret_cast<const char *>(app_field.bytes),
                                     app_field.size);
        }
        return true;
      });
    }
    return true;
  });
}

bool parse_remote_ime_batch_edit(const uint8_t *data, size_t size, ParsedRemoteMessage &message) {
  return for_each_proto_field(data, size, [&](const ProtoField &field) {
    if (field.number == 1 && field.wire_type == WireType::Varint) {
      message.ime_counter = static_cast<int32_t>(field.varint_value);
    } else if (field.number == 2 && field.wire_type == WireType::Varint) {
      message.field_counter = static_cast<int32_t>(field.varint_value);
    }
    return true;
  });
}

bool parse_remote_set_volume_level(const uint8_t *data, size_t size, ParsedRemoteMessage &message) {
  return for_each_proto_field(data, size, [&](const ProtoField &field) {
    if (field.number == 6 && field.wire_type == WireType::Varint) {
      message.volume_max = static_cast<uint32_t>(field.varint_value);
    } else if (field.number == 7 && field.wire_type == WireType::Varint) {
      message.volume_level = static_cast<uint32_t>(field.varint_value);
    } else if (field.number == 8 && field.wire_type == WireType::Varint) {
      message.volume_muted = field.varint_value != 0;
    }
    return true;
  });
}

bool parse_remote_message(const std::vector<uint8_t> &data, ParsedRemoteMessage &message) {
  return for_each_proto_field(data.data(), data.size(), [&](const ProtoField &field) {
    if (field.number == 1 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::Configure;
      return parse_remote_configure(field.bytes, field.size, message);
    }
    if (field.number == 2 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::SetActive;
      return true;
    }
    if (field.number == 3 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::Error;
      return true;
    }
    if (field.number == 8 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::PingRequest;
      return for_each_proto_field(field.bytes, field.size, [&](const ProtoField &ping_field) {
        if (ping_field.number == 1 && ping_field.wire_type == WireType::Varint) {
          message.ping_value = static_cast<int32_t>(ping_field.varint_value);
        }
        return true;
      });
    }
    if (field.number == 20 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::ImeKeyInject;
      return parse_remote_ime_key_inject(field.bytes, field.size, message);
    }
    if (field.number == 21 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::ImeBatchEdit;
      return parse_remote_ime_batch_edit(field.bytes, field.size, message);
    }
    if (field.number == 40 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::Start;
      return for_each_proto_field(field.bytes, field.size, [&](const ProtoField &start_field) {
        if (start_field.number == 1 && start_field.wire_type == WireType::Varint) {
          message.started = start_field.varint_value != 0;
        }
        return true;
      });
    }
    if (field.number == 50 && field.wire_type == WireType::LengthDelimited) {
      message.type = ParsedRemoteMessage::Type::SetVolumeLevel;
      return parse_remote_set_volume_level(field.bytes, field.size, message);
    }
    return true;
  });
}

std::vector<uint8_t> make_remote_configure(uint32_t active_features) {
  std::vector<uint8_t> device_info;
  append_uint32(3, 1, device_info);
  append_string(4, "1", device_info);
  append_string(5, "atvremote", device_info);
  append_string(6, "1.0.0", device_info);

  std::vector<uint8_t> configure;
  append_uint32(1, active_features, configure);
  append_message(2, device_info, configure);

  std::vector<uint8_t> message;
  append_message(1, configure, message);
  return message;
}

std::vector<uint8_t> make_remote_set_active(uint32_t active_features) {
  std::vector<uint8_t> set_active;
  append_uint32(1, active_features, set_active);

  std::vector<uint8_t> message;
  append_message(2, set_active, message);
  return message;
}

std::vector<uint8_t> make_remote_ping_response(int32_t ping_value) {
  std::vector<uint8_t> ping_response;
  append_uint32(1, static_cast<uint32_t>(ping_value), ping_response);

  std::vector<uint8_t> message;
  append_message(9, ping_response, message);
  return message;
}

std::vector<uint8_t> make_remote_key(uint32_t key_code, uint32_t direction) {
  std::vector<uint8_t> key;
  append_uint32(1, key_code, key);
  append_uint32(2, direction, key);

  std::vector<uint8_t> message;
  append_message(10, key, message);
  return message;
}

std::vector<uint8_t> make_remote_text(std::string_view text, int32_t ime_counter,
                                      int32_t field_counter) {
  int32_t cursor = text.empty() ? 0 : static_cast<int32_t>(text.size()) - 1;

  std::vector<uint8_t> ime_object;
  append_uint32(1, static_cast<uint32_t>(cursor), ime_object);
  append_uint32(2, static_cast<uint32_t>(cursor), ime_object);
  append_string(3, text, ime_object);

  std::vector<uint8_t> edit_info;
  append_uint32(1, 1, edit_info);
  append_message(2, ime_object, edit_info);

  std::vector<uint8_t> batch_edit;
  append_uint32(1, static_cast<uint32_t>(ime_counter), batch_edit);
  append_uint32(2, static_cast<uint32_t>(field_counter), batch_edit);
  append_message(3, edit_info, batch_edit);

  std::vector<uint8_t> message;
  append_message(21, batch_edit, message);
  return message;
}

std::string trim_nul(std::string value) {
  while (!value.empty() && value.back() == '\0')
    value.pop_back();
  return value;
}

std::string uppercase_hex(std::string_view input) {
  std::string normalized;
  normalized.reserve(input.size());
  for (char c : input) {
    if (std::isxdigit(static_cast<unsigned char>(c)))
      normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  }
  return normalized;
}

bool decode_hex(std::string_view hex, std::vector<uint8_t> &bytes) {
  if (hex.size() % 2 != 0)
    return false;
  bytes.clear();
  bytes.reserve(hex.size() / 2);
  for (size_t i = 0; i < hex.size(); i += 2) {
    auto high = std::toupper(static_cast<unsigned char>(hex[i]));
    auto low = std::toupper(static_cast<unsigned char>(hex[i + 1]));
    auto nibble = [](char c) -> int {
      if (c >= '0' && c <= '9')
        return c - '0';
      if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
      return -1;
    };
    int h = nibble(high);
    int l = nibble(low);
    if (h < 0 || l < 0)
      return false;
    bytes.push_back(static_cast<uint8_t>((h << 4) | l));
  }
  return true;
}

std::string sanitize_common_name(std::string_view input) {
  std::string value;
  value.reserve(input.size());
  for (char c : input) {
    if (c == ',' || c == ';' || c == '=')
      value.push_back('_');
    else
      value.push_back(c);
  }
  if (value.empty())
    value = "ESPP Android TV Remote";
  return value;
}

std::vector<uint8_t> mpi_to_bytes(const mbedtls_mpi &mpi) {
  size_t size = mbedtls_mpi_size(&mpi);
  std::vector<uint8_t> bytes(size ? size : 1, 0);
  if (size > 0)
    mbedtls_mpi_write_binary(&mpi, bytes.data(), bytes.size());
  return bytes;
}

bool extract_public_key_bytes(const mbedtls_pk_context &pk, std::vector<uint8_t> &modulus,
                              std::vector<uint8_t> &exponent) {
  if (!mbedtls_pk_can_do(&pk, MBEDTLS_PK_RSA))
    return false;
  auto *rsa = mbedtls_pk_rsa(pk);
  modulus = mpi_to_bytes(rsa->N);
  exponent = mpi_to_bytes(rsa->E);
  return true;
}

bool parse_certificate_public_key(std::string_view cert_pem, std::vector<uint8_t> &modulus,
                                  std::vector<uint8_t> &exponent) {
  std::vector<unsigned char> pem(cert_pem.begin(), cert_pem.end());
  pem.push_back('\0');
  mbedtls_x509_crt cert;
  mbedtls_x509_crt_init(&cert);
  int ret = mbedtls_x509_crt_parse(&cert, pem.data(), pem.size());
  if (ret != 0) {
    mbedtls_x509_crt_free(&cert);
    return false;
  }
  bool ok = extract_public_key_bytes(cert.pk, modulus, exponent);
  mbedtls_x509_crt_free(&cert);
  return ok;
}

bool generate_self_signed_identity(std::string_view common_name, std::string &cert_pem,
                                   std::string &key_pem) {
  mbedtls_pk_context key;
  mbedtls_x509write_cert crt;
  mbedtls_mpi serial;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_pk_init(&key);
  mbedtls_x509write_crt_init(&crt);
  mbedtls_mpi_init(&serial);
  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&ctr_drbg);

  bool ok = false;
  std::string subject;
  const char *personalization = "espp_android_tv_remote";
  int ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                  reinterpret_cast<const unsigned char *>(personalization),
                                  strlen(personalization));
  if (ret != 0)
    goto cleanup;

  ret = mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_RSA));
  if (ret != 0)
    goto cleanup;

  ret = mbedtls_rsa_gen_key(mbedtls_pk_rsa(key), mbedtls_ctr_drbg_random, &ctr_drbg, 2048, 65537);
  if (ret != 0)
    goto cleanup;

  ret = mbedtls_mpi_fill_random(&serial, 16, mbedtls_ctr_drbg_random, &ctr_drbg);
  if (ret != 0)
    goto cleanup;
  if (mbedtls_mpi_cmp_int(&serial, 0) == 0) {
    ret = mbedtls_mpi_lset(&serial, 1);
    if (ret != 0)
      goto cleanup;
  }

  subject = "CN=" + sanitize_common_name(common_name);
  mbedtls_x509write_crt_set_subject_key(&crt, &key);
  mbedtls_x509write_crt_set_issuer_key(&crt, &key);
  mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
  mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);

  ret = mbedtls_x509write_crt_set_subject_name(&crt, subject.c_str());
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_issuer_name(&crt, subject.c_str());
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_serial(&crt, &serial);
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_validity(&crt, "20240101000000", "20501231235959");
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_basic_constraints(&crt, 0, -1);
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_subject_key_identifier(&crt);
  if (ret != 0)
    goto cleanup;
  ret = mbedtls_x509write_crt_set_authority_key_identifier(&crt);
  if (ret != 0)
    goto cleanup;

  {
    std::vector<unsigned char> buffer(4096, 0);
    ret = mbedtls_x509write_crt_pem(&crt, buffer.data(), buffer.size(), mbedtls_ctr_drbg_random,
                                    &ctr_drbg);
    if (ret != 0)
      goto cleanup;
    cert_pem = reinterpret_cast<const char *>(buffer.data());
  }

  {
    std::vector<unsigned char> buffer(4096, 0);
    ret = mbedtls_pk_write_key_pem(&key, buffer.data(), buffer.size());
    if (ret != 0)
      goto cleanup;
    key_pem = reinterpret_cast<const char *>(buffer.data());
  }

  ok = true;

cleanup:
  mbedtls_pk_free(&key);
  mbedtls_x509write_crt_free(&crt);
  mbedtls_mpi_free(&serial);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  mbedtls_entropy_free(&entropy);
  return ok;
}

class TlsConnection {
public:
  explicit TlsConnection(espp::Logger &logger)
      : logger_(logger) {}

  ~TlsConnection() { close(); }

  bool connect(std::string_view host, uint16_t port, const std::string &certificate_pem,
               const std::string &private_key_pem, int timeout_ms, int handshake_timeout_ms,
               bool skip_common_name_check, std::error_code &ec) {
    close();
    tls_.reset(esp_tls_init());
    if (!tls_) {
      ec = espp::AndroidTvRemoteErrc::tls_error;
      return false;
    }

    esp_tls_cfg_t cfg = {};
    cfg.clientcert_buf = reinterpret_cast<const unsigned char *>(certificate_pem.c_str());
    cfg.clientcert_bytes = static_cast<unsigned int>(certificate_pem.size() + 1);
    cfg.clientkey_buf = reinterpret_cast<const unsigned char *>(private_key_pem.c_str());
    cfg.clientkey_bytes = static_cast<unsigned int>(private_key_pem.size() + 1);
    cfg.timeout_ms = timeout_ms;
    cfg.tls_handshake_timeout_ms = handshake_timeout_ms;
    cfg.skip_common_name = skip_common_name_check;

    int ret =
        esp_tls_conn_new_sync(host.data(), static_cast<int>(host.size()), port, &cfg, tls_.get());
    if (ret <= 0) {
      logger_.warn("TLS connect to {}:{} failed", std::string(host), port);
      close();
      ec = espp::AndroidTvRemoteErrc::connect_failed;
      return false;
    }

    if (esp_tls_get_conn_sockfd(tls_.get(), &sockfd_) != ESP_OK || sockfd_ < 0) {
      close();
      ec = espp::AndroidTvRemoteErrc::tls_error;
      return false;
    }
    return true;
  }

  void close() {
    sockfd_ = -1;
    if (tls_) {
      esp_tls_conn_destroy(tls_.release());
    }
    rx_buffer_.clear();
  }

  bool is_open() const { return tls_ != nullptr; }

  bool get_peer_public_key(std::vector<uint8_t> &modulus, std::vector<uint8_t> &exponent,
                           std::error_code &ec) const {
    if (!tls_) {
      ec = espp::AndroidTvRemoteErrc::not_connected;
      return false;
    }
    auto *ssl = static_cast<mbedtls_ssl_context *>(esp_tls_get_ssl_context(tls_.get()));
    if (!ssl) {
      ec = espp::AndroidTvRemoteErrc::tls_error;
      return false;
    }
    const mbedtls_x509_crt *peer_cert = mbedtls_ssl_get_peer_cert(ssl);
    if (!peer_cert || !extract_public_key_bytes(peer_cert->pk, modulus, exponent)) {
      ec = espp::AndroidTvRemoteErrc::protocol_error;
      return false;
    }
    return true;
  }

  bool write_frame(const std::vector<uint8_t> &payload, std::error_code &ec) {
    if (!tls_) {
      ec = espp::AndroidTvRemoteErrc::not_connected;
      return false;
    }

    std::vector<uint8_t> framed;
    framed.reserve(payload.size() + 8);
    append_varint(payload.size(), framed);
    framed.insert(framed.end(), payload.begin(), payload.end());

    size_t offset = 0;
    while (offset < framed.size()) {
      ssize_t written =
          esp_tls_conn_write(tls_.get(), framed.data() + offset, framed.size() - offset);
      if (written <= 0) {
        ec = espp::AndroidTvRemoteErrc::transport_error;
        return false;
      }
      offset += static_cast<size_t>(written);
    }
    return true;
  }

  bool read_frame(std::vector<uint8_t> &payload, int timeout_ms, std::error_code &ec) {
    payload.clear();
    for (;;) {
      if (try_extract_frame(payload)) {
        return true;
      }

      if (!tls_) {
        ec = espp::AndroidTvRemoteErrc::not_connected;
        return false;
      }

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sockfd_, &read_fds);
      timeval timeout = {
          .tv_sec = timeout_ms / 1000,
          .tv_usec = (timeout_ms % 1000) * 1000,
      };
      int ready = select(sockfd_ + 1, &read_fds, nullptr, nullptr, &timeout);
      if (ready == 0) {
        ec = espp::AndroidTvRemoteErrc::timeout;
        return false;
      }
      if (ready < 0) {
        ec = espp::AndroidTvRemoteErrc::transport_error;
        return false;
      }

      std::array<uint8_t, 1024> buffer{};
      ssize_t read = esp_tls_conn_read(tls_.get(), buffer.data(), buffer.size());
      if (read <= 0) {
        ec = espp::AndroidTvRemoteErrc::transport_error;
        return false;
      }
      rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + read);
    }
  }

private:
  bool try_extract_frame(std::vector<uint8_t> &payload) {
    if (rx_buffer_.empty())
      return false;

    uint64_t frame_size = 0;
    size_t prefix_size = 0;
    int shift = 0;
    while (prefix_size < rx_buffer_.size()) {
      uint8_t byte = rx_buffer_[prefix_size++];
      frame_size |= static_cast<uint64_t>(byte & 0x7f) << shift;
      if ((byte & 0x80) == 0)
        break;
      shift += 7;
      if (shift > 63)
        return false;
    }

    if (prefix_size == rx_buffer_.size() && (rx_buffer_.back() & 0x80)) {
      return false;
    }
    if (rx_buffer_.size() < prefix_size + frame_size)
      return false;

    payload.assign(rx_buffer_.begin() + prefix_size, rx_buffer_.begin() + prefix_size + frame_size);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + prefix_size + frame_size);
    return true;
  }

  espp::Logger &logger_;
  struct EspTlsDeleter {
    void operator()(esp_tls_t *tls) const {
      if (tls)
        esp_tls_conn_destroy(tls);
    }
  };
  std::unique_ptr<esp_tls_t, EspTlsDeleter> tls_{nullptr};
  int sockfd_{-1};
  std::vector<uint8_t> rx_buffer_;
};
} // namespace

namespace espp {

class AndroidTvRemoteCategory : public std::error_category {
public:
  const char *name() const noexcept override { return "android_tv_remote"; }

  std::string message(int ev) const override {
    switch (static_cast<AndroidTvRemoteErrc>(ev)) {
    case AndroidTvRemoteErrc::success:
      return "success";
    case AndroidTvRemoteErrc::invalid_configuration:
      return "invalid configuration";
    case AndroidTvRemoteErrc::invalid_argument:
      return "invalid argument";
    case AndroidTvRemoteErrc::not_ready:
      return "not ready";
    case AndroidTvRemoteErrc::nvs_error:
      return "nvs error";
    case AndroidTvRemoteErrc::discovery_failed:
      return "discovery failed";
    case AndroidTvRemoteErrc::pairing_failed:
      return "pairing failed";
    case AndroidTvRemoteErrc::tls_error:
      return "tls error";
    case AndroidTvRemoteErrc::connect_failed:
      return "connect failed";
    case AndroidTvRemoteErrc::handshake_failed:
      return "handshake failed";
    case AndroidTvRemoteErrc::transport_error:
      return "transport error";
    case AndroidTvRemoteErrc::timeout:
      return "timeout";
    case AndroidTvRemoteErrc::protocol_error:
      return "protocol error";
    case AndroidTvRemoteErrc::unsupported:
      return "unsupported";
    case AndroidTvRemoteErrc::not_connected:
      return "not connected";
    case AndroidTvRemoteErrc::canceled:
      return "canceled";
    }
    return "unknown android tv remote error";
  }
};

std::error_code make_error_code(AndroidTvRemoteErrc e) {
  static AndroidTvRemoteCategory category;
  return {static_cast<int>(e), category};
}

struct AndroidTvRemote::Impl {
  explicit Impl(AndroidTvRemote &owner, const Config &config)
      : owner(owner)
      , config(config)
      , logger({.tag = "AndroidTvRemote", .level = config.log_level})
      , control(logger) {}

  uint32_t requested_features() const {
    return kFeaturePing | kFeatureKey | kFeaturePower | kFeatureVolume | kFeatureAppLink |
           (config.transport.enable_ime ? kFeatureIme : 0u);
  }

  bool validate_config(std::error_code &ec) const {
    if (config.persistence.nvs_namespace.empty() || config.persistence.nvs_namespace.size() > 15 ||
        config.persistence.certificate_key.empty() ||
        config.persistence.certificate_key.size() > 15 ||
        config.persistence.private_key_key.empty() ||
        config.persistence.private_key_key.size() > 15) {
      ec = AndroidTvRemoteErrc::invalid_configuration;
      return false;
    }
    return true;
  }

  bool ensure_identity(std::error_code &ec) {
    if (!certificate_pem.empty() && !private_key_pem.empty())
      return true;

    if (!validate_config(ec))
      return false;

    espp::Nvs nvs;
    nvs.init(ec);
    if (ec) {
      ec = AndroidTvRemoteErrc::nvs_error;
      return false;
    }

    espp::NvsHandle handle(config.persistence.nvs_namespace, ec);
    if (ec) {
      ec = AndroidTvRemoteErrc::nvs_error;
      return false;
    }

    std::error_code read_ec;
    handle.get(config.persistence.certificate_key, certificate_pem, read_ec);
    if (!read_ec)
      certificate_pem = trim_nul(certificate_pem);
    read_ec.clear();
    handle.get(config.persistence.private_key_key, private_key_pem, read_ec);
    if (!read_ec)
      private_key_pem = trim_nul(private_key_pem);

    if (!certificate_pem.empty() && !private_key_pem.empty())
      return true;

    certificate_pem.clear();
    private_key_pem.clear();
    if (!generate_self_signed_identity(config.pairing.client_name, certificate_pem,
                                       private_key_pem)) {
      ec = AndroidTvRemoteErrc::tls_error;
      return false;
    }

    handle.set(config.persistence.certificate_key, certificate_pem, ec);
    if (ec) {
      ec = AndroidTvRemoteErrc::nvs_error;
      return false;
    }
    handle.set(config.persistence.private_key_key, private_key_pem, ec);
    if (ec) {
      ec = AndroidTvRemoteErrc::nvs_error;
      return false;
    }
    handle.commit(ec);
    if (ec) {
      ec = AndroidTvRemoteErrc::nvs_error;
      return false;
    }
    return true;
  }

  bool discover(std::vector<DeviceInfo> &devices, std::error_code &ec) {
    devices.clear();
    if (!validate_config(ec))
      return false;

    if (config.discovery.initialize_mdns) {
      esp_err_t mdns_err = mdns_init();
      if (mdns_err != ESP_OK && mdns_err != ESP_ERR_INVALID_STATE) {
        ec = AndroidTvRemoteErrc::discovery_failed;
        return false;
      }
    }

    mdns_result_t *results = nullptr;
    esp_err_t err = mdns_query_ptr("_googlecast", "_tcp", config.discovery.timeout_ms,
                                   config.discovery.max_results, &results);
    if (err != ESP_OK) {
      ec = AndroidTvRemoteErrc::discovery_failed;
      return false;
    }

    for (auto *result = results; result; result = result->next) {
      DeviceInfo device;
      if (result->instance_name)
        device.name = result->instance_name;
      if (result->hostname)
        device.hostname = result->hostname;
      device.port = config.transport.port;

      for (auto *address = result->addr; address; address = address->next) {
        if (address->addr.type == ESP_IPADDR_TYPE_V4) {
          in_addr in_addr_value{};
          in_addr_value.s_addr = address->addr.u_addr.ip4.addr;
          const char *text = inet_ntoa(in_addr_value);
          if (text) {
            device.host = text;
            break;
          }
        }
      }

      for (size_t i = 0; i < result->txt_count; i++) {
        const char *key = result->txt[i].key ? result->txt[i].key : "";
        const char *value = result->txt[i].value ? result->txt[i].value : "";
        device.txt[key] = value;
      }

      auto friendly_name = device.txt.find("fn");
      if (friendly_name != device.txt.end() && !friendly_name->second.empty()) {
        device.name = friendly_name->second;
      }

      if (device.host.empty() && !device.hostname.empty()) {
        esp_ip4_addr_t address{};
        if (mdns_query_a(device.hostname.c_str(), config.discovery.timeout_ms, &address) ==
            ESP_OK) {
          in_addr in_addr_value{};
          in_addr_value.s_addr = address.addr;
          const char *text = inet_ntoa(in_addr_value);
          if (text)
            device.host = text;
        }
      }

      if (!device.host.empty())
        devices.push_back(device);
    }
    mdns_query_results_free(results);
    return true;
  }

  bool pair(std::string_view host, const std::function<std::optional<std::string>()> &code_provider,
            std::error_code &ec) {
    if (host.empty()) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }
    if (!ensure_identity(ec))
      return false;

    TlsConnection pairing(logger);
    if (!pairing.connect(host, config.pairing.port, certificate_pem, private_key_pem,
                         static_cast<int>(config.transport.connect_timeout.count()),
                         static_cast<int>(config.transport.connect_timeout.count()),
                         config.transport.skip_common_name_check, ec)) {
      return false;
    }

    if (!pairing.write_frame(make_pairing_request(config.pairing.client_name), ec))
      return false;

    auto start = std::chrono::steady_clock::now();
    bool pairing_ready = false;
    while (!pairing_ready) {
      auto elapsed = std::chrono::steady_clock::now() - start;
      auto remaining = config.pairing.timeout - elapsed;
      if (remaining <= std::chrono::milliseconds::zero()) {
        ec = AndroidTvRemoteErrc::timeout;
        return false;
      }

      std::vector<uint8_t> frame;
      if (!pairing.read_frame(frame, static_cast<int>(remaining.count()), ec)) {
        return false;
      }

      ParsedPairingMessage message;
      if (!parse_pairing_message(frame, message)) {
        ec = AndroidTvRemoteErrc::protocol_error;
        return false;
      }
      if (message.status != kPairingStatusOk) {
        ec = AndroidTvRemoteErrc::pairing_failed;
        return false;
      }

      if (message.has_pairing_request_ack) {
        if (!pairing.write_frame(make_pairing_options(), ec))
          return false;
      } else if (message.has_options) {
        if (!pairing.write_frame(make_pairing_configuration(), ec))
          return false;
      } else if (message.has_configuration_ack) {
        pairing_ready = true;
      }
    }

    if (!code_provider) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }
    auto provided_code = code_provider();
    if (!provided_code) {
      ec = AndroidTvRemoteErrc::canceled;
      return false;
    }

    std::string code = uppercase_hex(*provided_code);
    if (code.size() != 6) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }

    std::vector<uint8_t> client_modulus;
    std::vector<uint8_t> client_exponent;
    if (!parse_certificate_public_key(certificate_pem, client_modulus, client_exponent)) {
      ec = AndroidTvRemoteErrc::tls_error;
      return false;
    }

    std::vector<uint8_t> server_modulus;
    std::vector<uint8_t> server_exponent;
    if (!pairing.get_peer_public_key(server_modulus, server_exponent, ec))
      return false;

    std::vector<uint8_t> secret_suffix;
    if (!decode_hex(code.substr(2), secret_suffix) || secret_suffix.size() != 2) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }

    std::vector<uint8_t> prefix;
    if (!decode_hex(code.substr(0, 2), prefix) || prefix.size() != 1) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    mbedtls_sha256_update(&sha, client_modulus.data(), client_modulus.size());
    mbedtls_sha256_update(&sha, client_exponent.data(), client_exponent.size());
    mbedtls_sha256_update(&sha, server_modulus.data(), server_modulus.size());
    mbedtls_sha256_update(&sha, server_exponent.data(), server_exponent.size());
    mbedtls_sha256_update(&sha, secret_suffix.data(), secret_suffix.size());
    std::vector<uint8_t> secret(32, 0);
    mbedtls_sha256_finish(&sha, secret.data());
    mbedtls_sha256_free(&sha);

    if (secret.front() != prefix.front()) {
      ec = AndroidTvRemoteErrc::pairing_failed;
      return false;
    }

    if (!pairing.write_frame(make_pairing_secret(secret), ec))
      return false;

    while (true) {
      auto elapsed = std::chrono::steady_clock::now() - start;
      auto remaining = config.pairing.timeout - elapsed;
      if (remaining <= std::chrono::milliseconds::zero()) {
        ec = AndroidTvRemoteErrc::timeout;
        return false;
      }

      std::vector<uint8_t> frame;
      if (!pairing.read_frame(frame, static_cast<int>(remaining.count()), ec)) {
        return false;
      }

      ParsedPairingMessage message;
      if (!parse_pairing_message(frame, message)) {
        ec = AndroidTvRemoteErrc::protocol_error;
        return false;
      }
      if (message.status != kPairingStatusOk) {
        ec = AndroidTvRemoteErrc::pairing_failed;
        return false;
      }
      if (message.has_secret_ack) {
        return true;
      }
    }
  }

  bool connect(std::string_view host, std::error_code &ec) {
    disconnect();
    if (host.empty()) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }
    if (!ensure_identity(ec))
      return false;

    {
      std::scoped_lock lock(state_mutex);
      active_features = requested_features();
      remote_started = false;
      async_error.reset();
      ime_counter = 0;
      ime_field_counter = 0;
      connected_host = std::string(host);
      connected = false;
      stop_requested = false;
    }

    if (!control.connect(host, config.transport.port, certificate_pem, private_key_pem,
                         static_cast<int>(config.transport.connect_timeout.count()),
                         static_cast<int>(config.transport.connect_timeout.count()),
                         config.transport.skip_common_name_check, ec)) {
      return false;
    }

    reader_thread = std::thread([this] { reader_loop(); });

    std::unique_lock lock(state_mutex);
    bool ready = state_cv.wait_for(lock, config.transport.handshake_timeout,
                                   [&] { return remote_started || async_error.has_value(); });
    if (!ready) {
      lock.unlock();
      ec = AndroidTvRemoteErrc::timeout;
      disconnect();
      return false;
    }
    if (async_error) {
      ec = *async_error;
      lock.unlock();
      disconnect();
      return false;
    }
    connected = true;
    return true;
  }

  void disconnect() {
    stop_requested = true;
    connected = false;
    control.close();
    if (reader_thread.joinable() && reader_thread.get_id() != std::this_thread::get_id()) {
      reader_thread.join();
    }
    {
      std::scoped_lock lock(state_mutex);
      remote_started = false;
    }
  }

  bool send_key(Key key, Action action, std::error_code &ec) {
    if (!connected) {
      ec = AndroidTvRemoteErrc::not_connected;
      return false;
    }
    std::scoped_lock lock(io_mutex);
    return control.write_frame(
        make_remote_key(static_cast<uint32_t>(key), static_cast<uint32_t>(action)), ec);
  }

  bool send_text(std::string_view text, std::error_code &ec) {
    if (!connected) {
      ec = AndroidTvRemoteErrc::not_connected;
      return false;
    }
    if (text.empty()) {
      ec = AndroidTvRemoteErrc::invalid_argument;
      return false;
    }
    if ((active_features & kFeatureIme) == 0) {
      ec = AndroidTvRemoteErrc::unsupported;
      return false;
    }
    std::scoped_lock lock(io_mutex);
    return control.write_frame(make_remote_text(text, ime_counter, ime_field_counter), ec);
  }

  void set_async_error(std::error_code ec) {
    {
      std::scoped_lock lock(state_mutex);
      if (!async_error)
        async_error = ec;
    }
    state_cv.notify_all();
  }

  void reader_loop() {
    while (!stop_requested) {
      std::vector<uint8_t> frame;
      std::error_code ec;
      if (!control.read_frame(frame, static_cast<int>(config.transport.read_poll_timeout.count()),
                              ec)) {
        if (ec == AndroidTvRemoteErrc::timeout)
          continue;
        if (!stop_requested)
          set_async_error(ec);
        break;
      }

      ParsedRemoteMessage message;
      if (!parse_remote_message(frame, message)) {
        set_async_error(AndroidTvRemoteErrc::protocol_error);
        break;
      }

      std::vector<uint8_t> response;
      switch (message.type) {
      case ParsedRemoteMessage::Type::Configure:
        active_features &= message.code1;
        response = make_remote_configure(active_features);
        break;
      case ParsedRemoteMessage::Type::SetActive:
        response = make_remote_set_active(active_features);
        break;
      case ParsedRemoteMessage::Type::PingRequest:
        response = make_remote_ping_response(message.ping_value);
        break;
      case ParsedRemoteMessage::Type::ImeBatchEdit:
        ime_counter = message.ime_counter;
        ime_field_counter = message.field_counter;
        break;
      case ParsedRemoteMessage::Type::ImeKeyInject:
        current_app = message.app_package;
        break;
      case ParsedRemoteMessage::Type::Start: {
        std::scoped_lock lock(state_mutex);
        remote_started = true;
      }
        state_cv.notify_all();
        break;
      case ParsedRemoteMessage::Type::SetVolumeLevel:
        volume_level = message.volume_level;
        volume_max = message.volume_max;
        volume_muted = message.volume_muted;
        break;
      case ParsedRemoteMessage::Type::Error:
        set_async_error(AndroidTvRemoteErrc::protocol_error);
        break;
      case ParsedRemoteMessage::Type::Unknown:
      case ParsedRemoteMessage::Type::KeyInject:
        break;
      }

      if (!response.empty()) {
        std::scoped_lock lock(io_mutex);
        if (!control.write_frame(response, ec)) {
          set_async_error(ec);
          break;
        }
      }
    }
    connected = false;
  }

  AndroidTvRemote &owner;
  Config config;
  Logger logger;
  TlsConnection control;
  std::string certificate_pem;
  std::string private_key_pem;
  std::string connected_host;
  std::string current_app;
  std::thread reader_thread;
  std::atomic<bool> connected{false};
  std::atomic<bool> stop_requested{false};
  std::mutex io_mutex;
  mutable std::mutex state_mutex;
  std::condition_variable state_cv;
  std::optional<std::error_code> async_error;
  uint32_t active_features{0};
  int32_t ime_counter{0};
  int32_t ime_field_counter{0};
  uint32_t volume_level{0};
  uint32_t volume_max{0};
  bool volume_muted{false};
  bool remote_started{false};
  std::optional<DeviceInfo> last_device_info;
};

AndroidTvRemote::AndroidTvRemote(const Config &config)
    : BaseComponent("AndroidTvRemote", config.log_level)
    , impl_(std::make_unique<Impl>(*this, config)) {}

AndroidTvRemote::~AndroidTvRemote() { disconnect(); }

bool AndroidTvRemote::discover(std::vector<DeviceInfo> &devices, std::error_code &ec) {
  bool ok = impl_->discover(devices, ec);
  if (ok && !devices.empty())
    impl_->last_device_info = devices.front();
  return ok;
}

bool AndroidTvRemote::pair(std::string_view host,
                           const std::function<std::optional<std::string>()> &code_provider,
                           std::error_code &ec) {
  return impl_->pair(host, code_provider, ec);
}

bool AndroidTvRemote::connect(std::string_view host, std::error_code &ec) {
  return impl_->connect(host, ec);
}

void AndroidTvRemote::disconnect() { impl_->disconnect(); }

bool AndroidTvRemote::is_connected() const { return impl_->connected; }

std::optional<AndroidTvRemote::DeviceInfo> AndroidTvRemote::get_last_device_info() const {
  std::scoped_lock lock(impl_->state_mutex);
  return impl_->last_device_info;
}

bool AndroidTvRemote::send_key(Key key, Action action, std::error_code &ec) {
  return impl_->send_key(key, action, ec);
}

bool AndroidTvRemote::send_text(std::string_view text, std::error_code &ec) {
  return impl_->send_text(text, ec);
}
} // namespace espp
