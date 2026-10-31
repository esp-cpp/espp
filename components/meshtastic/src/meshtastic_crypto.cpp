#include "meshtastic_crypto.hpp"

#include <array>
#include <cstring>

#include <mbedtls/build_info.h> // for MBEDTLS_VERSION_MAJOR
#if MBEDTLS_VERSION_MAJOR >= 4
// mbedtls 4.x (ESP-IDF v6+) moved the low-level AES API to a private header; the
// mbedtls_aes_* functions used below are unchanged and still available there.
// (PSA Crypto is the recommended long-term API.)
#include <mbedtls/private/aes.h>
#else
#include <mbedtls/aes.h>
#endif

namespace espp::meshtastic {

std::span<const uint8_t, 16> default_psk() {
  // the well-known Meshtastic default channel key
  static constexpr std::array<uint8_t, 16> psk = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                                  0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};
  return std::span<const uint8_t, 16>{psk};
}

std::vector<uint8_t> expand_psk(std::span<const uint8_t> psk) {
  if (psk.empty() || (psk.size() == 1 && psk[0] == 0)) {
    return {}; // encryption disabled
  }
  if (psk.size() == 1) {
    auto def = default_psk();
    std::vector<uint8_t> key(def.begin(), def.end());
    key.back() += psk[0] - 1;
    return key;
  }
  if (psk.size() == 16 || psk.size() == 32) {
    return std::vector<uint8_t>(psk.begin(), psk.end());
  }
  return {}; // invalid key length
}

bool crypt_payload(std::span<const uint8_t> key, uint32_t packet_id, uint32_t from_node,
                   std::span<uint8_t> data) {
  if (key.size() != 16 && key.size() != 32) {
    return false;
  }
  // nonce / initial counter block: packet id (LE u64) then sender (LE u64)
  uint8_t nonce[16] = {0};
  nonce[0] = packet_id & 0xff;
  nonce[1] = (packet_id >> 8) & 0xff;
  nonce[2] = (packet_id >> 16) & 0xff;
  nonce[3] = (packet_id >> 24) & 0xff;
  nonce[8] = from_node & 0xff;
  nonce[9] = (from_node >> 8) & 0xff;
  nonce[10] = (from_node >> 16) & 0xff;
  nonce[11] = (from_node >> 24) & 0xff;

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  int ret = mbedtls_aes_setkey_enc(&ctx, key.data(), key.size() * 8);
  if (ret != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  size_t nc_off = 0;
  uint8_t stream_block[16] = {0};
  ret = mbedtls_aes_crypt_ctr(&ctx, data.size(), &nc_off, nonce, stream_block, data.data(),
                              data.data());
  mbedtls_aes_free(&ctx);
  return ret == 0;
}

} // namespace espp::meshtastic
