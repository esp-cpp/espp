#include "switch2_pro_pairing.hpp"

#include <psa/crypto.h>

#include "esp_log.h"

namespace {
constexpr const char *kPairingTag = "switch2::pairing";
} // namespace

namespace espp::switch2 {

namespace {
std::array<uint8_t, 16> reversed(const std::array<uint8_t, 16> &in) {
  std::array<uint8_t, 16> out{};
  for (size_t i = 0; i < 16; ++i)
    out[i] = in[15 - i];
  return out;
}
} // namespace

std::array<uint8_t, 16> PairingCrypto::confirm(const std::array<uint8_t, 16> &ltk,
                                               const std::array<uint8_t, 16> &a2) {
  const auto key = reversed(ltk);
  const auto block = reversed(a2);
  std::array<uint8_t, 16> out{};

  // AES-128-ECB single-block encrypt via the PSA Crypto API (the supported
  // interface in mbedTLS 4.x / IDF 6; the classic mbedtls_aes_* API is private).
  if (psa_crypto_init() != PSA_SUCCESS) {
    ESP_LOGE(kPairingTag, "psa_crypto_init failed");
    return out; // zeros — self_test() flags it, live pairing fails cleanly
  }
  psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
  psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
  psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
  psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
  psa_set_key_bits(&attr, 128);

  psa_key_id_t key_id = 0;
  if (psa_import_key(&attr, key.data(), key.size(), &key_id) != PSA_SUCCESS) {
    psa_reset_key_attributes(&attr);
    return out; // zeros on failure; self_test() will flag it
  }
  size_t out_len = 0;
  psa_status_t st = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, block.data(), block.size(),
                                       out.data(), out.size(), &out_len);
  psa_destroy_key(key_id);
  psa_reset_key_attributes(&attr);
  if (st != PSA_SUCCESS || out_len != out.size()) {
    ESP_LOGE(kPairingTag, "psa_cipher_encrypt failed (status=%d, out_len=%u)", static_cast<int>(st),
             static_cast<unsigned>(out_len));
    out.fill(0); // don't return a partially-written block
    return out;
  }
  return out;
}

bool PairingCrypto::self_test() {
  const auto ltk = derive_ltk(golden::A1);
  if (ltk != golden::LTK)
    return false;
  const auto b2 = confirm(ltk, golden::A2);
  return b2 == golden::B2;
}

} // namespace espp::switch2
