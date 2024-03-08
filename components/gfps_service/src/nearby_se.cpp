#include "gfps.hpp"

static espp::Logger logger({.tag = "GFPS SE", .level = espp::gfps::LOG_LEVEL});

static constexpr char *kAntiSpoofingPrivateKey = CONFIG_GFPS_ANTISPOOFING_PRIVATE_KEY;
static std::string decoded_anti_spoofing_private_key;

// Generates a random number.
uint8_t nearby_platform_Rand() { return (uint8_t)esp_random(); }

#if !defined(NEARBY_PLATFORM_USE_MBEDTLS)

// Computes the sha256 incrementally. Sha256Start() is called first, then
// Sha256Update() one or more times, and finally Sha256Finish().
nearby_platform_status nearby_platform_Sha256Start() {
  logger.warn("nearby_platform_Sha256Start not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

// Intermediate sha256 compute.
//
// data   - Block of data to compute into sha256.
// length - Length of data to process.
nearby_platform_status nearby_platform_Sha256Update(const void *data, size_t length) {
  logger.warn("nearby_platform_Sha256Update not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

// Finishes sha256 compute.
//
// out - Contains the final 256 bit sha.
nearby_platform_status nearby_platform_Sha256Finish(uint8_t out[32]) {
  logger.warn("nearby_platform_Sha256Finish not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

// Encrypts a data block with AES128 in ECB mode.
//
// input - Input data block to be encrypted.
// output - Resulting encrypted block.
// key    - 128 bit key to use for encryption.
nearby_platform_status nearby_platform_Aes128Encrypt(const uint8_t input[AES_MESSAGE_SIZE_BYTES],
                                                     uint8_t output[AES_MESSAGE_SIZE_BYTES],
                                                     const uint8_t key[AES_MESSAGE_SIZE_BYTES]) {
  logger.warn("nearby_platform_Aes128Encrypt not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

// Decrypts a data block with AES128 in ECB mode.
//
// input - Input data block to be decrypted.
// output - Resulting decrypted block.
// key    - 128 bit key to use for decryption.
nearby_platform_status nearby_platform_Aes128Decrypt(const uint8_t input[AES_MESSAGE_SIZE_BYTES],
                                                     uint8_t output[AES_MESSAGE_SIZE_BYTES],
                                                     const uint8_t key[AES_MESSAGE_SIZE_BYTES]) {
  logger.warn("nearby_platform_Aes128Decrypt not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

#endif // !defined(NEARBY_PLATFORM_USE_MBEDTLS)

#if defined(NEARBY_PLATFORM_HAS_SE)

// Generates a shared sec256p1 secret using remote party public key and this
// device's private key.
//
// remote_party_public_key - Remote key.
// secret                  - 256 bit shared secret.
nearby_platform_status nearby_platform_GenSec256r1Secret(const uint8_t remote_party_public_key[64],
                                                         uint8_t secret[32]) {
  logger.warn("nearby_platform_GenSec256r1Secret not implemented");
  // TODO: Implement.
  return kNearbyStatusOK;
}

#endif // defined(NEARBY_PLATFORM_HAS_SE)

// from https://stackoverflow.com/a/44562527
std::string base64_decode(const std::string_view in) {
  // table from '+' to 'z'
  const uint8_t lookup[] = {62,  255, 62,  255, 63,  52,  53, 54, 55, 56, 57, 58, 59, 60, 61, 255,
                            255, 0,   255, 255, 255, 255, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
                            10,  11,  12,  13,  14,  15,  16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                            255, 255, 255, 255, 63,  255, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
                            36,  37,  38,  39,  40,  41,  42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
  static_assert(sizeof(lookup) == 'z' - '+' + 1);

  std::string out;
  int val = 0, valb = -8;
  for (uint8_t c : in) {
    if (c < '+' || c > 'z')
      break;
    c -= '+';
    if (lookup[c] >= 64)
      break;
    val = (val << 6) + lookup[c];
    valb += 6;
    if (valb >= 0) {
      out.push_back(char((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return out;
}

// Returns anti-spoofing 128 bit private key.
// Only used if the implementation also uses the
// nearby_platform_GenSec256r1Secret() routine defined in gen_secret.c.
// Return NULL if not implemented.
const uint8_t *nearby_platform_GetAntiSpoofingPrivateKey() {
  return reinterpret_cast<const uint8_t *>(decoded_anti_spoofing_private_key.data());
}

// Initializes secure element module
nearby_platform_status nearby_platform_SecureElementInit() {
  static std::string_view key(kAntiSpoofingPrivateKey);
  decoded_anti_spoofing_private_key = base64_decode(key);
  return kNearbyStatusOK;
}
