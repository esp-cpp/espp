#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace espp::meshtastic {

/// The well-known default channel pre-shared key (16 bytes, AES-128). A
/// 1-byte PSK of value N is shorthand for this key with (N - 1) added to the
/// final byte; the default channel uses N = 1 (i.e. exactly this key).
std::span<const uint8_t, 16> default_psk();

/// Expand a configured PSK into an AES key:
/// - empty or {0}: encryption disabled (returns empty)
/// - 1 byte with value N: the default key with (N - 1) added to its last byte
/// - 16 bytes: used directly as an AES-128 key
/// - 32 bytes: used directly as an AES-256 key
/// \param psk The configured PSK
/// \return The expanded key (0, 16, or 32 bytes)
std::vector<uint8_t> expand_psk(std::span<const uint8_t> psk);

/// Encrypt or decrypt a packet payload in place using AES-CTR with the
/// Meshtastic nonce construction (packet id and sender node number, little
/// endian). CTR mode is symmetric, so the same function is used for both
/// directions.
/// \param key The AES key (16 bytes for AES-128 or 32 bytes for AES-256)
/// \param packet_id The packet id from the header
/// \param from_node The sender node number from the header
/// \param data The payload to encrypt / decrypt, modified in place
/// \return True on success
bool crypt_payload(std::span<const uint8_t> key, uint32_t packet_id, uint32_t from_node,
                   std::span<uint8_t> data);

} // namespace espp::meshtastic
