#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "switch2_pro_protocol.hpp"

/// @file switch2_pro_pairing.hpp
/// @brief Switch 2 controller pairing key derivation (the cracked handshake).

namespace espp::switch2 {

/// Link key derivation for the Switch 2 pairing handshake.
///
/// The console sends a 16-byte "public key" A1; the controller replies with the
/// fixed constant B1 (CONTROLLER_KEY_B1). Both sides then form
/// `LTK = A1 ⊕ B1`. To confirm possession, the console sends a challenge A2 and
/// the controller returns `B2 = AES-128-ECB(reverse(LTK), reverse(A2))` (both
/// the key and the block are byte-reversed for the cipher operation).
struct PairingCrypto {
  /// LTK = A1 ⊕ B1.
  static std::array<uint8_t, 16> derive_ltk(const std::array<uint8_t, 16> &a1) {
    std::array<uint8_t, 16> ltk{};
    for (size_t i = 0; i < 16; ++i)
      ltk[i] = a1[i] ^ CONTROLLER_KEY_B1[i];
    return ltk;
  }

  /// B2 = AES-128-ECB(key = reverse(ltk), data = reverse(a2)).
  /// Returns the confirmation to send back to the console. Implemented in the
  /// .cpp against mbedTLS.
  static std::array<uint8_t, 16> confirm(const std::array<uint8_t, 16> &ltk,
                                         const std::array<uint8_t, 16> &a2);

  /// Runs derive_ltk()/confirm() against the golden vector and returns true iff
  /// both match. Intended to be logged at init as an on-device sanity check.
  static bool self_test();
};

} // namespace espp::switch2
