#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace espp {
/// Helpers for the fixed-size SSID / password fields of ESP-IDF's
/// `wifi_config_t` (`wifi_sta_config_t` and `wifi_ap_config_t`), shared by
/// espp::WifiSta and espp::WifiAp. Standard-library only (host-testable, see
/// test/wifi_credentials_host_test.cpp).
namespace wifi_credentials {

/// Size of the SSID fields (`wifi_sta_config_t::ssid`, `wifi_ap_config_t::ssid`).
static constexpr size_t kSsidFieldSize = 32;
/// Size of the password fields; a WPA passphrase is 8..63 characters and a
/// raw 64-hex-digit PSK fills the field.
static constexpr size_t kPasswordFieldSize = 64;

/// @brief Whether `value` fits a credential field of `field_size` bytes.
constexpr bool fits(std::string_view value, size_t field_size) {
  return value.size() <= field_size;
}

/**
 * @brief Store a credential in a fixed-size field, replacing it completely.
 *
 * The whole field is cleared before the copy, so a value shorter than the one
 * the field held before never keeps the old trailing bytes (a password
 * "1234567890" replaced by "abcdefgh" used to become "abcdefgh90"). A value
 * that fills the field exactly is allowed (ESP-IDF treats such a field as not
 * NUL-terminated); an empty value clears the field.
 *
 * @param field The destination array (its size is deduced).
 * @param value The new content.
 * @return true when the value fit and was stored; false (field untouched) when
 *         it is longer than the field.
 */
template <size_t N> bool set_field(uint8_t (&field)[N], std::string_view value) {
  if (!fits(value, N))
    return false;
  std::memset(field, 0, N);
  if (!value.empty())
    std::memcpy(field, value.data(), value.size());
  return true;
}

} // namespace wifi_credentials
} // namespace espp
