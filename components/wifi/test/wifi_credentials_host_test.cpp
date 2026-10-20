// Host-buildable test for the credential field helpers shared by WifiSta and
// WifiAp (include/wifi_credentials.hpp -- standard library only). Build & run:
//   c++ -std=c++23 -Wall -Wextra -Werror -I components/wifi/include \
//       components/wifi/test/wifi_credentials_host_test.cpp -o test && ./test

#include <cstdio>
#include <cstring>
#include <string>

#include "wifi_credentials.hpp"

namespace creds = espp::wifi_credentials;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// the field as ESP-IDF reads it: NUL-terminated unless it is completely full
static std::string as_string(const uint8_t *field, size_t n) {
  size_t len = 0;
  while (len < n && field[len] != 0)
    len++;
  return std::string(reinterpret_cast<const char *>(field), len);
}

int main() {
  // ---- the reported bug: a shorter password kept the old trailing bytes ----
  {
    uint8_t password[creds::kPasswordFieldSize];
    CHECK(creds::set_field(password, "1234567890"));
    CHECK(as_string(password, sizeof(password)) == "1234567890");
    CHECK(creds::set_field(password, "abcdefgh"));
    CHECK(as_string(password, sizeof(password)) == "abcdefgh"); // not "abcdefgh90"
    // every byte past the value is zero, not just the one after it
    bool clean = true;
    for (size_t i = 8; i < sizeof(password); i++)
      clean = clean && password[i] == 0;
    CHECK(clean);
  }
  // ---- the same for the SSID ----
  {
    uint8_t ssid[creds::kSsidFieldSize];
    CHECK(creds::set_field(ssid, "a-long-network-name"));
    CHECK(creds::set_field(ssid, "net"));
    CHECK(as_string(ssid, sizeof(ssid)) == "net");
  }
  // ---- an empty value clears the field (an open network) ----
  {
    uint8_t password[creds::kPasswordFieldSize];
    CHECK(creds::set_field(password, "secret"));
    CHECK(creds::set_field(password, ""));
    CHECK(as_string(password, sizeof(password)).empty());
  }
  // ---- a value that exactly fills the field is accepted, one byte more is refused ----
  {
    uint8_t ssid[creds::kSsidFieldSize];
    const std::string full(creds::kSsidFieldSize, 'x');
    CHECK(creds::set_field(ssid, full));
    CHECK(std::memcmp(ssid, full.data(), full.size()) == 0);
    const std::string previous = full;
    CHECK(!creds::set_field(ssid, full + "y"));                      // refused ...
    CHECK(std::memcmp(ssid, previous.data(), previous.size()) == 0); // ... and the field untouched
    uint8_t password[creds::kPasswordFieldSize];
    CHECK(creds::set_field(password, std::string(creds::kPasswordFieldSize, 'p')));
    CHECK(!creds::set_field(password, std::string(creds::kPasswordFieldSize + 1, 'p')));
  }
  // ---- fits() mirrors set_field()'s acceptance ----
  CHECK(creds::fits("", 32) && creds::fits(std::string(32, 'a'), 32) &&
        !creds::fits(std::string(33, 'a'), 32));

  if (g_failures == 0)
    std::printf("wifi_credentials_host_test: all checks passed\n");
  else
    std::printf("wifi_credentials_host_test: %d check(s) FAILED\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
