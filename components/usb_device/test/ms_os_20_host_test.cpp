// Host-buildable unit test for the Microsoft OS 2.0 descriptor set builder in
// include/ms_os_20.hpp. No ESP-IDF / TinyUSB headers required.
//
/* Build & run (a block comment so the line-continuation doesn't trip -Wcomment):
     c++ -std=c++23 -Wall -Wextra -Werror -I components/usb_device/include \
         components/usb_device/test/ms_os_20_host_test.cpp -o test && ./test       */

#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "ms_os_20.hpp"

using namespace espp::ms_os_20;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond);                                \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static uint16_t rd16(const std::vector<uint8_t> &d, size_t i) {
  return static_cast<uint16_t>(d[i] | (d[i + 1] << 8));
}
static uint32_t rd32(const std::vector<uint8_t> &d, size_t i) {
  return static_cast<uint32_t>(rd16(d, i) | (static_cast<uint32_t>(rd16(d, i + 2)) << 16));
}
static std::string ascii(const std::vector<uint8_t> &d, size_t i, size_t n) {
  std::string s;
  for (size_t k = 0; k < n && d[i + k] != 0; ++k)
    s.push_back(static_cast<char>(d[i + k]));
  return s;
}
static std::string utf16(const std::vector<uint8_t> &d, size_t i, size_t units) {
  std::string s;
  for (size_t k = 0; k < units; ++k) {
    const uint16_t c = rd16(d, i + 2 * k);
    if (c == 0)
      break;
    s.push_back(static_cast<char>(c));
  }
  return s;
}

// Walk a built set and check every length field against the bytes.
static void check_structure(const std::vector<uint8_t> &d, size_t expected_functions) {
  CHECK(rd16(d, 0) == kSetHeaderLen);
  CHECK(rd16(d, 2) == kSetHeaderDescriptor);
  CHECK(rd32(d, 4) == kWindowsVersion);
  CHECK(rd16(d, 8) == d.size());
  size_t i = kSetHeaderLen;
  CHECK(rd16(d, i) == kConfigurationSubsetHeaderLen);
  CHECK(rd16(d, i + 2) == kSubsetHeaderConfiguration);
  CHECK(d[i + 4] == 0 && d[i + 5] == 0);
  CHECK(rd16(d, i + 6) == d.size() - kSetHeaderLen);
  i += kConfigurationSubsetHeaderLen;
  size_t functions = 0;
  while (i < d.size()) {
    CHECK(rd16(d, i) == kFunctionSubsetHeaderLen);
    CHECK(rd16(d, i + 2) == kSubsetHeaderFunction);
    const size_t subset_len = rd16(d, i + 6);
    const size_t subset_end = i + subset_len;
    CHECK(subset_end <= d.size());
    if (subset_len < kFunctionSubsetHeaderLen || subset_end > d.size())
      break; // a short or runaway subset would never advance
    size_t j = i + kFunctionSubsetHeaderLen;
    bool saw_compat = false;
    while (j < subset_end) {
      const size_t len = rd16(d, j);
      const uint16_t type = rd16(d, j + 2);
      CHECK(len > 0 && j + len <= subset_end);
      if (len == 0)
        break; // a zero length would never advance
      if (type == kFeatureCompatibleId) {
        CHECK(len == kCompatibleIdFeatureLen);
        saw_compat = true;
      } else {
        CHECK(type == kFeatureRegProperty && len == kRegPropertyFeatureLen);
      }
      j += len;
    }
    CHECK(j == subset_end);
    CHECK(saw_compat);
    i = subset_end;
    ++functions;
  }
  CHECK(i == d.size());
  CHECK(functions == expected_functions);
}

int main() {
  // --- WinUSB only: the set TinyUSB's webusb_serial example ships (0xB2 bytes) ---
  {
    const Function fns[] = {{.first_interface = 2,
                             .compatible_id = kCompatibleIdWinUsb,
                             .device_interface_guid = true}};
    const auto d = descriptor_set(fns);
    CHECK(d.size() == 0xB2);
    CHECK(descriptor_set_length(fns) == 0xB2);
    check_structure(d, 1);
    const size_t fn = kSetHeaderLen + kConfigurationSubsetHeaderLen;
    CHECK(d[fn + 4] == 2); // bFirstInterface
    CHECK(rd16(d, fn + 6) == 0xB2 - 0x0A - 0x08);
    const size_t compat = fn + kFunctionSubsetHeaderLen;
    CHECK(ascii(d, compat + 4, 8) == "WINUSB");
    CHECK(ascii(d, compat + 12, 8).empty()); // sub-compatible ID blank
    const size_t prop = compat + kCompatibleIdFeatureLen;
    CHECK(rd16(d, prop) == kRegPropertyFeatureLen);
    CHECK(rd16(d, prop + 2) == kFeatureRegProperty);
    CHECK(rd16(d, prop + 4) == 7);    // REG_MULTI_SZ
    CHECK(rd16(d, prop + 6) == 0x2A); // name length
    CHECK(utf16(d, prop + 8, 21) == "DeviceInterfaceGUIDs");
    CHECK(rd16(d, prop + 8 + 0x2A) == 0x50); // data length
    CHECK(utf16(d, prop + 8 + 0x2A + 2, 40) == std::string(kDeviceInterfaceGuid));
    // the REG_MULTI_SZ ends with two UTF-16 NULs
    CHECK(rd16(d, d.size() - 4) == 0 && rd16(d, d.size() - 2) == 0);
  }

  // --- WinUSB (interface 0) + X-Input (interface 1): a composite controller ---
  {
    const Function fns[] = {
        {.first_interface = 0, .compatible_id = kCompatibleIdWinUsb, .device_interface_guid = true},
        {.first_interface = 1, .compatible_id = kCompatibleIdXusb10},
    };
    const auto d = descriptor_set(fns);
    CHECK(d.size() == 0xB2 + kFunctionSubsetHeaderLen + kCompatibleIdFeatureLen);
    check_structure(d, 2);
    const size_t fn2 = 0xB2;
    CHECK(d[fn2 + 4] == 1);
    CHECK(rd16(d, fn2 + 6) == kFunctionSubsetHeaderLen + kCompatibleIdFeatureLen);
    CHECK(ascii(d, fn2 + kFunctionSubsetHeaderLen + 4, 8) == "XUSB10");
  }

  // --- too long for the 16-bit length fields: refused, not truncated ---
  {
    std::vector<Function> many(
        kMaxSetLength / (kFunctionSubsetHeaderLen + kCompatibleIdFeatureLen) + 1,
        Function{.first_interface = 0, .compatible_id = kCompatibleIdXusb10});
    CHECK(descriptor_set_length(many) > kMaxSetLength);
    CHECK(descriptor_set(many).empty());
    many.pop_back();
    many.pop_back();
    CHECK(descriptor_set_length(many) <= kMaxSetLength);
    const auto d = descriptor_set(many);
    CHECK(d.size() == descriptor_set_length(many));
    CHECK(rd16(d, 8) == d.size());
  }

  // --- the other order, and an X-Input function alone ---
  {
    const Function fns[] = {
        {.first_interface = 0, .compatible_id = kCompatibleIdXusb10},
        {.first_interface = 1, .compatible_id = kCompatibleIdWinUsb, .device_interface_guid = true},
    };
    const auto d = descriptor_set(fns);
    check_structure(d, 2);
    CHECK(d[kSetHeaderLen + kConfigurationSubsetHeaderLen + 4] == 0);
    const Function alone[] = {{.first_interface = 0, .compatible_id = kCompatibleIdXusb10}};
    const auto a = descriptor_set(alone);
    CHECK(a.size() == kSetHeaderLen + kConfigurationSubsetHeaderLen + kFunctionSubsetHeaderLen +
                          kCompatibleIdFeatureLen);
    check_structure(a, 1);
  }

  // --- a compatible ID longer than 8 bytes is truncated, never overruns ---
  {
    const Function fns[] = {{.first_interface = 0, .compatible_id = "TOOLONGCOMPATID"}};
    const auto d = descriptor_set(fns);
    check_structure(d, 1);
    const size_t compat = kSetHeaderLen + kConfigurationSubsetHeaderLen + kFunctionSubsetHeaderLen;
    CHECK(ascii(d, compat + 4, 8) == "TOOLONGC");
  }

  // --- no functions: just the two headers ---
  {
    const auto d = descriptor_set(std::span<const Function>{});
    CHECK(d.size() == kSetHeaderLen + kConfigurationSubsetHeaderLen);
    check_structure(d, 0);
  }

  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
