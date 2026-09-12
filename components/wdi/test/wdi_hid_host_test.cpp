// Host-side unit test for the hid-rp-built WDI HID report descriptor. hid-rp is
// header-only and stdlib-only, so this builds on a host:
//
//   c++ -std=c++20 -Wall -Wextra -Werror \
//       -I components/wdi/include -isystem components/hid-rp/include \
//       -isystem components/hid-rp/detail/hid-rp/hid-rp \
//       components/wdi/test/wdi_hid_host_test.cpp -o wdi_hid_test && ./wdi_hid_test

#include <algorithm>
#include <cstdio>
#include <initializer_list>

#include "wdi_hid.hpp"

namespace wdi = espp::wdi;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// Count occurrences of a 2-byte item (tag,value) in the descriptor. The window
// slides by one byte, so overlapping matches are counted too -- fine here, as the
// (tag,value) pairs searched for don't overlap themselves.
template <typename D> static int count_item(const D &d, uint8_t tag, uint8_t value) {
  int n = 0;
  for (size_t i = 0; i + 1 < d.size(); ++i)
    if (d[i] == tag && d[i + 1] == value)
      ++n;
  return n;
}
template <typename D> static bool contains(const D &d, std::initializer_list<uint8_t> seq) {
  return std::search(d.begin(), d.end(), seq.begin(), seq.end()) != d.end();
}

int main() {
  const auto &d = wdi::kReportDescriptor;
  std::printf("wdi hid descriptor: %zu bytes\n", d.size());

  CHECK(!d.empty());
  // Vendor usage page 0xFF00: `06 00 FF`, then application collection `A1 01`.
  CHECK(contains(d, {0x06, 0x00, 0xFF}));
  CHECK(contains(d, {0xA1, 0x01}));
  // Five report-id items: `85 01`..`85 05`, each once.
  for (uint8_t id = 1; id <= 5; ++id)
    CHECK(count_item(d, 0x85, id) == 1);

  // Field-accurate layout (not opaque byte blobs):
  //  - Control:  2x SInt8 axes (one Input item, count 2) + 4x 32-bit flag fields
  //  - Feedback: 3x 32-bit flag fields + speed/profile + velocity + odometer + 4 reserved
  //  - Request-Feedback / Keepalive: 1 byte each; Keepalive-Response: 16 bytes
  // Both 1-bit (flag) and 8-bit (byte) field sizes must appear.
  CHECK(contains(d, {0x75, 0x01})); // report_size 1 (flag bits)
  CHECK(contains(d, {0x75, 0x08})); // report_size 8 (bytes)
  // Seven 32-bit flag fields total (4 Control + 3 Feedback): `95 20` (count 32).
  CHECK(count_item(d, 0x95, 0x20) == 7);
  CHECK(contains(d, {0x95, 0x02})); // axes: count 2
  CHECK(contains(d, {0x95, 0x10})); // Keepalive-Response: count 16
  CHECK(contains(d, {0x95, 0x04})); // Feedback reserved: count 4
  // Signed axes: logical minimum -127 (`15 81`) and maximum 127 (`25 7F`).
  CHECK(contains(d, {0x15, 0x81}));
  CHECK(contains(d, {0x25, 0x7F}));
  // Seven Input items (`81 02`): Control axes + 4 flags, Request-Feedback, Keepalive.
  CHECK(count_item(d, 0x81, 0x02) == 7);
  // Eight Output items (`91 02`): Feedback 3 flags + 4 byte fields, Keepalive-Response.
  CHECK(count_item(d, 0x91, 0x02) == 8);
  // Terminated by End Collection (`C0`).
  CHECK(d.back() == 0xC0);

  if (g_failures == 0) {
    std::printf("ALL WDI HID DESCRIPTOR TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
