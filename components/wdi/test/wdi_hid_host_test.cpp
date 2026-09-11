// Host-side unit test for the hid-rp-built WDI HID report descriptor. hid-rp is
// header-only and stdlib-only, so this builds on a host:
//
//   c++ -std=c++20 -Wall -Wextra -Werror \
//       -I components/wdi/include -I components/hid-rp/include \
//       -I components/hid-rp/detail/hid-rp/hid-rp \
//       components/wdi/test/wdi_hid_host_test.cpp -o wdi_hid_test && ./wdi_hid_test

#include <cstdio>

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

// Count non-overlapping occurrences of a 2-byte item (tag,value) in the descriptor.
template <typename D> static int count_item(const D &d, uint8_t tag, uint8_t value) {
  int n = 0;
  for (size_t i = 0; i + 1 < d.size(); ++i)
    if (d[i] == tag && d[i + 1] == value)
      ++n;
  return n;
}
template <typename D> static bool contains(const D &d, std::initializer_list<uint8_t> seq) {
  for (size_t i = 0; i + seq.size() <= d.size(); ++i) {
    bool ok = true;
    size_t j = 0;
    for (uint8_t b : seq)
      if (d[i + j++] != b) {
        ok = false;
        break;
      }
    if (ok)
      return true;
  }
  return false;
}

int main() {
  const auto &d = wdi::kReportDescriptor;
  std::printf("wdi hid descriptor: %zu bytes\n", d.size());

  CHECK(!d.empty());
  // Vendor usage page 0xFF00: `06 00 FF`.
  CHECK(contains(d, {0x06, 0x00, 0xFF}));
  // Application collection: `A1 01`.
  CHECK(contains(d, {0xA1, 0x01}));
  // Five report-id items: `85 01`..`85 05`.
  for (uint8_t id = 1; id <= 5; ++id)
    CHECK(count_item(d, 0x85, id) == 1);
  // Report counts: Control 18 (0x12), Feedback 19 (0x13), 1-byte reports (0x01),
  // Keepalive Response 16 (0x10) -- `95 <count>`.
  CHECK(count_item(d, 0x95, 0x12) == 1); // 18-byte Control
  CHECK(count_item(d, 0x95, 0x13) == 1); // 19-byte Feedback
  CHECK(count_item(d, 0x95, 0x10) == 1); // 16-byte Keepalive Response
  CHECK(count_item(d, 0x95, 0x01) == 2); // two 1-byte reports (Request Feedback + Keepalive)
  // Three Input items (`81 02`) and two Output items (`91 02`).
  CHECK(count_item(d, 0x81, 0x02) == 3);
  CHECK(count_item(d, 0x91, 0x02) == 2);
  // Report size 8 bits (`75 08`) and End Collection (`C0`).
  CHECK(contains(d, {0x75, 0x08}));
  CHECK(d.back() == 0xC0);

  if (g_failures == 0) {
    std::printf("ALL WDI HID DESCRIPTOR TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
