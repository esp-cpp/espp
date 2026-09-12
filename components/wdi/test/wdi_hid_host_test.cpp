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
#include <span>
#include <vector>

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

  // ---- looks_like_wdi_descriptor(): the host's "adopt this HID device?" gate ----
  std::printf("looks_like_wdi_descriptor\n");
  // Exact match against our own descriptor.
  CHECK(wdi::looks_like_wdi_descriptor(d));
  // Empty, or cut in half (report ids missing), or cut mid-item (malformed) -> no.
  // (Dropping just the trailing End Collection still parses as WDI -- by design,
  // the gate checks the vendor usage + report ids, not descriptor well-formedness.)
  CHECK(!wdi::looks_like_wdi_descriptor(std::span<const uint8_t>{}));
  CHECK(!wdi::looks_like_wdi_descriptor(std::span<const uint8_t>(d.data(), d.size() / 2)));
  CHECK(d[d.size() - 3] == 0x91 && d[d.size() - 2] == 0x02); // last item: Output, then C0
  CHECK(!wdi::looks_like_wdi_descriptor(std::span<const uint8_t>(d.data(), d.size() - 2)));
  CHECK(wdi::looks_like_wdi_descriptor(std::span<const uint8_t>(d.data(), d.size() - 1)));

  // Another implementation of the spec: a minimal descriptor with the vendor
  // usage page + usage 0x01 and report ids 1..5 (opaque byte-blob reports).
  auto blob_report = [](std::vector<uint8_t> &v, uint8_t id, uint8_t count, bool out) {
    v.insert(v.end(), {0x85, id});                         // Report ID
    v.insert(v.end(), {0x75, 0x08, 0x95, count});          // size 8, count N
    v.insert(v.end(), {0x09, 0x02});                       // Usage (arbitrary)
    v.insert(v.end(), {uint8_t(out ? 0x91 : 0x81), 0x02}); // Output/Input (Data,Var,Abs)
  };
  auto make_alt = [&](bool with_ka_response, uint8_t page_lo = 0x00, uint8_t page_hi = 0xFF,
                      uint8_t usage = 0x01) {
    std::vector<uint8_t> v{0x06, page_lo, page_hi, 0x09, usage, 0xA1, 0x01};
    v.insert(v.end(), {0x15, 0x81, 0x25, 0x7F}); // logical -127..127
    blob_report(v, 1, 18, false);
    blob_report(v, 2, 19, true);
    blob_report(v, 3, 1, false);
    blob_report(v, 4, 1, false);
    if (with_ka_response)
      blob_report(v, 5, 16, true);
    v.push_back(0xC0);
    return v;
  };
  const auto alt = make_alt(true);
  CHECK(alt.size() != d.size()); // i.e. this really exercises the parse path
  CHECK(wdi::looks_like_wdi_descriptor(alt));
  // Missing one of the five report ids -> not WDI.
  CHECK(!wdi::looks_like_wdi_descriptor(make_alt(false)));
  // Same reports on a different vendor page (0xFF01), or usage 0x02 -> not WDI.
  CHECK(!wdi::looks_like_wdi_descriptor(make_alt(true, 0x01, 0xFF)));
  CHECK(!wdi::looks_like_wdi_descriptor(make_alt(true, 0x00, 0xFF, 0x02)));

  // Item *data* must not masquerade as items: a generic-desktop descriptor whose
  // 4-byte Logical Maximum happens to contain the bytes `06 00 FF 09 01`-ish.
  {
    std::vector<uint8_t> v{0x05, 0x01, 0x09, 0x05, 0xA1, 0x01};
    v.insert(v.end(), {0x27, 0x06, 0x00, 0xFF, 0x09}); // Logical Max (4 bytes) = 09FF0006
    v.insert(v.end(), {0x09, 0x01});                   // Usage 0x01 (on page 0x01, not 0xFF00)
    for (uint8_t id = 1; id <= 5; ++id)
      blob_report(v, id, 8, false);
    v.push_back(0xC0);
    CHECK(!wdi::looks_like_wdi_descriptor(v));
  }
  // A long item (FE, bDataSize, bLongItemTag, data...) is valid HID and must be
  // skipped, not treated as a rejection...
  {
    auto v = make_alt(true);
    // insert after the collection open: 3 bytes of long-item payload
    v.insert(v.begin() + 7, {0xFE, 0x03, 0x42, 0xAA, 0xBB, 0xCC});
    CHECK(wdi::looks_like_wdi_descriptor(v));
    // ...and a long item straddling the vendor page + usage pair breaks the
    // "immediately followed by" requirement.
    auto w = make_alt(true);
    w.insert(w.begin() + 3, {0xFE, 0x00, 0x42});
    CHECK(!wdi::looks_like_wdi_descriptor(w));
  }
  // Truncated long item (declares more data than remains) / truncated short item.
  {
    auto v = make_alt(true);
    v.insert(v.end(), {0xFE, 0x10, 0x42}); // claims 16 data bytes, has none
    CHECK(!wdi::looks_like_wdi_descriptor(v));
    auto w = make_alt(true);
    w.insert(w.end(), {0xFE, 0x01}); // no room for even the tag byte
    CHECK(!wdi::looks_like_wdi_descriptor(w));
    auto x = make_alt(true);
    x.push_back(0x06); // 2-byte Usage Page item with no data
    CHECK(!wdi::looks_like_wdi_descriptor(x));
  }

  if (g_failures == 0) {
    std::printf("ALL WDI HID DESCRIPTOR TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
