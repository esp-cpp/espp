// Host-buildable unit tests for espp::Dispatcher. Build & run with:
//   c++ -std=c++20 -Werror -I components/dispatcher/include \
//       -I components/stream_frame/include \
//       components/dispatcher/test/dispatcher_host_test.cpp -o test && ./test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

namespace sf = espp::stream_frame;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static void test_module_of() {
  std::printf("test_module_of\n");
  // High nibble is the module id; the 0x80 reply bit lives in the high nibble
  // too, so requests and replies of one protocol map to *different* module ids
  // (device sees the request nibble; host would see the reply nibble).
  CHECK(espp::Dispatcher::module_of(0x01) == 0);  // OTA BEGIN
  CHECK(espp::Dispatcher::module_of(0x04) == 0);  // OTA ABORT
  CHECK(espp::Dispatcher::module_of(0x40) == 4);  // coredump GET_SUMMARY
  CHECK(espp::Dispatcher::module_of(0x43) == 4);  // coredump ERASE
  CHECK(espp::Dispatcher::module_of(0x50) == 5);  // CAN bridge (example)
  CHECK(espp::Dispatcher::module_of(0x81) == 8);  // OTA OK reply
  CHECK(espp::Dispatcher::module_of(0xC2) == 12); // coredump DATA reply
}

static void test_routing_and_coexistence() {
  std::printf("test_routing_and_coexistence\n");
  espp::Dispatcher d;
  std::vector<uint8_t> mod0_types, mod4_types, mod5_types;
  d.register_module(0, [&](uint8_t t, std::span<const uint8_t>) { mod0_types.push_back(t); });
  d.register_module(4, [&](uint8_t t, std::span<const uint8_t> p) {
    mod4_types.push_back(t);
    // echo payload length check for one case below
    if (t == 0x42)
      CHECK(p.size() == 3);
  });
  d.register_module(5, [&](uint8_t t, std::span<const uint8_t>) { mod5_types.push_back(t); });
  CHECK(d.has_module(0) && d.has_module(4) && d.has_module(5));
  CHECK(!d.has_module(1) && !d.has_module(12));

  // Interleave three protocols' frames on one stream, plus one frame for an
  // UNREGISTERED module (must be silently ignored).
  std::vector<uint8_t> stream;
  auto add = [&](const std::vector<uint8_t> &f) {
    stream.insert(stream.end(), f.begin(), f.end());
  };
  const uint8_t p3[] = {1, 2, 3};
  add(sf::build_frame(0x01));     // module 0
  add(sf::build_frame(0x50));     // module 5
  add(sf::build_frame(0x42, p3)); // module 4, 3-byte payload
  add(sf::build_frame(0x20));     // module 2 — unregistered, ignored
  add(sf::build_frame(0x02));     // module 0

  d.feed(stream);
  CHECK(mod0_types.size() == 2);
  CHECK(mod0_types.size() == 2 && mod0_types[0] == 0x01 && mod0_types[1] == 0x02);
  CHECK(mod4_types.size() == 1 && mod4_types[0] == 0x42);
  CHECK(mod5_types.size() == 1 && mod5_types[0] == 0x50);
  CHECK(d.dropped_bytes() == 0);
}

static void test_unregister_and_reset() {
  std::printf("test_unregister_and_reset\n");
  espp::Dispatcher d;
  int count = 0;
  d.register_module(0, [&](uint8_t, std::span<const uint8_t>) { ++count; });
  d.feed(sf::build_frame(0x01));
  CHECK(count == 1);
  d.unregister_module(0);
  d.feed(sf::build_frame(0x02));
  CHECK(count == 1); // no longer routed

  // reset() drops a partially-buffered frame so a stale prefix cannot stitch
  // onto later bytes.
  d.register_module(0, [&](uint8_t, std::span<const uint8_t>) { ++count; });
  auto frame = sf::build_frame(0x03);
  d.feed(std::span<const uint8_t>(frame.data(), 3)); // partial (header only)
  CHECK(d.buffered() > 0);
  d.reset();
  CHECK(d.buffered() == 0);
  d.feed(std::span<const uint8_t>(frame.data() + 3, frame.size() - 3)); // remainder alone
  CHECK(count == 1); // the split frame did NOT complete after reset
}

int main() {
  test_module_of();
  test_routing_and_coexistence();
  test_unregister_and_reset();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
