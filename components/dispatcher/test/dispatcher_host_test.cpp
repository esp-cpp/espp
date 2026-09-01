// Host-buildable unit tests for espp::Dispatcher (v2). Build & run with:
//   c++ -std=c++20 -Werror -I components/dispatcher/include \
//       -I components/stream_frame/include \
//       components/dispatcher/test/dispatcher_host_test.cpp -o test && ./test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <span>
#include <stdexcept>
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

static void test_routing_and_coexistence() {
  std::printf("test_routing_and_coexistence\n");
  espp::Dispatcher d;
  std::vector<uint8_t> mod0, mod4, mod200;
  bool mod4_saw_reply = false;
  d.register_module(0, [&](const sf::Frame &f) { mod0.push_back(f.type); });
  d.register_module(4, [&](const sf::Frame &f) {
    mod4.push_back(f.type);
    if (f.is_reply())
      mod4_saw_reply = true;
    if (f.type == 0x42)
      CHECK(f.payload.size() == 3);
  });
  // A full-byte module id well beyond the old nibble range (0..15).
  d.register_module(200, [&](const sf::Frame &f) { mod200.push_back(f.type); });
  CHECK(d.has_module(0) && d.has_module(4) && d.has_module(200));
  CHECK(!d.has_module(1) && !d.has_module(13));

  std::vector<uint8_t> stream;
  auto add = [&](const std::vector<uint8_t> &f) {
    stream.insert(stream.end(), f.begin(), f.end());
  };
  const uint8_t p3[] = {1, 2, 3};
  add(sf::build_frame(false, 0, 0x02));     // module 0 request
  add(sf::build_frame(true, 4, 0xC0));      // module 4 reply
  add(sf::build_frame(false, 4, 0x42, p3)); // module 4 request, 3-byte payload
  add(sf::build_frame(false, 7, 0x01));     // module 7 — unregistered, ignored
  add(sf::build_frame(false, 200, 0x99));   // module 200 request
  add(sf::build_frame(false, 0, 0x03));     // module 0 request

  d.feed(stream);
  CHECK(mod0.size() == 2 && mod0[0] == 0x02 && mod0[1] == 0x03);
  CHECK(mod4.size() == 2 && mod4[0] == 0xC0 && mod4[1] == 0x42 && mod4_saw_reply);
  CHECK(mod200.size() == 1 && mod200[0] == 0x99);
  CHECK(d.dropped_bytes() == 0);
}

static void test_register_replace_unregister() {
  std::printf("test_register_replace_unregister\n");
  espp::Dispatcher d;
  int a = 0, b = 0;
  d.register_module(3, [&](const sf::Frame &) { ++a; });
  d.feed(sf::build_frame(false, 3, 0x00));
  CHECK(a == 1 && b == 0);
  // Replacing the handler for a module routes to the new one.
  d.register_module(3, [&](const sf::Frame &) { ++b; });
  d.feed(sf::build_frame(false, 3, 0x00));
  CHECK(a == 1 && b == 1);
  // Unregister -> frames for the module are ignored.
  d.unregister_module(3);
  d.feed(sf::build_frame(false, 3, 0x00));
  CHECK(a == 1 && b == 1 && !d.has_module(3));
}

static void test_reset() {
  std::printf("test_reset\n");
  espp::Dispatcher d;
  int count = 0;
  d.register_module(0, [&](const sf::Frame &) { ++count; });
  auto frame = sf::build_frame(false, 0, 0x03);
  d.feed(std::span<const uint8_t>(frame.data(), 3)); // partial (header only)
  CHECK(d.buffered() > 0);
  d.reset();
  CHECK(d.buffered() == 0);
  d.feed(std::span<const uint8_t>(frame.data() + 3, frame.size() - 3)); // remainder alone
  CHECK(count == 0); // the split frame did NOT complete after reset
}

static void test_reentrant_unregister() {
  std::printf("test_reentrant_unregister\n");
  // A handler that unregisters its own module mid-dispatch must be safe: the
  // dispatcher DEFERS register/unregister requested during a dispatch until the
  // dispatch unwinds, so handlers_ is not reallocated and the running handler is
  // not destroyed while it executes (no per-frame handler copy needed).
  espp::Dispatcher d;
  int hits = 0;
  d.register_module(6, [&](const sf::Frame &) {
    ++hits;
    d.unregister_module(6);
  });
  d.feed(sf::build_frame(false, 6, 0x11)); // unregisters itself while dispatching
  d.feed(sf::build_frame(false, 6, 0x22)); // ignored now (unregistered)
  CHECK(hits == 1 && !d.has_module(6));
}

static void test_handler_exception_recovers() {
  std::printf("test_handler_exception_recovers\n");
  // A throwing handler must not wedge the dispatcher: the internal dispatch
  // depth is restored (so later registrations still apply), and pending ops
  // queued before the throw are applied on a subsequent dispatch.
  espp::Dispatcher d;
  d.register_module(1, [](const sf::Frame &) { throw std::runtime_error("boom"); });
  bool threw = false;
  try {
    d.feed(sf::build_frame(false, 1, 0x00));
  } catch (const std::runtime_error &) {
    threw = true;
  }
  CHECK(threw);
  // The dispatcher is not wedged: a NEW registration made after the throw takes
  // effect immediately (proving dispatch_depth_ was restored to 0).
  int hits = 0;
  d.register_module(2, [&](const sf::Frame &) { ++hits; });
  d.feed(sf::build_frame(false, 2, 0x00));
  CHECK(hits == 1 && d.has_module(2));
}

int main() {
  test_routing_and_coexistence();
  test_register_replace_unregister();
  test_reset();
  test_reentrant_unregister();
  test_handler_exception_recovers();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
