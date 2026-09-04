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
#include <string>
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

// Minimal reader for the describe() TLV payload ([len u8][bytes] strings).
struct TlvReader {
  std::span<const uint8_t> b;
  size_t p = 0;
  uint8_t u8() { return b[p++]; }
  std::string str() {
    const uint8_t n = u8();
    std::string s(reinterpret_cast<const char *>(&b[p]), n);
    p += n;
    return s;
  }
};

static void test_discovery() {
  std::printf("test_discovery\n");
  using D = espp::Dispatcher;
  espp::Dispatcher d;
  d.set_device_info("espp Hub", "1.2.3");
  d.register_module(0, [](const sf::Frame &) {},
                    {.name = "OTA", .app = "ota_console.html", .description = "Firmware update"});
  d.register_module(6, [](const sf::Frame &) {},
                    {.name = "MCP266", .app = "mcp266_console.html", .description = "Motors"});
  d.register_module(9, [](const sf::Frame &) {}); // no metadata -> not advertised

  const auto payload = d.describe();
  TlvReader r{payload};
  CHECK(r.u8() == D::kDiscoveryVersion);
  CHECK(r.u8() == 0); // reserved
  CHECK(r.str() == "espp Hub");
  CHECK(r.str() == "1.2.3");
  const uint8_t count = r.u8();
  CHECK(count == 2); // module 9 (no name) excluded; discovery module absent
  const uint8_t id0 = r.u8();
  const std::string n0 = r.str(), a0 = r.str(), de0 = r.str();
  CHECK(id0 == 0 && n0 == "OTA" && a0 == "ota_console.html" && de0 == "Firmware update");
  const uint8_t id1 = r.u8();
  const std::string n1 = r.str(), a1 = r.str(), de1 = r.str();
  CHECK(id1 == 6 && n1 == "MCP266" && a1 == "mcp266_console.html" && de1 == "Motors");
  CHECK(r.p == payload.size());

  // serve_discovery: a ListModules request produces one reply frame carrying the
  // describe() payload and echoing the request correlation id.
  std::vector<uint8_t> sent;
  d.serve_discovery(
      [&](std::span<const uint8_t> frame) { sent.assign(frame.begin(), frame.end()); });
  CHECK(d.has_module(D::kDiscoveryModule));
  CHECK(d.describe() == payload); // registering discovery must not list it
  d.feed(sf::build_frame(false, D::kDiscoveryModule,
                         static_cast<uint8_t>(D::Discovery::ListModules), {}, 0x1234));
  CHECK(!sent.empty());
  sf::StreamParser sp;
  const auto frames = sp.feed(sent);
  CHECK(frames.size() == 1);
  CHECK(frames[0].module == D::kDiscoveryModule && frames[0].is_reply());
  CHECK(frames[0].correlation.has_value() && *frames[0].correlation == 0x1234);
  CHECK(frames[0].payload == payload);

  // An echoed reply frame must NOT trigger another reply (avoid loops).
  sent.clear();
  d.feed(
      sf::build_frame(true, D::kDiscoveryModule, static_cast<uint8_t>(D::Discovery::ListModules)));
  CHECK(sent.empty());
}

static void test_discovery_payload_bound() {
  std::printf("test_discovery_payload_bound\n");
  using D = espp::Dispatcher;
  espp::Dispatcher d;
  const std::string big(255, 'x'); // max-length metadata (each record ~769 bytes)
  for (int i = 0; i < 40; ++i)
    d.register_module(static_cast<uint8_t>(i), [](const sf::Frame &) {},
                      {.name = big, .app = big, .description = big});
  const auto payload = d.describe();
  // 40 * ~769 bytes >> kMaxPayloadSize, so describe() must truncate to fit.
  CHECK(payload.size() <= sf::kMaxPayloadSize);
  // The count must match the records that actually fit, and the walk must consume
  // exactly the payload (self-consistent: no short/trailing bytes).
  TlvReader r{payload};
  r.u8();
  r.u8(); // version, reserved
  r.str();
  r.str(); // device name, fw
  const uint8_t count = r.u8();
  for (uint8_t m = 0; m < count; ++m) {
    r.u8();
    r.str();
    r.str();
    r.str();
  }
  CHECK(r.p == payload.size());
  CHECK(count > 0 && count < 40); // some fit, some were dropped
  // The resulting payload must be encodable as a frame (i.e. within the cap).
  CHECK(!sf::build_frame(true, D::kDiscoveryModule, static_cast<uint8_t>(D::Discovery::ListModules),
                         payload)
             .empty());
}

int main() {
  test_routing_and_coexistence();
  test_register_replace_unregister();
  test_reset();
  test_reentrant_unregister();
  test_discovery();
  test_discovery_payload_bound();
  // cppcheck-suppress throwInEntryPoint // the handler's throw is caught inside
  // the test (cppcheck can't trace it through the std::function / feed() call)
  test_handler_exception_recovers();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
