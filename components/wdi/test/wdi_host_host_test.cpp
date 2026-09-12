// Host-side unit test for the WDI **host** role (WdiHost). Deterministic: uses a
// fake clock and a mock send callback (no ESP-IDF, no real time).
//
//   c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
//       components/wdi/test/wdi_host_host_test.cpp -o wdi_host_test && ./wdi_host_test

#include <cstdio>
#include <optional>
#include <vector>

#include "wdi_host.hpp"

namespace wdi = espp::wdi;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

struct Sent {
  wdi::ReportId id;
  std::vector<uint8_t> payload;
};

// A WdiHost wired to a controllable clock and a sink that records sends.
struct Harness {
  uint32_t now = 5000;
  std::vector<Sent> sent;
  int connects = 0;
  int disconnects = 0;
  std::optional<wdi::ControlReport> last_control;

  espp::WdiHost make() {
    espp::WdiHost::Config cfg;
    cfg.now_ms = [this] { return now; };
    cfg.send = [this](wdi::ReportId id, std::span<const uint8_t> p) {
      sent.push_back({id, std::vector<uint8_t>(p.begin(), p.end())});
      return true;
    };
    cfg.on_control = [this](const wdi::ControlReport &c) { last_control = c; };
    cfg.on_connected = [this] { ++connects; };
    cfg.on_disconnected = [this] { ++disconnects; };
    uint8_t rnd[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    cfg.host_uuid = espp::WdiHost::make_host_uuid(
        static_cast<uint16_t>(wdi::ManufacturerId::LuciMobility), rnd);
    return espp::WdiHost(cfg);
  }
};

static void test_control_delivery() {
  std::printf("test_control_delivery\n");
  Harness h;
  auto host = h.make();
  CHECK(!host.is_connected());

  wdi::ControlReport c;
  c.x = 10;
  c.y = -20;
  c.set(wdi::ControlBit::DriveEnable);
  auto bytes = c.serialize();
  host.handle_input(wdi::ReportId::Control, bytes);

  CHECK(host.is_connected());
  CHECK(h.connects == 1);
  CHECK(h.last_control.has_value());
  CHECK(h.last_control->x == 10);
  CHECK(h.last_control->y == -20);
  CHECK(h.last_control->has(wdi::ControlBit::DriveEnable));
  CHECK(host.last_control().has_value());
}

static void test_keepalive_response() {
  std::printf("test_keepalive_response\n");
  Harness h;
  auto host = h.make();
  const uint8_t trig = wdi::kTriggerValue;
  host.handle_input(wdi::ReportId::Keepalive, {&trig, 1});

  CHECK(h.sent.size() == 1);
  CHECK(h.sent[0].id == wdi::ReportId::KeepaliveResponse);
  CHECK(h.sent[0].payload.size() == wdi::kKeepaliveResponseSize);
  // manufacturer id is big-endian in bytes 0..1
  auto uuid = wdi::HostUuid::parse(h.sent[0].payload);
  CHECK(uuid.has_value());
  CHECK(uuid->manufacturer_id() == static_cast<uint16_t>(wdi::ManufacturerId::LuciMobility));
}

static void test_request_feedback() {
  std::printf("test_request_feedback\n");
  Harness h;
  auto host = h.make();
  wdi::FeedbackReport fb;
  fb.set(wdi::FeedbackBit::DriveEnabled);
  fb.speed = 4;
  host.set_feedback(fb);

  const uint8_t trig = wdi::kTriggerValue;
  host.handle_input(wdi::ReportId::RequestFeedback, {&trig, 1});

  CHECK(h.sent.size() == 1);
  CHECK(h.sent[0].id == wdi::ReportId::Feedback);
  auto got = wdi::FeedbackReport::parse(h.sent[0].payload);
  CHECK(got.has_value());
  CHECK(got->has(wdi::FeedbackBit::DriveEnabled));
  CHECK(got->speed == 4);
}

static void test_watchdog_disconnect() {
  std::printf("test_watchdog_disconnect\n");
  Harness h;
  auto host = h.make();
  const uint8_t trig = wdi::kTriggerValue;
  host.handle_input(wdi::ReportId::Keepalive, {&trig, 1});
  CHECK(host.is_connected());

  // Not yet timed out (just under 3 windows).
  h.now += wdi::kHostKeepaliveWindowMs * 3 - 1;
  CHECK(!host.poll());
  CHECK(host.is_connected());
  CHECK(h.disconnects == 0);

  // Cross the 3-window threshold -> disconnect.
  h.now += 2;
  CHECK(host.poll());
  CHECK(!host.is_connected());
  CHECK(h.disconnects == 1);

  // Idempotent: further polls don't re-fire.
  CHECK(!host.poll());
  CHECK(h.disconnects == 1);

  // A new report reconnects.
  host.handle_input(wdi::ReportId::Keepalive, {&trig, 1});
  CHECK(host.is_connected());
  CHECK(h.connects == 2);
}

int main() {
  std::printf("WDI host-role host tests\n");
  test_control_delivery();
  test_keepalive_response();
  test_request_feedback();
  test_watchdog_disconnect();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHECK(s) FAILED\n", g_failures);
  return 1;
}
