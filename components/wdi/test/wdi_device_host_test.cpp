// Host-side unit test for the WDI device role (WdiDevice). Deterministic: uses a
// fake clock and a mock send callback (no ESP-IDF, no real time).
//
//   c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
//       components/wdi/test/wdi_device_host_test.cpp -o wdi_dev_test && ./wdi_dev_test

#include <cstdio>
#include <optional>
#include <vector>

#include "wdi.hpp"

namespace wdi = espp::wdi;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// A recorded outgoing report.
struct Sent {
  wdi::ReportId id;
  std::vector<uint8_t> payload;
};

// Build a WdiDevice wired to a controllable clock + a sink that records sends.
struct Harness {
  uint32_t now = 1000;    // fake ms clock, controlled by the test
  std::vector<Sent> sent; // every report the device transmitted
  bool send_ok = true;    // let a test make send() "fail"
  std::optional<wdi::FeedbackReport> last_feedback;
  std::optional<wdi::HostUuid> last_uuid;

  espp::WdiDevice make() {
    espp::WdiDevice::Config cfg;
    cfg.now_ms = [this] { return now; };
    cfg.send = [this](wdi::ReportId id, std::span<const uint8_t> p) {
      if (!send_ok)
        return false;
      sent.push_back({id, std::vector<uint8_t>(p.begin(), p.end())});
      return true;
    };
    cfg.on_feedback = [this](const wdi::FeedbackReport &f) { last_feedback = f; };
    cfg.on_keepalive_response = [this](const wdi::HostUuid &u) { last_uuid = u; };
    return espp::WdiDevice(cfg);
  }
};

static void test_control_send_and_reset() {
  std::printf("test_control_send_and_reset\n");
  Harness h;
  auto dev = h.make();

  wdi::ControlReport c;
  c.y = -100; // forward
  c.set(wdi::ControlBit::DriveEnable);
  CHECK(dev.send_control(c));
  CHECK(h.sent.size() == 1);
  CHECK(h.sent[0].id == wdi::ReportId::Control);
  CHECK(h.sent[0].payload.size() == wdi::kControlSize);
  // The control send reset the keepalive timer, so nothing is due yet.
  CHECK(dev.ms_until_keepalive() == wdi::kAppKeepaliveIntervalMs);
  CHECK(!dev.poll()); // not due
  CHECK(h.sent.size() == 1);
}

static void test_keepalive_timing() {
  std::printf("test_keepalive_timing\n");
  Harness h;
  auto dev = h.make();
  dev.send_control(wdi::ControlReport{}); // reset timer at now=1000
  const size_t base = h.sent.size();

  h.now += wdi::kAppKeepaliveIntervalMs - 1; // just before due
  CHECK(!dev.poll());
  CHECK(h.sent.size() == base);

  h.now += 1; // exactly at the interval
  CHECK(dev.poll());
  CHECK(h.sent.size() == base + 1);
  CHECK(h.sent.back().id == wdi::ReportId::Keepalive);
  CHECK(h.sent.back().payload.size() == 1 && h.sent.back().payload[0] == wdi::kTriggerValue);

  // The keepalive itself reset the timer, so the next one is a full interval away.
  CHECK(!dev.poll());
  h.now += wdi::kAppKeepaliveIntervalMs;
  CHECK(dev.poll());
  CHECK(h.sent.size() == base + 2);
}

static void test_request_feedback_resets_timer() {
  std::printf("test_request_feedback_resets_timer\n");
  Harness h;
  auto dev = h.make();
  dev.send_control(wdi::ControlReport{});
  h.now += wdi::kAppKeepaliveIntervalMs - 10;
  CHECK(dev.request_feedback()); // resets the timer 10ms before a keepalive was due
  CHECK(h.sent.back().id == wdi::ReportId::RequestFeedback);
  const size_t n = h.sent.size();
  h.now += 10; // would have been due if request_feedback hadn't reset it
  CHECK(!dev.poll());
  CHECK(h.sent.size() == n);
}

static void test_failed_send_does_not_reset_timer() {
  std::printf("test_failed_send_does_not_reset_timer\n");
  Harness h;
  auto dev = h.make();
  dev.send_control(wdi::ControlReport{}); // ok, timer reset at now=1000
  h.now += wdi::kAppKeepaliveIntervalMs;
  h.send_ok = false;
  CHECK(!dev.poll()); // keepalive due but send fails
  h.send_ok = true;
  CHECK(dev.poll()); // still due (a failed send must not reset the timer)
  CHECK(h.sent.back().id == wdi::ReportId::Keepalive);
}

static void test_handle_feedback_and_uuid() {
  std::printf("test_handle_feedback_and_uuid\n");
  Harness h;
  auto dev = h.make();

  wdi::FeedbackReport fb;
  fb.set(wdi::FeedbackBit::DriveEnabled);
  fb.speed = 4;
  fb.velocity_whole = 2;
  fb.velocity_tenths = 5;
  const auto fbytes = fb.serialize();
  dev.handle_output(wdi::ReportId::Feedback, fbytes);
  CHECK(h.last_feedback.has_value());
  CHECK(dev.last_feedback().has_value());
  if (h.last_feedback)
    CHECK(h.last_feedback->has(wdi::FeedbackBit::DriveEnabled) && h.last_feedback->speed == 4);

  std::array<uint8_t, wdi::kKeepaliveResponseSize> uuid{};
  uuid[0] = 0x00;
  uuid[1] = 0x0B; // LUCI (big-endian)
  dev.handle_output(wdi::ReportId::KeepaliveResponse, uuid);
  CHECK(h.last_uuid.has_value());
  CHECK(dev.host_uuid().has_value());
  if (dev.host_uuid())
    CHECK(dev.host_uuid()->manufacturer_id() == 0x000B);

  // A malformed (wrong-size) feedback payload is ignored, not delivered.
  h.last_feedback.reset();
  std::vector<uint8_t> bad(wdi::kFeedbackSize - 3, 0);
  dev.handle_output(wdi::ReportId::Feedback, bad);
  CHECK(!h.last_feedback.has_value());

  // An Input-report id fed to handle_output (wrong direction) is ignored.
  dev.handle_output(wdi::ReportId::Control, std::vector<uint8_t>(wdi::kControlSize, 0));
  // (no crash / no callback expectations)
}

int main() {
  test_control_send_and_reset();
  test_keepalive_timing();
  test_request_feedback_resets_timer();
  test_failed_send_does_not_reset_timer();
  test_handle_feedback_and_uuid();
  if (g_failures == 0) {
    std::printf("ALL WDI DEVICE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
