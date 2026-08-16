// Typed pub/sub test (Phase 5 of components/rtps/REFACTOR_PLAN.md).
//
// Exercises espp::Publisher<T> / espp::Subscriber<T>: two facade participants in
// one process exchange a reflectable message struct end-to-end, with no manual
// CDR (de)serialization in the test — the typed layer does it. Also confirms the
// zero-alloc publish path (reused buffer) round-trips a multi-field struct.
//
// Exits 0 when at least kRequired typed samples arrive within the deadline.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"

using namespace std::chrono_literals;

// A plain reflectable message struct — no base class, macros, or methods.
struct Telemetry {
  uint32_t seq{};
  float value{};
  std::string label;
};

int main() {
  constexpr int kRequired = 5;
  constexpr auto kDeadline = 20s;
  const char *topic = "rt/telemetry";
  const char *type = "espp::msg::dds_::Telemetry_";
  using R = espp::RtpsParticipant;

  std::mutex m;
  std::vector<Telemetry> got;
  std::atomic<int> count{0};

  R pub({.log_level = espp::Logger::Verbosity::WARN});
  R sub({.log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  espp::Publisher<Telemetry> publisher(pub, {
                                                .topic = topic,
                                                .type_name = type,
                                                .reliability = R::Reliability::RELIABLE,
                                            });
  espp::Subscriber<Telemetry> subscriber(sub, {
                                                  .topic = topic,
                                                  .type_name = type,
                                                  .reliability = R::Reliability::RELIABLE,
                                                  .on_message =
                                                      [&](const Telemetry &t) {
                                                        std::lock_guard<std::mutex> lk(m);
                                                        got.push_back(t);
                                                        count.fetch_add(1);
                                                      },
                                              });
  if (!publisher.is_valid() || !subscriber.is_valid()) {
    std::printf("FAIL: publisher/subscriber registration\n");
    return 1;
  }

  // Let SEDP match before publishing.
  std::this_thread::sleep_for(2s);

  int sent = 0;
  const auto start = std::chrono::steady_clock::now();
  while (count.load() < kRequired && std::chrono::steady_clock::now() - start < kDeadline) {
    publisher.publish(Telemetry{static_cast<uint32_t>(sent), 1.5f * sent, "sample"});
    sent++;
    std::this_thread::sleep_for(100ms);
  }

  const int n = count.load();
  std::printf("sent=%d received=%d\n", sent, n);
  // Verify the typed round-trip preserved fields (not just the count).
  bool fields_ok = false;
  {
    std::lock_guard<std::mutex> lk(m);
    fields_ok = std::any_of(got.begin(), got.end(), [](const Telemetry &t) {
      return t.label == "sample" && t.value == 1.5f * static_cast<float>(t.seq);
    });
  }
  pub.stop();
  sub.stop();
  if (n >= kRequired && fields_ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: received %d/%d, fields_ok=%d\n", n, kRequired, fields_ok);
  return 1;
}
