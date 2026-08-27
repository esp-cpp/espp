// Regression: remove_reader() must not hold the facade mutex_ while quiescing
// the deferred dispatcher. close() waits for the in-flight delivery, and that
// user callback may legally call back into the participant (e.g. publish(),
// which takes mutex_). If remove_reader held mutex_ across close(), the
// callback's publish() would block on mutex_ while close() waits for the
// callback - a deadlock.
//
// This reproduces it deterministically: a banded shared-port reader (deferred
// dispatch) whose on_sample publishes to another topic is removed WHILE its
// callback is executing. remove_reader() must return promptly.
//
// Exits 0 on success, 1 on failure/deadlock.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "cdr.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
// Expose the protected remove_reader() for this unit test.
struct TestParticipant : espp::RtpsParticipant {
  using espp::RtpsParticipant::remove_reader;
  using espp::RtpsParticipant::RtpsParticipant;
};

struct SeqMsg {
  uint32_t seq;
};

std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}
} // namespace

int main() {
  using Reliability = espp::RtpsParticipant::Reliability;
  const char *type = "espp::test::dds_::Seq_";
  const char *topic_a = "deadlock_in";  // banded reader here
  const char *topic_b = "deadlock_out"; // the callback publishes here

  // Separate publisher (samples on topic_a) and subscriber. The subscriber is
  // the participant under test: it owns the banded reader on topic_a and a
  // writer on topic_b that the reader's callback publishes to. Dedicated ports
  // disabled so the banded reader falls back to DEFERRED dispatch (whose
  // close() waits for the in-flight delivery).
  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  TestParticipant part(
      {.log_level = espp::Logger::Verbosity::WARN, .enable_dedicated_endpoint_ports = false});
  if (!pub.start() || !part.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (!pub.add_writer(
          {.topic = topic_a, .type_name = type, .reliability = Reliability::RELIABLE}) ||
      !part.add_writer(
          {.topic = topic_b, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }

  std::atomic<bool> in_callback{false};
  std::atomic<bool> gate_open{false};
  std::atomic<bool> callback_published{false};
  if (!part.add_reader({.topic = topic_a,
                        .type_name = type,
                        .reliability = Reliability::RELIABLE,
                        .on_sample =
                            [&](std::span<const uint8_t>) {
                              in_callback = true;
                              // stay in-flight until the remover has started
                              while (!gate_open.load()) {
                                std::this_thread::sleep_for(1ms);
                              }
                              // a supported callback action that takes mutex_
                              auto bytes = cdr::serialize<cdr::xcdr1>(SeqMsg{0});
                              if (bytes) {
                                part.publish(topic_b, u8_span(*bytes));
                              }
                              callback_published = true;
                            },
                        .band = espp::QosBand::High})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  // Drive samples until the callback is executing (deferred delivery in-flight).
  std::atomic<bool> stop_pub{false};
  std::thread pub_thread([&]() {
    while (!stop_pub.load() && !in_callback.load()) {
      auto bytes = cdr::serialize<cdr::xcdr1>(SeqMsg{1});
      if (bytes) {
        pub.publish(topic_a, u8_span(*bytes));
      }
      std::this_thread::sleep_for(10ms);
    }
  });

  const auto entered_deadline = std::chrono::steady_clock::now() + 10s;
  while (!in_callback.load() && std::chrono::steady_clock::now() < entered_deadline) {
    std::this_thread::sleep_for(2ms);
  }
  stop_pub = true;
  pub_thread.join();
  if (!in_callback.load()) {
    std::printf("FAIL: callback never entered (no delivery)\n");
    return 1;
  }

  // Remove the reader while its callback is in-flight. On a separate thread so
  // a deadlock is observable via the watchdog rather than hanging the test.
  std::atomic<bool> removed{false};
  std::atomic<bool> removed_ok{false};
  std::thread remover([&]() {
    // Capture the RESULT too: an implementation that bailed out early (never
    // exercising the deletion/quiesce under test) would otherwise still pass
    // the prompt-return assertion below.
    removed_ok = part.remove_reader(topic_a);
    removed = true;
  });
  // Give remove_reader() time to reach close()'s in-flight wait, then let the
  // callback proceed to its publish() (which needs mutex_).
  std::this_thread::sleep_for(100ms);
  gate_open = true;

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!removed.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  const bool ok = removed.load();
  if (ok) {
    remover.join();
  }
  // (If deadlocked, the remover thread is stuck; detach so the process can
  // report the failure rather than hang on join.)
  else {
    remover.detach();
  }

  if (!ok) {
    std::printf("FAIL: remove_reader() deadlocked (held mutex_ across close())\n");
    return 1;
  }
  if (!removed_ok.load()) {
    std::printf("FAIL: remove_reader() returned false (removal not exercised)\n");
    return 1;
  }
  if (!callback_published.load()) {
    std::printf("FAIL: callback's publish() never completed\n");
    return 1;
  }
  // Scenario 2: a callback removing ITS OWN reader. The engine's teardown
  // drain must recognize the caller's own in-flight dispatch (previously
  // reentrant via the recursive callback mutex; an unconditional
  // wait-for-zero would deadlock on the callback's own dispatch count).
  const char *topic_c = "deadlock_self";
  std::atomic<bool> self_removed{false};
  std::atomic<bool> self_removed_ok{false};
  if (!pub.add_writer(
          {.topic = topic_c, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: scenario-2 add_writer\n");
    return 1;
  }
  if (!part.add_reader({.topic = topic_c,
                        .type_name = type,
                        .reliability = Reliability::RELIABLE,
                        .on_sample = [&](std::span<const uint8_t>) {
                          if (!self_removed.exchange(true)) {
                            self_removed_ok = part.remove_reader(topic_c);
                          }
                        }})) {
    std::printf("FAIL: scenario-2 add_reader\n");
    return 1;
  }
  const auto self_deadline = std::chrono::steady_clock::now() + 10s;
  while (!self_removed.load() && std::chrono::steady_clock::now() < self_deadline) {
    auto bytes = cdr::serialize<cdr::xcdr1>(SeqMsg{2});
    if (bytes) {
      pub.publish(topic_c, u8_span(*bytes));
    }
    std::this_thread::sleep_for(10ms);
  }
  // The watchdog is the harness timeout: a self-wait deadlock would hang the
  // callback (and this loop's publisher would keep running) until the kill.
  if (!self_removed.load()) {
    std::printf("FAIL: scenario-2 callback never ran\n");
    return 1;
  }
  if (!self_removed_ok.load()) {
    std::printf("FAIL: scenario-2 remove_reader() from own callback failed\n");
    return 1;
  }

  part.stop();
  pub.stop();
  std::printf("PASS\n");
  return 0;
}
