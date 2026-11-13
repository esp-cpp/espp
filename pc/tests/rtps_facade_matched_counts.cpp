// RtpsParticipant::matched_writers() / matched_readers() over a real loopback,
// and the dispose path end to end: when the publisher removes its writer (which
// announces the SEDP dispose), the subscriber's matched-writer count must fall
// to zero at once, not when the peer's lease would have expired. (Stopping a
// participant announces nothing today - its peers rely on the lease - so the
// removal is what drives the dispose here.)
//
// Two participants in one process. The subscriber has TWO readers on the topic
// (a reliable and a best-effort one - add_reader() allows that); the count is
// of DISTINCT remote writers across both, so the one publisher counts once
// however many of the readers discovery pairs it with.
//
// Exits 0 on success, 1 on failure.

#include <chrono>
#include <cstdio>
#include <thread>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
// remove_writer() is protected (it exists for composite rollback); expose it.
struct TestParticipant : espp::RtpsParticipant {
  using espp::RtpsParticipant::remove_writer;
  using espp::RtpsParticipant::RtpsParticipant;
};

template <typename Pred> bool wait_until(Pred pred, std::chrono::milliseconds limit) {
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}
} // namespace

int main() {
  using Reliability = espp::RtpsParticipant::Reliability;
  const char *topic = "matched_counts";
  const char *type = "espp::test::dds_::Counts_";

  TestParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (sub.matched_writers(topic) != 0 || pub.matched_readers(topic) != 0) {
    std::printf("FAIL: counts for a topic without endpoints must be 0\n");
    return 1;
  }
  if (!pub.add_writer({.topic = topic, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  const auto noop = [](std::span<const uint8_t>) {};
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = noop}) ||
      !sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::BEST_EFFORT,
                       .on_sample = noop})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  // Discovery: the one remote writer, counted once across the two readers;
  // the writer sees at least one of the readers (every compatible one once
  // discovery matches all of them).
  if (!wait_until([&] { return sub.matched_writers(topic) == 1; }, 10s)) {
    std::printf("FAIL: matched_writers = %zu, expected 1 after discovery\n",
                sub.matched_writers(topic));
    return 1;
  }
  if (!wait_until([&] { return pub.matched_readers(topic) >= 1; }, 10s)) {
    std::printf("FAIL: matched_readers = %zu, expected >= 1 after discovery\n",
                pub.matched_readers(topic));
    return 1;
  }
  std::printf("discovered: matched_writers=%zu matched_readers=%zu\n", sub.matched_writers(topic),
              pub.matched_readers(topic));

  // The publisher removes its writer: the SEDP dispose must unmatch it within
  // a couple of seconds (the lease is far longer), on every reader it reached;
  // then a NEW writer on the topic must be matched again, and so on - a peer
  // that churns its publishers (ros2_churn_publisher.py) does exactly this.
  for (int round = 1; round <= 3; ++round) {
    const auto left = std::chrono::steady_clock::now();
    if (!pub.remove_writer(topic)) {
      std::printf("FAIL: remove_writer (round %d)\n", round);
      return 1;
    }
    if (!wait_until([&] { return sub.matched_writers(topic) == 0; }, 3s)) {
      std::printf("FAIL: matched_writers still %zu 3 s after the writer was removed (round %d)\n",
                  sub.matched_writers(topic), round);
      return 1;
    }
    const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - left);
    std::printf("round %d: unmatched %lld ms after the writer was removed\n", round,
                static_cast<long long>(took.count()));
    if (pub.matched_readers(topic) != 0) {
      std::printf("FAIL: matched_readers for a removed writer must be 0\n");
      return 1;
    }
    if (!pub.add_writer(
            {.topic = topic, .type_name = type, .reliability = Reliability::RELIABLE})) {
      std::printf("FAIL: add_writer (round %d)\n", round);
      return 1;
    }
    if (!wait_until([&] { return sub.matched_writers(topic) == 1; }, 10s)) {
      std::printf("FAIL: a new writer on the topic was not matched again (round %d)\n", round);
      return 1;
    }
  }
  pub.stop();
  sub.stop();
  std::printf("PASS\n");
  return 0;
}
