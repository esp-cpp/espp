// Regression: Domain::deleteReader() must not hold the Domain mutex while the
// reader drains its in-flight dispatches.
//
// reset() waits for the reader's running callback, and that callback may
// legitimately create an endpoint - add_writer() reaches Domain::createWriter,
// which takes the Domain mutex. With deleteReader() holding that mutex across
// reset(), the callback's add_writer() waited for the mutex while reset()
// waited for the callback: a deadlock between a thread removing a reader and
// the reader's own delivery. (The facade already releases ITS mutex for this
// phase; the engine's was the one left.)
//
// Reproduced deterministically: an INLINE (Normal-band) reader whose
// on_sample waits on a gate and then calls add_writer() for a new topic;
// remove_reader() is called from another thread while the callback is in
// flight, then the gate opens. Both must complete promptly and the
// add_writer() must have succeeded.
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
  const char *topic_in = "delete_reader_in";   // the reader under removal
  const char *topic_new = "delete_reader_new"; // the callback creates this writer

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  TestParticipant part({.log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !part.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (!pub.add_writer(
          {.topic = topic_in, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }

  std::atomic<bool> in_callback{false};
  std::atomic<bool> gate_open{false};
  std::atomic<bool> callback_done{false};
  std::atomic<bool> callback_added_writer{false};
  if (!part.add_reader(
          {.topic = topic_in,
           .type_name = type,
           .reliability = Reliability::RELIABLE,
           .on_sample = [&](std::span<const uint8_t>) {
             if (in_callback.exchange(true)) {
               return; // only the first delivery plays
             }
             while (!gate_open.load()) {
               std::this_thread::sleep_for(1ms);
             }
             // Creating an endpoint from a callback is allowed;
             // it takes the facade mutex and then the Domain's.
             callback_added_writer = part.add_writer(
                 {.topic = topic_new, .type_name = type, .reliability = Reliability::RELIABLE});
             callback_done = true;
           }})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  // Drive samples until the callback is executing.
  std::atomic<bool> stop_pub{false};
  std::thread pub_thread([&]() {
    while (!stop_pub.load() && !in_callback.load()) {
      auto bytes = cdr::serialize<cdr::xcdr1>(SeqMsg{1});
      if (bytes) {
        pub.publish(topic_in, u8_span(*bytes));
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

  // Remove the reader while its callback is in flight; on its own thread so a
  // deadlock shows up as a timeout rather than a hang.
  std::atomic<bool> removed{false};
  std::atomic<bool> removed_ok{false};
  std::thread remover([&]() {
    removed_ok = part.remove_reader(topic_in);
    removed = true;
  });
  // Let remove_reader() reach reset()'s drain, then release the callback into
  // its add_writer().
  std::this_thread::sleep_for(200ms);
  gate_open = true;

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while ((!removed.load() || !callback_done.load()) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  const bool ok = removed.load() && callback_done.load();
  if (ok) {
    remover.join();
  } else {
    remover.detach(); // stuck: report instead of hanging on join
  }
  if (!ok) {
    std::printf("FAIL: deadlock - remove_reader %s, callback's add_writer %s\n",
                removed.load() ? "returned" : "stuck", callback_done.load() ? "returned" : "stuck");
    return 1;
  }
  if (!removed_ok.load()) {
    std::printf("FAIL: remove_reader() returned false (removal not exercised)\n");
    return 1;
  }
  if (!callback_added_writer.load()) {
    std::printf("FAIL: the callback's add_writer() failed\n");
    return 1;
  }
  part.stop();
  pub.stop();
  std::printf("PASS\n");
  return 0;
}
