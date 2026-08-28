// Writer create/publish/delete churn under load - the writer-side analog of
// rtps_banded_churn. Guards the writer-teardown lifetime class found in review:
// a reliable writer's publish() submits guaranteed progress() jobs keyed by the
// writer, so deleting it races any parked/queued job (formerly: use-after-free
// of the reset history, sends on a released dedicated port, stale jobs against
// a reused pool slot). The engine now cancels parked jobs, generation-guards
// accepted ones, and quiesces in-flight progress() under the writer mutex -
// this test churns exactly that window, repeatedly.
//
//  Phase 1 (hostile churn): while a persistent flood writer keeps the
//  transport pool busy, a banded (dedicated-port capable) writer on a second
//  topic is repeatedly created, flood-published (no pacing - guaranteed
//  progress() jobs park under saturation), and IMMEDIATELY deleted with those
//  jobs still parked/in flight. Any lifetime bug is a crash/terminate (and a
//  sanitizer report under the ASan/TSan CI leg); a leak of the dedicated-port
//  ration eventually exhausts the ration and surfaces as add_writer failures.
//
//  Phase 2 (verified churn): the same create/publish/delete cycle, but each
//  iteration waits for at least one sample to arrive end-to-end before the
//  delete - proving the churned writer slot is fully functional after every
//  reuse (a stale-generation bug that ate samples would fail here).
//
// The test must complete well under the external timeout the harness applies;
// a deadlocked delete shows up as a timeout kill.
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "cdr.hpp"
#include "rtps_participant.hpp"

#include "rtps_common.hpp"

struct StringMsg {
  std::string data;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

namespace {
// Expose the protected remove_writer() for this test (same pattern as
// rtps_remove_reader_deadlock exposing remove_reader()).
struct TestParticipant : espp::RtpsParticipant {
  using espp::RtpsParticipant::remove_writer;
  using espp::RtpsParticipant::RtpsParticipant;
};
} // namespace

using namespace std::chrono_literals;

int main() {
  using Reliability = espp::RtpsParticipant::Reliability;
  const char *type = "std_msgs::msg::dds_::String_";
  const char *flood_topic = "writer_churn_flood";
  const char *churn_topic = "writer_churn_topic";

  std::string ip;
  // Portable interface discovery (rtps_common.hpp builds on POSIX and MSVC);
  // the loopback fallback means no usable interface was found.
  ip = rtps_test::guess_local_ipv4();
  if (ip.rfind("127.", 0) == 0) {
    std::printf("FAIL: no usable IPv4 interface\n");
    return 1;
  }

  TestParticipant pub({.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  std::atomic<int> flood_received{0};
  std::atomic<int> churn_received{0};
  if (!pub.add_writer(
          {.topic = flood_topic, .type_name = type, .reliability = Reliability::RELIABLE}) ||
      !sub.add_reader(
          {.topic = flood_topic,
           .type_name = type,
           .reliability = Reliability::RELIABLE,
           .on_sample = [&](std::span<const uint8_t>) { flood_received.fetch_add(1); }}) ||
      !sub.add_reader(
          {.topic = churn_topic,
           .type_name = type,
           .reliability = Reliability::RELIABLE,
           .on_sample = [&](std::span<const uint8_t>) { churn_received.fetch_add(1); }})) {
    std::printf("FAIL: persistent endpoint setup\n");
    return 1;
  }

  // Persistent flood keeps the transport pool + reactor busy for the whole
  // test so writer deletion always races live traffic.
  std::atomic<bool> flood{true};
  std::thread flooder([&]() {
    int i = 0;
    while (flood.load()) {
      auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"flood " + std::to_string(i++)});
      if (bytes) {
        (void)pub.publish(flood_topic, u8_span(*bytes));
      }
      std::this_thread::sleep_for(1ms);
    }
  });

  // Wait for the persistent pair to match (discovery settled) before churning.
  const auto match_deadline = std::chrono::steady_clock::now() + 15s;
  while (flood_received.load() == 0 && std::chrono::steady_clock::now() < match_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  if (flood_received.load() == 0) {
    std::printf("FAIL: flood topic never delivered (discovery)\n");
    flood = false;
    flooder.join();
    return 1;
  }

  // Baseline for the flood-progress assertion below: flood_received was
  // already required to be nonzero above, so the final check must require an
  // INCREASE across the churn (a plain nonzero check would be vacuous).
  const int flood_before_churn = flood_received.load();

  // ---- Phase 1: hostile churn - delete with progress() jobs in flight ------
  constexpr int kHostileIterations = 15;
  constexpr int kBurst = 25;
  for (int iter = 0; iter < kHostileIterations; ++iter) {
    // Alternate bands so both the dedicated-port path (banded) and the shared
    // path get churned.
    const auto band = (iter % 2 == 0) ? espp::QosBand::High : espp::QosBand::Normal;
    if (!pub.add_writer({.topic = churn_topic,
                         .type_name = type,
                         .reliability = Reliability::RELIABLE,
                         .band = band})) {
      std::printf("FAIL: add_writer iteration %d (leaked ration slot?)\n", iter);
      flood = false;
      flooder.join();
      return 1;
    }
    for (int i = 0; i < kBurst; ++i) {
      auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"churn " + std::to_string(i)});
      if (bytes) {
        (void)pub.publish(churn_topic, u8_span(*bytes));
      }
    }
    // Delete immediately: the burst's guaranteed progress() jobs are still
    // queued/parked. This is the raced window.
    if (!pub.remove_writer(churn_topic)) {
      std::printf("FAIL: remove_writer iteration %d\n", iter);
      flood = false;
      flooder.join();
      return 1;
    }
  }
  std::printf("phase 1 OK: %d hostile churn iterations (churn samples so far: %d)\n",
              kHostileIterations, churn_received.load());

  // ---- Phase 2: verified churn - every reused slot must still deliver ------
  // Between verified iterations, let the SEDP disposal of the previous writer
  // propagate before announcing its replacement: endpoints are pooled, so a
  // rapid re-add can reuse the previous GUID, and if the reader processes the
  // new announce BEFORE the old dispose it keeps stale reliable-stream state
  // for that GUID and drops the new writer's samples as duplicates. That
  // ordering hazard is inherent to same-topic churn on a shared pool (peers
  // see two announcements racing), not the writer-teardown lifetime class this
  // test guards - phase 1 covers the hostile no-settle path.
  constexpr auto kSettle = 250ms;
  constexpr int kVerifiedIterations = 5;
  std::this_thread::sleep_for(kSettle);
  for (int iter = 0; iter < kVerifiedIterations; ++iter) {
    const int before = churn_received.load();
    if (!pub.add_writer({.topic = churn_topic,
                         .type_name = type,
                         .reliability = Reliability::RELIABLE,
                         .band = espp::QosBand::High})) {
      std::printf("FAIL: verified add_writer iteration %d\n", iter);
      flood = false;
      flooder.join();
      return 1;
    }
    // Publish until at least one sample lands (reliable recovery covers the
    // pre-match window), then delete.
    const auto deliver_deadline = std::chrono::steady_clock::now() + 10s;
    while (churn_received.load() == before && std::chrono::steady_clock::now() < deliver_deadline) {
      auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"verified " + std::to_string(iter)});
      if (bytes) {
        (void)pub.publish(churn_topic, u8_span(*bytes));
      }
      std::this_thread::sleep_for(20ms);
    }
    if (churn_received.load() == before) {
      std::printf("FAIL: churned writer slot never delivered (iteration %d)\n", iter);
      flood = false;
      flooder.join();
      return 1;
    }
    if (!pub.remove_writer(churn_topic)) {
      std::printf("FAIL: verified remove_writer iteration %d\n", iter);
      flood = false;
      flooder.join();
      return 1;
    }
    std::this_thread::sleep_for(kSettle); // let the dispose land before the next announce
  }
  std::printf("phase 2 OK: %d verified churn iterations\n", kVerifiedIterations);

  // Flood must have kept MAKING PROGRESS across all the churn (>= ~40 ms
  // worth of 1 ms-paced samples is a lenient floor that still catches a
  // wedged flood, which would show zero new deliveries).
  const int flood_final = flood_received.load();
  flood = false;
  flooder.join();
  constexpr int kMinFloodProgress = 40;
  if (flood_final - flood_before_churn < kMinFloodProgress) {
    std::printf("FAIL: flood stalled during churn (before=%d, after=%d)\n", flood_before_churn,
                flood_final);
    return 1;
  }

  pub.stop();
  sub.stop();
  std::printf("PASS (flood=%d, churn=%d samples)\n", flood_final, churn_received.load());
  return 0;
}
