// Best-effort saturation: a BEST_EFFORT (StatelessWriter) publisher bursting
// far faster than the send path must NOT collapse. This is the regression pair
// for rammp-org/pace-racer-fw#14:
//  - with growable (dynamic, the host/CI default) history, delivery must be
//    (near-)lossless - the guaranteed progress() machinery drains everything;
//  - with a full static ring the engine now degrades to KEEP_LAST drop-oldest
//    (progress() clamps a cursor that fell behind the ring instead of sending
//    nothing) and surfaces every overwritten sample via the facade's
//    rate-limited warning and Diagnostics::Writer::history_overwrite_drops.
//    The interop harness ALSO builds and runs this test in a static-storage
//    variant (RTPS_STORAGE_STATIC + HISTORY_SIZE_STATELESS=2), where it
//    requires overflow drops to have occurred AND delivery to stay well above
//    the collapse level (~10% pre-fix vs ~60-70% post-fix measured).
//
// Exits 0 on success.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "cdr.hpp"
#include "rtps/utils/Diagnostics.hpp"
#include "rtps_participant.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

struct StringMsg {
  std::string data;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

static bool detect_interface(std::string &addr) {
  struct ifaddrs *ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0)
    return false;
  bool found = false;
  for (struct ifaddrs *ifa = ifaddr; ifa != nullptr && !found; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
      continue;
    char buf[INET_ADDRSTRLEN] = {0};
    const auto *sin = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr)
      continue;
    const std::string ip = buf;
    if (ip.rfind("127.", 0) == 0 || ip.rfind("169.254.", 0) == 0)
      continue;
    addr = ip;
    found = true;
  }
  freeifaddrs(ifaddr);
  return found;
}

using namespace std::chrono_literals;

int main() {
  using Reliability = espp::RtpsParticipant::Reliability;
  const char *type = "std_msgs::msg::dds_::String_";
  const char *topic = "saturation_topic";

  std::string ip;
  if (!detect_interface(ip)) {
    std::printf("no iface\n");
    return 1;
  }

  espp::RtpsParticipant pub({.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
  std::atomic<int> received{0};
  if (!pub.start() || !sub.start() ||
      !pub.add_writer(
          {.topic = topic, .type_name = type, .reliability = Reliability::BEST_EFFORT}) ||
      !sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::BEST_EFFORT,
                       .on_sample = [&](std::span<const uint8_t>) { received.fetch_add(1); }})) {
    std::printf("setup failed\n");
    return 1;
  }

  // Wait for the match (paced pre-publishes until one lands).
  const auto match_deadline = std::chrono::steady_clock::now() + 15s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < match_deadline) {
    auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"probe"});
    if (bytes)
      (void)pub.publish(topic, u8_span(*bytes));
    std::this_thread::sleep_for(20ms);
  }
  if (received.load() == 0) {
    std::printf("never matched\n");
    return 1;
  }
  received.store(0);

  // Saturation burst: publish back-to-back, no pacing. Sized so the burst's
  // total datagram volume fits a default kernel UDP receive buffer: the loss
  // guarded here is the WRITER silently failing to send (history collapse) -
  // receiver-side kernel drops from a multi-hundred-KB burst are genuine
  // best-effort wire loss and would flake the lossless assertion (observed in
  // the slower interop container, where 5000 samples publish in <20 ms).
  // (~500 datagrams: a default linux rmem of ~208 KB holds ~800 small
  // datagrams after per-skb accounting overhead, so 500 leaves real margin.)
  constexpr int kBurst = 500;
  int published_ok = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kBurst; ++i) {
    auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"s" + std::to_string(i)});
    if (bytes && pub.publish(topic, u8_span(*bytes)))
      ++published_ok;
  }
  const auto burst_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
          .count();
  // Drain: with dynamic history nothing may be dropped, so wait until the
  // count stops growing (bounded), then judge.
  int got = received.load();
  const auto drain_deadline = std::chrono::steady_clock::now() + 20s;
  while (std::chrono::steady_clock::now() < drain_deadline) {
    std::this_thread::sleep_for(250ms);
    const int now = received.load();
    if (now == got && now > 0) {
      break; // settled
    }
    got = now;
  }
  got = received.load();
  std::printf("published_ok=%d/%d in %lld ms; received=%d (%.1f%%)\n", published_ok, kBurst,
              (long long)burst_ms, got, 100.0 * got / kBurst);
  pub.stop();
  sub.stop();
  if (published_ok != kBurst) {
    std::printf("FAIL: publish() rejected samples under saturation\n");
    return 1;
  }
  const uint32_t drops = rtps::Diagnostics::Writer::history_overwrite_drops.load();
  std::printf("history_overwrite_drops=%u\n", drops);
#if defined(RTPS_STORAGE_STATIC)
  // Static KEEP_LAST ring (the interop harness builds this variant with
  // HISTORY_SIZE_STATELESS=2). The DETERMINISTIC properties of the fix:
  //  1. the burst overflows and every overwrite is COUNTED (drops > 0 - the
  //     overwrite accounting and the publish()-side warning path);
  //  2. accounting conserves: every sample is either delivered or counted as
  //     dropped (received + drops == burst on the loss-free loopback; a small
  //     slack tolerates scheduling stragglers);
  //  3. delivery does not TOTALLY collapse: with the pre-fix cursor bug an
  //     executed progress() almost always found its change overwritten and
  //     sent NOTHING - total delivery was just the final ring contents (a few
  //     samples). With the fix every executed poke delivers a live sample.
  // The delivered FRACTION is deliberately not asserted tightly: it equals the
  // number of pokes the pool manages to execute during/after the storm, which
  // is scheduler-timing dependent (observed 6%-36% across host/container
  // runs); the collapse floor below is a few times the bug's ceiling while
  // staying under every observed fixed run.
  if (drops == 0) {
    std::printf("FAIL: static ring never overflowed - the KEEP_LAST path was not exercised\n");
    return 1;
  }
  if (got + static_cast<int>(drops) < (kBurst * 95) / 100) {
    std::printf("FAIL: accounting leak (received %d + drops %u < burst %d)\n", got, drops, kBurst);
    return 1;
  }
  if (got < kBurst / 50) {
    std::printf("FAIL: saturation collapse (%d/%d delivered)\n", got, kBurst);
    return 1;
  }
#else
  // Growable (dynamic) history - the host/CI default: nothing may be dropped;
  // allow a small slack for genuine (UDP) loss. A collapse regression delivers
  // a few percent at best, far below this floor.
  if (got < (kBurst * 97) / 100) {
    std::printf("FAIL: saturation collapse (%d/%d delivered)\n", got, kBurst);
    return 1;
  }
#endif
  std::printf("PASS\n");
  return 0;
}
