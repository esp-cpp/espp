// Shutdown/teardown-under-load stress for per-endpoint priority:
//
//  Phase 1 (churn): while a publisher floods a reliable topic, the subscriber
//  domain repeatedly creates a banded (dedicated-port) reader, receives live
//  traffic on it, and deletes it - exercising releaseReceivePort() with
//  datagrams in flight on the released socket and immediate fd-number reuse by
//  the next dedicated port. This is the reproducer for the CI shutdown hang:
//  a stale select() readiness bit aliased onto a reused fd dispatched a
//  handler with no data, whose unbounded blocking recvfrom wedged
//  SocketReactor::stop() forever.
//
//  Phase 2 (stop under load): a banded SHARED-port reader (dedicated ports
//  disabled at the engine level is facade behavior; here ration cap 0) with
//  deferred banded dispatch receives a flood, and the domains are stopped
//  WHILE deliveries are in flight.
//
// The test must complete well under the external timeout the harness applies;
// any shutdown hang shows up as a timeout kill.
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps/entities/Domain.hpp"
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

static bool detect_interface(std::string &addr, rtps::Ip4AddressBytes &bytes) {
  struct ifaddrs *ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    return false;
  }
  bool found = false;
  for (struct ifaddrs *ifa = ifaddr; ifa != nullptr && !found; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    char buf[INET_ADDRSTRLEN] = {0};
    const auto *sin = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
      continue;
    }
    const std::string ip = buf;
    if (ip.rfind("127.", 0) == 0 || ip.rfind("169.254.", 0) == 0) {
      continue;
    }
    addr = ip;
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
      bytes = {static_cast<uint8_t>(a), static_cast<uint8_t>(b), static_cast<uint8_t>(c),
               static_cast<uint8_t>(d)};
      found = true;
    }
  }
  freeifaddrs(ifaddr);
  return found;
}

using namespace std::chrono_literals;

int main() {
  constexpr int kChurnIterations = 25;
  const char *topic = "churn_topic";
  const char *type = "std_msgs::msg::dds_::String_";

  std::string ip;
  rtps::Ip4AddressBytes ip_bytes{};
  if (!detect_interface(ip, ip_bytes)) {
    std::printf("FAIL: no usable IPv4 interface\n");
    return 1;
  }

  // ---- Phase 1: dedicated-port churn under flood --------------------------
  {
    espp::RtpsParticipant pub(
        {.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
    if (!pub.start() ||
        !pub.add_writer({.topic = topic,
                         .type_name = type,
                         .reliability = espp::RtpsParticipant::Reliability::RELIABLE})) {
      std::printf("FAIL: pub setup\n");
      return 1;
    }
    // Flood: publish continuously from a thread until told to stop.
    std::atomic<bool> flood{true};
    std::thread flooder([&]() {
      int i = 0;
      while (flood.load()) {
        auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"churn " + std::to_string(i++)});
        if (bytes) {
          (void)pub.publish(topic, u8_span(*bytes));
        }
        std::this_thread::sleep_for(2ms);
      }
    });

    rtps::Domain sub_domain(ip_bytes);
    rtps::Participant *part = sub_domain.createParticipant();
    if (part == nullptr) {
      std::printf("FAIL: sub participant\n");
      flood = false;
      flooder.join();
      return 1;
    }
    if (!sub_domain.completeInit()) {
      std::printf("FAIL: sub completeInit\n");
      flood = false;
      flooder.join();
      return 1;
    }

    static std::atomic<int> received{0};
    int churned = 0;
    uint32_t first_dedicated_port = 0; // asserted bindable again after the churn
    for (int iter = 0; iter < kChurnIterations; ++iter) {
      rtps::Reader *reader = sub_domain.createReader(*part, topic, type, /*reliable=*/true,
                                                     {0, 0, 0, 0}, {.band = espp::QosBand::High});
      if (reader == nullptr) {
        std::printf("FAIL: createReader iter %d\n", iter);
        flood = false;
        flooder.join();
        return 1;
      }
      if (!reader->m_attributes.hasDedicatedPort) {
        std::printf("FAIL: no dedicated port at iter %d\n", iter);
        flood = false;
        flooder.join();
        return 1;
      }
      if (first_dedicated_port == 0) {
        first_dedicated_port = reader->m_attributes.unicastLocator.port;
      }
      const int before = received.load();
      reader->registerCallback(
          [](void *, const rtps::ReaderCacheChange &) { received.fetch_add(1); }, nullptr);
      // Wait until live traffic flows over THIS dedicated port. This is a
      // REQUIREMENT: an iteration that never sees a sample has not exercised
      // the delete-with-traffic-in-flight race the test exists for, so it
      // fails rather than silently churning idle sockets. The first iteration
      // gets a longer deadline for SPDP/SEDP discovery and matching.
      const auto deadline = std::chrono::steady_clock::now() + (iter == 0 ? 15s : 5s);
      while (received.load() < before + 2 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(5ms);
      }
      if (received.load() < before + 2) {
        std::printf("FAIL: iter %d saw no live samples on its dedicated port (received %d)\n", iter,
                    received.load() - before);
        flood = false;
        flooder.join();
        return 1;
      }
      // Delete the reader (closing/releasing its dedicated port) WHILE the
      // publisher is still sending to it - the next iteration's dedicated
      // port immediately reuses the freed slot (and likely the fd number).
      if (!sub_domain.deleteReader(*part, reader)) {
        std::printf("FAIL: deleteReader iter %d\n", iter);
        flood = false;
        flooder.join();
        return 1;
      }
      ++churned;
    }

    flood = false;
    flooder.join();
    std::printf("phase1: churned %d dedicated-port readers, received %d samples\n", churned,
                received.load());
    // Prompt fd/port release: the retired sockets must drain via the
    // reactor's removal-completion callbacks (NOT accumulate until stop) -
    // and the very first iteration's dedicated port must be bindable again
    // by a fresh reuse-disabled socket, all BEFORE the domains stop.
    {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (sub_domain.getTransport().retiredSocketCount() > 0 &&
             std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
      }
      const std::size_t retired = sub_domain.getTransport().retiredSocketCount();
      if (retired != 0) {
        std::printf("FAIL: %zu retired sockets still parked after churn\n", retired);
        return 1;
      }
      bool rebindable = false;
      const auto bind_deadline = std::chrono::steady_clock::now() + 2s;
      while (!rebindable && std::chrono::steady_clock::now() < bind_deadline) {
        espp::UdpSocket probe({.log_level = espp::Logger::Verbosity::NONE});
        espp::UdpSocket::ReceiveConfig rc;
        rc.port = static_cast<uint16_t>(first_dedicated_port);
        rebindable = probe.is_valid() && probe.disable_reuse() && probe.bind(rc);
        if (!rebindable) {
          std::this_thread::sleep_for(20ms);
        }
      }
      if (!rebindable) {
        std::printf("FAIL: first dedicated port %u not released before stop\n",
                    static_cast<unsigned>(first_dedicated_port));
        return 1;
      }
      std::printf("phase1: retired sockets drained, port %u released before stop\n",
                  static_cast<unsigned>(first_dedicated_port));
    }
    // Teardown with the peer still matched: sub_domain and pub stop here. A
    // shutdown hang (the CI failure mode) trips the harness timeout.
    sub_domain.stop();
    pub.stop();
    if (churned != kChurnIterations) {
      std::printf("FAIL: churn incomplete\n");
      return 1;
    }
  }

  // ---- Phase 2: stop() while deferred deliveries are in flight ------------
  {
    espp::RtpsParticipant pub(
        {.interface_address = ip, .log_level = espp::Logger::Verbosity::WARN});
    espp::RtpsParticipant sub({.interface_address = ip,
                               .log_level = espp::Logger::Verbosity::WARN,
                               .enable_dedicated_endpoint_ports = false});
    if (!pub.start() || !sub.start()) {
      std::printf("FAIL: phase2 start\n");
      return 1;
    }
    if (!pub.add_writer({.topic = topic,
                         .type_name = type,
                         .reliability = espp::RtpsParticipant::Reliability::RELIABLE})) {
      std::printf("FAIL: phase2 writer\n");
      return 1;
    }
    std::atomic<int> received{0};
    // Banded shared-port reader -> deferred banded dispatch; the callback
    // dawdles so deliveries are IN FLIGHT (and queued) when stop() runs.
    if (!sub.add_reader({.topic = topic,
                         .type_name = type,
                         .reliability = espp::RtpsParticipant::Reliability::RELIABLE,
                         .on_sample =
                             [&received](std::span<const uint8_t>) {
                               received.fetch_add(1);
                               std::this_thread::sleep_for(20ms);
                             },
                         .band = espp::QosBand::High})) {
      std::printf("FAIL: phase2 reader\n");
      return 1;
    }
    std::atomic<bool> flood{true};
    std::thread flooder([&]() {
      int i = 0;
      while (flood.load()) {
        auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"stop-load " + std::to_string(i++)});
        if (bytes) {
          (void)pub.publish(topic, u8_span(*bytes));
        }
        std::this_thread::sleep_for(2ms);
      }
    });
    // Wait for the pipeline to be visibly active, then stop UNDER load.
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (received.load() < 3 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(5ms);
    }
    const int seen = received.load();
    sub.stop(); // deliveries in flight + queued in the deferred dispatcher
    flood = false;
    flooder.join();
    pub.stop();
    std::printf("phase2: received %d before stop-under-load\n", seen);
    if (seen < 3) {
      std::printf("FAIL: phase2 no traffic before stop\n");
      return 1;
    }
  }

  std::printf("PASS\n");
  return 0;
}
