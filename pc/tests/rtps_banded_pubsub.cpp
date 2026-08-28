// In-process loopback proving a banded subscriber on a DEDICATED unicast port
// interoperates end-to-end:
//
//   publisher: espp::RtpsParticipant facade, plain reliable writer (Normal).
//   subscriber: engine-level rtps::Domain so the test can see the reader's
//     attributes - the reader is created at QosBand::High and must be granted a
//     dedicated port (asserted, incl. the documented port range).
//
// The subscriber's SEDP announcement carries ONLY its per-endpoint unicast
// locator (the dedicated port) - the publisher's writer sends DATA exclusively
// to that announced locator (ReaderProxy::remoteLocator) - so end-to-end sample
// delivery proves the traffic flowed through the dedicated socket, not the
// shared user-unicast port.
//
// Exits 0 when at least kRequired samples arrive within the deadline.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps/entities/Domain.hpp"
#include "rtps/utils/udpUtils.hpp"
#include "rtps_participant.hpp"

#include "rtps_common.hpp"

struct StringMsg {
  std::string data;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

// First non-loopback, non-link-local IPv4 interface (same rule the facade's
// auto-detection uses); both sides must share it so their locators match.

using namespace std::chrono_literals;

int main() {
  constexpr int kRequired = 5;
  constexpr auto kDeadline = 20s;
  const char *topic = "banded_loopback";
  const char *type = "std_msgs::msg::dds_::String_";

  std::string ip;
  rtps::Ip4AddressBytes ip_bytes{};
  // Portable interface discovery (rtps_common.hpp builds on POSIX and MSVC);
  // the loopback fallback means no usable interface was found.
  ip = rtps_test::guess_local_ipv4();
  unsigned ip_a = 0, ip_b = 0, ip_c = 0, ip_d = 0;
  if (ip.rfind("127.", 0) == 0 ||
      std::sscanf(ip.c_str(), "%u.%u.%u.%u", &ip_a, &ip_b, &ip_c, &ip_d) != 4) {
    std::printf("FAIL: no usable IPv4 interface\n");
    return 1;
  }
  ip_bytes = {static_cast<uint8_t>(ip_a), static_cast<uint8_t>(ip_b), static_cast<uint8_t>(ip_c),
              static_cast<uint8_t>(ip_d)};

  // Publisher: facade, default config, plain reliable writer.
  espp::RtpsParticipant pub({.interface_address = ip, .log_level = espp::Logger::Verbosity::INFO});
  if (!pub.start()) {
    std::printf("FAIL: pub start\n");
    return 1;
  }
  if (!pub.add_writer({.topic = topic,
                       .type_name = type,
                       .reliability = espp::RtpsParticipant::Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }

  // Subscriber: engine-level domain so the dedicated port is observable.
  rtps::Domain sub_domain(ip_bytes);
  rtps::Participant *part = sub_domain.createParticipant();
  if (part == nullptr) {
    std::printf("FAIL: sub createParticipant\n");
    return 1;
  }
  rtps::Reader *reader = sub_domain.createReader(*part, topic, type, /*reliable=*/true,
                                                 {0, 0, 0, 0}, {.band = espp::QosBand::High});
  if (reader == nullptr) {
    std::printf("FAIL: sub createReader\n");
    return 1;
  }
  if (!reader->m_attributes.hasDedicatedPort) {
    std::printf("FAIL: banded reader was not granted a dedicated port\n");
    return 1;
  }
  const auto dedicated_port = reader->m_attributes.unicastLocator.port;
  const rtps::Ip4Port_t dedicated_base = 7400 + 250 * rtps::Config::DOMAIN_ID + 100;
  if (dedicated_port < dedicated_base ||
      dedicated_port > static_cast<uint32_t>(7400 + 250 * rtps::Config::DOMAIN_ID + 249)) {
    std::printf("FAIL: dedicated port %u outside the documented range\n",
                static_cast<unsigned>(dedicated_port));
    return 1;
  }
  if (dedicated_port == rtps::getUserUnicastPort(part->m_participantId)) {
    std::printf("FAIL: dedicated port equals the shared user port\n");
    return 1;
  }
  std::printf("subscriber dedicated port: %u (shared would be %u)\n",
              static_cast<unsigned>(dedicated_port),
              static_cast<unsigned>(rtps::getUserUnicastPort(part->m_participantId)));

  static std::atomic<int> received{0};
  reader->registerCallback(
      [](void *, const rtps::ReaderCacheChange &change) {
        std::vector<uint8_t> payload(change.getDataSize());
        if (payload.empty() || !change.copyInto(payload.data(), change.getDataSize())) {
          return;
        }
        if (cdr::deserialize<StringMsg>(
                std::as_bytes(std::span<const uint8_t>(payload.data(), payload.size())))) {
          received.fetch_add(1);
        }
      },
      nullptr);

  if (!sub_domain.completeInit()) {
    std::printf("FAIL: sub completeInit\n");
    return 1;
  }

  int sent = 0;
  const auto start = std::chrono::steady_clock::now();
  while (received.load() < kRequired && std::chrono::steady_clock::now() - start < kDeadline) {
    auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"banded sample " + std::to_string(sent)});
    if (bytes && pub.publish(topic, u8_span(*bytes))) {
      sent++;
    }
    std::this_thread::sleep_for(100ms);
  }

  const int n = received.load();
  std::printf("sent=%d received=%d (via dedicated port %u)\n", sent, n,
              static_cast<unsigned>(dedicated_port));
  pub.stop();
  sub_domain.stop();
  if (n >= kRequired) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
