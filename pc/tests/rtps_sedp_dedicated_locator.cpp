// Engine-level checks for per-endpoint priority (dedicated unicast ports):
//
//  1. A banded endpoint (reader or writer) is granted a dedicated unicast port
//     from the documented range (7400 + 250*domain + 100 + n) and its SEDP
//     announcement carries that port in the standard PID_UNICAST_LOCATOR
//     parameter - verified byte-for-byte against the parameter encoding, plus a
//     round-trip parse.
//  2. A default (Normal, no dscp) endpoint keeps the shared user-unicast port
//     and no dedicated flag - i.e. the pre-band behavior is unchanged.
//  3. The ration (DomainConfig::max_prioritized_endpoint_ports) is enforced:
//     endpoints beyond the cap fall back to the shared port.
//  4. Deleting a dedicated-port endpoint returns its port to the ration.
//  5. With enable_dedicated_endpoint_ports=false no dedicated port is granted.
//  6. The ration is a TRUE fd bound: a released socket whose fd is still open
//     (removal completion pending behind a busy worker pool) counts against
//     the cap; once it closes, the slot is usable again.
//  7. A fully-occupied probe window advances the allocator: after a failed
//     allocation the next attempt probes fresh ports and succeeds.
//
// Exits 0 on success, 1 on the first failed check.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <span>
#include <thread>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/utils/udpUtils.hpp"

namespace {

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

std::vector<uint8_t> serialize_attributes(const rtps::TopicData &attributes) {
  std::vector<uint8_t> buf(1024, 0);
  rtps::CdrSink sink{rtps::asWritableBytes(buf.data(), buf.size())};
  rtps::CdrWriter writer(sink);
  if (!attributes.serializeInto(writer)) {
    return {};
  }
  buf.resize(sink.size());
  return buf;
}

// The expected PID_UNICAST_LOCATOR parameter for a UDPv4 locator on kIp:port -
// the exact bytes a peer parses: pid(2) len(2) kind(4) port(4) address(16),
// all little-endian, address IPv4-mapped in the last 4 bytes.
std::vector<uint8_t> expected_unicast_locator_param(uint32_t port) {
  std::vector<uint8_t> p;
  const auto push_u16 = [&p](uint16_t v) {
    p.push_back(static_cast<uint8_t>(v & 0xFF));
    p.push_back(static_cast<uint8_t>(v >> 8));
  };
  const auto push_u32 = [&p](uint32_t v) {
    for (int i = 0; i < 4; ++i) {
      p.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
  };
  push_u16(static_cast<uint16_t>(rtps::SMElement::ParameterId::PID_UNICAST_LOCATOR));
  push_u16(sizeof(rtps::FullLengthLocator)); // 24
  push_u32(static_cast<uint32_t>(rtps::LocatorKind_t::LOCATOR_KIND_UDPv4));
  push_u32(port);
  for (int i = 0; i < 12; ++i) {
    p.push_back(0);
  }
  p.insert(p.end(), kIp.begin(), kIp.end());
  return p;
}

// Poll (up to ~2 s) until a fresh reuse-disabled socket can bind the port -
// i.e. the previously bound fd was actually closed, not parked until stop().
bool wait_for_port_released(uint16_t port) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    espp::UdpSocket probe({.log_level = espp::Logger::Verbosity::NONE});
    espp::UdpSocket::ReceiveConfig rc;
    rc.port = port;
    if (probe.is_valid() && probe.disable_reuse() && probe.bind(rc)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

// Poll (up to ~2 s) until the transport reports no retired sockets pending.
bool wait_for_no_retired_sockets(rtps::EsppTransport &transport) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    if (transport.retiredSocketCount() == 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool contains(std::span<const uint8_t> haystack, std::span<const uint8_t> needle) {
  if (needle.empty() || haystack.size() < needle.size()) {
    return false;
  }
  for (size_t i = 0; i + needle.size() <= haystack.size(); ++i) {
    if (std::memcmp(haystack.data() + i, needle.data(), needle.size()) == 0) {
      return true;
    }
  }
  return false;
}

bool run_checks() {
  const rtps::Ip4Port_t dedicated_base = 7400 + 250 * rtps::Config::DOMAIN_ID + 100;
  const rtps::Ip4Port_t dedicated_end = 7400 + 250 * rtps::Config::DOMAIN_ID + 249;

  {
    rtps::DomainConfig cfg;
    cfg.max_prioritized_endpoint_ports = 2; // small ration for check 3
    rtps::Domain domain(kIp, cfg);
    rtps::Participant *part = domain.createParticipant();
    CHECK(part != nullptr, "createParticipant");
    const rtps::Ip4Port_t shared_port = rtps::getUserUnicastPort(part->m_participantId);

    // 1a. Banded reader -> dedicated port, announced via PID_UNICAST_LOCATOR.
    rtps::Reader *banded_reader =
        domain.createReader(*part, "prio_topic", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High, .dscp = espp::Dscp::Ef});
    CHECK(banded_reader != nullptr, "banded reader created");
    CHECK(banded_reader->m_attributes.hasDedicatedPort, "banded reader has dedicated port");
    const auto reader_port = banded_reader->m_attributes.unicastLocator.port;
    CHECK(reader_port >= dedicated_base && reader_port <= dedicated_end,
          "reader port in the documented dedicated range");
    CHECK(reader_port != shared_port, "reader port differs from the shared user port");
    CHECK(banded_reader->m_attributes.band == espp::QosBand::High, "band recorded");

    const auto sedp = serialize_attributes(banded_reader->m_attributes);
    CHECK(!sedp.empty(), "SEDP serialization");
    const auto expected = expected_unicast_locator_param(reader_port);
    CHECK(contains(sedp, expected),
          "SEDP bytes contain the dedicated-port PID_UNICAST_LOCATOR parameter");
    // Round-trip: a peer parsing the announcement sees the dedicated port.
    rtps::TopicData parsed;
    CHECK(parsed.readFromBuffer(std::span<const uint8_t>(sedp.data(), sedp.size())),
          "SEDP round-trip parse");
    CHECK(parsed.unicastLocator.port == reader_port, "parsed locator carries the dedicated port");

    // 2. Default endpoint: shared port, no dedicated flag (pre-band behavior).
    rtps::Reader *normal_reader =
        domain.createReader(*part, "normal_topic", "NormalType", /*reliable=*/true);
    CHECK(normal_reader != nullptr, "normal reader created");
    CHECK(!normal_reader->m_attributes.hasDedicatedPort, "normal reader has no dedicated port");
    CHECK(normal_reader->m_attributes.unicastLocator.port == shared_port,
          "normal reader keeps the shared user port");
    const auto normal_sedp = serialize_attributes(normal_reader->m_attributes);
    CHECK(contains(normal_sedp, expected_unicast_locator_param(shared_port)),
          "normal reader announces the shared port");

    // 1b. Banded writer -> its own dedicated port too.
    rtps::Writer *banded_writer =
        domain.createWriter(*part, "prio_out", "PrioType", /*reliable=*/true,
                            /*enforceUnicast=*/false, {.band = espp::QosBand::Critical});
    CHECK(banded_writer != nullptr, "banded writer created");
    CHECK(banded_writer->m_attributes.hasDedicatedPort, "banded writer has dedicated port");
    const auto writer_port = banded_writer->m_attributes.unicastLocator.port;
    CHECK(writer_port >= dedicated_base && writer_port <= dedicated_end,
          "writer port in the dedicated range");
    CHECK(writer_port != reader_port, "writer and reader ports are distinct");

    // 3. Ration exhausted (cap 2, both used): fall back to the shared port.
    const rtps::Reader *over_cap =
        domain.createReader(*part, "over_cap", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(over_cap != nullptr, "over-cap reader still created");
    CHECK(!over_cap->m_attributes.hasDedicatedPort, "over-cap reader fell back to shared port");
    CHECK(over_cap->m_attributes.unicastLocator.port == shared_port,
          "over-cap reader announces the shared port");
    CHECK(over_cap->m_attributes.band == espp::QosBand::High,
          "over-cap reader keeps its band (for deferred dispatch)");

    // 4. Deleting a dedicated-port endpoint returns its port to the ration
    //    AND promptly releases the underlying fd/port (the retired socket is
    //    destroyed by the reactor's removal-completion callback, not held
    //    until stop()): binding a fresh reuse-disabled socket to the released
    //    port must succeed well before the domain stops.
    CHECK(domain.deleteReader(*part, banded_reader), "delete banded reader");
    CHECK(wait_for_port_released(static_cast<uint16_t>(reader_port)),
          "released dedicated port is bindable again before stop()");
    CHECK(wait_for_no_retired_sockets(domain.getTransport()),
          "no retired sockets accumulate after the release");
    const rtps::Reader *after_delete =
        domain.createReader(*part, "after_delete", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(after_delete != nullptr, "post-delete reader created");
    CHECK(after_delete->m_attributes.hasDedicatedPort,
          "released port made room for a new dedicated port");
  }

  // 6. TRUE fd bound: a released-but-not-yet-closed socket (its reactor
  //    dispatch is queued behind a saturated worker pool, deferring the
  //    removal completion) counts against the ration.
  {
    rtps::DomainConfig cfg;
    cfg.max_prioritized_endpoint_ports = 1;
    rtps::Domain domain(kIp, cfg);
    rtps::Participant *part = domain.createParticipant();
    CHECK(part != nullptr, "createParticipant (fd bound)");

    // Block BOTH transport pool workers so a dispatch for the dedicated
    // socket stays queued (in flight from the reactor's perspective) across
    // the deleteReader() below - deferring the removal completion.
    std::atomic<bool> release_workers{false};
    const auto blocker = [&release_workers]() {
      while (!release_workers.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
    CHECK(domain.getTransport().submit(blocker), "block worker 1");
    CHECK(domain.getTransport().submit(blocker), "block worker 2");

    // Create a banded reader, make its dedicated socket readable so the loop
    // queues a dispatch, then delete it: the retired socket must stay parked
    // (count 1) until the workers drain. Collection timing is the only
    // nondeterminism, so retry with a fresh reader if the dispatch was not
    // yet queued when the delete happened.
    bool pending_retirement = false;
    for (int attempt = 0; attempt < 5 && !pending_retirement; ++attempt) {
      rtps::Reader *banded =
          domain.createReader(*part, ("fdbound" + std::to_string(attempt)).c_str(), "PrioType",
                              /*reliable=*/true, {0, 0, 0, 0}, {.band = espp::QosBand::High});
      CHECK(banded != nullptr, "fd-bound reader created");
      CHECK(banded->m_attributes.hasDedicatedPort, "fd-bound reader has dedicated port");
      const auto port = banded->m_attributes.unicastLocator.port;
      espp::UdpSocket sender({.log_level = espp::Logger::Verbosity::NONE});
      const std::vector<uint8_t> junk{0x00, 0x01, 0x02, 0x03};
      CHECK(sender.send(junk, {.ip_address = "127.0.0.1", .port = port}),
            "send datagram to dedicated port");
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      CHECK(domain.deleteReader(*part, banded), "delete fd-bound reader");
      pending_retirement = domain.getTransport().retiredSocketCount() > 0;
    }
    CHECK(pending_retirement, "a retirement stayed pending behind the blocked pool");

    // Cap 1 and one retired-but-open fd: a new banded reader must NOT get a
    // dedicated port - the cap is a bound on real fds, not registry entries.
    const rtps::Reader *while_pending =
        domain.createReader(*part, "while_pending", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(while_pending != nullptr, "reader created while retirement pending");
    CHECK(!while_pending->m_attributes.hasDedicatedPort,
          "retired-but-open fd counts against the ration");

    // Drain the pool: the queued dispatch runs, the removal completes, the
    // retired fd closes - and the ration slot becomes usable again.
    release_workers = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (domain.getTransport().retiredSocketCount() > 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(domain.getTransport().retiredSocketCount() == 0, "retired socket closed after drain");
    const rtps::Reader *after_drain =
        domain.createReader(*part, "after_drain", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(after_drain != nullptr, "reader created after drain");
    CHECK(after_drain->m_attributes.hasDedicatedPort, "ration slot usable after the fd closed");
  }

  // 7. Probe-window advance: when the current window is fully occupied (here
  //    by external reuse-disabled sockets), the allocator must move PAST the
  //    failed window so a later allocation succeeds on fresh ports.
  {
    rtps::DomainConfig cfg;
    rtps::Domain domain(kIp, cfg);
    rtps::Participant *part = domain.createParticipant();
    CHECK(part != nullptr, "createParticipant (probe window)");

    // Occupy the entire first probe window externally.
    std::vector<std::unique_ptr<espp::UdpSocket>> squatters;
    for (uint16_t i = 0; i < 16; ++i) {
      auto sock = std::make_unique<espp::UdpSocket>(
          espp::UdpSocket::Config{.log_level = espp::Logger::Verbosity::NONE});
      espp::UdpSocket::ReceiveConfig rc;
      rc.port = static_cast<uint16_t>(dedicated_base + i);
      CHECK(sock->is_valid() && sock->disable_reuse() && sock->bind(rc), "squatter bind");
      squatters.push_back(std::move(sock));
    }

    // First allocation: whole window occupied -> falls back to shared port...
    const rtps::Reader *blocked =
        domain.createReader(*part, "blocked_window", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(blocked != nullptr, "reader created against occupied window");
    CHECK(!blocked->m_attributes.hasDedicatedPort, "occupied window falls back");

    // ...but the allocator advanced past the window: the next allocation
    // probes fresh ports and succeeds (squatters still bound).
    const rtps::Reader *advanced =
        domain.createReader(*part, "advanced_window", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::High});
    CHECK(advanced != nullptr, "reader created after window advance");
    CHECK(advanced->m_attributes.hasDedicatedPort, "allocator advanced past the failed window");
    CHECK(advanced->m_attributes.unicastLocator.port >= dedicated_base + 16,
          "new port comes from beyond the occupied window");
  }

  // 5. Dedicated ports disabled: banded endpoints stay on the shared port.
  {
    rtps::DomainConfig cfg;
    cfg.enable_dedicated_endpoint_ports = false;
    rtps::Domain domain(kIp, cfg);
    rtps::Participant *part = domain.createParticipant();
    CHECK(part != nullptr, "createParticipant (disabled)");
    const rtps::Reader *reader =
        domain.createReader(*part, "prio_topic", "PrioType", /*reliable=*/true, {0, 0, 0, 0},
                            {.band = espp::QosBand::Critical});
    CHECK(reader != nullptr, "reader created (disabled)");
    CHECK(!reader->m_attributes.hasDedicatedPort, "no dedicated port when disabled");
    CHECK(reader->m_attributes.unicastLocator.port ==
              rtps::getUserUnicastPort(part->m_participantId),
          "shared port when disabled");
  }

  return true;
}

} // namespace

int main() {
  if (!run_checks()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
