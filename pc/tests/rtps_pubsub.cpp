// Host loopback pub/sub test for the embeddedRTPS engine (components/rtps).
//
// Phase 0a of components/rtps/REFACTOR_PLAN.md: establish a host-buildable,
// runnable baseline of the engine BEFORE any refactoring, so every later phase can be
// checked against it. Two participants in one Domain discover each other via SPDP/SEDP
// and exchange CDR string samples (best-effort both ends; see the pool-sizing
// note at the createWriter call).
//
// Exits 0 when at least kRequiredSamples samples arrive within the deadline; 1 otherwise.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "rtps/entities/Domain.hpp"
#include "rtps/utils/CdrBuffer.hpp"

#include "rtps_common.hpp"

using namespace std::chrono_literals;

namespace {
constexpr int kRequiredSamples = 5;
constexpr auto kDeadline = 20s;
constexpr const char *kTopic = "espp_loopback";
constexpr const char *kType = "std_msgs::msg::String";

std::atomic<int> g_received{0};

void reader_cb(void * /*callee*/, const rtps::ReaderCacheChange &change) {
  // 4-byte CDR encapsulation header + at least a string length prefix
  if (change.getDataSize() < 8) {
    return;
  }
  g_received.fetch_add(1);
}

bool publish_string(rtps::Writer *writer, const char *text) {
  uint8_t cdr_buf[4 + 4 + 128];
  cdr_buf[0] = 0x00; // CDR_LE encapsulation
  cdr_buf[1] = 0x01;
  cdr_buf[2] = 0x00;
  cdr_buf[3] = 0x00;
  // CDR string body: uint32 length (incl. null terminator) + chars + null.
  rtps::CdrSink sink{rtps::asWritableBytes(cdr_buf + 4, sizeof(cdr_buf) - 4)};
  rtps::CdrWriter writer_cdr(sink);
  const auto len = static_cast<uint32_t>(std::strlen(text) + 1);
  writer_cdr.write<uint32_t>(len);
  rtps::writeBytes(writer_cdr, reinterpret_cast<const uint8_t *>(text), len);
  if (!writer_cdr.ok()) {
    return false;
  }
  const auto total = static_cast<rtps::DataSize_t>(4 + sink.size());
  return writer->newChange(rtps::ChangeKind_t::ALIVE, cdr_buf, total) != nullptr;
}
} // namespace

int main() {
  const std::string ip_str = rtps_test::guess_local_ipv4();
  rtps::Ip4AddressBytes ip{};
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
    std::printf("FAIL: could not parse local ip '%s'\n", ip_str.c_str());
    return 1;
  }
  ip = {static_cast<uint8_t>(a), static_cast<uint8_t>(b), static_cast<uint8_t>(c),
        static_cast<uint8_t>(d)};
  std::printf("local ip: %s\n", ip_str.c_str());

  static rtps::Domain domain(ip);

  // Participants must exist before completeInit() starts discovery.
  rtps::Participant *pub_part = domain.createParticipant();
  rtps::Participant *sub_part = domain.createParticipant();
  if (pub_part == nullptr || sub_part == nullptr) {
    std::printf("FAIL: could not create participants\n");
    return 1;
  }

  if (!domain.completeInit()) {
    std::printf("FAIL: completeInit\n");
    return 1;
  }

  // Best-effort on both ends: with MAX_NUM_PARTICIPANTS=2 the SEDP builtins consume
  // the entire desktop stateful pools (2 participants x 2 SEDP writers = 4 = cap), so
  // the loopback baseline uses the spare stateless slots. Reliable QoS is exercised
  // against FastDDS in the Phase 0c interop harness (single local participant).
  rtps::Writer *writer = domain.createWriter(*pub_part, kTopic, kType, /*reliable=*/false);
  rtps::Reader *reader = domain.createReader(*sub_part, kTopic, kType, /*reliable=*/false);
  if (writer == nullptr || reader == nullptr) {
    std::printf("FAIL: could not create writer/reader\n");
    domain.stop();
    return 1;
  }
  if (reader->registerCallback(reader_cb, nullptr) == 0) {
    std::printf("FAIL: registerCallback\n");
    domain.stop();
    return 1;
  }

  const auto start = std::chrono::steady_clock::now();
  int sent = 0;
  while (g_received.load() < kRequiredSamples &&
         std::chrono::steady_clock::now() - start < kDeadline) {
    char text[64];
    std::snprintf(text, sizeof(text), "hello espp rtps %d", sent);
    if (publish_string(writer, text)) {
      sent++;
    }
    std::this_thread::sleep_for(100ms);
  }

  const int received = g_received.load();
  std::printf("sent=%d received=%d\n", sent, received);
  domain.stop();

  if (received >= kRequiredSamples) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: received %d < %d\n", received, kRequiredSamples);
  return 1;
}
