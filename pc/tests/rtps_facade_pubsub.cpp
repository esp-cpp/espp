// In-process facade loopback: two espp::RtpsParticipant instances (two Domains)
// in one process discover each other and exchange samples.
//
// This exercises the Phase 2a unicast-port probing fix (REFACTOR_PLAN.md): the
// second participant's Domain finds the first's ports taken (bind with reuse
// disabled fails loudly) and probes forward to the next participant id.
// Before that fix, both Domains silently shared the same ports and could never
// discover each other.
//
// Exits 0 when at least kRequired samples arrive within the deadline.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;

int main() {
  constexpr int kRequired = 5;
  constexpr auto kDeadline = 20s;
  const char *topic = "facade_loopback";
  const char *type = "std_msgs::msg::dds_::String_";

  std::atomic<int> received{0};
  using Reliability = espp::RtpsParticipant::Reliability;

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::INFO});
  espp::RtpsParticipant sub({.log_level = espp::Logger::Verbosity::INFO});

  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (!pub.add_writer({.topic = topic, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = [&received](std::span<const uint8_t> payload) {
                         espp::CdrReader reader(payload);
                         std::string text;
                         if (reader.read_string(text)) {
                           received.fetch_add(1);
                         }
                       }})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  espp::CdrWriter writer;
  int sent = 0;
  const auto start = std::chrono::steady_clock::now();
  while (received.load() < kRequired && std::chrono::steady_clock::now() - start < kDeadline) {
    writer.reset();
    writer.write_string("facade loopback " + std::to_string(sent));
    if (pub.publish(topic, writer.buffer())) {
      sent++;
    }
    std::this_thread::sleep_for(100ms);
  }

  const int n = received.load();
  std::printf("sent=%d received=%d\n", sent, n);
  pub.stop();
  sub.stop();
  if (n >= kRequired) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
