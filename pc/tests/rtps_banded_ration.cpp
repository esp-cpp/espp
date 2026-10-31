// Ration exhaustion end-to-end: the subscriber caps dedicated endpoint ports at
// 1 (max_prioritized_endpoint_ports=1) but registers TWO banded (High) readers.
// The first gets the dedicated port; the second exceeds the ration, logs a
// warning, and falls back to the shared port with deferred banded dispatch.
// Both must still receive every published sample.
//
// Exits 0 when both readers receive at least kRequired samples in time.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

struct StringMsg {
  std::string data;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

using namespace std::chrono_literals;

int main() {
  constexpr int kRequired = 5;
  constexpr auto kDeadline = 30s;
  const char *topic_a = "ration_topic_a";
  const char *topic_b = "ration_topic_b";
  const char *type = "std_msgs::msg::dds_::String_";
  using Reliability = espp::RtpsParticipant::Reliability;

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::INFO});
  espp::RtpsParticipant sub(
      {.log_level = espp::Logger::Verbosity::INFO, .max_prioritized_endpoint_ports = 1});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (!pub.add_writer(
          {.topic = topic_a, .type_name = type, .reliability = Reliability::RELIABLE}) ||
      !pub.add_writer(
          {.topic = topic_b, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }

  std::atomic<int> received_a{0};
  std::atomic<int> received_b{0};
  const auto count_into = [](std::atomic<int> &counter) {
    return [&counter](std::span<const uint8_t> payload) {
      if (cdr::deserialize<StringMsg>(std::as_bytes(payload))) {
        counter.fetch_add(1);
      }
    };
  };
  // Reader A takes the single dedicated port; reader B exhausts the ration and
  // must fall back (warning logged) to shared-port deferred dispatch.
  if (!sub.add_reader({.topic = topic_a,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = count_into(received_a),
                       .band = espp::QosBand::High})) {
    std::printf("FAIL: add_reader a\n");
    return 1;
  }
  if (!sub.add_reader({.topic = topic_b,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = count_into(received_b),
                       .band = espp::QosBand::High})) {
    std::printf("FAIL: add_reader b\n");
    return 1;
  }

  int sent = 0;
  const auto start = std::chrono::steady_clock::now();
  while ((received_a.load() < kRequired || received_b.load() < kRequired) &&
         std::chrono::steady_clock::now() - start < kDeadline) {
    auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{"ration sample " + std::to_string(sent)});
    if (bytes) {
      const bool a = pub.publish(topic_a, u8_span(*bytes));
      const bool b = pub.publish(topic_b, u8_span(*bytes));
      if (a && b) {
        sent++;
      }
    }
    std::this_thread::sleep_for(100ms);
  }

  const int a = received_a.load();
  const int b = received_b.load();
  std::printf("sent=%d received_a=%d received_b=%d\n", sent, a, b);
  pub.stop();
  sub.stop();
  if (a >= kRequired && b >= kRequired) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
