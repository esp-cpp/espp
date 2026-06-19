// Self-contained RTPS test: two participants (a publisher and a subscriber) in one process exchange
// std_msgs/msg/UInt32 samples over best-effort CDR-over-RTPS, exercising SPDP/SEDP discovery and
// the user-data path end to end. Exits 0 if the subscriber received samples, 1 otherwise.
//
// Usage: rtps_pubsub [advertised_ipv4] [run_seconds]

#include <atomic>
#include <chrono>
#include <thread>

#include "espp.hpp"
#include "rtps_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  espp::Logger logger({.tag = "rtps_pubsub", .level = espp::Logger::Verbosity::INFO});

  const std::string address = argc > 1 ? argv[1] : rtps_test::guess_local_ipv4();
  const int run_seconds = argc > 2 ? std::atoi(argv[2]) : 8;
  const std::string topic = "espp/test/counter";
  logger.info("advertising on {} for {}s, topic '{}'", address, run_seconds, topic);

  std::atomic<uint32_t> received_count{0};
  std::atomic<uint32_t> last_received{0};

  // --- Subscriber participant ---
  espp::RtpsParticipant subscriber({
      .node_name = "espp_pubsub_subscriber",
      .participant_id = 11,
      .advertised_address = address,
      .announce_period = 200ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  subscriber.add_reader({
      .topic_name = topic,
      .on_sample =
          [&](std::span<const uint8_t> cdr) {
            if (auto value = rtps_test::deserialize_uint32(cdr)) {
              received_count++;
              last_received = *value;
            }
          },
  });

  // --- Publisher participant ---
  espp::RtpsParticipant publisher({
      .node_name = "espp_pubsub_publisher",
      .participant_id = 10,
      .advertised_address = address,
      .announce_period = 200ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  publisher.add_writer({.topic_name = topic});

  if (!subscriber.start() || !publisher.start()) {
    logger.error("Failed to start participants (is multicast networking available?)");
    return 1;
  }

  // Give SPDP/SEDP discovery a moment to match the writer and reader.
  logger.info("waiting for discovery...");
  for (int i = 0; i < 50; i++) {
    if (!publisher.discovered_readers().empty() && !subscriber.discovered_writers().empty()) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  logger.info("discovered {} remote reader(s), {} remote writer(s)",
              publisher.discovered_readers().size(), subscriber.discovered_writers().size());

  uint32_t sent_count = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(run_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    uint32_t value = ++sent_count;
    if (publisher.publish(topic, rtps_test::serialize_uint32(value))) {
      logger.info("published {} -> received so far {} (last={})", value, received_count.load(),
                  last_received.load());
    }
    std::this_thread::sleep_for(500ms);
  }

  publisher.stop();
  subscriber.stop();

  logger.info("done: sent {}, received {}, last value {}", sent_count, received_count.load(),
              last_received.load());
  if (received_count == 0) {
    logger.error("subscriber received no samples");
    return 1;
  }
  return 0;
}
