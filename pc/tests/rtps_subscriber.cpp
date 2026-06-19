// Standalone RTPS subscriber: announces a reader and prints received std_msgs/msg/UInt32 samples.
// Pair it with rtps_publisher (C++), python/rtps_publisher.py, or python/rtps_host.py.
//
// Usage: rtps_subscriber [topic] [advertised_ipv4]

#include <atomic>
#include <chrono>
#include <thread>

#include "espp.hpp"
#include "rtps_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  espp::Logger logger({.tag = "rtps_subscriber", .level = espp::Logger::Verbosity::INFO});

  const std::string topic = argc > 1 ? argv[1] : "espp/test/counter";
  const std::string address = argc > 2 ? argv[2] : rtps_test::guess_local_ipv4();

  std::atomic<uint32_t> count{0};

  espp::RtpsParticipant participant({
      .node_name = "espp_subscriber",
      .participant_id = 12,
      .advertised_address = address,
      .announce_period = 500ms,
      .on_participant_discovered =
          [&logger](const auto &proxy) {
            logger.info("discovered participant '{}' at {}", proxy.name, proxy.address);
          },
  });
  participant.add_reader({
      .topic_name = topic,
      .on_sample =
          [&](std::span<const uint8_t> cdr) {
            if (auto value = rtps_test::deserialize_uint32(cdr)) {
              logger.info("received {} (#{})", *value, ++count);
            }
          },
  });

  if (!participant.start()) {
    logger.error("Failed to start participant (is multicast networking available?)");
    return 1;
  }
  logger.info("subscribed to '{}' on {} (Ctrl-C to stop)", topic, address);

  while (true) {
    std::this_thread::sleep_for(5s);
    logger.info("status: {} samples received, {} known publisher(s)", count.load(),
                participant.discovered_writers().size());
  }
  return 0;
}
