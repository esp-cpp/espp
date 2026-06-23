// Standalone RTPS publisher: announces a writer and periodically publishes std_msgs/msg/UInt32
// samples. Pair it with rtps_subscriber (C++), python/rtps_subscriber.py, or python/rtps_host.py.
//
// Usage: rtps_publisher [topic] [advertised_ipv4] [period_ms]

#include <chrono>
#include <thread>

#include "espp.hpp"
#include "rtps_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  espp::Logger logger({.tag = "rtps_publisher", .level = espp::Logger::Verbosity::INFO});

  const std::string topic = argc > 1 ? argv[1] : "espp/test/counter";
  const std::string address = argc > 2 ? argv[2] : rtps_test::guess_local_ipv4();
  const int period_ms = argc > 3 ? std::atoi(argv[3]) : 1000;

  espp::RtpsParticipant participant({
      .node_name = "espp_publisher",
      .participant_id = 10,
      .advertised_address = address,
      .announce_period = 500ms,
      .on_endpoint_discovered =
          [&logger](const auto &endpoint) {
            logger.info("discovered {} '{}'", endpoint.is_reader ? "reader" : "writer",
                        endpoint.topic_name);
          },
  });
  participant.add_writer({.topic_name = topic});

  if (!participant.start()) {
    logger.error("Failed to start participant (is multicast networking available?)");
    return 1;
  }
  logger.info("publishing on '{}' from {} every {}ms (Ctrl-C to stop)", topic, address, period_ms);

  uint32_t value = 0;
  while (true) {
    ++value;
    bool sent = participant.publish(topic, rtps_test::serialize_uint32(value));
    logger.info("publish {} -> {}", value, sent ? "sent" : "no destinations yet");
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
  return 0;
}
