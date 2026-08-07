// Standalone embeddedRTPS publisher test for host/PC. It creates an embeddedRTPS Domain,
// participant, and writer, then periodically publishes a uint32 payload.
//
// Usage: rtps_embedded_publisher [topic] [local_ipv4] [period_ms]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "espp.hpp"
#include "rtps/entities/Domain.hpp"
#include "rtps_common.hpp"

namespace {

bool parse_ipv4(const std::string &ip, rtps::Ip4AddressBytes &out) {
  unsigned int b0 = 0, b1 = 0, b2 = 0, b3 = 0;
  if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &b0, &b1, &b2, &b3) != 4) {
    return false;
  }
  if (b0 > 255 || b1 > 255 || b2 > 255 || b3 > 255) {
    return false;
  }
  out = {static_cast<uint8_t>(b0), static_cast<uint8_t>(b1), static_cast<uint8_t>(b2),
         static_cast<uint8_t>(b3)};
  return true;
}

} // namespace

int main(int argc, char **argv) {
  espp::Logger logger({.tag = "rtps_embedded_pub", .level = espp::Logger::Verbosity::INFO});

  const std::string topic = argc > 1 ? argv[1] : "rtps_emb_pub";
  const std::string local_ip_string = argc > 2 ? argv[2] : rtps_test::guess_local_ipv4();
  const int period_ms = argc > 3 ? std::atoi(argv[3]) : 1000;

  rtps::Ip4AddressBytes local_ip{0, 0, 0, 0};
  if (!parse_ipv4(local_ip_string, local_ip)) {
    logger.error("Invalid IPv4 address '{}'", local_ip_string);
    return 1;
  }

  rtps::Domain domain(local_ip);
  rtps::Participant *participant = domain.createParticipant();
  if (participant == nullptr) {
    logger.error("Failed to create embeddedRTPS participant");
    return 1;
  }

  // Keep topic/type names short because embeddedRTPS desktop config caps lengths.
  rtps::Writer *writer =
      domain.createWriter(*participant, topic.c_str(), "std_msgs::msg::String", false);
  if (writer == nullptr) {
    logger.error("Failed to create embeddedRTPS writer for topic '{}'", topic);
    return 1;
  }

  if (!domain.completeInit()) {
    logger.error("Failed to start embeddedRTPS domain");
    return 1;
  }

  logger.info("publishing on '{}' from {} every {}ms (Ctrl-C to stop)", topic, local_ip_string,
              period_ms);

  uint32_t value = 0;
  while (true) {
    ++value;
    const std::string message = std::to_string(value);
    const rtps::CacheChange *change = writer->newChange(
        rtps::ChangeKind_t::ALIVE, reinterpret_cast<const uint8_t *>(message.c_str()),
        static_cast<rtps::DataSize_t>(message.size() + 1));
    logger.info("publish {} -> {}", value, change != nullptr ? "queued" : "dropped");
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  return 0;
}