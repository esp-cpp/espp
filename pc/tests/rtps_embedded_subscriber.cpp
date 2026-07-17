// Standalone embeddedRTPS subscriber test for host/PC. It creates an embeddedRTPS Domain,
// participant, and reader, then logs received std_msgs::msg::String payloads.
//
// Usage: rtps_embedded_subscriber [topic] [local_ipv4] [run_seconds]

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "espp.hpp"
#include "rtps/entities/Domain.h"
#include "rtps_common.hpp"

namespace {

struct SubscriberState {
  std::atomic<uint32_t> received_count{0};
  std::string last_message;
  std::mutex mutex;
};

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

void on_sample(void *arg, const rtps::ReaderCacheChange &change) {
  if (arg == nullptr) {
    return;
  }
  auto *state = static_cast<SubscriberState *>(arg);
  const auto size = change.getDataSize();
  if (size == 0) {
    return;
  }

  std::string message(size, '\0');
  if (!change.copyInto(reinterpret_cast<uint8_t *>(message.data()),
                       static_cast<rtps::DataSize_t>(message.size()))) {
    return;
  }

  if (!message.empty() && message.back() == '\0') {
    message.pop_back();
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->last_message = std::move(message);
  }
  state->received_count++;
}

} // namespace

int main(int argc, char **argv) {
  espp::Logger logger({.tag = "rtps_embedded_sub", .level = espp::Logger::Verbosity::INFO});

  const std::string topic = argc > 1 ? argv[1] : "rtps_emb_pub";
  const std::string local_ip_string = argc > 2 ? argv[2] : rtps_test::guess_local_ipv4();
  const int run_seconds = argc > 3 ? std::atoi(argv[3]) : 0;

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
  rtps::Reader *reader =
      domain.createReader(*participant, topic.c_str(), "std_msgs::msg::String", false);
  if (reader == nullptr) {
    logger.error("Failed to create embeddedRTPS reader for topic '{}'", topic);
    return 1;
  }

  SubscriberState state;
  if (reader->registerCallback(on_sample, &state) == 0) {
    logger.error("Failed to register reader callback");
    return 1;
  }

  if (!domain.completeInit()) {
    logger.error("Failed to start embeddedRTPS domain");
    return 1;
  }

  logger.info("subscribing on '{}' from {} (run_seconds={} / 0=forever)", topic,
              local_ip_string, run_seconds);

  const auto start_time = std::chrono::steady_clock::now();
  while (true) {
    std::string last_message;
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      last_message = state.last_message;
    }
    logger.info("received {} samples, last='{}'", state.received_count.load(),
                last_message);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (run_seconds > 0) {
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                           start_time)
              .count();
      if (elapsed >= run_seconds) {
        break;
      }
    }
  }

  domain.stop();
  return state.received_count.load() > 0 ? 0 : 1;
}