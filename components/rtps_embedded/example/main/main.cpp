#include <chrono>
#include <thread>

#include "esp32-ethernet-kit.hpp"

#include "cdr.hpp"
#include "logger.hpp"
#include "rtps_participant.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

// std_msgs/msg/String: the reflection-driven cdr component serializes any
// reflectable struct straight to the DDS wire format (cdr::serialize<cdr::xcdr1>
// emits the 4-byte encapsulation header + classic-CDR body ROS 2 speaks).
struct StringMsg {
  std::string data;
};

// The cdr component works in std::byte; the facade publish/on_sample API uses
// uint8_t spans - bridge the two views (same bytes, different value type).
inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "rtps_example", .level = espp::Logger::Verbosity::INFO});

  //! [rtps example]
  // Bring up Ethernet (DHCP server on 192.168.4.1/24 so a directly-attached PC
  // gets an address); any espp network interface works - the RTPS participant
  // only needs the interface's IPv4 address.
  auto &board = espp::Esp32EthernetKit::get();
  bool eth_ok = board.initialize_ethernet({
      .mode = espp::Esp32EthernetKit::DhcpMode::SERVER,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
  });
  if (!eth_ok) {
    logger.error("Ethernet initialization failed");
    return;
  }
  logger.info("Waiting for Ethernet link...");
  while (!board.is_ethernet_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  auto eth_ip = board.ethernet_ip();
  const std::string interface_address =
      fmt::format("{}.{}.{}.{}", esp_ip4_addr1_16(&eth_ip), esp_ip4_addr2_16(&eth_ip),
                  esp_ip4_addr3_16(&eth_ip), esp_ip4_addr4_16(&eth_ip));
  logger.info("Ethernet up, IP {}", interface_address);

  // RTPS/DDS participant (embeddedRTPS engine behind the espp facade). The
  // topics pair with the FastDDS host peer in example/pc/host_pubsub.cpp; for
  // ROS 2 instead, use topic "rt/<name>" with type "<pkg>::msg::dds_::<Type>_"
  // (e.g. "rt/chatter" + "std_msgs::msg::dds_::String_").
  constexpr const char *pub_topic = "mcu_to_pc";
  constexpr const char *sub_topic = "pc_to_mcu";
  constexpr const char *type_name = "std_msgs::msg::String";

  static espp::RtpsParticipant participant({
      .interface_address = interface_address,
      .on_publisher_matched = [&]() { logger.info("publisher matched a remote reader"); },
      .on_subscriber_matched = [&]() { logger.info("subscriber matched a remote writer"); },
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!participant.start()) {
    logger.error("Failed to start the RTPS participant");
    return;
  }

  // Reliable writer: publishes are HEARTBEAT/ACKNACK-acknowledged and
  // retransmitted to matched readers.
  if (!participant.add_writer({
          .topic = pub_topic,
          .type_name = type_name,
          .reliability = espp::RtpsParticipant::Reliability::RELIABLE,
      })) {
    logger.error("Failed to add the writer");
    return;
  }

  // Best-effort reader: samples arrive as CDR-encapsulated bytes; decode with
  // the reflection-driven cdr::deserialize.
  if (!participant.add_reader({
          .topic = sub_topic,
          .type_name = type_name,
          .on_sample =
              [&](std::span<const uint8_t> cdr_payload) {
                if (auto msg = cdr::deserialize<StringMsg>(std::as_bytes(cdr_payload)); msg) {
                  logger.info("rx: {}", msg->data);
                }
              },
      })) {
    logger.error("Failed to add the reader");
    return;
  }

  // Publish a counter periodically; serialization via the cdr component.
  static uint32_t counter = 0;
  espp::Timer publish_timer({
      .name = "rtps_pub",
      .period = std::chrono::milliseconds(CONFIG_RTPS_EXAMPLE_ANNOUNCE_PERIOD_MS),
      .callback =
          [&]() {
            auto bytes = cdr::serialize<cdr::xcdr1>(StringMsg{fmt::format("msg {}", counter++)});
            if (bytes && participant.publish(pub_topic, u8_span(*bytes))) {
              logger.info("tx: msg {}", counter - 1);
            } else {
              logger.warn("tx dropped (history full)");
            }
            return false; // keep the timer running
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  logger.info("started: pub='{}' sub='{}' type='{}'", pub_topic, sub_topic, type_name);
  //! [rtps example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
