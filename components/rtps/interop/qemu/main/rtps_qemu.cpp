// RTPS interop firmware for QEMU (components/rtps/interop/qemu).
//
// The device side of the QEMU interop harness (../run_qemu_interop.sh): the
// embedded RTPS engine - the esp32 configuration profile, lwIP, FreeRTOS - on
// QEMU's OpenCores Ethernet, talking to FastDDS, CycloneDDS / ROS 2 and the
// espp host engine on a tap device. The harness drives everything from the
// host side and asserts on the "RTPSTEST ..." lines this firmware logs:
//
//   RTPSTEST ready ip=<ip>           participant started
//   RTPSTEST pub_matched/sub_matched a writer / reader gained a remote peer
//   RTPSTEST tx <n>                  reliable publisher sent "qemu <n>"
//   RTPSTEST rx <data>               reliable subscriber received <data>
//   RTPSTEST be_rx <data>            best-effort subscriber received <data>
//   RTPSTEST big_rx len=<n>          fragmented-sample subscriber received n bytes
//   RTPSTEST srv <a>+<b>=<sum>       service request served
//   RTPSTEST status up=<s> tx= rx= be_rx= big_rx= heap=<free> min_heap=<low water>
//                                    every 2 s, for the soak checks
//
// Topics (ROS 2 names: "rt/" prefix stripped, std_msgs/msg/String):
//   rt/qemu_tx     reliable, device publishes "qemu <n>" every 200 ms
//   rt/qemu_rx     reliable, device subscribes
//   rt/qemu_be_tx  best-effort, device publishes "qemu_be <n>" every 200 ms
//   rt/qemu_be_rx  best-effort, device subscribes
//   rt/qemu_big    reliable, device subscribes (fragmented samples)
//   /qemu_add_two_ints  example_interfaces/srv/AddTwoInts service the device serves

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <esp_heap_caps.h>
#include <esp_netif.h>

#include "qemu.hpp"

#include "logger.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "rtps_service.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

// std_msgs/msg/String as a reflectable struct (ROS 2 CDR)
struct StringMsg {
  std::string data;
};

// example_interfaces/srv/AddTwoInts
struct AddReq {
  int64_t a;
  int64_t b;
};
struct AddResp {
  int64_t sum;
};

static std::string ip4_to_string(esp_ip4_addr_t ip) {
  char buf[16] = {};
  esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
  return buf;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "rtps_qemu", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting the RTPS QEMU interop firmware");

  auto &qemu = espp::Qemu::get();

  // Network: a static address (CI, on the harness's tap device) or a DHCP
  // client (a developer's idf.py qemu on the user-mode NAT), per Kconfig.
  espp::Qemu::EthernetConfig eth_config{};
  eth_config.mode = espp::Qemu::DhcpMode::CLIENT;
#if CONFIG_RTPS_QEMU_STATIC_IP
  eth_config.ip_info.ip.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_IP);
  eth_config.ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_NETMASK);
  eth_config.ip_info.gw.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_GATEWAY);
#endif
  std::atomic<bool> have_ip{false};
  eth_config.on_got_ip = [&](esp_ip4_addr_t ip) {
    logger.info("Ethernet up: {}", ip4_to_string(ip));
    have_ip = true;
  };
  eth_config.on_link_down = [&]() { logger.warn("Ethernet link down"); };
  if (!qemu.initialize_ethernet(eth_config)) {
    logger.error("RTPSTEST fatal ethernet");
    return;
  }
  // wait for the address (DHCP takes a second on the NAT; static is immediate)
  for (int i = 0; i < 300 && !have_ip; ++i) {
    std::this_thread::sleep_for(100ms);
  }
  if (!have_ip) {
    logger.error("RTPSTEST fatal no_ip");
    return;
  }
  const std::string interface_address = ip4_to_string(qemu.ethernet_ip());
  esp_netif_ip_info_t ip_info{};
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("ETH_DEF"), &ip_info);
  const std::string interface_netmask = ip4_to_string(ip_info.netmask);

  // Counters for the status line (the callbacks run on the engine's threads)
  std::atomic<uint32_t> tx_count{0};
  std::atomic<uint32_t> rx_count{0};
  std::atomic<uint32_t> be_rx_count{0};
  std::atomic<uint32_t> big_rx_count{0};
  std::atomic<uint32_t> pub_matches{0};
  std::atomic<uint32_t> sub_matches{0};

  espp::RtpsParticipant participant({
      .interface_address = interface_address,
      .interface_netmask = interface_netmask,
      .on_publisher_matched =
          [&]() {
            pub_matches++;
            logger.info("RTPSTEST pub_matched");
          },
      .on_subscriber_matched =
          [&]() {
            sub_matches++;
            logger.info("RTPSTEST sub_matched");
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!participant.start()) {
    logger.error("RTPSTEST fatal participant");
    return;
  }

  using Reliability = espp::RtpsParticipant::Reliability;
  constexpr const char *type_name = "std_msgs::msg::dds_::String_";

  espp::Publisher<StringMsg> publisher(participant, {
                                                        .topic = "rt/qemu_tx",
                                                        .type_name = type_name,
                                                        .reliability = Reliability::RELIABLE,
                                                    });
  espp::Subscriber<StringMsg> subscriber(participant, {
                                                          .topic = "rt/qemu_rx",
                                                          .type_name = type_name,
                                                          .reliability = Reliability::RELIABLE,
                                                          .on_message =
                                                              [&](const StringMsg &msg) {
                                                                rx_count++;
                                                                logger.info("RTPSTEST rx {}",
                                                                            msg.data);
                                                              },
                                                      });
  espp::Publisher<StringMsg> be_publisher(participant, {
                                                           .topic = "rt/qemu_be_tx",
                                                           .type_name = type_name,
                                                           .reliability = Reliability::BEST_EFFORT,
                                                       });
  espp::Subscriber<StringMsg> be_subscriber(participant,
                                            {
                                                .topic = "rt/qemu_be_rx",
                                                .type_name = type_name,
                                                .reliability = Reliability::BEST_EFFORT,
                                                .on_message =
                                                    [&](const StringMsg &msg) {
                                                      be_rx_count++;
                                                      logger.info("RTPSTEST be_rx {}", msg.data);
                                                    },
                                            });
  // Samples larger than one UDP datagram arrive as DATA_FRAG and are
  // reassembled (CONFIG_RTPS_ENABLE_FRAGMENTATION); only the length is logged.
  espp::Subscriber<StringMsg> big_subscriber(
      participant, {
                       .topic = "rt/qemu_big",
                       .type_name = type_name,
                       .reliability = Reliability::RELIABLE,
                       .on_message =
                           [&](const StringMsg &msg) {
                             big_rx_count++;
                             logger.info("RTPSTEST big_rx len={}", msg.data.size());
                           },
                   });
  if (!publisher.is_valid() || !subscriber.is_valid() || !be_publisher.is_valid() ||
      !be_subscriber.is_valid() || !big_subscriber.is_valid()) {
    logger.error("RTPSTEST fatal endpoints");
    return;
  }

#ifdef RTPS_WITH_RPC
  espp::ServiceServer<AddReq, AddResp> add_service(
      participant, {
                       .service = "/qemu_add_two_ints",
                       .type_name = "example_interfaces::srv::dds_::AddTwoInts",
                       .handler =
                           [&](const AddReq &r) {
                             logger.info("RTPSTEST srv {}+{}={}", r.a, r.b, r.a + r.b);
                             return AddResp{r.a + r.b};
                           },
                   });
  if (!add_service.is_valid()) {
    logger.error("RTPSTEST fatal service");
    return;
  }
#endif

  // Periodic publishers: one reliable, one best-effort
  uint32_t counter = 0;
  espp::Timer publish_timer({
      .name = "rtps_pub",
      .period = 200ms,
      .callback =
          [&]() {
            const uint32_t n = counter++;
            if (publisher.publish(StringMsg{"qemu " + std::to_string(n)})) {
              tx_count++;
              logger.info("RTPSTEST tx {}", n);
            }
            be_publisher.publish(StringMsg{"qemu_be " + std::to_string(n)});
            return false;
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  logger.info("RTPSTEST ready ip={}", interface_address);

  // Status line every 2 s: the soak and leak checks read heap and min_heap
  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(2s);
    const auto up =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0)
            .count();
    logger.info("RTPSTEST status up={} tx={} rx={} be_rx={} big_rx={} pub_matches={} "
                "sub_matches={} heap={} min_heap={}",
                up, tx_count.load(), rx_count.load(), be_rx_count.load(), big_rx_count.load(),
                pub_matches.load(), sub_matches.load(), heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
  }
}
