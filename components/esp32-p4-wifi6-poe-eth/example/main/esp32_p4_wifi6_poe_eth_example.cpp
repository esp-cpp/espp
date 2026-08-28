#include <chrono>
#include <thread>

#include <sdkconfig.h>

#include "esp32-p4-wifi6-poe-eth.hpp"
#include "logger.hpp"

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
#include <lwip/ip4_addr.h>
#endif

using namespace std::chrono_literals;
using DhcpMode = espp::Esp32P4Wifi6PoeEth::DhcpMode;
using EthConfig = espp::Esp32P4Wifi6PoeEth::EthernetConfig;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "P4Wifi6PoeEth", .level = espp::Logger::Verbosity::INFO});
  logger.info("ESP32-P4-WIFI6-POE-ETH example starting");

  // Example 1: get the singleton board instance
  //! [esp32 p4 wifi6 poe eth get instance]
  auto &board = espp::Esp32P4Wifi6PoeEth::get();
  //! [esp32 p4 wifi6 poe eth get instance]

  // Boot button: log presses / releases
  board.initialize_boot_button([&](const espp::Interrupt::Event &event) {
    logger.info("Boot button {}", event.active ? "pressed" : "released");
  });

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
  // Example 2: init as DHCP server -- the board assigns IPs to connected hosts.
  // ServerConfig::ip_info zero -> default 192.168.4.1/24. Supply Kconfig
  // values (EXAMPLE_ETH_SERVER_IP / _NETMASK / _GW) for a custom static IP.
  //! [esp32 p4 wifi6 poe eth dhcp server]
  espp::Esp32P4Wifi6PoeEth::ServerConfig srv_cfg;
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_IP, reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.ip));
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_NETMASK,
               reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.netmask));
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_GW, reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.gw));
  srv_cfg.on_client_assigned = [&](esp_ip4_addr_t ip, std::array<uint8_t, 6> mac) {
    logger.info("Client assigned {}.{}.{}.{} (mac {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})",
                esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip),
                esp_ip4_addr4_16(&ip), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  };
  bool eth_ok = board.initialize_ethernet({
      .mode = DhcpMode::SERVER,
      .server_config = srv_cfg,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            logger.info("DHCP server up at {}.{}.{}.{}", esp_ip4_addr1_16(&ip),
                        esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
          },
      .on_lost_ip = [&]() { logger.warn("Ethernet lost IP"); },
  });
  //! [esp32 p4 wifi6 poe eth dhcp server]
#else
  // Example 3: init as DHCP client -- the board acquires an IP from the network.
  //! [esp32 p4 wifi6 poe eth dhcp client]
  bool eth_ok = board.initialize_ethernet({
      .mode = DhcpMode::CLIENT,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            logger.info("DHCP lease acquired: {}.{}.{}.{}", esp_ip4_addr1_16(&ip),
                        esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
          },
      .on_lost_ip = [&]() { logger.warn("Ethernet lost IP"); },
  });
  //! [esp32 p4 wifi6 poe eth dhcp client]
#endif

  if (!eth_ok) {
    logger.error("Ethernet initialization failed");
    return;
  }

  // SERVER mode: connected as soon as the cable is plugged in (static IP).
  // CLIENT mode: connected once the DHCP lease is granted (up to ~30 s).
  logger.info("Waiting for Ethernet...");
  for (int i = 0; i < 60 && !board.is_ethernet_connected(); ++i) {
    std::this_thread::sleep_for(500ms);
  }

  if (board.is_ethernet_connected()) {
    auto ip = board.ethernet_ip();
    logger.info("Connected. IP: {}.{}.{}.{}", esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
  } else {
    logger.warn("Ethernet not ready within 30 s");
  }

  while (true) {
    std::this_thread::sleep_for(5s);
    if (board.is_ethernet_connected()) {
      auto ip = board.ethernet_ip();
      logger.info("Ethernet up, IP: {}.{}.{}.{} (boot button {})", esp_ip4_addr1_16(&ip),
                  esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip),
                  board.boot_button_state() ? "pressed" : "released");
    } else {
      logger.warn("Ethernet not connected");
    }
  }
}
