#include <chrono>
#include <thread>

#include <sdkconfig.h>

#include "esp32-ethernet-kit.hpp"
#include "logger.hpp"

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
#include <lwip/ip4_addr.h>
#endif

using namespace std::chrono_literals;
using DhcpMode = espp::Esp32EthernetKit::DhcpMode;

extern "C" void app_main(void) {
  //! [esp32 ethernet kit example]
  espp::Logger logger({.tag = "EthKitExample", .level = espp::Logger::Verbosity::INFO});
  logger.info("ESP32-Ethernet-Kit A V1.2 example starting");

  auto &board = espp::Esp32EthernetKit::get();

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
  // DHCP server mode: ESP32 assigns IPs to connected hosts using a static IP.
  espp::Esp32EthernetKit::ServerConfig srv_cfg;
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_IP,
               reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.ip));
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_NETMASK,
               reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.netmask));
  ip4addr_aton(CONFIG_EXAMPLE_ETH_SERVER_GW,
               reinterpret_cast<ip4_addr_t *>(&srv_cfg.ip_info.gw));

  bool eth_ok = board.initialize_ethernet(
      [&](esp_ip4_addr_t ip) {
        logger.info("DHCP server up at {}.{}.{}.{}",
                    esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                    esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
      },
      DhcpMode::SERVER, srv_cfg);
#else
  // DHCP client mode (default): ESP32 acquires an IP from the network.
  bool eth_ok = board.initialize_ethernet(
      [&](esp_ip4_addr_t ip) {
        logger.info("DHCP lease acquired: {}.{}.{}.{}",
                    esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                    esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
      },
      DhcpMode::CLIENT);
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
    logger.info("Connected. IP: {}.{}.{}.{}",
                esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
  } else {
    logger.warn("Ethernet not ready within 30 s");
  }

  while (true) {
    std::this_thread::sleep_for(5s);
    if (board.is_ethernet_connected()) {
      auto ip = board.ethernet_ip();
      logger.info("Ethernet up, IP: {}.{}.{}.{}",
                  esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                  esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
    } else {
      logger.warn("Ethernet not connected");
    }
  }
  //! [esp32 ethernet kit example]
}

