#include <chrono>
#include <thread>

#include <esp_netif.h>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP32-P4-ETH Example", .level = espp::Logger::Verbosity::INFO});

  //! [esp32 p4 eth example]
  auto &board = espp::Esp32P4Eth::get();
  bool ok = board.initialize_ethernet({
      .on_link_up = [&]() { logger.info("Link up"); },
      .on_link_down = [&]() { logger.warn("Link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            char buf[16] = {0};
            esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
            logger.info("Got IP: {}", buf);
          },
  });
  if (!ok) {
    logger.error("Failed to initialize Ethernet");
  }
  //! [esp32 p4 eth example]

  while (true) {
    logger.info("connected={} ip={}", board.is_ethernet_connected(), board.ethernet_ip().addr != 0);
    std::this_thread::sleep_for(5s);
  }
}
