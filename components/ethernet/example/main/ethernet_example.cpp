#include <chrono>
#include <thread>

#include <sdkconfig.h>

#include <esp_netif.h>

#if CONFIG_ESPP_ETHERNET_W5500
#include <driver/spi_master.h>
#include <esp_err.h>
#endif

#include "ethernet.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

namespace {
espp::Logger logger({.tag = "Ethernet Example", .level = espp::Logger::Verbosity::INFO});

// Log the interface status forever (the callbacks below do the event-driven part).
[[noreturn]] void monitor(espp::Ethernet &eth) {
  while (true) {
    logger.info("link={} connected={} ip={} mac={}", eth.link_up(), eth.is_connected(),
                eth.get_ip_address(), eth.get_mac_address());
    std::this_thread::sleep_for(5s);
  }
}

espp::Ethernet::Config with_callbacks(espp::Ethernet::Config config) {
  config.hostname = "espp-eth";
  config.log_level = espp::Logger::Verbosity::INFO;
  config.on_link_up = []() { logger.info("Link up"); };
  config.on_link_down = []() { logger.warn("Link down"); };
  config.on_got_ip = [](esp_ip4_addr_t ip) {
    char buf[16] = {0};
    esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
    logger.info("Got IP: {}", buf);
  };
  config.on_lost_ip = []() { logger.warn("Lost IP"); };
  return config;
}
} // namespace

extern "C" void app_main(void) {
#if CONFIG_ESPP_ETHERNET_W5500
  //! [ethernet spi example]
  // Bring up the SPI bus the W5500 lives on (adjust pins for your board).
  spi_bus_config_t buscfg = {};
  buscfg.miso_io_num = 13;
  buscfg.mosi_io_num = 11;
  buscfg.sclk_io_num = 12;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 1600;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // One reactor-owned W5500 over that bus, DHCP client.
  espp::Ethernet eth(with_callbacks({
      .interface = espp::Ethernet::SpiConfig{.host = SPI2_HOST,
                                             .cs_gpio = 10,
                                             .int_gpio = 14,
                                             .reset_gpio = 21,
                                             .chip = espp::Ethernet::SpiChip::W5500},
  }));
  eth.initialize();
  //! [ethernet spi example]
  monitor(eth);
#elif SOC_EMAC_SUPPORTED
  //! [ethernet rmii example]
  // RMII on the ESP32-Ethernet-Kit (IP101 PHY on GPIO0 ref-clock). Adjust for your board.
  espp::Ethernet eth(with_callbacks({
      .interface = espp::Ethernet::RmiiConfig{.mdc_gpio = 23,
                                              .mdio_gpio = 18,
                                              .phy_addr = 1,
                                              .phy_reset_gpio = 5,
                                              .clock_ext_in = true,
                                              .clock_gpio = 0},
  }));
  eth.initialize();
  //! [ethernet rmii example]
  monitor(eth);
#else
  logger.error("No ethernet interface available: this SoC has no EMAC and "
               "CONFIG_ESPP_ETHERNET_W5500 is not enabled.");
  while (true) {
    std::this_thread::sleep_for(5s);
  }
#endif
}
