#include "esp32-p4-function-ev-board.hpp"

#if CONFIG_ESP_P4_EV_BOARD_ETHERNET

namespace espp {

bool Esp32P4FunctionEvBoard::initialize_ethernet(const ethernet_link_callback_t &on_link_up) {
  if (ethernet_ && ethernet_->is_initialized()) {
    logger_.warn("Ethernet already initialized");
    return true;
  }

  // Warn the user if they are initializing ethernet after the BOOT button, since
  // the BOOT button is wired to the PHY's RMII_TXD1 pin (GPIO35).
  if (button_callback_) {
    logger_.warn(
        "Initializing Ethernet while BOOT button is initialized. The BOOT button is connected to "
        "the PHY's RMII_TXD1 pin, so the boot button will not work while Ethernet is enabled!");
  }

  logger_.info("Initializing Ethernet (EMAC + IP101 RMII PHY)");

  // Board-specific RMII configuration for the ESP32-P4 Function EV Board. Unlike
  // the ESP32, the P4's EMAC data-plane pins are routable, so they are supplied
  // explicitly (MDC=31, MDIO=52, REF_CLK in=50, TX_EN=49, TXD0=34, TXD1=35,
  // CRS_DV=28, RXD0=29, RXD1=30; IP101 PHY at addr 1, reset on GPIO51).
  espp::Ethernet::Config eth_config{};
  eth_config.interface = espp::Ethernet::RmiiConfig{
      .mdc_gpio = 31,
      .mdio_gpio = 52,
      .phy_addr = 1,
      .phy_reset_gpio = 51,
      .clock_ext_in = true,
      .clock_gpio = 50,
      .data_pins =
          espp::Ethernet::RmiiConfig::DataPins{
              .tx_en = 49, .txd0 = 34, .txd1 = 35, .crs_dv = 28, .rxd0 = 29, .rxd1 = 30},
  };
  // The board's single link callback fires when an IP is acquired.
  eth_config.on_got_ip = on_link_up;

  ethernet_ = std::make_unique<espp::Ethernet>(eth_config);
  return ethernet_->initialize();
}

} // namespace espp

#endif // CONFIG_ESP_P4_EV_BOARD_ETHERNET
