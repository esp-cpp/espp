#include "esp32-ethernet-kit.hpp"

namespace espp {

Esp32EthernetKit::Esp32EthernetKit()
    : BaseComponent("Esp32EthernetKit") {}

bool Esp32EthernetKit::initialize_ethernet() { return initialize_ethernet(EthernetConfig{}); }

bool Esp32EthernetKit::initialize_ethernet(const EthernetConfig &config) {
  if (ethernet_ && ethernet_->is_initialized()) {
    logger_.warn("Ethernet already initialized");
    return true;
  }

  logger_.info("Initializing Ethernet (EMAC + IP101GRI RMII, DHCP {})",
               config.mode == DhcpMode::SERVER ? "server" : "client");

  // The board-specific part: RMII pins + IP101GRI PHY on the fixed ESP32 IO_MUX
  // data-plane pins, with the external 50 MHz ref-clock on GPIO0.
  espp::Ethernet::Config eth_config{};
  eth_config.interface = espp::Ethernet::RmiiConfig{
      .mdc_gpio = eth_mdc_io,
      .mdio_gpio = eth_mdio_io,
      .phy_addr = eth_phy_addr,
      .phy_reset_gpio = eth_phy_reset_gpio,
      .clock_ext_in = true,
      .clock_gpio = rmii_clk_gpio,
  };
  eth_config.mode = (config.mode == DhcpMode::SERVER) ? espp::Ethernet::DhcpMode::SERVER
                                                      : espp::Ethernet::DhcpMode::CLIENT;
  if (config.mode == DhcpMode::SERVER) {
    eth_config.ip_info = config.server_config.ip_info;
    eth_config.on_client_assigned = config.server_config.on_client_assigned;
  }
  eth_config.on_link_up = config.on_link_up;
  eth_config.on_link_down = config.on_link_down;
  eth_config.on_got_ip = config.on_got_ip;
  eth_config.on_lost_ip = config.on_lost_ip;

  ethernet_ = std::make_unique<espp::Ethernet>(eth_config);
  return ethernet_->initialize();
}

} // namespace espp
