#include "esp32-p4-eth.hpp"

namespace espp {

Esp32P4Eth::Esp32P4Eth()
    : BaseComponent("Esp32P4Eth") {}

bool Esp32P4Eth::initialize_ethernet() { return initialize_ethernet(EthernetConfig{}); }

bool Esp32P4Eth::initialize_ethernet(const EthernetConfig &config) {
  if (ethernet_ && ethernet_->is_initialized()) {
    logger_.warn("Ethernet already initialized");
    return true;
  }

  logger_.info("Initializing Ethernet (EMAC + IP101GRI RMII, DHCP {})",
               config.mode == DhcpMode::SERVER ? "server" : "client");

  espp::Ethernet::Config eth_config{};
  eth_config.interface = espp::Ethernet::RmiiConfig{
      .mdc_gpio = eth_mdc_io,
      .mdio_gpio = eth_mdio_io,
      .phy_addr = eth_phy_addr,
      .phy_reset_gpio = eth_phy_reset_gpio,
      .clock_ext_in = true,
      .clock_gpio = eth_ref_clk_io,
      .data_pins = espp::Ethernet::RmiiConfig::DataPins{.tx_en = eth_tx_en_io,
                                                        .txd0 = eth_txd0_io,
                                                        .txd1 = eth_txd1_io,
                                                        .crs_dv = eth_crs_dv_io,
                                                        .rxd0 = eth_rxd0_io,
                                                        .rxd1 = eth_rxd1_io},
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
