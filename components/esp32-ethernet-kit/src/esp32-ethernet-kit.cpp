#include "esp32-ethernet-kit.hpp"

#include "esp_idf_version.h"
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <esp_eth.h>
#include <esp_eth_mac_esp.h>
#include <esp_eth_netif_glue.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <lwip/ip4_addr.h>

#include <algorithm>

namespace espp {

Esp32EthernetKit::Esp32EthernetKit()
    : BaseComponent("Esp32EthernetKit") {}

void Esp32EthernetKit::ethernet_event_handler(void *arg, esp_event_base_t /*event_base*/,
                                              int32_t event_id, void * /*event_data*/) {
  auto *self = static_cast<Esp32EthernetKit *>(arg);
  if (!self) {
    return;
  }
  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED: {
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    if (self->eth_handle_) {
      esp_eth_ioctl(self->eth_handle_, ETH_CMD_G_SPEED, &speed);
      esp_eth_ioctl(self->eth_handle_, ETH_CMD_G_DUPLEX_MODE, &duplex);
    }
    self->logger_.info("Ethernet link up: {} Mbps, {} duplex", speed == ETH_SPEED_100M ? 100 : 10,
                       duplex == ETH_DUPLEX_FULL ? "full" : "half");
    if (self->on_link_up_) {
      self->on_link_up_();
    }
    // In server mode the IP is static — fire got_ip immediately
    if (self->dhcp_mode_ == DhcpMode::SERVER) {
      self->ethernet_ip_ = self->server_ip_info_.ip;
      self->ethernet_connected_ = true;
      if (self->on_got_ip_) {
        self->on_got_ip_(self->server_ip_info_.ip);
      }
    }
    break;
  }
  case ETHERNET_EVENT_DISCONNECTED:
    self->logger_.info("Ethernet link down");
    if (self->ethernet_connected_) {
      self->ethernet_connected_ = false;
      if (self->on_lost_ip_) {
        self->on_lost_ip_();
      }
    }
    self->ethernet_ip_ = {};
    if (self->on_link_down_) {
      self->on_link_down_();
    }
    break;
  case ETHERNET_EVENT_START:
    self->logger_.info("Ethernet started");
    break;
  case ETHERNET_EVENT_STOP:
    self->logger_.info("Ethernet stopped");
    break;
  default:
    break;
  }
}

void Esp32EthernetKit::ethernet_got_ip_handler(void *arg, esp_event_base_t /*event_base*/,
                                               int32_t /*event_id*/, void *event_data) {
  auto *self = static_cast<Esp32EthernetKit *>(arg);
  if (!self) {
    return;
  }
  auto *event = static_cast<ip_event_got_ip_t *>(event_data);
  self->ethernet_ip_ = event->ip_info.ip;
  self->ethernet_connected_ = true;
  self->logger_.info("Ethernet got IP: {}.{}.{}.{}", esp_ip4_addr1_16(&event->ip_info.ip),
                     esp_ip4_addr2_16(&event->ip_info.ip), esp_ip4_addr3_16(&event->ip_info.ip),
                     esp_ip4_addr4_16(&event->ip_info.ip));
  if (self->on_got_ip_) {
    self->on_got_ip_(event->ip_info.ip);
  }
}

void Esp32EthernetKit::ethernet_lost_ip_handler(void *arg, esp_event_base_t /*event_base*/,
                                                int32_t /*event_id*/, void * /*event_data*/) {
  auto *self = static_cast<Esp32EthernetKit *>(arg);
  if (!self) {
    return;
  }
  self->logger_.info("Ethernet lost IP");
  self->ethernet_connected_ = false;
  self->ethernet_ip_ = {};
  if (self->on_lost_ip_) {
    self->on_lost_ip_();
  }
}

void Esp32EthernetKit::ethernet_client_ip_handler(void *arg, esp_event_base_t /*event_base*/,
                                                  int32_t /*event_id*/, void *event_data) {
  auto *self = static_cast<Esp32EthernetKit *>(arg);
  if (!self) {
    return;
  }
  auto *event = static_cast<ip_event_ap_staipassigned_t *>(event_data);
  // Filter: only handle leases from our Ethernet DHCP server netif
  if (event->esp_netif != self->eth_netif_) {
    return;
  }
  std::array<uint8_t, 6> mac;
  std::copy(std::begin(event->mac), std::end(event->mac), mac.begin());
  self->logger_.info(
      "DHCP server assigned {}.{}.{}.{} to {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
      esp_ip4_addr1_16(&event->ip), esp_ip4_addr2_16(&event->ip), esp_ip4_addr3_16(&event->ip),
      esp_ip4_addr4_16(&event->ip), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (self->client_ip_callback_) {
    self->client_ip_callback_(event->ip, mac);
  }
}

bool Esp32EthernetKit::initialize_ethernet() { return initialize_ethernet(EthernetConfig{}); }

bool Esp32EthernetKit::initialize_ethernet(const EthernetConfig &config) {
  if (ethernet_initialized_) {
    logger_.warn("Ethernet already initialized");
    return true;
  }

  const DhcpMode mode = config.mode;
  logger_.info("Initializing Ethernet (EMAC + IP101GRI RMII, DHCP {})",
               mode == DhcpMode::SERVER ? "server" : "client");
  dhcp_mode_ = mode;
  on_link_up_ = config.on_link_up;
  on_link_down_ = config.on_link_down;
  on_got_ip_ = config.on_got_ip;
  on_lost_ip_ = config.on_lost_ip;
  client_ip_callback_ = config.server_config.on_client_assigned;

  esp_eth_mac_t *mac = nullptr;
  esp_eth_phy_t *phy = nullptr;
  bool eth_handler_registered = false;
  bool got_ip_handler_registered = false;
  bool lost_ip_handler_registered = false;
  bool client_ip_handler_registered = false;

  auto fail = [&](const char *message, esp_err_t err) {
    logger_.error("{}: {}", message, esp_err_to_name(err));

    if (client_ip_handler_registered) {
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                   &Esp32EthernetKit::ethernet_client_ip_handler);
    }
    if (lost_ip_handler_registered) {
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_LOST_IP,
                                   &Esp32EthernetKit::ethernet_lost_ip_handler);
    }
    if (got_ip_handler_registered) {
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                   &Esp32EthernetKit::ethernet_got_ip_handler);
    }
    if (eth_handler_registered) {
      esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID,
                                   &Esp32EthernetKit::ethernet_event_handler);
    }

    if (eth_handle_) {
      // Best-effort stop in case the driver was already started.
      esp_eth_stop(eth_handle_);
    }
    if (eth_glue_) {
      esp_eth_del_netif_glue(eth_glue_);
      eth_glue_ = nullptr;
    }
    if (eth_handle_) {
      esp_eth_driver_uninstall(eth_handle_);
      eth_handle_ = nullptr;
    }
    if (phy) {
      phy->del(phy);
      phy = nullptr;
    }
    if (mac) {
      mac->del(mac);
      mac = nullptr;
    }
    if (eth_netif_) {
      esp_netif_destroy(eth_netif_);
      eth_netif_ = nullptr;
    }

    // Reset observable runtime state so retries start from a clean slate.
    ethernet_initialized_ = false;
    ethernet_connected_ = false;
    ethernet_ip_ = {};
    server_ip_info_ = {};
    dhcp_mode_ = DhcpMode::CLIENT;
    on_link_up_ = nullptr;
    on_link_down_ = nullptr;
    on_got_ip_ = nullptr;
    on_lost_ip_ = nullptr;
    client_ip_callback_ = nullptr;

    return false;
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

  // ESP32-Ethernet-Kit A V1.2 uses an external 50 MHz oscillator connected to
  // GPIO0 as the RMII reference clock (EMAC_CLK_EXT_IN). GPIO0 is also the BOOT
  // strapping pin; see the class-level warning about the conflict.
  //
  // Note: on ESP32 the RMII data-plane pins are fixed via IO_MUX (TX_EN=21,
  // TXD0=19, TXD1=22, CRS_DV=27, RXD0=25, RXD1=26) so SOC_EMAC_USE_MULTI_IO_MUX
  // is not defined and emac_dataif_gpio does not exist in the struct.
  // SOC_EMAC_RMII_CLK_OUT_INTERNAL_LOOPBACK=1 on ESP32, so clock_config_out_in
  // is also not present in the struct.
  eth_esp32_emac_config_t esp32_emac_config = {
    .smi_gpio = {.mdc_num = eth_mdc_io, .mdio_num = eth_mdio_io},
    .interface = EMAC_DATA_INTERFACE_RMII,
    .clock_config = {.rmii = {.clock_mode = EMAC_CLK_EXT_IN,
                              .clock_gpio = static_cast<emac_rmii_clock_gpio_t>(rmii_clk_gpio)}},
    .dma_burst_len = ETH_DMA_BURST_LEN_32,
    .intr_priority = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    .mdc_freq_hz = 0,
#endif
  };
#pragma GCC diagnostic pop

  phy_config.phy_addr = eth_phy_addr;
  phy_config.reset_gpio_num = eth_phy_reset_gpio;

  logger_.info("Creating ESP32 EMAC");
  mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
  if (!mac) {
    return fail("Failed to create EMAC", ESP_FAIL);
  }

  logger_.info("Creating IP101GRI PHY");
  phy = esp_eth_phy_new_ip101(&phy_config);
  if (!phy) {
    return fail("Failed to create PHY", ESP_FAIL);
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
#pragma GCC diagnostic pop

  eth_handle_ = nullptr;
  logger_.info("Installing Ethernet driver");
  esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle_);
  if (ret != ESP_OK) {
    return fail("esp_eth_driver_install failed", ret);
  }
  // Ownership of MAC/PHY objects transfers to the driver after install.
  mac = nullptr;
  phy = nullptr;

  ret = esp_netif_init();
  if (ret != ESP_OK) {
    return fail("esp_netif_init failed", ret);
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    return fail("esp_event_loop_create_default failed", ret);
  }

  // Create Ethernet netif — server mode uses a proper DHCP-server-flagged netif so that
  // esp_netif allocates dhcps_t and wires up dhcps_set_new_lease_cb, which then fires
  // IP_EVENT_AP_STAIPASSIGNED for every lease. Client mode uses the standard ETH default.
  if (mode == DhcpMode::SERVER) {
    esp_netif_ip_info_t ip_info = config.server_config.ip_info;
    if (ip_info.ip.addr == 0) {
      IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
      IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
      IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    }
    server_ip_info_ = ip_info;

    // ESP_NETIF_FLAG_AUTOUP causes the DHCP server to start at ETHERNET_EVENT_START
    // (before the cable is connected), so dhcps_set_new_lease_cb is properly registered
    // and will fire IP_EVENT_AP_STAIPASSIGNED on each lease.
    esp_netif_inherent_config_t dhcps_cfg = {};
    dhcps_cfg.flags = (esp_netif_flags_t)(ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP);
    dhcps_cfg.ip_info = &ip_info;
    dhcps_cfg.if_key = "ETH_DHCPS";
    dhcps_cfg.if_desc = "eth";
    dhcps_cfg.route_prio = 50;
    esp_netif_config_t netif_cfg = {};
    netif_cfg.base = &dhcps_cfg;
    netif_cfg.stack = ESP_NETIF_NETSTACK_DEFAULT_ETH;
    eth_netif_ = esp_netif_new(&netif_cfg);
  } else {
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif_ = esp_netif_new(&netif_cfg);
  }
  if (!eth_netif_) {
    return fail("Failed to create Ethernet netif", ESP_FAIL);
  }

  eth_glue_ = esp_eth_new_netif_glue(eth_handle_);
  if (!eth_glue_) {
    return fail("Failed to create Ethernet netif glue", ESP_FAIL);
  }
  ret = esp_netif_attach(eth_netif_, eth_glue_);
  if (ret != ESP_OK) {
    return fail("esp_netif_attach failed", ret);
  }

  ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethernet_event_handler, this);
  if (ret != ESP_OK) {
    return fail("Failed to register Ethernet event handler", ret);
  }
  eth_handler_registered = true;

  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ethernet_got_ip_handler, this);
  if (ret != ESP_OK) {
    return fail("Failed to register Ethernet got-IP handler", ret);
  }
  got_ip_handler_registered = true;

  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &ethernet_lost_ip_handler, this);
  if (ret != ESP_OK) {
    return fail("Failed to register Ethernet lost-IP handler", ret);
  }
  lost_ip_handler_registered = true;

  if (mode == DhcpMode::SERVER) {
    // IP_EVENT_AP_STAIPASSIGNED fires for every client the DHCP server serves
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                     &ethernet_client_ip_handler, this);
    if (ret != ESP_OK) {
      return fail("Failed to register DHCP-server client-IP handler", ret);
    }
    client_ip_handler_registered = true;
  }

  ret = esp_eth_start(eth_handle_);
  if (ret != ESP_OK) {
    return fail("esp_eth_start failed", ret);
  }

  ethernet_initialized_ = true;
  logger_.info("Ethernet initialized (DHCP {})",
               mode == DhcpMode::SERVER ? "server" : "client — waiting for link/DHCP");
  return true;
}

} // namespace espp
