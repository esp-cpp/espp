#include "ethernet.hpp"

#include <algorithm>

#include "esp_idf_version.h"
// Provide fallbacks so this file's ESP-IDF version checks are well-defined even
// when esp_idf_version.h is not available (e.g. under cppcheck).
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <esp_event.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <lwip/ip4_addr.h>

#include <driver/gpio.h>

#if SOC_EMAC_SUPPORTED
#include <esp_eth_mac_esp.h>
#include <esp_eth_phy.h>
#endif

#if CONFIG_ESPP_ETHERNET_W5500
#include <esp_eth_mac_w5500.h>
#include <esp_eth_phy_w5500.h>
#endif
#if CONFIG_ESPP_ETHERNET_DM9051
#include <esp_eth_mac_dm9051.h>
#include <esp_eth_phy_dm9051.h>
#endif
#if CONFIG_ESPP_ETHERNET_ENC28J60
#include <esp_eth_enc28j60.h>
#endif

namespace espp {

Ethernet::Ethernet(const Config &config)
    : BaseComponent("Ethernet", config.log_level)
    , config_(config) {}

Ethernet::~Ethernet() { deinitialize(); }

//////////////////////////////////////////////////////////////////////////////
// Event handlers
//////////////////////////////////////////////////////////////////////////////

void Ethernet::eth_event_handler(void *arg, esp_event_base_t /*base*/, int32_t id, void *data) {
  auto *self = static_cast<Ethernet *>(arg);
  if (!self) {
    return;
  }
  // ETH_EVENT is registered with ESP_EVENT_ANY_ID, so when multiple espp::Ethernet
  // instances are active each handler sees every driver's events. The payload is a
  // pointer to the originating esp_eth_handle_t; ignore events from other drivers so
  // we don't corrupt our own link/connection state.
  if (!data || *static_cast<esp_eth_handle_t *>(data) != self->eth_handle_) {
    return;
  }
  switch (id) {
  case ETHERNET_EVENT_CONNECTED: {
    self->link_up_ = true;
    auto sd = self->link_speed_duplex();
    if (sd) {
      self->logger_.info("Link up: {} Mbps, {} duplex", sd->first, sd->second ? "full" : "half");
    } else {
      self->logger_.info("Link up");
    }
    if (self->config_.on_link_up) {
      self->config_.on_link_up();
    }
    // In server mode the IP is static, so surface got-IP immediately.
    if (self->config_.mode == DhcpMode::SERVER) {
      self->ip_addr_ = self->server_ip_info_.ip.addr;
      self->connected_ = true;
      if (self->config_.on_got_ip) {
        self->config_.on_got_ip(self->server_ip_info_.ip);
      }
    }
    break;
  }
  case ETHERNET_EVENT_DISCONNECTED:
    self->logger_.info("Link down");
    self->link_up_ = false;
    if (self->connected_.exchange(false)) {
      if (self->config_.on_lost_ip) {
        self->config_.on_lost_ip();
      }
    }
    self->ip_addr_ = 0;
    if (self->config_.on_link_down) {
      self->config_.on_link_down();
    }
    break;
  case ETHERNET_EVENT_START:
    self->logger_.debug("Started");
    break;
  case ETHERNET_EVENT_STOP:
    self->logger_.debug("Stopped");
    self->link_up_ = false;
    self->connected_ = false;
    self->ip_addr_ = 0;
    break;
  default:
    break;
  }
}

void Ethernet::got_ip_handler(void *arg, esp_event_base_t /*base*/, int32_t /*id*/, void *data) {
  auto *self = static_cast<Ethernet *>(arg);
  auto *event = static_cast<ip_event_got_ip_t *>(data);
  if (!self || !event) {
    return;
  }
  // IP_EVENT is global; only handle got-IP for our own netif.
  if (event->esp_netif != self->eth_netif_) {
    return;
  }
  self->ip_addr_ = event->ip_info.ip.addr;
  self->connected_ = true;
  self->logger_.info("Got IP: {}.{}.{}.{}", esp_ip4_addr1_16(&event->ip_info.ip),
                     esp_ip4_addr2_16(&event->ip_info.ip), esp_ip4_addr3_16(&event->ip_info.ip),
                     esp_ip4_addr4_16(&event->ip_info.ip));
  if (self->config_.on_got_ip) {
    self->config_.on_got_ip(event->ip_info.ip);
  }
}

void Ethernet::lost_ip_handler(void *arg, esp_event_base_t /*base*/, int32_t /*id*/, void *data) {
  auto *self = static_cast<Ethernet *>(arg);
  // The lost-IP event carries an ip_event_got_ip_t whose esp_netif identifies the
  // interface, so we can filter to our own netif just like got_ip_handler.
  auto *event = static_cast<ip_event_got_ip_t *>(data);
  if (!self || !event) {
    return;
  }
  if (event->esp_netif != self->eth_netif_) {
    return;
  }
  self->logger_.info("Lost IP");
  self->connected_ = false;
  self->ip_addr_ = 0;
  if (self->config_.on_lost_ip) {
    self->config_.on_lost_ip();
  }
}

void Ethernet::client_ip_handler(void *arg, esp_event_base_t /*base*/, int32_t /*id*/, void *data) {
  auto *self = static_cast<Ethernet *>(arg);
  auto *event = static_cast<ip_event_ap_staipassigned_t *>(data);
  if (!self || !event) {
    return;
  }
  // Only handle leases from our own DHCP-server netif.
  if (event->esp_netif != self->eth_netif_) {
    return;
  }
  MacAddress mac;
  std::copy(std::begin(event->mac), std::end(event->mac), mac.begin());
  self->logger_.info("DHCP server assigned {}.{}.{}.{}", esp_ip4_addr1_16(&event->ip),
                     esp_ip4_addr2_16(&event->ip), esp_ip4_addr3_16(&event->ip),
                     esp_ip4_addr4_16(&event->ip));
  if (self->config_.on_client_assigned) {
    self->config_.on_client_assigned(event->ip, mac);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Interface-specific MAC + PHY creation
//////////////////////////////////////////////////////////////////////////////

bool Ethernet::create_mac_phy(esp_eth_mac_t **mac, esp_eth_phy_t **phy, bool &needs_isr,
                              bool &needs_mac_assign, std::error_code &ec) {
  needs_isr = false;
  needs_mac_assign = false;

  // ---- Pre-built driver escape hatch ----
  if (auto *drv = std::get_if<DriverConfig>(&config_.interface)) {
    if (!drv->mac || !drv->phy) {
      logger_.error("DriverConfig requires non-null mac and phy");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    *mac = drv->mac;
    *phy = drv->phy;
    needs_isr = drv->needs_isr_service;
    needs_mac_assign = drv->needs_mac_assignment;
    return true;
  }

  // ---- RMII (internal EMAC) ----
  if ([[maybe_unused]] auto *rmii = std::get_if<RmiiConfig>(&config_.interface)) {
#if SOC_EMAC_SUPPORTED
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    // Zero-init then assign so we avoid both -Wmissing-field-initializers and the
    // out-of-order designated-init hard error under C++20.
    eth_esp32_emac_config_t emac_cfg = {};
    emac_cfg.smi_gpio.mdc_num = rmii->mdc_gpio;
    emac_cfg.smi_gpio.mdio_num = rmii->mdio_gpio;
    emac_cfg.interface = EMAC_DATA_INTERFACE_RMII;
    emac_cfg.clock_config.rmii.clock_mode = rmii->clock_ext_in ? EMAC_CLK_EXT_IN : EMAC_CLK_OUT;
    // clock_gpio is a plain int in IDF v6.0 but an `emac_rmii_clock_gpio_t` enum on
    // ESP32 in v5.x, so cast to the field's actual type to build across both.
    emac_cfg.clock_config.rmii.clock_gpio =
        static_cast<decltype(emac_cfg.clock_config.rmii.clock_gpio)>(rmii->clock_gpio);
    emac_cfg.dma_burst_len = ETH_DMA_BURST_LEN_32;
    emac_cfg.intr_priority = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    // mdc_freq_hz was added to eth_esp32_emac_config_t in ESP-IDF v6.0.
    emac_cfg.mdc_freq_hz = 0;
#endif
#if SOC_EMAC_USE_MULTI_IO_MUX || SOC_EMAC_MII_USE_GPIO_MATRIX
    if (rmii->data_pins) {
      emac_cfg.emac_dataif_gpio.rmii.tx_en_num = rmii->data_pins->tx_en;
      emac_cfg.emac_dataif_gpio.rmii.txd0_num = rmii->data_pins->txd0;
      emac_cfg.emac_dataif_gpio.rmii.txd1_num = rmii->data_pins->txd1;
      emac_cfg.emac_dataif_gpio.rmii.crs_dv_num = rmii->data_pins->crs_dv;
      emac_cfg.emac_dataif_gpio.rmii.rxd0_num = rmii->data_pins->rxd0;
      emac_cfg.emac_dataif_gpio.rmii.rxd1_num = rmii->data_pins->rxd1;
    }
#endif
#if !SOC_EMAC_RMII_CLK_OUT_INTERNAL_LOOPBACK
    emac_cfg.clock_config_out_in.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config_out_in.rmii.clock_gpio =
        static_cast<decltype(emac_cfg.clock_config_out_in.rmii.clock_gpio)>(-1);
#endif

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = rmii->phy_addr;
    phy_config.reset_gpio_num = rmii->phy_reset_gpio;

    *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_config);
    if (!*mac) {
      logger_.error("Failed to create ESP32 EMAC");
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    // v1 uses the generic 802.3 PHY driver (drives IP101/LAN87xx/DP83848/RTL8201/KSZ8041).
    *phy = esp_eth_phy_new_generic(&phy_config);
    if (!*phy) {
      logger_.error("Failed to create generic PHY");
      (*mac)->del(*mac);
      *mac = nullptr;
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    needs_mac_assign = config_.mac_address.has_value(); // EMAC has a factory MAC otherwise
    return true;
#else
    logger_.error("RMII requested but this SoC has no internal EMAC (SOC_EMAC_SUPPORTED=0)");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  }

  // ---- SPI (external MAC+PHY chip) ----
  auto *spi = std::get_if<SpiConfig>(&config_.interface);
  if (!spi) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 0;
  devcfg.clock_speed_hz = spi->clock_speed_hz;
  devcfg.queue_size = 20;
  devcfg.spics_io_num = spi->cs_gpio;
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = spi->phy_addr;
  phy_config.reset_gpio_num = spi->reset_gpio;
  needs_isr = (spi->int_gpio >= 0);
  needs_mac_assign = true; // SPI chips generally have no factory MAC

  switch (spi->chip) {
  case SpiChip::W5500:
#if CONFIG_ESPP_ETHERNET_W5500
  {
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi->host, &devcfg);
    w5500_config.base.int_gpio_num = spi->int_gpio;
    *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    if (*mac) {
      *phy = esp_eth_phy_new_w5500(&phy_config);
    }
    break;
  }
#else
    logger_.error("W5500 selected but CONFIG_ESPP_ETHERNET_W5500 is not enabled");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  case SpiChip::DM9051:
#if CONFIG_ESPP_ETHERNET_DM9051
  {
    eth_dm9051_config_t dm_config = ETH_DM9051_DEFAULT_CONFIG(spi->host, &devcfg);
    dm_config.int_gpio_num = spi->int_gpio;
    *mac = esp_eth_mac_new_dm9051(&dm_config, &mac_config);
    if (*mac) {
      *phy = esp_eth_phy_new_dm9051(&phy_config);
    }
    break;
  }
#else
    logger_.error("DM9051 selected but CONFIG_ESPP_ETHERNET_DM9051 is not enabled");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  case SpiChip::ENC28J60:
#if CONFIG_ESPP_ETHERNET_ENC28J60
  {
    eth_enc28j60_config_t enc_config = ETH_ENC28J60_DEFAULT_CONFIG(spi->host, &devcfg);
    enc_config.int_gpio_num = spi->int_gpio;
    *mac = esp_eth_mac_new_enc28j60(&enc_config, &mac_config);
    if (*mac) {
      *phy = esp_eth_phy_new_enc28j60(&phy_config);
    }
    break;
  }
#else
    logger_.error("ENC28J60 selected but CONFIG_ESPP_ETHERNET_ENC28J60 is not enabled");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  }

  if (!*mac || !*phy) {
    logger_.error("Failed to create SPI MAC/PHY");
    if (*phy) {
      (*phy)->del(*phy);
      *phy = nullptr;
    }
    if (*mac) {
      (*mac)->del(*mac);
      *mac = nullptr;
    }
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// initialize / deinitialize
//////////////////////////////////////////////////////////////////////////////

bool Ethernet::initialize(std::error_code &ec) {
  ec.clear();
  if (initialized_.load()) {
    return true;
  }

  const bool server_mode = (config_.mode == DhcpMode::SERVER);
  const bool is_spi = std::holds_alternative<SpiConfig>(config_.interface);

  esp_eth_mac_t *mac = nullptr;
  esp_eth_phy_t *phy = nullptr;
  bool needs_isr = false;
  bool needs_mac_assign = false;
  bool driver_installed = false;

  auto fail = [&](const char *msg, esp_err_t err, std::errc code) {
    logger_.error("{}: {}", msg, esp_err_to_name(err));
    if (client_ip_handler_registered_) {
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &client_ip_handler);
      client_ip_handler_registered_ = false;
    }
    if (handlers_registered_) {
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_LOST_IP, &lost_ip_handler);
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler);
      esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler);
      handlers_registered_ = false;
    }
    if (eth_handle_) {
      esp_eth_stop(eth_handle_);
    }
    if (eth_glue_) {
      esp_eth_del_netif_glue(eth_glue_);
      eth_glue_ = nullptr;
    }
    if (driver_installed && eth_handle_) {
      esp_eth_driver_uninstall(eth_handle_); // also frees the mac + phy it owns
      eth_handle_ = nullptr;
    } else {
      // driver not installed yet: free any mac/phy we created ourselves
      if (phy) {
        phy->del(phy);
      }
      if (mac) {
        mac->del(mac);
      }
    }
    if (eth_netif_) {
      esp_netif_destroy(eth_netif_);
      eth_netif_ = nullptr;
    }
    link_up_ = false;
    connected_ = false;
    ip_addr_ = 0;
    ec = std::make_error_code(code);
    return false;
  };

  // TCP/IP stack + default event loop (both process-global singletons).
  esp_err_t err = esp_netif_init();
  if (err != ESP_OK) {
    return fail("esp_netif_init failed", err, std::errc::io_error);
  }
  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return fail("esp_event_loop_create_default failed", err, std::errc::io_error);
  }

  // Create the MAC + PHY for the configured interface.
  if (!create_mac_phy(&mac, &phy, needs_isr, needs_mac_assign, ec)) {
    return false; // create_mac_phy already logged + freed
  }

  // SPI chips with an INT line need the GPIO ISR service (the driver adds a
  // handler but not the service). ESP_ERR_INVALID_STATE = already installed.
  if (needs_isr) {
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
      return fail("gpio_install_isr_service failed", isr_err, std::errc::io_error);
    }
  }

  // Install the driver (takes ownership of mac + phy).
  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  err = esp_eth_driver_install(&eth_config, &eth_handle_);
  if (err != ESP_OK) {
    return fail("esp_eth_driver_install failed", err, std::errc::io_error);
  }
  driver_installed = true;

  // Assign a MAC address if the chip needs one or the user provided one.
  if (needs_mac_assign) {
    MacAddress mac_addr{};
    if (config_.mac_address) {
      mac_addr = *config_.mac_address;
    } else {
      err = esp_read_mac(mac_addr.data(), ESP_MAC_ETH);
      if (err != ESP_OK) {
        return fail("esp_read_mac failed", err, std::errc::io_error);
      }
    }
    err = esp_eth_ioctl(eth_handle_, ETH_CMD_S_MAC_ADDR, mac_addr.data());
    if (err != ESP_OK) {
      return fail("Failed to set MAC address", err, std::errc::io_error);
    }
  }

  // Create the netif (server mode gets a DHCP-server-flagged netif).
  if (server_mode) {
    esp_netif_ip_info_t ip_info = config_.ip_info;
    if (ip_info.ip.addr == 0) {
      IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
      IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
      IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    }
    server_ip_info_ = ip_info;
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
    return fail("esp_netif_new failed", ESP_FAIL, std::errc::not_enough_memory);
  }

  eth_glue_ = esp_eth_new_netif_glue(eth_handle_);
  if (!eth_glue_) {
    return fail("esp_eth_new_netif_glue failed", ESP_FAIL, std::errc::not_enough_memory);
  }
  err = esp_netif_attach(eth_netif_, eth_glue_);
  if (err != ESP_OK) {
    return fail("esp_netif_attach failed", err, std::errc::io_error);
  }

  // Event handlers. Set handlers_registered_ as soon as the first one is
  // registered so fail() unregisters whatever got registered (unregistering a
  // not-registered handler is a harmless no-op), and validate every return.
  err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, this);
  if (err != ESP_OK) {
    return fail("register ETH_EVENT failed", err, std::errc::io_error);
  }
  handlers_registered_ = true;
  err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, this);
  if (err != ESP_OK) {
    return fail("register IP_EVENT_ETH_GOT_IP failed", err, std::errc::io_error);
  }
  err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &lost_ip_handler, this);
  if (err != ESP_OK) {
    return fail("register IP_EVENT_ETH_LOST_IP failed", err, std::errc::io_error);
  }
  if (server_mode) {
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &client_ip_handler, this);
    if (err != ESP_OK) {
      return fail("register client-IP handler failed", err, std::errc::io_error);
    }
    client_ip_handler_registered_ = true;
  }

  // Hostname (non-fatal).
  if (!config_.hostname.empty()) {
    err = esp_netif_set_hostname(eth_netif_, config_.hostname.c_str());
    if (err != ESP_OK) {
      logger_.warn("Failed to set hostname '{}': {}", config_.hostname, esp_err_to_name(err));
    }
  }

  // Static IP in client mode (ip.addr != 0): stop DHCP client, apply the IP.
  if (!server_mode && config_.ip_info.ip.addr != 0) {
    esp_err_t stop_err = esp_netif_dhcpc_stop(eth_netif_);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
      return fail("esp_netif_dhcpc_stop failed", stop_err, std::errc::io_error);
    }
    err = esp_netif_set_ip_info(eth_netif_, &config_.ip_info);
    if (err != ESP_OK) {
      return fail("esp_netif_set_ip_info failed", err, std::errc::io_error);
    }
  }

  err = esp_eth_start(eth_handle_);
  if (err != ESP_OK) {
    return fail("esp_eth_start failed", err, std::errc::io_error);
  }

  initialized_ = true;
  logger_.info("Ethernet initialized ({}, DHCP {})", is_spi ? "SPI" : "RMII",
               server_mode ? "server" : "client");
  return true;
}

bool Ethernet::initialize() {
  std::error_code ec;
  if (!initialize(ec)) {
    logger_.error("initialize failed: {}", ec.message());
    return false;
  }
  return true;
}

bool Ethernet::deinitialize(std::error_code &ec) {
  ec.clear();
  if (!initialized_.load() && !eth_handle_ && !eth_netif_) {
    return true;
  }
  logger_.info("Deinitializing Ethernet");
  if (client_ip_handler_registered_) {
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &client_ip_handler);
    client_ip_handler_registered_ = false;
  }
  if (handlers_registered_) {
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_LOST_IP, &lost_ip_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler);
    esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler);
    handlers_registered_ = false;
  }
  if (eth_handle_) {
    esp_eth_stop(eth_handle_);
  }
  if (eth_glue_) {
    esp_eth_del_netif_glue(eth_glue_);
    eth_glue_ = nullptr;
  }
  if (eth_handle_) {
    esp_eth_driver_uninstall(eth_handle_); // frees the mac + phy it owns
    eth_handle_ = nullptr;
  }
  if (eth_netif_) {
    esp_netif_destroy(eth_netif_);
    eth_netif_ = nullptr;
  }
  initialized_ = false;
  link_up_ = false;
  connected_ = false;
  ip_addr_ = 0;
  return true;
}

void Ethernet::deinitialize() {
  std::error_code ec;
  if (!deinitialize(ec)) {
    logger_.error("deinitialize failed: {}", ec.message());
  }
}

//////////////////////////////////////////////////////////////////////////////
// Getters
//////////////////////////////////////////////////////////////////////////////

esp_ip4_addr_t Ethernet::ip() const {
  esp_ip4_addr_t addr{};
  addr.addr = ip_addr_.load();
  return addr;
}

std::string Ethernet::get_ip_address() const {
  esp_ip4_addr_t addr{};
  addr.addr = ip_addr_.load();
  char buf[16] = {0};
  esp_ip4addr_ntoa(&addr, buf, sizeof(buf));
  return std::string(buf);
}

std::string Ethernet::get_mac_address() const {
  if (!eth_handle_) {
    return "";
  }
  MacAddress mac{};
  if (esp_eth_ioctl(eth_handle_, ETH_CMD_G_MAC_ADDR, mac.data()) != ESP_OK) {
    return "";
  }
  return fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", mac[0], mac[1], mac[2], mac[3],
                     mac[4], mac[5]);
}

std::optional<std::pair<int, bool>> Ethernet::link_speed_duplex() const {
  if (!eth_handle_ || !link_up_.load()) {
    return std::nullopt;
  }
  eth_speed_t speed = ETH_SPEED_10M;
  eth_duplex_t duplex = ETH_DUPLEX_HALF;
  esp_eth_ioctl(eth_handle_, ETH_CMD_G_SPEED, &speed);
  esp_eth_ioctl(eth_handle_, ETH_CMD_G_DUPLEX_MODE, &duplex);
  return std::make_pair(speed == ETH_SPEED_100M ? 100 : 10, duplex == ETH_DUPLEX_FULL);
}

} // namespace espp
