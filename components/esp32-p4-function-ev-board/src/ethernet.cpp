#include "esp32-p4-function-ev-board.hpp"

#if CONFIG_ESP_P4_EV_BOARD_ETHERNET

#include <esp_eth.h>
#include <esp_eth_mac_esp.h>
#include <esp_eth_netif_glue.h>
#include <esp_event.h>
#include <esp_netif.h>

#include "esp_eth_phy_ip101.h" // vendored IP101 PHY driver (third_party/)

namespace espp {

// IP101 PHY: reset GPIO and address. The RMII data/clock/MDIO pins for the
// ESP32-P4 Function EV Board are the ESP-IDF ESP32-P4 defaults
// (ETH_ESP32_EMAC_DEFAULT_CONFIG): MDC=31, MDIO=52, REF_CLK in=50, TX_EN=49,
// TXD0=34, TXD1=35, CRS_DV=28, RXD0=29, RXD1=30.
static constexpr int kPhyResetGpio = 51;
static constexpr int kPhyAddr = 1;

void Esp32P4FunctionEvBoard::ethernet_event_handler(void *arg, esp_event_base_t /*event_base*/,
                                                    int32_t event_id, void * /*event_data*/) {
  auto *self = static_cast<Esp32P4FunctionEvBoard *>(arg);
  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED: {
    // Log the negotiated speed/duplex. A 10 Mbps or half-duplex result usually
    // indicates an autonegotiation/duplex mismatch, which presents as "DHCP works
    // but unicast (ping) fails".
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    if (self->eth_handle_) {
      esp_eth_ioctl(self->eth_handle_, ETH_CMD_G_SPEED, &speed);
      esp_eth_ioctl(self->eth_handle_, ETH_CMD_G_DUPLEX_MODE, &duplex);
    }
    self->logger_.info("Ethernet link up: {} Mbps, {} duplex", speed == ETH_SPEED_100M ? 100 : 10,
                       duplex == ETH_DUPLEX_FULL ? "full" : "half");
    break;
  }
  case ETHERNET_EVENT_DISCONNECTED:
    self->logger_.info("Ethernet link down");
    self->ethernet_connected_ = false;
    self->ethernet_ip_ = {};
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

void Esp32P4FunctionEvBoard::ethernet_got_ip_handler(void *arg, esp_event_base_t /*event_base*/,
                                                     int32_t /*event_id*/, void *event_data) {
  auto *self = static_cast<Esp32P4FunctionEvBoard *>(arg);
  auto *event = static_cast<ip_event_got_ip_t *>(event_data);
  self->ethernet_ip_ = event->ip_info.ip;
  self->ethernet_connected_ = true;
  // Note: the espp logger uses fmt-style ({}) formatting, not printf-style, so
  // format the IPv4 octets explicitly rather than using IPSTR/IP2STR.
  self->logger_.info("Ethernet got IP: {}.{}.{}.{}", esp_ip4_addr1_16(&event->ip_info.ip),
                     esp_ip4_addr2_16(&event->ip_info.ip), esp_ip4_addr3_16(&event->ip_info.ip),
                     esp_ip4_addr4_16(&event->ip_info.ip));
  if (self->ethernet_link_callback_) {
    self->ethernet_link_callback_(event->ip_info.ip);
  }
}

bool Esp32P4FunctionEvBoard::initialize_ethernet(const ethernet_link_callback_t &on_link_up) {
  if (ethernet_initialized_) {
    logger_.warn("Ethernet already initialized");
    return true;
  }

  // warn the user if they are initializing ethernet after initializing the boot
  // button, since they share a pin.
  if (button_callback_) {
    logger_.warn(
        "Initializing Ethernet while BOOT button is initialized. The BOOT button is connected to "
        "the PHY's RMII_TXD1 pin, so the boot button will not work while Ethernet is enabled!");
  }

  logger_.info("Initializing Ethernet (EMAC + IP101 EMAC)");
  ethernet_link_callback_ = on_link_up;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  // NOTE: we can't use the ETH_ESP32_EMAC_DEFAULT_CONFIG macro because it's out
  // of order which is a hard error in c++20 and above.
  eth_esp32_emac_config_t esp32_emac_config = {
      .smi_gpio = {.mdc_num = 31, .mdio_num = 52},
      .interface = EMAC_DATA_INTERFACE_RMII,
      .clock_config = {.rmii = {.clock_mode = EMAC_CLK_EXT_IN, .clock_gpio = 50}},
      .dma_burst_len = ETH_DMA_BURST_LEN_32,
      .intr_priority = 0,
      .emac_dataif_gpio = {.rmii = {.tx_en_num = 49,
                                    .txd0_num = 34,
                                    .txd1_num = 35,
                                    .crs_dv_num = 28,
                                    .rxd0_num = 29,
                                    .rxd1_num = 30}},
      .clock_config_out_in = {.rmii = {.clock_mode = EMAC_CLK_EXT_IN, .clock_gpio = -1}},
      .mdc_freq_hz = 0,
  };
#pragma GCC diagnostic pop

  // Update PHY config based on board specific configuration
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = 51;

  // Update vendor specific MAC config based on board configuration
  esp32_emac_config.smi_gpio.mdc_num = 31;
  esp32_emac_config.smi_gpio.mdio_num = 52;

  logger_.info("Creating ESP32 EMAC");
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
  if (!mac) {
    logger_.error("Failed to create EMAC");
    return false;
  }

  logger_.info("Creating generic PHY (IP101)");
  esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
  if (!phy) {
    logger_.error("Failed to create generic PHY");
    return false;
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
#pragma GCC diagnostic pop
  eth_handle_ = nullptr;
  logger_.info("Installing Ethernet driver");
  esp_err_t ret = esp_eth_driver_install(&config, &eth_handle_);
  if (ret != ESP_OK) {
    logger_.error("esp_eth_driver_install failed: {}", esp_err_to_name(ret));
    return false;
  }

  ret = esp_netif_init();
  if (ret != ESP_OK) {
    logger_.error("esp_netif_init failed: {}", esp_err_to_name(ret));
    return false;
  }
  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    logger_.error("esp_event_loop_create_default failed: {}", esp_err_to_name(ret));
    return false;
  }

  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  eth_netif_ = esp_netif_new(&netif_cfg);
  if (!eth_netif_) {
    logger_.error("Failed to create Ethernet netif");
    return false;
  }

  eth_glue_ = esp_eth_new_netif_glue(eth_handle_);
  ret = esp_netif_attach(eth_netif_, static_cast<esp_eth_netif_glue_handle_t>(eth_glue_));
  if (ret != ESP_OK) {
    logger_.error("esp_netif_attach failed: {}", esp_err_to_name(ret));
    return false;
  }

  esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethernet_event_handler, this);
  esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ethernet_got_ip_handler, this);

  ret = esp_eth_start(eth_handle_);
  if (ret != ESP_OK) {
    logger_.error("esp_eth_start failed: {}", esp_err_to_name(ret));
    return false;
  }

  ethernet_initialized_ = true;
  logger_.info("Ethernet initialized; waiting for link/DHCP");
  return true;
}

} // namespace espp

#endif // CONFIG_ESP_P4_EV_BOARD_ETHERNET
