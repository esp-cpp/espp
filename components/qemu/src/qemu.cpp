#include "qemu.hpp"

#include <esp_eth.h>
#include <esp_eth_driver.h>
#include <esp_eth_phy.h>
#if CONFIG_ETH_USE_OPENETH
#include <esp_eth_mac_openeth.h>
#endif

namespace espp {

Qemu::Qemu()
    : BaseComponent("Qemu") {}

/////////////////////////////////////////////////////////////////////////////
// Ethernet
/////////////////////////////////////////////////////////////////////////////

bool Qemu::initialize_ethernet() { return initialize_ethernet(EthernetConfig{}); }

bool Qemu::initialize_ethernet(const EthernetConfig &config) {
  if (ethernet_ && ethernet_->is_initialized()) {
    logger_.warn("Ethernet already initialized");
    return true;
  }
#if !CONFIG_ETH_USE_OPENETH
  logger_.error("The OpenCores Ethernet MAC driver is not enabled: set CONFIG_ETH_USE_OPENETH "
                "(Component config -> Ethernet -> Support OpenCores Ethernet MAC)");
  return false;
#else
  logger_.info("Initializing Ethernet (OpenCores MAC, DHCP {})",
               config.mode == DhcpMode::SERVER ? "server" : "client");

  // The emulated MAC has no pins and no real PHY: QEMU answers the MII
  // management registers itself, so the generic 802.3 PHY driver is enough,
  // and the link is up as soon as the driver starts, so a short auto-
  // negotiation timeout keeps start-up quick. The MAC address comes from QEMU
  // (-nic ...,mac=...; 52:54:00:12:34:56 by default), read from the MAC's
  // address registers by the driver, so nothing is assigned here.
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  esp_eth_mac_t *mac = esp_eth_mac_new_openeth(&mac_config);
  if (mac == nullptr) {
    logger_.error("Could not create the OpenCores MAC (not running in QEMU?)");
    return false;
  }
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = -1;
  phy_config.autonego_timeout_ms = 100;
  esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
  if (phy == nullptr) {
    logger_.error("Could not create the PHY driver");
    mac->del(mac);
    return false;
  }

  Ethernet::Config eth_config{};
  eth_config.interface = Ethernet::DriverConfig{
      .mac = mac,
      .phy = phy,
      .needs_isr_service = false,
      .needs_mac_assignment = false,
  };
  eth_config.mode =
      config.mode == DhcpMode::SERVER ? Ethernet::DhcpMode::SERVER : Ethernet::DhcpMode::CLIENT;
  eth_config.hostname = config.hostname;
  eth_config.ip_info = config.ip_info;
  eth_config.on_link_up = config.on_link_up;
  eth_config.on_link_down = config.on_link_down;
  eth_config.on_got_ip = config.on_got_ip;
  eth_config.on_lost_ip = config.on_lost_ip;
  eth_config.on_client_assigned = config.on_client_assigned;
  eth_config.log_level = logger_.get_verbosity();

  ethernet_ = std::make_shared<Ethernet>(eth_config);
  if (!ethernet_->initialize()) {
    logger_.error("Ethernet initialization failed");
    ethernet_.reset();
    return false;
  }
  return true;
#endif
}

/////////////////////////////////////////////////////////////////////////////
// SD card
/////////////////////////////////////////////////////////////////////////////

#if SOC_SDMMC_HOST_SUPPORTED
bool Qemu::initialize_sdcard() { return initialize_sdcard(SdCardConfig{}); }

bool Qemu::initialize_sdcard(const SdCardConfig &config) {
  if (sdcard_ && sdcard_->is_mounted()) {
    logger_.warn("SD card already mounted");
    return true;
  }
  logger_.info("Initializing SD card (SD/MMC slot 1, {}-bit) at {}", config.bus_width,
               config.mount_point);
  // QEMU emulates the SD/MMC host (ESP32 machine) and the card behind it is
  // the -drive ...,if=sd image. The slot's pins are the chip's fixed ones; the
  // emulator does not care which.
  SdCard::Config sd_config{};
  sd_config.interface = SdCard::SdmmcConfig{.slot = 1, .bus_width = config.bus_width};
  sd_config.mount_point = config.mount_point;
  sd_config.mount_on_initialize = true;
  sd_config.format_if_mount_failed = config.format_if_mount_failed;
  sd_config.log_level = logger_.get_verbosity();
  sdcard_ = std::make_shared<SdCard>(sd_config);
  std::error_code ec;
  if (!sdcard_->initialize(ec)) {
    logger_.error("SD card initialization failed: {} (is an image attached with -drive "
                  "file=sd.img,if=sd,format=raw?)",
                  ec.message());
    sdcard_.reset();
    return false;
  }
  return true;
}
#endif

} // namespace espp
