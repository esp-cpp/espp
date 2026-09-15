#include "m5stack-tab5.hpp"

#include <esp_vfs_fat.h>
#include <sd_pwr_ctrl_by_on_chip_ldo.h>
#include <sdmmc_cmd.h>

namespace espp {

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_sdcard(const M5StackTab5::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");

  esp_err_t ret;
  // Options for mounting the filesystem. If format_if_mount_failed is set to
  // true, SD card will be partitioned and formatted in case when mounting
  // fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.
  logger_.debug("Using SDMMC peripheral");

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40MHz
  host.slot = SDMMC_HOST_SLOT_0;

  // The ESP32-P4 powers the SD card's IO pads from its internal LDO (LDO_VO4).
  // Without a power control handle that rail stays off, the bus floats and
  // card init fails (timeouts, or errors on the first data transfer). Same as
  // M5Stack's own Tab5 BSP.
  if (sd_pwr_ctrl_handle_ == nullptr) {
    sd_pwr_ctrl_ldo_config_t ldo_config{};
    ldo_config.ldo_chan_id = sd_ldo_channel;
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = nullptr;
    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
      logger_.error("Failed to create the SD power control driver: {}", esp_err_to_name(ret));
      return false;
    }
    sd_pwr_ctrl_handle_ = pwr_ctrl_handle;
  }
  host.pwr_ctrl_handle = static_cast<sd_pwr_ctrl_handle_t>(sd_pwr_ctrl_handle_);

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.clk = sd_clk_io;
  slot_config.cmd = sd_cmd_io;
  slot_config.d0 = sd_dat0_io;
  slot_config.d1 = sd_dat1_io;
  slot_config.d2 = sd_dat2_io;
  slot_config.d3 = sd_dat3_io;
  slot_config.width = 4;

  logger_.debug("Mounting filesystem");
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. ");
    } else {
      logger_.warn("Failed to initialize the card ({}). "
                   "Make sure SD card is present and lines have pull-up resistors in place.",
                   esp_err_to_name(ret));
    }
    sd_pwr_ctrl_del_on_chip_ldo(static_cast<sd_pwr_ctrl_handle_t>(sd_pwr_ctrl_handle_));
    sd_pwr_ctrl_handle_ = nullptr;
    return false;
  }

  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard_);

  sd_card_initialized_ = true;

  return true;
}

bool M5StackTab5::is_sd_card_available() const { return sd_card_initialized_; }

bool M5StackTab5::get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const {
  if (!sd_card_initialized_) {
    return false;
  }

  uint64_t total_bytes = 0, free_bytes = 0;
  esp_err_t ret = esp_vfs_fat_info(mount_point, &total_bytes, &free_bytes);
  if (ret != ESP_OK) {
    logger_.error("Failed to get SD card information ({})", esp_err_to_name(ret));
    return false;
  }

  if (size_mb) {
    *size_mb = total_bytes / (1024 * 1024);
  }
  if (free_mb) {
    *free_mb = free_bytes / (1024 * 1024);
  }

  return true;
}

} // namespace espp
