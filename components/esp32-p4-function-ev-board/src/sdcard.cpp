#include "esp32-p4-function-ev-board.hpp"

#include <esp_vfs_fat.h>
#include <sd_pwr_ctrl_by_on_chip_ldo.h>
#include <sdmmc_cmd.h>

namespace espp {

bool Esp32P4FunctionEvBoard::initialize_sdcard(const SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card (4-bit SDMMC)");

  esp_vfs_fat_sdmmc_mount_config_t mount_config{};
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  host.slot = SDMMC_HOST_SLOT_0;

  // The ESP32-P4 powers the SD card via an internal LDO (LDO_VO4). Configure the
  // power control handle so the host can enable that rail.
  sd_pwr_ctrl_ldo_config_t ldo_config{};
  ldo_config.ldo_chan_id = sd_ldo_channel;
  sd_pwr_ctrl_handle_t pwr_ctrl_handle = nullptr;
  esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
  if (ret != ESP_OK) {
    logger_.error("Failed to create SD power control driver: {}", esp_err_to_name(ret));
    return false;
  }
  host.pwr_ctrl_handle = pwr_ctrl_handle;
  sd_pwr_ctrl_handle_ = pwr_ctrl_handle;

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 4;
  slot_config.clk = sd_clk_io;
  slot_config.cmd = sd_cmd_io;
  slot_config.d0 = sd_d0_io;
  slot_config.d1 = sd_d1_io;
  slot_config.d2 = sd_d2_io;
  slot_config.d3 = sd_d3_io;

  logger_.debug("Mounting filesystem");
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem.");
    } else {
      logger_.warn("Failed to initialize the card ({}). "
                   "Make sure an SD card is inserted.",
                   esp_err_to_name(ret));
    }
    sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
    sd_pwr_ctrl_handle_ = nullptr;
    return false;
  }

  logger_.info("Filesystem mounted");
  sdmmc_card_print_info(stdout, sdcard_);
  sd_card_initialized_ = true;
  return true;
}

bool Esp32P4FunctionEvBoard::get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const {
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
