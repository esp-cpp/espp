#include "m5stack-tab5.hpp"

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
  espp::SdCard::SdmmcConfig sdmmc;
  sdmmc.slot = SDMMC_HOST_SLOT_0;
  sdmmc.bus_width = 4;
  sdmmc.clk = sd_clk_io;
  sdmmc.cmd = sd_cmd_io;
  sdmmc.d0 = sd_dat0_io;
  sdmmc.d1 = sd_dat1_io;
  sdmmc.d2 = sd_dat2_io;
  sdmmc.d3 = sd_dat3_io;
  sdmmc.frequency_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz
  // The ESP32-P4 powers the SD card's IO pads from its internal LDO (LDO_VO4):
  // without it the bus floats and card init fails. Same as M5Stack's own BSP.
  sdmmc.ldo_channel = sd_ldo_channel;
  sdcard_ = std::make_unique<espp::SdCard>(espp::SdCard::Config{
      .interface = sdmmc,
      .mount_point = mount_point,
      .format_if_mount_failed = config.format_if_mount_failed,
      .max_files = config.max_files,
      .allocation_unit_size = config.allocation_unit_size,
      .log_level = get_log_level(),
  });
  std::error_code ec;
  if (!sdcard_->initialize(ec)) {
    logger_.error("Failed to initialize the SD card: {}", ec.message());
    sdcard_.reset();
    return false;
  }
  logger_.info("Filesystem mounted at {}", mount_point);
  sdcard_->print_info(stdout);
  sd_card_initialized_ = true;
  return true;
}

bool M5StackTab5::is_sd_card_available() const { return sd_card_initialized_; }

bool M5StackTab5::get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const {
  if (!sd_card_initialized_ || !sdcard_) {
    return false;
  }
  const auto volume = sdcard_->volume_info();
  if (!volume) {
    logger_.error("Failed to get SD card information (volume not mounted)");
    return false;
  }
  if (size_mb) {
    *size_mb = volume->total_bytes / (1024 * 1024);
  }
  if (free_mb) {
    *free_mb = volume->free_bytes / (1024 * 1024);
  }
  return true;
}

} // namespace espp
