#include "t-dongle-s3.hpp"

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool TDongleS3::initialize_sdcard(const TDongleS3::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");
  espp::SdCard::SdmmcConfig sdmmc;
  sdmmc.bus_width = 4;
  sdmmc.clk = sdcard_clk;
  sdmmc.cmd = sdcard_cmd;
  sdmmc.d0 = sdcard_d0;
  sdmmc.d1 = sdcard_d1;
  sdmmc.d2 = sdcard_d2;
  sdmmc.d3 = sdcard_d3;
  sdmmc.frequency_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz
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
  return true;
}
