#include "t-deck.hpp"

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool TDeck::initialize_sdcard(const TDeck::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  // ensure that the SPI bus is initialized (shared with the display and the radio)
  if (!init_spi_bus()) {
    logger_.error("Failed to initialize SPI bus.");
    return false;
  }

  logger_.info("Initializing SD card");
  espp::SdCard::SpiConfig spi;
  spi.host = spi_num;
  spi.cs = sdcard_cs;
  spi.initialize_bus = false; // the BSP owns the (shared) bus
  sdcard_ = std::make_unique<espp::SdCard>(espp::SdCard::Config{
      .interface = spi,
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
