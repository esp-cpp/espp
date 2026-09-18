#include "m5stack-cardputer.hpp"

using namespace espp;

////////////////////////
// uSD Card Functions //
////////////////////////

bool M5StackCardputer::initialize_sdcard(const SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");

  // The uSD card is on its own SPI bus (not shared with the LCD, but shared
  // with the LoRa+GPS Cap's radio)
  if (!ensure_expansion_spi_bus()) {
    logger_.error("Failed to initialize SPI bus for SD card");
    return false;
  }

  espp::SdCard::SpiConfig spi;
  spi.host = sdcard_spi_num;
  spi.cs = sdcard_cs;
  spi.initialize_bus = false; // the BSP owns the (shared) expansion bus
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
    // release the card (and its SPI device) before touching the bus
    sdcard_.reset();
    // only free the bus if the LoRa radio isn't using it
    if (!lora_) {
      spi_bus_free(sdcard_spi_num);
      expansion_spi_bus_initialized_ = false;
    }
    return false;
  }

  logger_.info("Filesystem mounted at {}", mount_point);
  sdcard_->print_info(stdout);
  return true;
}
