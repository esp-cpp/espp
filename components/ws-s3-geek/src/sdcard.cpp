#include "ws-s3-geek.hpp"

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool WsS3Geek::initialize_sdcard(const WsS3Geek::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");
  // The card is driven in SPI mode on its own bus (SPI3): the component
  // initializes the bus and frees it again when the card is released.
  espp::SdCard::SpiConfig spi;
  spi.host = sdcard_spi_num;
  spi.cs = sdcard_cs;
  spi.initialize_bus = true;
  spi.mosi = sdcard_mosi;
  spi.miso = sdcard_miso;
  spi.sclk = sdcard_clk;
  spi.max_transfer_size = SPI_MAX_TRANSFER_BYTES;
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
