#include "lilygo-t5-47.hpp"

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// SPI bus (shared by the microSD card and the board's other SPI peripherals)
/////////////////////////////////////////////////////////////////////////////

bool LilyGoT547::init_spi_bus() {
  if (spi_) {
    return spi_->initialized();
  }
  logger_.info("Initializing SPI bus (host {}, sclk={}, mosi={}, miso={})",
               static_cast<int>(spi_num), static_cast<int>(spi_sclk_io),
               static_cast<int>(spi_mosi_io), static_cast<int>(spi_miso_io));
  spi_ = std::make_unique<Spi>(Spi::Config{
      .host = spi_num,
      .sclk_io_num = spi_sclk_io,
      .mosi_io_num = spi_mosi_io,
      .miso_io_num = spi_miso_io,
      .max_transfer_sz = SPI_MAX_TRANSFER_BYTES,
      .dma_channel = SPI_DMA_CH_AUTO,
      .log_level = get_log_level(),
  });
  if (!spi_->initialized()) {
    logger_.error("Failed to initialize SPI bus");
    spi_.reset();
    return false;
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// microSD Card
/////////////////////////////////////////////////////////////////////////////

bool LilyGoT547::initialize_sdcard(const LilyGoT547::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("microSD card already initialized!");
    return false;
  }

  // The microSD card shares the board's SPI bus; make sure it is up.
  if (!init_spi_bus()) {
    logger_.error("Failed to initialize SPI bus for the microSD card");
    return false;
  }

  logger_.info("Initializing microSD card (CS={})", static_cast<int>(sdcard_cs));
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
    logger_.error("Failed to initialize the microSD card: {}", ec.message());
    sdcard_.reset();
    return false;
  }
  logger_.info("microSD card mounted at {}", mount_point);
  sdcard_->print_info(stdout);
  return true;
}
