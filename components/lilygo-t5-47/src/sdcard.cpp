#include "lilygo-t5-47.hpp"

#include <cstring>

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

  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  // The SPI bus is already initialized (init_spi_bus above), so use the host on
  // our spi_num and only attach the card's chip-select on this slot.
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = spi_num;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sdcard_cs;
  slot_config.host_id = static_cast<spi_host_device_t>(host.slot);

  logger_.debug("Mounting filesystem at {}", mount_point);
  esp_err_t ret =
      esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem (set format_if_mount_failed to format the card)");
    } else {
      logger_.error("Failed to initialize the microSD card ({}). Make sure the card is inserted "
                    "and the lines have pull-ups.",
                    esp_err_to_name(ret));
    }
    sdcard_ = nullptr;
    return false;
  }

  logger_.info("microSD card mounted at {}", mount_point);
  sdmmc_card_print_info(stdout, sdcard_);
  return true;
}
