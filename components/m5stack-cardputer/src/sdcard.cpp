#include "m5stack-cardputer.hpp"

#include <cstring>

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

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = sdcard_spi_num;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sdcard_cs;
  slot_config.host_id = static_cast<spi_host_device_t>(host.slot);

  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  logger_.debug("Mounting filesystem");
  auto ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. If you want the card to be formatted, set the "
                    "format_if_mount_failed field in the SdCardConfig.");
    } else {
      logger_.error("Failed to initialize the card ({}). Make sure SD card lines have pull-up "
                    "resistors in place.",
                    esp_err_to_name(ret));
    }
    // only free the bus if the LoRa radio isn't using it
    if (!lora_) {
      spi_bus_free(sdcard_spi_num);
      expansion_spi_bus_initialized_ = false;
    }
    sdcard_ = nullptr;
    return false;
  }

  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard_);

  return true;
}
