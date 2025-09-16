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

  esp_err_t ret;
  // Options for mounting the filesystem. If format_if_mount_failed is set to
  // true, SD card will be partitioned and formatted in case when mounting
  // fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  spi_bus_config_t bus_cfg;
  memset(&bus_cfg, 0, sizeof(bus_cfg));
  bus_cfg.mosi_io_num = sdcard_mosi;
  bus_cfg.miso_io_num = sdcard_miso;
  bus_cfg.sclk_io_num = sdcard_clk;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = SPI_MAX_TRANSFER_BYTES;
  ret = spi_bus_initialize(sdcard_spi_num, &bus_cfg,
                           SDSPI_DEFAULT_DMA); // SPI_DMA_CH_AUTO); // SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize bus.");
    return false;
  }

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = sdcard_spi_num; // Use SPI1 host
  // host.max_freq_khz = 20 * 1000;

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  spi_host_device_t host_id = (spi_host_device_t)host.slot;
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sdcard_cs;
  slot_config.host_id = host_id;

  logger_.debug("Mounting filesystem");
  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  // // Use settings defined above to initialize SD card and mount FAT filesystem.
  // // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // // Please check its source code and implement error recovery when developing
  // // production applications.
  // logger_.debug("Using SDMMC peripheral");

  // // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  // sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  // host.max_freq_khz = SDMMC_FREQ_DEFAULT;

  // // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  // sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  // slot_config.clk = sdcard_clk;
  // slot_config.cmd = sdcard_cmd;
  // slot_config.d0 = sdcard_d0;
  // slot_config.d1 = sdcard_d1;
  // slot_config.d2 = sdcard_d2;
  // slot_config.d3 = sdcard_d3;

  // logger_.debug("Mounting filesystem");
  // ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. ");
      return false;
    } else {
      logger_.error("Failed to initialize the card ({}). "
                    "Make sure SD card lines have pull-up resistors in place.",
                    esp_err_to_name(ret));
      return false;
    }
    return false;
  }

  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard_);

  return true;
}
