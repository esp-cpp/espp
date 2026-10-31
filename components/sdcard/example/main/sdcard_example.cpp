// SD card example.
//
// Brings up a microSD card with espp::SdCard over SDMMC (SDIO) or SPI, then
// shows the two-step model the component adds on top of ESP-IDF: the card is
// probed once, and its FAT volume can be mounted and unmounted any number of
// times while the card stays initialized (which is what lets the same card be
// handed to a USB host, see the README).
//
// The card must already carry a FAT filesystem unless
// CONFIG_SDCARD_EXAMPLE_FORMAT_IF_MOUNT_FAILED is enabled.

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "sdkconfig.h"

#include "logger.hpp"
#include "sdcard.hpp"

using namespace std::chrono_literals;

static void list_files(espp::Logger &logger, const std::string &path) {
  std::error_code ec;
  logger.info("Files in {}:", path);
  size_t count = 0;
  for (const auto &entry : std::filesystem::directory_iterator(path, ec)) {
    std::error_code size_ec;
    const bool dir = entry.is_directory(size_ec);
    const auto size = dir ? 0 : entry.file_size(size_ec);
    logger.info("  {}{} ({} bytes)", entry.path().filename().string(), dir ? "/" : "", size);
    if (++count >= 20) {
      logger.info("  ...");
      break;
    }
  }
  if (ec)
    logger.error("could not list {}: {}", path, ec.message());
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SD Card", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting SD card example");

  //! [sdcard example]
  espp::SdCard::Config config;
  config.mount_point = "/sdcard";
#ifdef CONFIG_SDCARD_EXAMPLE_FORMAT_IF_MOUNT_FAILED
  config.format_if_mount_failed = true; // erases a card with no FAT filesystem
#endif
  config.log_level = espp::Logger::Verbosity::INFO;
#if CONFIG_SDCARD_EXAMPLE_INTERFACE_SDMMC
  espp::SdCard::SdmmcConfig sdmmc;
#ifdef CONFIG_SDCARD_EXAMPLE_SDMMC_BUS_WIDTH_1
  sdmmc.bus_width = 1;
#else
  sdmmc.bus_width = 4;
#endif
  sdmmc.clk = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_CLK);
  sdmmc.cmd = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_CMD);
  sdmmc.d0 = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_D0);
  sdmmc.d1 = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_D1);
  sdmmc.d2 = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_D2);
  sdmmc.d3 = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SDMMC_D3);
  sdmmc.ldo_channel = CONFIG_SDCARD_EXAMPLE_SDMMC_LDO_CHANNEL;
  config.interface = sdmmc;
#else
  espp::SdCard::SpiConfig spi;
  spi.host = SPI2_HOST;
  spi.initialize_bus = true; // nothing else is on this bus
  spi.mosi = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SPI_MOSI);
  spi.miso = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SPI_MISO);
  spi.sclk = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SPI_SCLK);
  spi.cs = static_cast<gpio_num_t>(CONFIG_SDCARD_EXAMPLE_SPI_CS);
  config.interface = spi;
#endif

  espp::SdCard sdcard(config);
  std::error_code ec;
  if (!sdcard.initialize(ec)) { // probes the card and mounts it at /sdcard
    logger.error("SD card initialization failed: {}", ec.message());
    return;
  }
  //! [sdcard example]

  if (const auto card = sdcard.card_info()) {
    logger.info("Card '{}': {} MiB, {} kHz, {}-bit, {}", card->name,
                card->capacity_bytes / (1024 * 1024), card->frequency_khz, card->bus_width,
                card->high_capacity ? "SDHC/SDXC" : "SDSC");
  }
  if (const auto volume = sdcard.volume_info()) {
    logger.info("Volume: {} MiB total, {} MiB free", volume->total_bytes / (1024 * 1024),
                volume->free_bytes / (1024 * 1024));
  }

  // Ordinary file I/O while the volume is mounted.
  const std::string counter_path = sdcard.mount_point() + "/boots.txt";
  int boots = 0;
  if (std::ifstream in(counter_path); in)
    in >> boots;
  ++boots;
  if (std::ofstream out(counter_path, std::ios::trunc); out)
    out << boots << "\n";
  else
    logger.error("could not write {}", counter_path);
  logger.info("boot #{} recorded", boots);
  list_files(logger, sdcard.mount_point());

  // The card stays initialized while its volume is unmounted: this is the window
  // in which another owner (e.g. a USB host through espp::UsbDevice's MSC
  // function) may use sdcard.card() directly.
  if (!sdcard.unmount(ec))
    logger.error("unmount failed: {}", ec.message());
  logger.info("volume unmounted; the card is still initialized: {}", sdcard.is_initialized());
  std::this_thread::sleep_for(1s);
  if (!sdcard.mount(ec))
    logger.error("mount failed: {}", ec.message());
  else
    logger.info("volume mounted again; boots.txt still says {}", boots);

  logger.info("Done. The card stays mounted at {}.", sdcard.mount_point());
  while (true)
    std::this_thread::sleep_for(1s);
}
