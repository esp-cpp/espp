// USB mass storage (MSC) example.
//
// Exposes a FAT partition in the ESP32-S3's flash -- or an SD card, picked in
// menuconfig -- as a USB drive using espp::UsbDevice's MSC function, and shows
// the ownership model: the application reads and writes files through the VFS
// while it owns the medium, the host gets the drive when it mounts the device,
// and the application gets it back when the host ejects it (or the cable is
// unplugged).
//
// On boot the app writes a boot counter and a README to the volume. Plug the
// native USB port into a PC: the drive appears with those files. Add or edit
// files, then eject the drive: the app lists what it now sees, including the
// host's changes.
//
// The medium (menuconfig "MSC Example Configuration"):
// - a FAT partition in flash (default),
// - an SD card probed with espp::SdCard (no mount) on configurable SDMMC pins,
// - the SD card of the LilyGo T-Dongle-S3 through its BSP: initialize_sdcard()
//   mounts it, sdcard_component()->unmount() releases the volume, sdcard() is
//   handed to the MSC function -- the pattern for any espp BSP with a uSD slot.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "logger.hpp"
#include "sdcard.hpp"
#include "usb_device.hpp"

#if CONFIG_MSC_EXAMPLE_MEDIUM_BSP_T_DONGLE_S3
#include "t-dongle-s3.hpp"
#endif

using namespace std::chrono_literals;
using MscOwner = espp::UsbDevice::MscOwner;
using MscEvent = espp::UsbDevice::MscEvent;

static constexpr const char *kBasePath = "/msc";

static void list_files(espp::Logger &logger) {
  std::error_code ec;
  size_t garbled = 0;
  logger.info("Files on the volume:");
  for (const auto &entry : std::filesystem::directory_iterator(kBasePath, ec)) {
    std::error_code size_ec;
    const auto size = entry.is_regular_file(size_ec) ? entry.file_size(size_ec) : 0;
    // A damaged directory (e.g. an erased flash sector) yields names full of 0xFF
    // bytes: print them safely and count them instead of sending raw bytes to the
    // console.
    std::string name = entry.path().filename().string();
    bool printable = true;
    for (auto &c : name) {
      if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) >= 0x7F) {
        c = '?';
        printable = false;
      }
    }
    if (!printable) {
      ++garbled;
      continue;
    }
    logger.info("  {}{} ({} bytes)", name, entry.is_directory(size_ec) ? "/" : "", size);
  }
  if (ec)
    logger.error("could not list {}: {}", kBasePath, ec.message());
  if (garbled > 0)
    logger.warn("{} directory entries are unreadable: the volume is damaged. Erase the storage "
                "partition (or reformat the drive from the host) to start clean.",
                garbled);
}

static void write_boot_files(espp::Logger &logger) {
  const std::string counter_path = std::string(kBasePath) + "/boots.txt";
  int boots = 0;
  if (std::ifstream in(counter_path); in)
    in >> boots;
  ++boots;
  if (std::ofstream out(counter_path, std::ios::trunc); out)
    out << boots << "\n";
  else
    logger.error("could not write {}", counter_path);

  // Create the README only once: rewriting it every boot would discard edits the
  // host made to it.
  const std::string readme_path = std::string(kBasePath) + "/README.txt";
  std::error_code exists_ec;
  if (!std::filesystem::exists(readme_path, exists_ec)) {
    if (std::ofstream readme(readme_path); readme) {
      readme << "Written by the espp usb_device msc_example.\n"
             << "Add files here, eject the drive, and the device lists them.\n";
    }
  }
  logger.info("boot #{} recorded on the volume", boots);
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MSC", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB mass storage example");

  // Hand-overs happen in the TinyUSB task: only note them here, act on them in
  // the main loop.
  std::atomic<bool> app_regained{false};

  //! [msc_example]
  espp::UsbDevice::Config cfg;
  cfg.product = "espp MSC Example";
  cfg.log_level = espp::Logger::Verbosity::INFO;
  // Stay invisible to the host until the boot files are written: with
  // auto_handover a host that enumerates takes the medium immediately, which
  // would unmount /msc in the middle of the application's writes.
  cfg.connect_on_initialize = false;

  espp::UsbDevice::MscMedium medium;
  medium.base_path = kBasePath;
  medium.initial_owner = MscOwner::App; // write the boot files before a host takes it
#if CONFIG_MSC_EXAMPLE_MEDIUM_FLASH
  medium.type = espp::UsbDevice::MscMedium::Type::FlashPartition;
  medium.partition_label = "storage"; // `data, fat` partition in partitions.csv
  medium.volume_label = "ESPP MSC";   // the name the host shows for the drive
  // Safe here: this is the only FAT volume on the device (see the header docs).
  medium.format_if_unformatted = true;
#else
  // An SD card: initialized by the firmware, but NOT mounted -- the MSC function
  // mounts it at base_path while the app owns it. Never formatted by the example
  // (a card with no filesystem raises MscEvent::FormatRequired; format it on the
  // PC).
  medium.type = espp::UsbDevice::MscMedium::Type::SdCard;
#if CONFIG_MSC_EXAMPLE_MEDIUM_SDCARD
  espp::SdCard::SdmmcConfig sdmmc;
#if CONFIG_MSC_EXAMPLE_SDMMC_BUS_WIDTH_1
  sdmmc.bus_width = 1;
#else
  sdmmc.bus_width = 4;
  sdmmc.d1 = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_D1);
  sdmmc.d2 = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_D2);
  sdmmc.d3 = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_D3);
#endif
  sdmmc.clk = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_CLK);
  sdmmc.cmd = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_CMD);
  sdmmc.d0 = static_cast<gpio_num_t>(CONFIG_MSC_EXAMPLE_SDMMC_D0);
  sdmmc.ldo_channel = CONFIG_MSC_EXAMPLE_SDMMC_LDO_CHANNEL;
  espp::SdCard::Config sd_config;
  sd_config.interface = sdmmc;
  sd_config.mount_on_initialize = false; // probe only; the MSC function mounts it
  sd_config.log_level = espp::Logger::Verbosity::INFO;
  // Declared before the UsbDevice below so it outlives it.
  static espp::SdCard sdcard(sd_config);
  if (std::error_code sd_ec; !sdcard.initialize(sd_ec)) {
    logger.error("Failed to initialize the SD card: {}", sd_ec.message());
    return;
  }
  sdcard.print_info(stdout);
  medium.sd_card = sdcard.card();
#else // CONFIG_MSC_EXAMPLE_MEDIUM_BSP_T_DONGLE_S3
  auto &board = espp::TDongleS3::get();
  // The BSP mounts the volume at its own mount point; release it so the MSC
  // function can own the card (it re-mounts it at base_path for the app).
  if (!board.initialize_sdcard({})) {
    logger.error("Failed to initialize the SD card (is a FAT-formatted card inserted?)");
    return;
  }
  if (std::error_code sd_ec; !board.sdcard_component()->unmount(sd_ec)) {
    logger.error("Failed to unmount the SD card: {}", sd_ec.message());
    return;
  }
  medium.sd_card = board.sdcard();
#endif
#endif

  espp::UsbDevice::MscFunction msc;
  msc.interface_name = "espp MSC Example";
  msc.media = {medium};
  msc.auto_handover = true; // host takes the drive on mount, app gets it back on eject
  msc.on_event = [&](size_t lun, MscEvent event, MscOwner owner) {
    if (event == MscEvent::OwnerChanged) {
      logger.info("medium {} now owned by the {}", lun, owner == MscOwner::App ? "app" : "host");
      if (owner == MscOwner::App)
        app_regained = true;
    } else if (event == MscEvent::FormatRequired) {
      logger.warn("medium {} has no filesystem", lun);
    }
  };
  cfg.msc = msc;

  espp::UsbDevice usb(cfg);
  std::error_code ec;
  if (!usb.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }
  //! [msc_example]

  if (const auto capacity = usb.msc_capacity(0))
    logger.info("volume: {} sectors x {} bytes = {} KiB", capacity->sector_count,
                capacity->sector_size, capacity->bytes() / 1024);

  // initialize() handed the medium to the app, which raised OwnerChanged(App):
  // that is not a return from the host, so do not list the files twice.
  app_regained = false;

  // Still detached, so no host can take the medium: write to it now, then attach.
  if (usb.msc_owner(0) == MscOwner::App) {
    write_boot_files(logger);
    list_files(logger);
  }
  usb.connect();

  logger.info("Ready. Connect the native USB port to a PC; eject the drive to hand it back.");
  while (true) {
    std::this_thread::sleep_for(500ms);
    if (app_regained.exchange(false) && usb.msc_owner(0) == MscOwner::App)
      list_files(logger); // shows whatever the host added / changed
  }
}
