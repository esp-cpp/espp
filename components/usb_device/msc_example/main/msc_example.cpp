// USB mass storage (MSC) example.
//
// Exposes a FAT partition in the ESP32-S3's flash as a USB drive using
// espp::UsbDevice's MSC function, and shows the ownership model: the
// application reads and writes files through the VFS while it owns the medium,
// the host gets the drive when it mounts the device, and the application gets
// it back when the host ejects it (or the cable is unplugged).
//
// On boot the app writes a boot counter and a README to the volume. Plug the
// native USB port into a PC: the drive appears with those files. Add or edit
// files, then eject the drive: the app lists what it now sees, including the
// host's changes. An SD card works the same way (see the README).

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "logger.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;
using MscOwner = espp::UsbDevice::MscOwner;
using MscEvent = espp::UsbDevice::MscEvent;

static constexpr const char *kBasePath = "/msc";

static void list_files(espp::Logger &logger) {
  std::error_code ec;
  logger.info("Files on the volume:");
  for (const auto &entry : std::filesystem::directory_iterator(kBasePath, ec)) {
    std::error_code size_ec;
    const auto size = entry.is_regular_file(size_ec) ? entry.file_size(size_ec) : 0;
    logger.info("  {}{} ({} bytes)", entry.path().filename().string(),
                entry.is_directory(size_ec) ? "/" : "", size);
  }
  if (ec)
    logger.error("could not list {}: {}", kBasePath, ec.message());
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

  if (std::ofstream readme(std::string(kBasePath) + "/README.txt", std::ios::trunc); readme) {
    readme << "Written by the espp usb_device msc_example.\n"
           << "Add files here, eject the drive, and the device lists them.\n";
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

  espp::UsbDevice::MscMedium flash;
  flash.type = espp::UsbDevice::MscMedium::Type::FlashPartition;
  flash.partition_label = "storage"; // `data, fat` partition in partitions.csv
  flash.base_path = kBasePath;
  flash.volume_label = "ESPP MSC"; // the name the host shows for the drive
  // Safe here: this is the only FAT volume on the device (see the header docs).
  flash.format_if_unformatted = true;
  flash.initial_owner = MscOwner::App; // write the boot files before a host takes it

  espp::UsbDevice::MscFunction msc;
  msc.interface_name = "espp MSC Example";
  msc.media = {flash};
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

  // The app owns the medium until a host mounts the device: write to it now.
  if (usb.msc_owner(0) == MscOwner::App) {
    write_boot_files(logger);
    list_files(logger);
  }

  logger.info("Ready. Connect the native USB port to a PC; eject the drive to hand it back.");
  while (true) {
    std::this_thread::sleep_for(500ms);
    if (app_regained.exchange(false) && usb.msc_owner(0) == MscOwner::App)
      list_files(logger); // shows whatever the host added / changed
  }
}
