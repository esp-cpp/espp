#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "sdkconfig.h"

#include "esp_heap_caps.h"

#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_control.hpp"
#include "system_info.hpp"
#include "system_service.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

// USB device reference example: one espp::UsbDevice, three interface classes.
//
//   - CDC-ACM (a serial port): the espp framed protocol, so the hosted web
//     consoles can talk to the device over Web Serial (and the espp_* CLIs
//     over the serial port)
//   - vendor-specific / WebUSB: the same framed protocol over bulk endpoints,
//     for WebUSB and the espp_* CLIs over libusb
//   - MSC (a USB drive): a wear-levelled FAT partition in flash, exposed to the
//     host as a removable drive
//
// Each framed link has its own espp::DispatcherWorker (a bounded receive queue
// + worker task feeding a Dispatcher), on which the standard espp USB services
// are registered and advertised for discovery:
//   - espp::SystemService   (module 7, `espp.system`):   device identity /
//     status, reboot, reboot into the ROM bootloader (download mode)
//   - espp::MonitorService  (module 8, `espp.monitor`):  heap regions and the
//     task table, on request or streamed
//   - espp::OtaService      (module 0, `espp.ota`):      firmware update
//   - espp::CoreDumpService (module 4, `espp.coredump`): last-crash report +
//     core dump download / erase
// The hosted system / OTA / coredump consoles and the Device Hub work against
// this device over either framed interface.

static constexpr const char *kMscBasePath = "/usb"; // where the app sees the FAT volume

// Write a small README onto the drive while the application owns it (before
// the host takes it), so the drive explains itself when mounted.
static void write_drive_readme(espp::Logger &logger) {
  const std::string path = std::string(kMscBasePath) + "/README.txt";
  // stdio, not iostreams: keeps the example's binary and heap footprint small
  if (FILE *probe = std::fopen(path.c_str(), "r"); probe != nullptr) {
    std::fclose(probe);
    logger.info("drive README already present");
    return;
  }
  FILE *readme = std::fopen(path.c_str(), "w");
  if (readme == nullptr) {
    logger.error("could not write {}", path);
    return;
  }
  static constexpr const char *kText =
      "espp USB Device example\n"
      "=======================\n"
      "This drive is a wear-levelled FAT partition in the ESP32-S3's flash,\n"
      "exposed over USB mass storage by espp::UsbDevice.\n"
      "The same device also presents a CDC serial port and a vendor/WebUSB\n"
      "interface carrying the espp framed protocol: system info / reboot,\n"
      "heap + task monitor, OTA update and core-dump download.\n"
      "Open https://esp-cpp.github.io/espp/apps/ to use them from a browser.\n";
  const bool ok = std::fputs(kText, readme) >= 0;
  std::fclose(readme);
  if (ok)
    logger.info("wrote {}", path);
  else
    logger.error("could not write {}", path);
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "USB Device", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB device example (CDC + vendor/WebUSB + MSC)");

  //! [usb_device_example]
  logger.info("System:\n{}", espp::SystemInfo::to_string());

  // The OTA and core-dump engines, shared by the per-transport services below.
  // A panic core-dumps to the `coredump` partition (partitions.csv) and is
  // reported here on the next boot; the coredump console downloads / erases
  // it. OTA alternates between ota_0 / ota_1 with host-driven rollback
  // confirmation, exactly as the ota example does.
  espp::CoreDump core_dump({.log_level = espp::Logger::Verbosity::INFO});
  const std::string crash_report = core_dump.format_report();
  if (crash_report.empty())
    logger.info("Clean boot history (reset reason: {})",
                espp::CoreDump::reset_reason_name(espp::CoreDump::reset_reason()));
  else
    logger.error("Previous abnormal reset:\n{}", crash_report);
  espp::Ota ota({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO});
  if (ota.is_pending_verify())
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update): it rolls back on "
                "the next reset unless the host confirms it (MARK_VALID from the OTA console)");

  // --- the composite USB device: CDC + vendor (WebUSB) + MSC ------------------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = 0x0d38; // distinct from the other espp examples so a host filter is specific
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp USB Device";
  usb_cfg.connect_on_initialize = false; // write the drive's README before the host can mount it
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp USB Device (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp USB Device (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/system_console.html";
  usb_cfg.vendor = vendor;
  // MSC: the `storage` FAT partition, owned by the application first (so it
  // can write the README), then handed to the host when it mounts the device
  // (auto_handover); ejecting the drive gives it back to the application.
  using MscOwner = espp::UsbDevice::MscOwner;
  using MscEvent = espp::UsbDevice::MscEvent;
  std::atomic<bool> drive_regained{false};
  espp::UsbDevice::MscMedium medium;
  medium.type = espp::UsbDevice::MscMedium::Type::FlashPartition;
  medium.partition_label = "storage"; // `data, fat` partition in partitions.csv
  medium.base_path = kMscBasePath;
  medium.volume_label = "ESPP USB";    // the name the host shows for the drive
  medium.format_if_unformatted = true; // the only FAT volume on this device
  medium.initial_owner = MscOwner::App;
  espp::UsbDevice::MscFunction msc;
  msc.interface_name = "espp USB Device (MSC)";
  msc.media = {medium};
  msc.auto_handover = true;
  msc.on_event = [&](size_t lun, MscEvent event, MscOwner owner) {
    if (event == MscEvent::OwnerChanged) {
      logger.info("drive {} now owned by the {}", lun, owner == MscOwner::App ? "app" : "host");
      if (owner == MscOwner::App)
        drive_regained = true;
    } else if (event == MscEvent::FormatRequired) {
      logger.warn("drive {} has no filesystem", lun);
    } else if (event == MscEvent::FormatFailed || event == MscEvent::OwnerChangeFailed) {
      logger.error("drive {}: {}", lun,
                   event == MscEvent::FormatFailed ? "format failed" : "hand-over failed");
    }
  };
  usb_cfg.msc = msc;
  espp::UsbDevice usb(usb_cfg);

  // Replies go back on the stream the request came in on: one send function
  // per transport. All services share each transport and only serialize their
  // OWN frames (a streamed monitor event and a system reply come from
  // different tasks), so every device->host write on a transport goes through
  // one application-level mutex; write_vendor / write_cdc are all-or-nothing
  // per call, so a frame is never truncated or interleaved.
  std::mutex vendor_tx_mutex, cdc_tx_mutex;
  auto vendor_send = [&](std::span<const uint8_t> frame) {
    std::lock_guard<std::mutex> lock(vendor_tx_mutex);
    usb.write_vendor(frame);
  };
  auto cdc_send = [&](std::span<const uint8_t> frame) {
    std::lock_guard<std::mutex> lock(cdc_tx_mutex);
    usb.write_cdc(frame);
  };

  // The application decides whether a reboot may happen right now: this demo
  // permits every request and logs it. A real application would refuse (or
  // defer) while, say, the host is writing to the drive.
  auto reboot_request = [&](espp::SystemService::RebootKind kind) {
    logger.warn("Host requested a {}; allowing it",
                kind == espp::SystemService::RebootKind::Bootloader ? "reboot into the bootloader"
                                                                    : "reboot");
    return true;
  };

  // One service instance per transport (they are cheap; each replies on its
  // own stream). Declared BEFORE the workers that call into them.
  espp::SystemService vendor_system({.send = vendor_send,
                                     .on_reboot_request = reboot_request,
                                     .log_level = espp::Logger::Verbosity::INFO});
  espp::SystemService cdc_system({.send = cdc_send,
                                  .on_reboot_request = reboot_request,
                                  .log_level = espp::Logger::Verbosity::INFO});
  espp::MonitorService vendor_monitor(
      {.send = vendor_send,
       .task_config = {.name = "monitor_v", .stack_size_bytes = 6 * 1024},
       .log_level = espp::Logger::Verbosity::INFO});
  espp::MonitorService cdc_monitor(
      {.send = cdc_send,
       .task_config = {.name = "monitor_c", .stack_size_bytes = 6 * 1024},
       .log_level = espp::Logger::Verbosity::INFO});
  // OTA and core dump: one service per transport over the shared engines (each
  // OtaService only touches the session IT began, so the two cannot interfere).
  espp::OtaService vendor_ota(ota,
                              {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::OtaService cdc_ota(ota, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService vendor_coredump(
      core_dump, {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService cdc_coredump(
      core_dump, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});

  // One DispatcherWorker per byte stream, so the services never run on the
  // TinyUSB task. Registering a service routes its module to it and advertises
  // it for discovery; serve_discovery() answers the hub's ListModules query.
  // On an RX overflow the OTA service aborts a transfer it owned and tells the
  // host (an OTA image with dropped bytes is unusable).
  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "usb_rx_vendor", .stack_size_bytes = 8192}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send,
       .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
       .task_config = {.name = "usb_rx_cdc", .stack_size_bytes = 8192}});
  vendor_link.register_module(vendor_system);
  vendor_link.register_module(vendor_monitor);
  vendor_link.register_module(vendor_ota);
  vendor_link.register_module(vendor_coredump);
  cdc_link.register_module(cdc_system);
  cdc_link.register_module(cdc_monitor);
  cdc_link.register_module(cdc_ota);
  cdc_link.register_module(cdc_coredump);
  vendor_link.serve_discovery(usb_cfg.product);
  cdc_link.serve_discovery(usb_cfg.product);

  // RX plumbing: the TinyUSB callbacks just queue the bytes for the workers.
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec)) {
    logger.error("Failed to initialize USB device: {}", usb_ec.message());
    return;
  }
  // The drive is ours until the host mounts the device: describe it, then
  // let the host in.
  if (const auto capacity = usb.msc_capacity(0))
    logger.info("drive: {} sectors x {} bytes = {} KiB", capacity->sector_count,
                capacity->sector_size, capacity->bytes() / 1024);
  if (usb.msc_owner(0) == MscOwner::App)
    write_drive_readme(logger);
  usb.connect();
  //! [usb_device_example]

  logger.info("Enumerated as VID 0x{:04x} PID 0x{:04x} '{}': CDC (framed protocol / Web Serial), "
              "vendor (framed protocol / WebUSB, landing page https://{}), MSC (drive '{}')",
              usb_cfg.vid, usb_cfg.pid, usb_cfg.product, vendor.landing_page_url,
              medium.volume_label);
  logger.info("Ready: open the system / OTA / coredump consoles or the Device Hub, or mount the "
              "drive");

  // Idle; all work happens in the dispatcher workers, the monitor stream tasks
  // and TinyUSB. A small heartbeat shows the device is alive and what the
  // drive is doing.
  while (true) {
    std::this_thread::sleep_for(10s);
    if (drive_regained.exchange(false))
      logger.info("the host ejected the drive; the application owns it again");
    logger.info("alive: free heap {} B (min {} B), drive owned by the {}",
                heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT),
                usb.msc_owner(0) == MscOwner::App ? "app" : "host");
  }
}
