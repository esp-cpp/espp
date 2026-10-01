#include <chrono>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "sdkconfig.h"

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

// System info + control over USB example.
//
// Exposes the standard espp USB services on the USB vendor (WebUSB) and CDC
// (Web Serial) interfaces, routed by one espp::DispatcherWorker per transport:
//   - espp::SystemService   (module 7, `espp.system`):   device identity /
//     status, reboot, reboot into the ROM bootloader (download mode)
//   - espp::MonitorService  (module 8, `espp.monitor`):  heap regions and the
//     task table, on request or streamed
//   - espp::OtaService      (module 0, `espp.ota`):      firmware update
//   - espp::CoreDumpService (module 4, `espp.coredump`): last-crash report +
//     core dump download / erase
// The hosted system console (components/system/web/system_console.html) talks
// to the first two, the OTA and coredump consoles to the others, and the
// Device Hub lists all of them through discovery. Every espp USB example
// serves this same set.

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "System Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting system info + control example");

  //! [system_example]
  // Boot banner: everything SystemInfo knows, in one string.
  logger.info("System:\n{}", espp::SystemInfo::to_string());
  logger.info("Reboot into the bootloader is {} on this chip",
              espp::SystemControl::bootloader_reboot_supported() ? "supported" : "NOT supported");

  // The OTA and core-dump engines, shared by the per-transport services below.
  // A panic core-dumps to the `coredump` partition (sdkconfig / partitions.csv)
  // and is reported here on the next boot; the coredump console downloads /
  // erases it. OTA alternates between ota_0 / ota_1 with host-driven rollback
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

  // USB composite device: a vendor/WebUSB function and a CDC function, both
  // carrying the same framed protocol.
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = 0x0d37; // distinct from the espp default so the webapp filter is specific
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp System";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp System (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp System (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/system_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // Replies go back on the stream the request came in on: one send function
  // per transport. Both services share each transport and only serialize
  // their OWN frames (a streamed monitor event and a system reply come from
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
  // defer) while, say, a motor is running or a file is being written.
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

  // One DispatcherWorker per byte stream: a bounded receive queue + worker
  // task feeding its Dispatcher, so the services never run on the TinyUSB
  // task. Registering a service routes its module to it and advertises it for
  // discovery; serve_discovery() answers the hub's ListModules query.
  // On an RX overflow the OTA service aborts a transfer it owned and tells the
  // host (an OTA image with dropped bytes is unusable).
  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "system_rx_vendor", .stack_size_bytes = 8192}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send,
       .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
       .task_config = {.name = "system_rx_cdc", .stack_size_bytes = 8192}});
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
  //! [system_example]

  // RX plumbing: the TinyUSB callbacks just queue the bytes for the workers.
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {}", usb_ec.message());
  else
    logger.info("Ready. Connect the native USB port and open the system console "
                "(components/system/web/system_console.html or https://{}); the OTA and "
                "coredump consoles and the Device Hub work too",
                vendor.landing_page_url);

  // Idle; all work happens in the dispatcher workers and the monitor stream task.
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
