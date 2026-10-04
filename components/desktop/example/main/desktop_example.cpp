#include <chrono>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "sdkconfig.h"

#include "console_capture.hpp"
#include "coredump.hpp"
#include "coredump_service.hpp"
#include "desktop.hpp"
#include "desktop_service.hpp"
#include "dispatcher_worker.hpp"
#include "file_system.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "nvs.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_info.hpp"
#include "system_service.hpp"
#include "usb_device.hpp"

#include "about_app.hpp"
#include "counter_app.hpp"
#include "files_app.hpp"
#include "log_viewer_app.hpp"
#include "settings_app.hpp"
#include "system_monitor_app.hpp"
#include "task_manager_app.hpp"

using namespace std::chrono_literals;

// Desktop over USB example.
//
// A browser-rendered windowed desktop: this firmware registers apps and
// describes their windows / widgets through espp::Desktop; the hosted desktop
// web app (components/desktop/web/desktop.html) draws and operates them over
// the USB vendor (WebUSB) or CDC (Web Serial) interface, routed by one
// espp::DispatcherWorker per transport next to the standard espp USB services:
//   - espp::DesktopService  (module 9, `espp.desktop`):  this desktop
//   - espp::SystemService   (module 7, `espp.system`):   device info, reboot
//   - espp::MonitorService  (module 8, `espp.monitor`):  heap / task stats
//   - espp::OtaService      (module 0, `espp.ota`):      firmware update
//   - espp::CoreDumpService (module 4, `espp.coredump`): last-crash report
// Apps: Counter, About, System Monitor, Task Manager, Log Viewer, Files (+
// Editor), Settings (main/apps/*.hpp; Counter is the API reference).

extern "C" void app_main(void) {
  //! [console_capture]
  // First thing: tee stdout / stderr into a ring the Log Viewer app reads, so
  // the boot log is captured too. The UART console keeps working.
#if CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE
  {
    std::error_code ec;
    espp::ConsoleCapture::install(
        {.capacity_bytes = CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE_BYTES, .tee_to_console = true}, ec);
  }
#endif
  //! [console_capture]

  espp::Logger logger({.tag = "Desktop Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting desktop example");

  // NVS (Counter / Settings) and the LittleFS partition (Files).
  {
    std::error_code ec;
    espp::Nvs nvs;
    nvs.init(ec);
    if (ec)
      logger.error("NVS init failed: {}", ec.message());
  }
  auto &fs = espp::FileSystem::get();
  logger.info("LittleFS at {}: {} / {} KiB used", espp::FileSystem::get_mount_point(),
              fs.get_used_space() / 1024, fs.get_total_space() / 1024);

  // The OTA and core-dump engines, shared by the per-transport services below
  // (exactly as in the system example).
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

  //! [desktop_example]
  // The desktop: one per device. Apps register with it; every app callback
  // runs on its task.
  const auto sysinfo = espp::SystemInfo::collect();
  espp::Desktop desktop(
      {.device_name = "espp Desktop",
       .firmware = fmt::format("{} {}", sysinfo.project_name, sysinfo.app_version),
       .task_config = {.name = "desktop", .stack_size_bytes = 10 * 1024},
       .log_level = espp::Logger::Verbosity::INFO});
  register_counter_app(desktop);
  register_about_app(desktop);
  register_system_monitor_app(desktop);
  register_task_manager_app(desktop);
  register_log_viewer_app(desktop);
  register_files_app(desktop);
  register_settings_app(desktop, "espp Desktop");
  desktop_example::apply_saved_settings(desktop);

  // USB composite device: a vendor/WebUSB function and a CDC function, both
  // carrying the same framed protocol.
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = 0x0d38; // distinct from the espp default so the webapp filter is specific
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp Desktop";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp Desktop (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp Desktop (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/desktop.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // One send function per transport, serialized by one mutex each, so the
  // services' frames never interleave. write_vendor / write_cdc are
  // all-or-nothing: they wait (bounded, 250 ms) for FIFO room for the WHOLE
  // frame and never queue a partial one, and return false when the host did
  // not drain in time (unplugged, or the page is not reading) -- the desktop
  // then flags that transport as needing a resync, and the browser resyncs
  // with GET_DESKTOP when it reconnects.
  std::mutex vendor_tx_mutex, cdc_tx_mutex;
  auto vendor_send = [&](std::span<const uint8_t> frame) {
    std::lock_guard<std::mutex> lock(vendor_tx_mutex);
    return usb.write_vendor(frame);
  };
  auto cdc_send = [&](std::span<const uint8_t> frame) {
    std::lock_guard<std::mutex> lock(cdc_tx_mutex);
    return usb.write_cdc(frame);
  };

  // One service instance per transport (they are cheap; each replies on its
  // own stream). Declared BEFORE the workers that call into them.
  espp::DesktopService vendor_desktop(
      desktop, {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::DesktopService cdc_desktop(desktop,
                                   {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::SystemService vendor_system({.send = vendor_send});
  espp::SystemService cdc_system({.send = cdc_send});
  espp::MonitorService vendor_monitor(
      {.send = vendor_send, .task_config = {.name = "monitor_v", .stack_size_bytes = 6 * 1024}});
  espp::MonitorService cdc_monitor(
      {.send = cdc_send, .task_config = {.name = "monitor_c", .stack_size_bytes = 6 * 1024}});
  espp::OtaService vendor_ota(ota, {.send = vendor_send});
  espp::OtaService cdc_ota(ota, {.send = cdc_send});
  espp::CoreDumpService vendor_coredump(core_dump, {.send = vendor_send});
  espp::CoreDumpService cdc_coredump(core_dump, {.send = cdc_send});

  // One DispatcherWorker per byte stream: a bounded receive queue + worker
  // task feeding its Dispatcher, so the services never run on the TinyUSB
  // task. Registering a service routes its module to it and advertises it for
  // discovery; serve_discovery() answers the hub's ListModules query.
  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "desktop_rx_vendor", .stack_size_bytes = 8192}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send,
       .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
       .task_config = {.name = "desktop_rx_cdc", .stack_size_bytes = 8192}});
  for (auto *link : {&vendor_link, &cdc_link}) {
    const bool v = link == &vendor_link;
    link->register_module(v ? vendor_desktop : cdc_desktop);
    link->register_module(v ? vendor_system : cdc_system);
    link->register_module(v ? vendor_monitor : cdc_monitor);
    link->register_module(v ? vendor_ota : cdc_ota);
    link->register_module(v ? vendor_coredump : cdc_coredump);
    link->serve_discovery(usb_cfg.product);
  }
  //! [desktop_example]

  // RX plumbing: the TinyUSB callbacks just queue the bytes for the workers.
  // When the host unplugs, stop streaming to it: the next GET_DESKTOP re-attaches.
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });
  // Both run on the TinyUSB task: detach() only flips a flag, *_write_clear
  // touches only the TX FIFOs, and request_reset() defers the queue + parser
  // reset to each worker (so a half frame of the old session can never eat
  // the first bytes of the next one).
  usb.set_unmount_callback([&]() {
    vendor_desktop.detach();
    cdc_desktop.detach();
    usb.vendor_write_clear();
    usb.cdc_write_clear();
    vendor_link.request_reset();
    cdc_link.request_reset();
  });
  usb.set_mount_callback([&]() {
    usb.vendor_write_clear();
    usb.cdc_write_clear();
    vendor_link.request_reset();
    cdc_link.request_reset();
  });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {}", usb_ec.message());
  else
    logger.info("Ready. Connect the native USB port and open the desktop "
                "(components/desktop/web/desktop.html or https://{})",
                vendor.landing_page_url);

  // Idle; all work happens in the desktop task and the dispatcher workers.
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
