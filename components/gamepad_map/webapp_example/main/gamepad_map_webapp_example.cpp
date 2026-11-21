// USB <-> gamepad map web console example.
//
// Serves an espp::GamepadProfiles store (default map + per-controller-type
// profiles in a LittleFS file) to the hosted gamepad map console web app over
// USB, on the vendor (WebUSB) OR CDC (Web Serial) interface, through
// espp::GamepadMapService (protocol: detail/gamepad_map_protocol.hpp,
// dispatcher module 11). The console edits the maps and shows the input before
// and after mapping, live.
//
// There is no real controller here: a demo controller task generates input
// (sticks circling, triggers ramping, buttons cycling) and reads one real
// button (BOOT, CONFIG_EXAMPLE_BUTTON_GPIO) as its South button, so "learn"
// can be tried with a press. In a real application the input comes from a
// decoder (hid_rp::GamepadDecoder + to_gamepad_state()) and the mapped state
// goes on to a USB controller, an emulator, ...
//
// The system console / logs go to UART0.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "driver/gpio.h"

#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "file_system.hpp"
#include "gamepad_map.hpp"
#include "gamepad_map_service.hpp"
#include "gamepad_profiles.hpp"
#include "gamepad_state_formatters.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_service.hpp"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Gamepad Map Console", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB<->gamepad map console example");

  // --- Standard USB services' engines -----------------------------------------
  // Every espp USB example serves system info + reboot (module 7), the heap /
  // task monitor (8), OTA (0) and core dump (4) next to its own protocol, so
  // the hosted system / OTA / coredump consoles and the Device Hub work
  // against it.
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

  // --- The profile store --------------------------------------------------------
  auto &fs = espp::FileSystem::get();
  espp::GamepadProfiles profiles({
      .file_path = fs.get_root_path() / "gamepad_profiles.bin",
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // The demo controller's identity: it gets its own profile when one is saved
  // for this VID:PID from the console.
  static constexpr uint16_t demo_vid = 0x1209;
  static constexpr uint16_t demo_pid = 0x0001;
  static const std::string demo_name = "Demo controller";

  // The map the input task applies; swapped (under the mutex) when the console
  // changes the demo controller's map.
  std::mutex map_mutex;
  espp::GamepadMap active_map;

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol ------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp Gamepad Map Console";
#if CONFIG_EXAMPLE_USB_PORT_FULL_SPEED
  usb_cfg.port = 0;
#elif CONFIG_EXAMPLE_USB_PORT_HIGH_SPEED
  usb_cfg.port = 1;
#endif
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp Gamepad Map (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/gamepad_map_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp Gamepad Map (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  // Each transport's `send` is called from the dispatcher worker (replies) and
  // the service's stream task (INPUT events); UsbDevice's write_*() do not
  // serialize writers, so guard them with a mutex to keep frames atomic.
  std::mutex tx_mutex;
  auto vendor_send = [&](std::span<const uint8_t> f) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    if (!usb.write_vendor(f))
      logger.warn_rate_limited("dropped a {}-byte vendor frame (TX backpressure)", f.size());
  };
  auto cdc_send = [&](std::span<const uint8_t> f) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    if (!usb.write_cdc(f))
      logger.warn_rate_limited("dropped a {}-byte CDC frame (TX backpressure)", f.size());
  };

  //! [gamepad_map_webapp_example]
  // One GamepadMapService per transport, both serving the one profile store.
  // When the console changes the map the connected controller uses, the
  // service hands it over: swap the map the input path applies.
  auto on_map_changed = [&](const espp::GamepadMap &map) {
    std::lock_guard<std::mutex> lock(map_mutex);
    active_map = map;
    logger.info("Map changed from the console: {} rule(s)", map.rules.size());
  };
  espp::GamepadMapService vendor_map({.send = vendor_send,
                                      .profiles = &profiles,
                                      .on_active_map_changed = on_map_changed,
                                      .task_config = {.name = "gpmap_v", .stack_size_bytes = 4096},
                                      .log_level = espp::Logger::Verbosity::INFO});
  espp::GamepadMapService cdc_map({.send = cdc_send,
                                   .profiles = &profiles,
                                   .on_active_map_changed = on_map_changed,
                                   .task_config = {.name = "gpmap_c", .stack_size_bytes = 4096},
                                   .log_level = espp::Logger::Verbosity::INFO});

  // A controller connected: tell every service, and start from its map.
  {
    std::lock_guard<std::mutex> lock(map_mutex);
    active_map = vendor_map.set_controller(demo_vid, demo_pid, demo_name);
    cdc_map.set_controller(demo_vid, demo_pid, demo_name);
  }
  //! [gamepad_map_webapp_example]

  // The standard services (system 7 / monitor 8 / OTA 0 / core dump 4), one per
  // transport through the same mutex-guarded senders.
  auto reboot_request = [&](espp::SystemService::RebootKind kind) {
    logger.warn("Host requested a {}; allowing it",
                kind == espp::SystemService::RebootKind::Bootloader ? "reboot into the bootloader"
                                                                    : "reboot");
    return true;
  };
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
  espp::OtaService vendor_ota(ota,
                              {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::OtaService cdc_ota(ota, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService vendor_coredump(
      core_dump, {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService cdc_coredump(
      core_dump, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});

  // One bounded RX queue + worker task per transport, so request handling
  // (which saves to flash) never runs in the TinyUSB callback context. On an RX
  // overflow the OTA service aborts a transfer it owned.
  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "gpmap_vendor", .stack_size_bytes = 16384}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send,
       .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
       .task_config = {.name = "gpmap_cdc", .stack_size_bytes = 16384}});
  vendor_link.register_module(vendor_map); // module 11 + its discovery metadata
  cdc_link.register_module(cdc_map);
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

  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  // --- The demo controller --------------------------------------------------------
  const int button_gpio = CONFIG_EXAMPLE_BUTTON_GPIO;
  if (button_gpio >= 0) {
    const gpio_config_t io{.pin_bit_mask = 1ULL << button_gpio,
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io);
  }
  using B = espp::GamepadState::Button;
  using A = espp::GamepadState::Axis;
  const auto start = std::chrono::steady_clock::now();
  espp::GamepadState last_mapped;
  espp::Task input_task({
      .callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
        // a 100 Hz "controller": the left stick circles every 4 s, the right
        // stick sweeps X, the triggers ramp, one face / shoulder button at a
        // time cycles every half second, the real button is South
        const float t =
            std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        espp::GamepadState raw;
        raw.set(A::LeftX, static_cast<int32_t>(std::cos(t * 1.5708f) * 30000));
        raw.set(A::LeftY, static_cast<int32_t>(std::sin(t * 1.5708f) * 30000));
        raw.set(A::RightX, static_cast<int32_t>(std::sin(t * 0.5f) * 32767));
        raw.set(A::LeftTrigger, static_cast<int32_t>((std::sin(t) * 0.5f + 0.5f) * 32767));
        raw.set(A::RightTrigger, static_cast<int32_t>((std::cos(t) * 0.5f + 0.5f) * 32767));
        static constexpr B cycle[] = {B::East, B::West, B::North, B::L1, B::R1, B::Start};
        raw.set(cycle[static_cast<int>(t * 2) % std::size(cycle)], true);
        if (button_gpio >= 0)
          raw.set(B::South, gpio_get_level(static_cast<gpio_num_t>(button_gpio)) == 0);

        espp::GamepadState mapped;
        {
          std::lock_guard<std::mutex> lock(map_mutex);
          active_map.apply(raw, mapped);
        }
        // the mapped state would go on to a USB controller / an emulator here
        vendor_map.publish_input(raw, mapped);
        cdc_map.publish_input(raw, mapped);
        if (mapped.buttons != last_mapped.buttons)
          logger.debug("mapped: {}", mapped);
        last_mapped = mapped;

        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 10ms);
        return false; // keep running
      },
      .task_config = {.name = "demo controller", .stack_size_bytes = 4096},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  input_task.start();

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {} — no host transport available",
                 usb_ec.message());
  else
    logger.info("Gamepad map console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
