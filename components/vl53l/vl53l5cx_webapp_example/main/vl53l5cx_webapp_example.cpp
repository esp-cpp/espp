// USB <-> VL53L5CX web console example.
//
// Runs the espp::Vl53l5cx multizone ToF driver on an ESP32-S3 / ESP32-P4 and
// exposes it to a browser over USB: the hosted VL53L5CX console web app
// connects on the vendor (WebUSB) OR CDC (Web Serial) interface, configures the
// sensor (resolution, frequency, integration time, sharpener, target order,
// ranging mode, power mode, motion indicator), starts / stops ranging and
// receives every ranging frame, which it shows as a live 3D view of the 4x4 /
// 8x8 zone distances. All I2C work happens on the device inside
// espp::Vl53l5cxService (protocol: vl53l5cx_protocol.hpp, dispatcher module
// 10); the web app only sends requests and decodes frames.
//
// Wiring: the sensor on I2C (SDA / SCL from the example configuration; the
// ESP32-P4 dev kits use SDA 7 / SCL 8), 3.3 V, GND; LPn tied high (or left to
// the breakout's pull-up). The system console/logs go to UART0.

#include <chrono>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_service.hpp"
#include "task.hpp"
#include "usb_device.hpp"
#include "vl53l5cx.hpp"
#include "vl53l5cx_service.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "VL53L5CX Console", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB<->VL53L5CX console example");

  // --- Standard USB services' engines -----------------------------------------
  // Every espp USB example serves system info + reboot (module 7), the heap /
  // task monitor (8), OTA (0) and core dump (4) next to its own protocol, so
  // the hosted system / OTA / coredump consoles and the Device Hub work
  // against it. A panic core-dumps to the `coredump` partition and is
  // reported on the next boot; OTA rollback confirmation is host-driven.
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

  // --- I2C + the sensor --------------------------------------------------------
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
  logger.info("Creating I2C on port {} with SDA {} and SCL {} at {} Hz", i2c_port, i2c_sda, i2c_scl,
              i2c_clock_speed);
  // The transaction timeout must cover the driver's longest transfers (512-byte
  // firmware chunks, ~1.4 KB result frames): derived from the clock
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .timeout_ms = espp::Vl53l5cx::i2c_timeout_ms(i2c_clock_speed),
                 .clk_speed = i2c_clock_speed});
  // The constructor reboots the sensor and uploads its firmware (about two
  // seconds at 400 kHz). A failure is not fatal here: the web app can retry
  // with INITIALIZE once the wiring is fixed. The callbacks take the address
  // from the driver, so SET_I2C_ADDRESS keeps working.
  espp::Vl53l5cx sensor(espp::Vl53l5cx::Config {
    .device_address = espp::Vl53l5cx::DEFAULT_ADDRESS,
#if CONFIG_EXAMPLE_VARIANT_VL53L7CX
    .variant = espp::Vl53l5cx::Variant::VL53L7CX,
#else
      .variant = espp::Vl53l5cx::Variant::VL53L5CX,
#endif
    .write = [&i2c](uint8_t address, const uint8_t *data,
                    size_t length) { return i2c.write(address, data, length); },
    .read = [&i2c](uint8_t address, uint8_t *data,
                   size_t length) { return i2c.read(address, data, length); },
    .log_level = espp::Logger::Verbosity::INFO,
  });
  // One sensor: every Vl53l5cx call -- from both services and from the poll
  // task below -- is serialized on one mutex.
  std::mutex sensor_mutex;
  if (!sensor.is_initialized()) {
    logger.error("The VL53L5CX did not initialize; check the wiring, then INITIALIZE from the "
                 "web app");
  } else {
    std::error_code ec;
    std::lock_guard<std::mutex> lock(sensor_mutex);
#if CONFIG_EXAMPLE_RESOLUTION_8X8
    const auto resolution = espp::Vl53l5cx::Resolution::ZONES_8X8;
#else
    const auto resolution = espp::Vl53l5cx::Resolution::ZONES_4X4;
#endif
    // resolution first (it bounds the ranging frequency), then the frame rate
    if (!sensor.set_resolution(resolution, ec))
      logger.error("Failed to set the resolution: {}", ec.message());
    else if (!sensor.set_ranging_frequency_hz(CONFIG_EXAMPLE_RANGING_FREQUENCY_HZ, ec))
      logger.error("Failed to set the ranging frequency: {}", ec.message());
#if CONFIG_EXAMPLE_START_RANGING_AT_BOOT
    else if (!sensor.start_ranging(ec))
      logger.error("Failed to start ranging: {}", ec.message());
    else
      logger.info("Ranging at {} Hz, {} zones", CONFIG_EXAMPLE_RANGING_FREQUENCY_HZ,
                  static_cast<int>(resolution));
#endif
  }

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol ------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp VL53L5CX Console";
#if CONFIG_EXAMPLE_USB_PORT_FULL_SPEED
  usb_cfg.port = 0;
#elif CONFIG_EXAMPLE_USB_PORT_HIGH_SPEED
  usb_cfg.port = 1;
#endif
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp VL53L5CX (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/vl53l5cx_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp VL53L5CX (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  //! [vl53l5cx_webapp_example]
  // Vendor (WebUSB) and CDC (Web Serial) are independent byte streams, so each
  // gets its own Vl53l5cxService (replies and the frame stream go back on the
  // stream the host is using) and its own DispatcherWorker (one bounded RX
  // queue + worker task feeding one parser, so the blocking sensor command
  // handler never runs in the TinyUSB callback context). Both services share
  // the one driver and its mutex; the poll task below feeds both.
  //
  // Each transport's `send` is called from TWO tasks -- the dispatcher worker
  // (request + discovery replies) and the poll task (the frame stream) -- and
  // UsbDevice's write_*() do not serialize writers, so guard each with a mutex
  // to keep frames atomic on the wire.
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
  espp::Vl53l5cxService vendor_service(sensor, {.send = vendor_send,
                                                .sensor_mutex = &sensor_mutex,
                                                .log_level = espp::Logger::Verbosity::INFO});
  espp::Vl53l5cxService cdc_service(sensor, {.send = cdc_send,
                                             .sensor_mutex = &sensor_mutex,
                                             .log_level = espp::Logger::Verbosity::INFO});

  // The standard services (system 7 / monitor 8 / OTA 0 / core dump 4), one per
  // transport through the same mutex-guarded senders. A reboot request is
  // permitted as is (the sensor is re-initialized at boot).
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

  // On an RX overflow the OTA service aborts a transfer it owned and tells the
  // host (an OTA image with dropped bytes is unusable).
  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "vl53l5cx_vendor", .stack_size_bytes = 16384}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send,
       .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
       .task_config = {.name = "vl53l5cx_cdc", .stack_size_bytes = 16384}});
  vendor_link.register_module(vendor_service); // module 10 + its discovery metadata
  cdc_link.register_module(cdc_service);
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

  // The TinyUSB callbacks just queue bytes for the workers.
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  // The one poll loop: read every frame the sensor produces (a frame read is
  // ~1.4 KB of I2C, about 40 ms at 400 kHz) and hand it to both services,
  // which forward it on the transport(s) whose host enabled the stream.
  espp::Vl53l5cx::RangingData frame;
  uint32_t frames = 0;
  auto last_report = std::chrono::steady_clock::now();
  espp::Task poll_task({
      .callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
        if (!sensor.is_initialized() || !sensor.is_ranging()) {
          std::unique_lock<std::mutex> lock(m);
          cv.wait_for(lock, 50ms);
          return false;
        }
        bool have_frame = false;
        {
          std::lock_guard<std::mutex> lock(sensor_mutex);
          std::error_code ec;
          if (sensor.is_data_ready(ec)) {
            have_frame = sensor.get_ranging_data(frame, ec);
            if (!have_frame)
              logger.warn_rate_limited("Failed to read a frame: {}", ec.message());
          }
        }
        if (!have_frame) {
          // nothing new yet (an I2C error is logged by the driver); check again soon
          std::unique_lock<std::mutex> lock(m);
          cv.wait_for(lock, 3ms);
          return false;
        }
        vendor_service.on_frame(frame);
        cdc_service.on_frame(frame);
        frames++;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= 10s) {
          logger.info("{} frames read ({} zones); vendor stream {}, CDC stream {}", frames,
                      frame.num_zones(), vendor_service.stream_enabled() ? "on" : "off",
                      cdc_service.stream_enabled() ? "on" : "off");
          last_report = now;
        }
        return false; // keep polling
      },
      .task_config = {.name = "vl53l5cx poll", .stack_size_bytes = 8 * 1024},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  poll_task.start();
  //! [vl53l5cx_webapp_example]

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {} — no host transport available",
                 usb_ec.message());
  else
    logger.info("VL53L5CX console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
