// USB telemetry -> Serial Plotter web app example.
//
// Streams a few synthetic float channels from an ESP32-S3 to the browser over
// USB using the espp::Telemetry emitter (a small binary protocol carried on the
// stream_frame framing, dispatcher module 3). The hosted `serial_plotter.html`
// web app connects on the vendor (WebUSB) OR CDC (Web Serial) interface, reads
// the SCHEMA (channel names), and plots the SAMPLE stream live — the binary,
// higher-rate, device-timestamped counterpart to the app's text/CSV transport.
//
// Replace the synthetic generator below with your real signals: build a
// std::array<float, N> in channel (schema) order and call telemetry.emit(...).
// The system console/logs go to the separate built-in USB-Serial-JTAG.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <span>
#include <thread>
#include <utility>
#include <vector>

#include "dispatcher.hpp"
#include "logger.hpp"
#include "stream_frame.hpp"
#include "task.hpp"
#include "telemetry_service.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;
namespace sf = espp::stream_frame;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Telemetry", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB telemetry (Serial Plotter) example");

  // --- the telemetry emitter: one float per named channel, in schema order ---
  espp::Telemetry telemetry({
      .channels = {"sine", "cosine", "noise", "ramp"},
      .stream_on_start = true, // stream immediately; the web app can pause via SET_STREAM
      .period_ms = 10,         // default 100 Hz (the web app may request another rate)
      .log_level = espp::Logger::Verbosity::WARN,
  });

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol -------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp Serial Plotter";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp Telemetry (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/serial_plotter.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp Telemetry (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  // Reply / stream on whichever transport the host last talked on (one at a time).
  enum class Transport { Vendor, Cdc };
  std::atomic<Transport> active_transport{Transport::Vendor};
  std::mutex tx_mutex;
  // Every device->host write (samples, request replies, discovery replies) goes
  // through this one tx_mutex-guarded helper so the sample-producer task and the
  // RX worker never write the TinyUSB FIFO at the same time.
  auto send_to = [&](Transport dest, std::span<const uint8_t> bytes) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    const bool ok = (dest == Transport::Cdc) ? usb.write_cdc(bytes) : usb.write_vendor(bytes);
    if (!ok)
      logger.warn_rate_limited("dropped a {}-byte frame (USB TX backpressure or disconnect)",
                               bytes.size());
  };
  // The emitter sends to the active transport (set by the RX worker per request).
  telemetry.set_send(
      [&](std::span<const uint8_t> bytes) { send_to(active_transport.load(), bytes); });

  // --- dispatchers: route module-3 frames to the telemetry service + discovery
  espp::Dispatcher vendor_dispatcher, cdc_dispatcher;
  const espp::Dispatcher::ModuleInfo info{.name = "Serial Plotter",
                                          .app = "serial_plotter.html",
                                          .description = "Live device telemetry plotting"};
  auto handler = [&](const sf::Frame &frame) { telemetry.handle(frame); };
  vendor_dispatcher.register_module(espp::Telemetry::kModule, handler, info);
  cdc_dispatcher.register_module(espp::Telemetry::kModule, handler, info);
  vendor_dispatcher.set_device_info(usb_cfg.product);
  cdc_dispatcher.set_device_info(usb_cfg.product);
  vendor_dispatcher.serve_discovery(
      [&](std::span<const uint8_t> f) { send_to(Transport::Vendor, f); });
  cdc_dispatcher.serve_discovery([&](std::span<const uint8_t> f) { send_to(Transport::Cdc, f); });

  // --- USB RX plumbing: queue in the TinyUSB callback, dispatch from a worker -
  std::mutex rx_mutex;
  std::condition_variable rx_cv;
  std::deque<std::pair<Transport, std::vector<uint8_t>>> rx_queue;
  size_t rx_queued_bytes = 0;
  bool rx_overflow = false;
  static constexpr size_t kMaxQueuedRxBytes = 8 * sf::kMaxFrameSize;
  auto enqueue_rx = [&](Transport source, std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex);
      if (rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        rx_queue.clear();
        rx_queued_bytes = 0;
        rx_overflow = true;
      } else {
        rx_queue.emplace_back(source, std::vector<uint8_t>(data.begin(), data.end()));
        rx_queued_bytes += data.size();
      }
    }
    rx_cv.notify_one();
  };
  usb.set_vendor_receive_callback(
      [&](std::span<const uint8_t> data) { enqueue_rx(Transport::Vendor, data); });
  usb.set_cdc_receive_callback(
      [&](std::span<const uint8_t> data) { enqueue_rx(Transport::Cdc, data); });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec)) {
    logger.error("Failed to initialize USB device: {} - no host transport available; aborting",
                 usb_ec.message());
    return;
  }

  espp::Task rx_task(
      {.callback = [&](std::mutex &, std::condition_variable &) -> bool {
         std::deque<std::pair<Transport, std::vector<uint8_t>>> chunks;
         bool overflowed = false;
         {
           std::unique_lock<std::mutex> lock(rx_mutex);
           rx_cv.wait_for(lock, 100ms, [&] { return !rx_queue.empty() || rx_overflow; });
           std::swap(chunks, rx_queue);
           rx_queued_bytes = 0;
           overflowed = rx_overflow;
           rx_overflow = false;
         }
         if (overflowed) {
           vendor_dispatcher.reset();
           cdc_dispatcher.reset();
           return false;
         }
         for (const auto &[source, chunk] : chunks) {
           // Single writer of active_transport: match the chunk being dispatched
           // so replies/samples go back on the transport the request arrived on.
           active_transport.store(source);
           (source == Transport::Vendor ? vendor_dispatcher : cdc_dispatcher).feed(chunk);
         }
         return false;
       },
       .task_config = {.name = "telemetry_rx", .stack_size_bytes = 8192}});
  rx_task.start();

  // --- synthetic signal producer: emit one SAMPLE per period ------------------
  // Replace this with your real signals. Values are in schema (channel) order.
  const auto t0 = std::chrono::steady_clock::now();
  espp::Task gen_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         const float t =
             std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
         const float two_pi = 6.2831853f;
         const float noise = static_cast<float>(std::rand()) / RAND_MAX * 0.4f - 0.2f;
         const std::array<float, 4> values = {
             std::sin(two_pi * 1.0f * t), // sine @ 1 Hz
             std::cos(two_pi * 0.5f * t), // cosine @ 0.5 Hz
             noise,                       // uniform noise
             std::fmod(t, 2.0f) - 1.0f,   // -1..1 sawtooth ramp
         };
         telemetry.emit(values); // no-op while streaming is paused
         std::unique_lock<std::mutex> lock(m);
         cv.wait_for(lock, std::chrono::milliseconds(std::max<uint16_t>(1, telemetry.period_ms())));
         return false; // keep running
       },
       .task_config = {.name = "telemetry_gen", .stack_size_bytes = 4096}});
  gen_task.start();

  logger.info("Telemetry ready. Open the Serial Plotter web app and connect over WebUSB / Web "
              "Serial (channels: sine, cosine, noise, ramp).");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
