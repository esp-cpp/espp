// USB telemetry -> Serial Plotter web app example.
//
// Streams a few synthetic float channels from an ESP32-S3 to the browser over
// USB using the espp::Telemetry emitter (a small binary protocol carried on the
// stream_frame framing, dispatcher module 3). The hosted `telemetry.html`
// web app connects on the vendor (WebUSB) interface, reads the SCHEMA (channel
// names), and plots the SAMPLE stream live — the binary, higher-rate,
// device-timestamped counterpart to the app's text/CSV Web Serial transport.
//
// Replace the synthetic generator below with your real signals: build a
// std::array<float, N> in channel (schema) order and call telemetry.emit(...).
// The system console/logs go to the separate built-in USB-Serial-JTAG.

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "dispatcher.hpp"
#include "logger.hpp"
#include "stream_frame.hpp"
#include "task.hpp"
#include "telemetry.hpp"
#include "timer.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;
namespace sf = espp::stream_frame;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Telemetry", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB telemetry (Serial Plotter) example");

  // --- the telemetry emitter: one float per named channel, in schema order ---
  espp::Telemetry telemetry({
      .channels = {"sine", "cosine", "noise", "ramp"},
      // Start paused: only stream once a host connects and sends SET_STREAM, so
      // the vendor TX FIFO doesn't fill with un-drained telemetry before anyone
      // is reading (which a reconnecting host would then have to parse past).
      .stream_on_start = false,
      .period_ms = 10, // default 100 Hz (the web app may request another rate)
      .log_level = espp::Logger::Verbosity::WARN,
  });

  // --- USB: the vendor (WebUSB) interface carries the telemetry protocol ------
  // The browser Serial Plotter plots binary telemetry over WebUSB. (The
  // Telemetry service itself is transport-agnostic - CDC / UART / a socket work
  // too - but the web app's Web Serial path parses text/CSV, so this example
  // streams the binary protocol over WebUSB only.) The console/logs go to the
  // separate built-in USB-Serial-JTAG.
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp Serial Plotter";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp Telemetry (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/telemetry.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // Every device->host write (samples, request replies, discovery replies) goes
  // through this one tx_mutex-guarded helper so the sample-producer task and the
  // RX worker never write the TinyUSB vendor FIFO at the same time.
  std::mutex tx_mutex;
  auto send = [&](std::span<const uint8_t> bytes) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    if (usb.write_vendor(bytes))
      return;
    // Backpressure: the host is not draining fast enough (e.g. a slow reader).
    // write_vendor is all-or-nothing, so nothing partial went out — just drop
    // THIS frame. Do NOT clear the FIFO here: it may already hold a SCHEMA (from
    // set_channels) plus earlier SAMPLEs, and clearing would discard that ordered
    // backlog, letting a later new-width SAMPLE reach the host with no preceding
    // schema. Dropping the current (newest) frame preserves the queued ordering;
    // the host drains what's there when it catches up. Stale-backlog clearing on
    // host-gone is handled separately by the mount/unmount callbacks below.
    logger.warn_rate_limited("vendor TX backpressure; dropped a {}-byte frame", bytes.size());
  };
  telemetry.set_send(send);

  // --- dispatcher: route module-3 frames to the telemetry service + discovery -
  espp::Dispatcher dispatcher;
  const espp::Dispatcher::ModuleInfo info{.name = "Serial Plotter",
                                          .app = "telemetry.html",
                                          .description = "Live device telemetry plotting"};
  dispatcher.register_module(
      espp::Telemetry::kModule, [&](const sf::Frame &frame) { telemetry.handle(frame); }, info);
  dispatcher.set_device_info(usb_cfg.product);
  dispatcher.serve_discovery(send);

  // --- USB RX plumbing: queue in the TinyUSB callback, dispatch from a worker -
  std::mutex rx_mutex;
  std::condition_variable rx_cv;
  std::deque<std::vector<uint8_t>> rx_queue;
  size_t rx_queued_bytes = 0;
  bool rx_overflow = false;
  bool rx_reset = false; // a (re)enumeration asked for a parser reset; done in rx_task
  static constexpr size_t kMaxQueuedRxBytes = 8 * sf::kMaxFrameSize;
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex);
      if (rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        rx_queue.clear();
        rx_queued_bytes = 0;
        rx_overflow = true;
      } else {
        rx_queue.emplace_back(data.begin(), data.end());
        rx_queued_bytes += data.size();
      }
    }
    rx_cv.notify_one();
  });

  // A physical unplug/replug re-enumerates the device: stop streaming, drop any
  // stale vendor TX backlog, and reset the frame parser so the next host starts
  // clean. (A WebUSB tab close does NOT unmount, so that case is handled by the
  // backpressure clear in `send` above.)
  // These run on the TinyUSB task, concurrently with rx_task's dispatcher.feed().
  // The dispatcher's frame parser is not thread-safe, so don't reset it here -
  // drop any queued pre-reset chunks and flag a reset for rx_task to perform,
  // so all parser access stays on the one thread. (set_streaming is atomic and
  // vendor_write_clear touches only the TX FIFO, so both are fine to call here.)
  auto request_parser_reset = [&] {
    {
      std::lock_guard<std::mutex> lock(rx_mutex);
      rx_queue.clear();
      rx_queued_bytes = 0;
      rx_reset = true;
    }
    rx_cv.notify_one();
  };
  usb.set_unmount_callback([&] {
    telemetry.set_streaming(false);
    usb.vendor_write_clear();
    request_parser_reset();
  });
  usb.set_mount_callback([&] {
    usb.vendor_write_clear();
    request_parser_reset();
  });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec)) {
    logger.error("Failed to initialize USB device: {} - no host transport available; aborting",
                 usb_ec.message());
    return;
  }

  espp::Task rx_task({.callback = [&](std::mutex &, std::condition_variable &) -> bool {
                        std::deque<std::vector<uint8_t>> chunks;
                        bool overflowed = false, do_reset = false;
                        {
                          std::unique_lock<std::mutex> lock(rx_mutex);
                          rx_cv.wait_for(lock, 100ms, [&] {
                            return !rx_queue.empty() || rx_overflow || rx_reset;
                          });
                          std::swap(chunks, rx_queue);
                          rx_queued_bytes = 0;
                          overflowed = rx_overflow;
                          rx_overflow = false;
                          do_reset = rx_reset;
                          rx_reset = false;
                        }
                        // All parser mutation happens here on the one rx thread.
                        // Reset first (enumeration change or overflow), then feed
                        // any chunks that arrived after the reset was requested.
                        if (do_reset || overflowed)
                          dispatcher.reset();
                        if (overflowed)
                          return false; // chunks were dropped on overflow
                        for (const auto &chunk : chunks)
                          dispatcher.feed(chunk);
                        return false;
                      },
                      .task_config = {.name = "telemetry_rx", .stack_size_bytes = 8192}});
  rx_task.start();

  // --- synthetic signal producer: emit one SAMPLE per period ------------------
  // Replace this with your real signals. Values are in schema (channel) order.
  const auto t0 = std::chrono::steady_clock::now();
  espp::Timer gen_timer(
      {.period = std::chrono::milliseconds(telemetry.period_ms()),
       .callback = [&]() -> bool {
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
         return false;           // keep running
       },
       .task_config = {.name = "telemetry_gen", .stack_size_bytes = 4096}});
  gen_timer.start();

  logger.info("Telemetry ready. Open the Serial Plotter web app and connect over WebUSB "
              "(channels: sine, cosine, noise, ramp).");

  // The web app can change the sample rate via SET_STREAM, which updates the
  // Telemetry period atomic but not the running producer timer. Poll it and
  // retune the timer when it changes so the advertised rate is actually honored
  // (set_period() is called here, off the timer's own callback thread).
  uint16_t applied_period_ms = telemetry.period_ms();
  while (true) {
    std::this_thread::sleep_for(200ms);
    const uint16_t want = telemetry.period_ms();
    if (want != 0 && want != applied_period_ms) {
      gen_timer.set_period(std::chrono::milliseconds(want));
      applied_period_ms = want;
      logger.debug("retuned producer timer to {} ms", want);
    }
  }
}
