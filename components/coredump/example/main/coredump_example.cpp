#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"

#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher.hpp"
#include "logger.hpp"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

// Deliberate test crashes, selectable from the CDC console (type the command +
// enter — the web console's Web Serial input works too) or the BOOT button.
// Each demonstrates a different row of the next-boot crash report:
//   nullptr  -> StoreProhibited panic  -> core dump + backtrace
//   assert   -> assert failure panic   -> core dump + backtrace
//   divzero  -> IntegerDivideByZero    -> core dump + backtrace
//   hang     -> interrupts-off forever -> INT_WDT reset, NO core dump (the
//               report explains the reset reason instead)
enum class CrashKind : uint8_t { None, NullPointer, Assert, DivideByZero, Hang };

// Example-specific "trigger a test crash" command: a stream_frame with
// module = kCrashModule, type = kMsgTriggerCrash, payload = [CrashKind]. Its
// own dispatcher module keeps it cleanly separate from the core-dump protocol
// (module 4); the CDC text console keeps working for Web Serial / terminal
// users too.
static constexpr uint8_t kCrashModule = 1;
static constexpr uint8_t kMsgTriggerCrash = 0x00;

[[noreturn]] static void perform_crash(CrashKind kind) {
  switch (kind) {
  case CrashKind::Assert:
    assert(false && "deliberate test crash (assert)");
    perform_crash(CrashKind::NullPointer); // only reached if asserts are compiled out (NDEBUG)
    // cppcheck-suppress unreachableCode // explicit no-fallthrough marker after [[noreturn]] call
    __builtin_unreachable();
  case CrashKind::DivideByZero: {
    volatile int numerator = 42;
    volatile int divisor = 0;
    // cppcheck-suppress zerodiv // deliberate: this command EXISTS to crash
    volatile int result = numerator / divisor; // IntegerDivideByZero panic
    (void)result;
    __builtin_unreachable();
  }
  case CrashKind::Hang:
    // Interrupts off + never yield: the interrupt watchdog resets the chip
    // WITHOUT writing a core dump — the next boot reports the reset reason.
    portDISABLE_INTERRUPTS();
    for (;;) {
    }
  case CrashKind::NullPointer:
  default: {
    volatile uint32_t *null_ptr = nullptr;
    // cppcheck-suppress nullPointer // deliberate: this command EXISTS to crash
    *null_ptr = 0xdead; // StoreProhibited panic
    __builtin_unreachable();
  }
  }
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "CoreDump Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting core dump over USB example");

  //! [coredump_example]
  // --------------------------------------------------------------------------
  // Core dump accessor: crash report + raw image access for the service below.
  // --------------------------------------------------------------------------
  espp::CoreDump core_dump({.log_level = espp::Logger::Verbosity::INFO});
  const std::string report = core_dump.format_report();
  if (report.empty()) {
    logger.info("Clean boot history (reset reason: {})",
                espp::CoreDump::reset_reason_name(espp::CoreDump::reset_reason()));
  } else {
    logger.error("Previous abnormal reset:\n{}", report);
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH // avoid a constant-condition warning when disabled
    if (core_dump.has_core_dump())
      logger.info("Core dump image in flash: {} bytes (download / erase it with the web console)",
                  core_dump.image_size());
#endif
  }

  // --------------------------------------------------------------------------
  // USB composite device: a vendor/WebUSB function (framed protocol only) and
  // a CDC function carrying the system console PLUS the same framed protocol
  // (the web console's Web Serial transport separates text from frames).
  // --------------------------------------------------------------------------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = 0x0d36; // distinct from the espp default so the webapp filter is specific
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp CoreDump";
  usb_cfg.log_level = espp::Logger::Verbosity::INFO;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp CoreDump (console)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp CoreDump (WebUSB)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/coredump_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // One CoreDumpService per byte stream (each owns its own frame parser),
  // both sharing the same espp::CoreDump. Replies go back on the stream the
  // request came in on.
  espp::CoreDumpService vendor_service(
      core_dump, {.send = [&](std::span<const uint8_t> frame) { usb.write_vendor(frame); },
                  .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService cdc_service(
      core_dump, {.send = [&](std::span<const uint8_t> frame) { usb.write_cdc(frame); },
                  .log_level = espp::Logger::Verbosity::INFO});
  //! [coredump_example]

  // --------------------------------------------------------------------------
  // RX plumbing: bytes arrive in the TinyUSB task context; queue them and feed
  // the services from a worker task (ERASE can block for tens of ms and must
  // not stall the USB stack). Tag each chunk with its source stream.
  // --------------------------------------------------------------------------
  enum class Source : uint8_t { Vendor, Cdc };
  std::mutex rx_mutex;
  std::condition_variable rx_cv;
  std::deque<std::pair<Source, std::vector<uint8_t>>> rx_queue;
  size_t rx_queued_bytes = 0;
  // Set when a chunk was dropped for a source; the worker then resets that
  // stream's parser(s) so a frame straddling the gap resynchronizes at once
  // instead of staying half-parsed until the next magic happens to appear.
  bool rx_drop_vendor = false, rx_drop_cdc = false;
  // The protocol is one-request-in-flight, so a well-behaved host queues at
  // most ~one frame; cap the queue so a misbehaving host cannot exhaust RAM.
  static constexpr size_t kMaxQueuedRxBytes = 8 * espp::stream_frame::kMaxFrameSize;
  auto enqueue_rx = [&](Source source, std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex);
      if (rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        // drop this chunk and flag the source so the worker resets its
        // parser(s) before feeding the next chunk (see rx_drop_* above)
        (source == Source::Vendor ? rx_drop_vendor : rx_drop_cdc) = true;
        return;
      }
      rx_queue.emplace_back(source, std::vector<uint8_t>(data.begin(), data.end()));
      rx_queued_bytes += data.size();
    }
    rx_cv.notify_one();
  };
  usb.set_vendor_receive_callback(
      [&](std::span<const uint8_t> data) { enqueue_rx(Source::Vendor, data); });

  // The CDC RX path additionally implements the tiny line-oriented test-crash
  // console: printable characters accumulate into a line; enter runs it. The
  // SAME bytes are also queued for the frame parser — text never forms a
  // valid frame and frames never form a command line, so both decoders can
  // safely watch the one stream.
  std::atomic<CrashKind> pending_crash{CrashKind::None};
  auto request_crash = [&](CrashKind kind, const char *name) {
    logger.warn("Test crash requested: {} (crashing from the main task in ~1s...)", name);
    pending_crash = kind;
  };
  std::mutex line_mutex;
  std::string cdc_line;
  auto handle_command = [&](const std::string &line) {
    if (line == "crash" || line == "nullptr")
      request_crash(CrashKind::NullPointer, "null-pointer write");
    else if (line == "assert")
      request_crash(CrashKind::Assert, "failed assert");
    else if (line == "divzero")
      request_crash(CrashKind::DivideByZero, "integer divide by zero");
    else if (line == "hang")
      request_crash(CrashKind::Hang, "interrupts-off hang (INT_WDT, no core dump)");
    else if (line == "report")
      logger.info("Crash report:\n{}", report.empty() ? "(clean boot history)" : report);
    else if (line == "help" || line == "?")
      logger.info("commands: crash | assert | divzero | hang | report | help");
    else if (!line.empty())
      logger.warn("unknown command '{}' (try 'help')", line);
  };
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(line_mutex);
      for (const uint8_t byte : data) {
        if (byte == '\r' || byte == '\n') {
          if (!cdc_line.empty()) {
            handle_command(cdc_line);
            cdc_line.clear();
          }
        } else if (byte >= 0x20 && byte < 0x7F && cdc_line.size() < 64) {
          cdc_line.push_back(static_cast<char>(byte));
        } else if (byte < 0x20 || byte >= 0x7F) {
          cdc_line.clear(); // binary (frame) bytes: not a command line
        }
      }
    }
    enqueue_rx(Source::Cdc, data);
  });

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec)) {
    logger.error("Failed to initialize USB device: {}", usb_ec.message());
  } else {
    // Route the SYSTEM console (stdout/stderr — all espp/fmt and esp_log
    // output) to the CDC interface: TinyUSB owns the S3's only USB PHY, so
    // this replaces the unusable USB-Serial-JTAG console. Attach any serial
    // terminal (`screen /dev/tty.usbmodem*`) or the web console (Web Serial)
    // for live logs. Panic backtraces cannot appear live (TinyUSB dies with
    // the panic) — they are captured by the flash core dump and reported on
    // the next boot.
    if (tinyusb_console_init(TINYUSB_CDC_ACM_0) != ESP_OK)
      logger.warn("Could not route the console to USB CDC");
  }

  // Route each byte stream through a Dispatcher: the core-dump protocol on
  // module 4 (CoreDumpService::kModule) and the example's WebUSB crash trigger
  // on module 1 (kCrashModule). One parser per stream, module-routed — no more
  // running two parsers over the same bytes.
  espp::Dispatcher vendor_dispatcher, cdc_dispatcher;
  auto handle_cmd_frame = [&](const espp::stream_frame::Frame &f) {
    // host->device request only: ignore reply-flagged frames (echo/loopback)
    if (f.is_reply() || f.type != kMsgTriggerCrash || f.payload.size() != 1)
      return;
    switch (static_cast<CrashKind>(f.payload[0])) {
    case CrashKind::NullPointer:
      request_crash(CrashKind::NullPointer, "null-pointer write (WebUSB)");
      break;
    case CrashKind::Assert:
      request_crash(CrashKind::Assert, "failed assert (WebUSB)");
      break;
    case CrashKind::DivideByZero:
      request_crash(CrashKind::DivideByZero, "integer divide by zero (WebUSB)");
      break;
    case CrashKind::Hang:
      request_crash(CrashKind::Hang, "interrupts-off hang (WebUSB)");
      break;
    default:
      logger.warn("TRIGGER_CRASH: unknown kind {}", static_cast<int>(f.payload[0]));
      break;
    }
  };
  // Register the protocols on each stream's dispatcher. The service answers
  // requests, so ignore reply-flagged frames (handle_frame() takes only
  // type+payload, so the direction check lives here).
  vendor_dispatcher.register_module(espp::CoreDumpService::kModule,
                                    [&](const espp::stream_frame::Frame &f) {
                                      if (!f.is_reply())
                                        vendor_service.handle_frame(f.type, f.payload);
                                    });
  vendor_dispatcher.register_module(kCrashModule, handle_cmd_frame);
  cdc_dispatcher.register_module(espp::CoreDumpService::kModule,
                                 [&](const espp::stream_frame::Frame &f) {
                                   if (!f.is_reply())
                                     cdc_service.handle_frame(f.type, f.payload);
                                 });

  espp::Task rx_task({.callback = [&](std::mutex &, std::condition_variable &) -> bool {
                        std::deque<std::pair<Source, std::vector<uint8_t>>> chunks;
                        bool drop_vendor = false, drop_cdc = false;
                        {
                          std::unique_lock<std::mutex> lock(rx_mutex);
                          rx_cv.wait_for(lock, 100ms, [&] { return !rx_queue.empty(); });
                          std::swap(chunks, rx_queue);
                          rx_queued_bytes = 0;
                          drop_vendor = rx_drop_vendor;
                          drop_cdc = rx_drop_cdc;
                          rx_drop_vendor = rx_drop_cdc = false;
                        }
                        if (drop_vendor)
                          vendor_dispatcher.reset();
                        if (drop_cdc)
                          cdc_dispatcher.reset();
                        for (const auto &[source, bytes] : chunks) {
                          if (source == Source::Vendor)
                            vendor_dispatcher.feed(bytes);
                          else
                            cdc_dispatcher.feed(bytes);
                        }
                        return false; // don't stop the task
                      },
                      .task_config = {.name = "coredump_rx", .stack_size_bytes = 8192}});
  rx_task.start();

  // --------------------------------------------------------------------------
  // BOOT button (GPIO0): press to trigger the null-pointer test crash.
  // --------------------------------------------------------------------------
  static constexpr gpio_num_t kBootButton = GPIO_NUM_0;
  {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << kBootButton;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
  }

  logger.info("Ready. Connect the native USB port and open the web console "
              "(components/coredump/web/coredump_console.html or https://{})",
              vendor.landing_page_url);
  logger.info("Trigger a test crash: press the BOOT button, or type 'help' on this console");

  bool cdc_was_connected = false;
  int button_pressed_polls = 0;
  while (true) {
    std::this_thread::sleep_for(50ms);
    // BOOT button held for a couple of polls -> crash (debounce).
    if (gpio_get_level(kBootButton) == 0) {
      if (++button_pressed_polls == 3)
        request_crash(CrashKind::NullPointer, "BOOT button null-pointer write");
    } else {
      button_pressed_polls = 0;
    }
    // Crash from here (the main task) so the report names a recognizable
    // task, and after a delay so the CDC console gets to flush the warning.
    if (pending_crash != CrashKind::None) {
      std::this_thread::sleep_for(1s);
      perform_crash(pending_crash.load());
    }
    // A terminal attaching to the CDC console missed the boot output; re-log
    // the previous-crash summary for it once per connection.
    const bool cdc_connected = usb.is_cdc_connected();
    if (cdc_connected && !cdc_was_connected && !report.empty())
      logger.error("Previous abnormal reset:\n{}", report);
    cdc_was_connected = cdc_connected;
  }
}
