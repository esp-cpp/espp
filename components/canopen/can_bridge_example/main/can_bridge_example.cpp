// USB <-> CAN (TWAI) bridge example.
//
// Turns an ESP32-S3 into a WebUSB / Web Serial CAN interface: the hosted CAN
// console web app connects over the USB vendor interface and can
//   - send CAN frames (as a normal, ACK-ing bus participant — "master"), and
//   - inspect the bus (stream every received frame; in listen-only mode the
//     node is a passive sniffer that never ACKs/transmits).
//
// Both the vendor (WebUSB) and CDC (Web Serial) interfaces carry the SAME
// framed protocol (espp stream_frame codec, routed by an espp::DispatcherWorker
// per transport; this example owns module id 5 — see can_bridge_protocol.hpp),
// so the web app can connect over either transport. The system console/logs go
// to the separate built-in USB-Serial-JTAG. Wire the TX/RX GPIOs to a CAN
// transceiver (e.g. SN65HVD230) on a terminated bus.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "dispatcher_worker.hpp"
#include "logger.hpp"
#include "stream_frame.hpp"
#include "twai.hpp"
#include "usb_device.hpp"

#include "can_bridge_protocol.hpp"

using namespace std::chrono_literals;
namespace sf = espp::stream_frame;

// TWAI GPIOs — change to match your board / transceiver wiring.
static constexpr int kCanTxGpio = 17;
static constexpr int kCanRxGpio = 16;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "CAN Bridge", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB<->CAN bridge example");

  // --- USB: vendor (WebUSB) for the framed protocol + CDC for the console ----
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp CAN Bridge";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp CAN Bridge (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/can_bridge_console.html";
  usb_cfg.vendor = vendor;
  // The CDC interface carries the SAME framed protocol as the vendor interface,
  // so the web app can connect over Web Serial as well as WebUSB. (The system
  // console/logs go to the built-in USB-Serial-JTAG, kept separate — see
  // sdkconfig.defaults.)
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp CAN Bridge (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  // Device -> host sends come from THREE tasks (each transport's dispatcher
  // worker replying to requests, and the TWAI receive task streaming frames),
  // so serialize them.
  enum class Transport { Vendor, Cdc };
  std::mutex tx_mutex;
  // All device->host writes (request replies, streamed CAN_RX frames, AND
  // discovery replies) go through this one tx_mutex-guarded helper so the
  // concurrent senders never write the TinyUSB FIFO at once.
  auto send_to = [&](Transport dest, std::span<const uint8_t> bytes) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    // write_vendor/write_cdc are all-or-nothing (no truncated frame): they
    // bounded-wait (~250 ms) for the host to drain the TX FIFO, then return
    // false and drop the WHOLE frame if it still does not fit / the host is
    // disconnected. We send from ordinary tasks (dispatcher workers + TWAI
    // receive task), not the TinyUSB callback, so the drain-wait path applies.
    // For a best-effort CAN monitor a drop is acceptable; surface it
    // rate-limited rather than silently discarding a reply/CAN_RX frame.
    const bool ok = (dest == Transport::Cdc) ? usb.write_cdc(bytes) : usb.write_vendor(bytes);
    if (!ok)
      logger.warn_rate_limited("dropped a {}-byte frame (USB TX backpressure or disconnect)",
                               bytes.size());
  };
  // Request replies always go back on the transport the request arrived on:
  // the protocol handler takes that transport's `send` (see the per-worker
  // registration below). Unsolicited CAN_RX frames (from the TWAI receive
  // task, no request context) go to whichever transport the host last sent a
  // request on -- only one is connected at a time.
  using send_fn = espp::DispatcherWorker::send_fn;
  std::mutex stream_mutex; // guards stream_send (workers write, TWAI task reads)
  send_fn stream_send;
  auto build_frame = [](uint8_t type, std::span<const uint8_t> payload = {}) {
    // Reply/event types (kCanRx/kOk/kError/kStatus) carry the high bit; map it
    // to the frame reply flag. All CAN-bridge frames are module kModuleId.
    const bool reply = (type & 0x80) != 0;
    return sf::build_frame(reply, can_bridge::kModuleId, type, payload);
  };
  auto send_frame = [&](const send_fn &send, uint8_t type, std::span<const uint8_t> payload = {}) {
    send(build_frame(type, payload));
  };
  auto reply_error = [&](const send_fn &send, const std::error_code &ec,
                         const std::string &context) {
    std::vector<uint8_t> p;
    sf::put_u32(p, static_cast<uint32_t>(ec.value()));
    const std::string msg = context + ": " + ec.message();
    p.insert(p.end(), msg.begin(), msg.end());
    send_frame(send, can_bridge::kError, p);
  };

  // --- CAN bus state (recreated on START so baudrate/mode can change) ---------
  std::mutex bus_mutex; // guards twai + config below
  std::unique_ptr<espp::Twai> twai;
  uint32_t baudrate = 1000000;
  uint8_t mode = can_bridge::kModeNormal;
  std::atomic<uint32_t> rx_count{0}, tx_count{0}, err_count{0};

  // TWAI receive task context: stream each frame to the host as CAN_RX.
  auto on_can_rx = [&](const espp::Twai::Message &m) {
    can_bridge::CanFrame f;
    f.id = m.id;
    f.extended = m.extended;
    f.rtr = m.rtr;
    f.dlc = m.dlc;
    f.data = m.data;
    rx_count.fetch_add(1);
    // Hold stream_mutex across the send so a worker cannot swap stream_send
    // out from under us (the send itself locks tx_mutex; always taken after
    // stream_mutex).
    std::lock_guard<std::mutex> lock(stream_mutex);
    if (stream_send)
      send_frame(stream_send, can_bridge::kCanRx, can_bridge::encode_frame(f));
  };
  auto on_can_err = [&](twai_error_flags_t) { err_count.fetch_add(1); };

  auto send_status = [&](const send_fn &send) {
    std::vector<uint8_t> p;
    {
      std::lock_guard<std::mutex> lock(bus_mutex);
      const bool running = static_cast<bool>(twai);
      sf::put_u32(p, baudrate);
      p.push_back(mode);
      p.push_back(running ? 1 : 0);
    }
    sf::put_u32(p, rx_count.load());
    sf::put_u32(p, tx_count.load());
    sf::put_u32(p, err_count.load());
    send_frame(send, can_bridge::kStatus, p);
  };

  auto start_bus = [&](std::error_code &ec) -> bool {
    std::lock_guard<std::mutex> lock(bus_mutex);
    if (twai)
      return true; // already running
    espp::Twai::Config cfg;
    cfg.tx_gpio = kCanTxGpio;
    cfg.rx_gpio = kCanRxGpio;
    cfg.baudrate = baudrate;
    cfg.mode = (mode == can_bridge::kModeListenOnly) ? espp::Twai::Mode::LISTEN_ONLY
                                                     : espp::Twai::Mode::NORMAL;
    cfg.auto_start = false; // start() explicitly so we get an error code
    cfg.on_receive = on_can_rx;
    cfg.on_error = on_can_err;
    auto node = std::make_unique<espp::Twai>(cfg);
    if (!node->start(ec))
      return false; // node destructs, uninstalling the driver
    twai = std::move(node);
    return true;
  };
  auto stop_bus = [&]() {
    std::lock_guard<std::mutex> lock(bus_mutex);
    if (twai) {
      std::error_code ec;
      twai->stop(ec);
      twai.reset();
    }
  };

  // --- CAN bridge protocol handler (dispatcher module id 5) ------------------
  // `send` transmits on the transport the frame arrived on (each worker
  // registers the handler with its own sender), so replies never cross streams.
  auto handle_can_frame = [&](const espp::stream_frame::Frame &frame, const send_fn &send) {
    // host->device requests only: ignore reply-flagged frames (0xD_ replies are
    // what we SEND; an echoed reply must not re-enter the request handler)
    if (frame.is_reply())
      return;
    // The host is talking on this transport: stream CAN_RX frames there too.
    {
      std::lock_guard<std::mutex> lock(stream_mutex);
      stream_send = send;
    }
    const uint8_t type = frame.type;
    std::span<const uint8_t> payload = frame.payload;
    std::error_code ec;
    switch (type) {
    case can_bridge::kCanTx: {
      can_bridge::CanFrame f;
      if (!can_bridge::decode_frame(payload, f)) {
        reply_error(send, std::make_error_code(std::errc::invalid_argument), "malformed CAN_TX");
        break;
      }
      std::lock_guard<std::mutex> lock(bus_mutex);
      if (!twai) {
        reply_error(send, std::make_error_code(std::errc::not_connected), "bus not started");
        break;
      }
      espp::Twai::Message m;
      m.id = f.id;
      m.extended = f.extended;
      m.rtr = f.rtr;
      m.dlc = f.dlc;
      m.data = f.data;
      if (twai->transmit(m, ec)) {
        tx_count.fetch_add(1);
        send_frame(send, can_bridge::kOk);
      } else {
        reply_error(send, ec, "transmit failed");
      }
      break;
    }
    case can_bridge::kSetConfig: {
      if (payload.size() < 6) {
        reply_error(send, std::make_error_code(std::errc::invalid_argument),
                    "SET_CONFIG needs u32 baudrate + u8 mode + u8 reserved");
        break;
      }
      if (payload[4] > can_bridge::kModeListenOnly) {
        reply_error(send, std::make_error_code(std::errc::invalid_argument),
                    "SET_CONFIG mode must be 0 (normal) or 1 (listen-only)");
        break;
      }
      {
        std::lock_guard<std::mutex> lock(bus_mutex);
        if (twai) {
          reply_error(send, std::make_error_code(std::errc::device_or_resource_busy),
                      "stop the bus before reconfiguring");
          break;
        }
        baudrate = sf::get_u32(payload);
        mode = payload[4];
      }
      send_frame(send, can_bridge::kOk);
      send_status(send);
      break;
    }
    case can_bridge::kStart:
      if (start_bus(ec)) {
        send_frame(send, can_bridge::kOk);
        send_status(send);
      } else {
        reply_error(send, ec, "start failed");
      }
      break;
    case can_bridge::kStop:
      stop_bus();
      send_frame(send, can_bridge::kOk);
      send_status(send);
      break;
    case can_bridge::kGetStatus:
      send_status(send);
      break;
    default:
      reply_error(send, std::make_error_code(std::errc::not_supported),
                  "unknown CAN bridge message");
      break;
    }
  };

  // Vendor (WebUSB) and CDC (Web Serial) are independent byte streams, so each
  // gets its OWN DispatcherWorker: one bounded RX queue + worker task feeding
  // one Dispatcher (one parser), so a frame split across reads on one
  // transport is never stitched onto bytes from the other, and the handler
  // (transmit() can block up to its timeout) never runs in the TinyUSB
  // callback context. On an RX overflow each worker resynchronizes its own
  // parser.
  espp::DispatcherWorker vendor_link(
      {.send = [&](std::span<const uint8_t> f) { send_to(Transport::Vendor, f); },
       .task_config = {.name = "can_bridge_vendor", .stack_size_bytes = 8192}});
  espp::DispatcherWorker cdc_link(
      {.send = [&](std::span<const uint8_t> f) { send_to(Transport::Cdc, f); },
       .task_config = {.name = "can_bridge_cdc", .stack_size_bytes = 8192}});
  // Advertise the CAN-bridge module for capability discovery so the browser
  // Device Hub can list and link it. The handler is registered on each worker
  // with THAT worker's sender captured, so replies go back on the stream the
  // request came from.
  const espp::Dispatcher::ModuleInfo can_info{.name = "CAN Bridge",
                                              .app = "can_bridge_console.html",
                                              .description =
                                                  "Raw CAN 2.0 bridge (WebUSB / Web Serial)"};
  for (auto *link : {&vendor_link, &cdc_link}) {
    link->register_module(
        can_bridge::kModuleId,
        [&, send = link->sender()](const sf::Frame &f) { handle_can_frame(f, send); }, can_info);
    link->serve_discovery(usb_cfg.product);
  }

  // --- USB RX plumbing: the TinyUSB callbacks just queue for the workers -----
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  std::error_code usb_ec;
  const bool usb_ok = usb.initialize(usb_ec);
  if (!usb_ok)
    logger.error("Failed to initialize USB device: {} — no host transport available",
                 usb_ec.message());

  if (usb_ok) {
    logger.info("CAN bridge ready. Connect the CAN console web app over WebUSB / Web Serial.");
    logger.info("Bus starts stopped; the host sets baudrate/mode (SET_CONFIG) then START.");
  }

  // Idle; all work happens in the TWAI receive task and the dispatcher workers.
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
