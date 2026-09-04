// USB <-> MCP266 web console example.
//
// Runs the espp::Mcp266 driver on an ESP32-S3 and exposes it to a browser over
// USB: the hosted MCP266 console web app connects on the vendor (WebUSB) OR CDC
// (Web Serial) interface and can configure the position loops, command moves,
// and view live per-axis status (position / velocity / DS402 state) plus device
// telemetry (battery, temperature). All CANopen/DS402 work happens on the
// device behind a small high-level protocol (see mcp266_protocol.hpp); the web
// app needs no CANopen knowledge.
//
// Wiring: the ESP32-S3 is the CANopen MASTER of the MCP266 node, so connect the
// TWAI TX/RX GPIOs to a 3.3 V CAN transceiver on a terminated bus at the
// baudrate configured on the MCP266 (Basicmicro Motion Studio). Set kNodeId to
// the MCP266's configured CANopen node id. The system console/logs go to the
// separate built-in USB-Serial-JTAG.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "canopen_client.hpp"
#include "dispatcher.hpp"
#include "logger.hpp"
#include "mcp266.hpp"
#include "stream_frame.hpp"
#include "task.hpp"
#include "twai.hpp"
#include "usb_device.hpp"

#include "mcp266_protocol.hpp"

using namespace std::chrono_literals;
namespace sf = espp::stream_frame;
namespace proto = mcp266_protocol;
using Axis = espp::Mcp266::Axis;

// --- device configuration (change to match your board / MCP266) --------------
static constexpr int kCanTxGpio = 17;
static constexpr int kCanRxGpio = 16;
static constexpr uint32_t kCanBaudrate = 1000000;
static constexpr uint8_t kNodeId = 10; // the MCP266's CANopen node id (Motion Studio)

// --- little-endian payload helpers -------------------------------------------
static uint32_t rd_u32(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}
static int32_t rd_i32(std::span<const uint8_t> p, size_t off) {
  return static_cast<int32_t>(rd_u32(p, off));
}
static int16_t rd_i16(std::span<const uint8_t> p, size_t off) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[off]) |
                              (static_cast<uint16_t>(p[off + 1]) << 8));
}
static uint16_t rd_u16(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[off]) |
                               (static_cast<uint16_t>(p[off + 1]) << 8));
}
static void put_u16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back(x & 0xFF);
  v.push_back((x >> 8) & 0xFF);
}
static void put_i32(std::vector<uint8_t> &v, int32_t x) {
  const auto u = static_cast<uint32_t>(x);
  v.push_back(u & 0xFF);
  v.push_back((u >> 8) & 0xFF);
  v.push_back((u >> 16) & 0xFF);
  v.push_back((u >> 24) & 0xFF);
}
static Axis axis_of(uint8_t b) { return b == proto::kAxisM2 ? Axis::M2 : Axis::M1; }

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MCP266 Console", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB<->MCP266 console example (node id {})", kNodeId);

  // --- CAN transport + CANopen client + MCP266 driver ------------------------
  // The Twai receive task feeds process_frame(); the MCP266 SDO transactions run
  // on OTHER tasks (the USB RX worker + the status streamer), satisfying
  // CanopenClient's "pump RX from a different task" contract.
  static espp::CanopenClient *client_ptr = nullptr;
  espp::Twai twai({
      .tx_gpio = kCanTxGpio,
      .rx_gpio = kCanRxGpio,
      .baudrate = kCanBaudrate,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 10,
      .on_receive =
          [](const espp::Twai::Message &m) {
            if (client_ptr)
              client_ptr->process_frame(espp::CanopenClient::CanFrame{
                  .id = m.id, .extended = m.extended, .rtr = m.rtr, .dlc = m.dlc, .data = m.data});
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  espp::CanopenClient client({
      .node_id = kNodeId,
      .send =
          [&twai](const espp::CanopenClient::CanFrame &f) {
            espp::Twai::Message m{
                .id = f.id, .extended = f.extended, .rtr = f.rtr, .dlc = f.dlc, .data = f.data};
            std::error_code tx_ec;
            return twai.transmit(m, tx_ec);
          },
      .sdo_timeout = 500ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  client_ptr = &client;
  espp::Mcp266 mcp(client, {.log_level = espp::Logger::Verbosity::WARN});

  // A single CANopen SDO channel: serialize every Mcp266 call (command handler +
  // status streamer both issue SDO, and only one transaction may be in flight).
  std::mutex mcp_mutex;

  std::error_code twai_ec;
  const bool twai_ok = twai.initialize(twai_ec);
  if (!twai_ok)
    logger.error("Failed to initialize TWAI: {}", twai_ec.message());
  else {
    // Best-effort node start so status works immediately; the web app can re-run
    // it (START) if the node is not on the bus yet.
    std::error_code ec;
    std::lock_guard<std::mutex> lock(mcp_mutex);
    if (mcp.start(ec))
      logger.info("MCP266 node started");
    else
      logger.warn("MCP266 not started ({}); use START from the web app once it is on the bus",
                  ec.message());
  }

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol ------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp MCP266 Console";
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp MCP266 (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/mcp266_console.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp MCP266 (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  // Reply on whichever transport the host last talked on (only one at a time).
  enum class Transport { Vendor, Cdc };
  std::atomic<Transport> active_transport{Transport::Vendor};
  std::mutex tx_mutex;
  auto send = [&](std::span<const uint8_t> bytes) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    const bool ok = (active_transport.load() == Transport::Cdc) ? usb.write_cdc(bytes)
                                                                : usb.write_vendor(bytes);
    if (!ok)
      logger.warn_rate_limited("dropped a {}-byte frame (USB TX backpressure or disconnect)",
                               bytes.size());
  };
  auto send_frame = [&](uint8_t type, std::span<const uint8_t> payload = {}) {
    const bool reply = (type & 0x80) != 0; // 0xE_ reply types set the frame reply flag
    send(sf::build_frame(reply, proto::kModuleId, type, payload));
  };
  auto send_ok = [&](uint8_t request_type) {
    const uint8_t p[] = {request_type};
    send_frame(proto::kOk, p);
  };
  auto reply_error = [&](uint8_t request_type, const std::error_code &ec, const std::string &ctx) {
    std::vector<uint8_t> p;
    p.push_back(request_type);
    sf::put_u32(p, static_cast<uint32_t>(ec.value()));
    const std::string msg = ctx + ": " + ec.message();
    p.insert(p.end(), msg.begin(), msg.end());
    send_frame(proto::kError, p);
  };

  // --- STATUS snapshot: read both axes + device telemetry, send a STATUS frame
  auto send_status = [&]() {
    std::vector<uint8_t> p;
    uint8_t flags = 0;
    bool any_ok = false;
    {
      std::lock_guard<std::mutex> lock(mcp_mutex);
      std::error_code ec;
      for (Axis axis : {Axis::M1, Axis::M2}) {
        int32_t position = 0, velocity = 0;
        uint16_t statusword = 0;
        if (mcp.read_encoder(axis, position, ec))
          any_ok = true;
        mcp.read_speed(axis, velocity, ec);
        mcp.read_statusword(axis, statusword, ec);
        put_i32(p, position);
        put_i32(p, velocity);
        put_u16(p, statusword);
      }
      float volts = 0.0f, temp_c = 0.0f;
      mcp.read_main_battery_voltage(volts, ec);
      mcp.read_temperature(temp_c, ec);
      put_u16(p, static_cast<uint16_t>(volts * 10.0f + 0.5f));
      put_u16(p, static_cast<uint16_t>(temp_c * 10.0f + 0.5f));
    }
    if (any_ok)
      flags |= proto::kStatusFlagOnline;
    p.push_back(flags);
    send_frame(proto::kStatus, p);
  };

  // --- status streaming task -------------------------------------------------
  // A STATUS snapshot performs eight blocking SDO reads while holding the shared
  // MCP mutex (see send_status), so a too-small period would starve command
  // handling and flood the CAN bus. Clamp the host-requested period into a safe
  // window: >= 50 ms (<= 20 Hz) leaves headroom for the eight SDO round-trips,
  // and <= 10 s keeps the stream responsive. A period of 0 selects the default.
  static constexpr uint16_t kDefaultStreamPeriodMs = 200;
  static constexpr uint16_t kMinStreamPeriodMs = 50;
  static constexpr uint16_t kMaxStreamPeriodMs = 10000;
  std::atomic<bool> stream_enabled{false};
  std::atomic<uint32_t> stream_period_ms{kDefaultStreamPeriodMs};
  espp::Task status_task({.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
                            if (stream_enabled.load())
                              send_status();
                            std::unique_lock<std::mutex> lock(m);
                            cv.wait_for(lock,
                                        std::chrono::milliseconds(
                                            stream_enabled.load() ? stream_period_ms.load() : 200));
                            return false; // keep running
                          },
                          .task_config = {.name = "mcp266_status", .stack_size_bytes = 8192}});
  status_task.start();

  // --- command handler (runs on the RX worker task, so SDO calls may block) --
  auto handle = [&](const sf::Frame &frame) {
    if (frame.is_reply())
      return; // 0xE_ replies are what we SEND; never re-enter the request path
    const uint8_t type = frame.type;
    std::span<const uint8_t> pl = frame.payload;
    std::error_code ec;
    auto need = [&](size_t n) -> bool {
      if (pl.size() < n) {
        reply_error(type, std::make_error_code(std::errc::invalid_argument), "short payload");
        return false;
      }
      return true;
    };
    // Axis-addressed requests: require the length AND a valid axis selector, so an
    // unexpected pl[0] cannot silently fall through to M1 and command the wrong
    // motor (axis_of() only distinguishes 1 == M2 from everything-else == M1).
    auto need_axis = [&](size_t n) -> bool {
      if (!need(n))
        return false;
      if (pl[0] > proto::kAxisM2) {
        reply_error(type, std::make_error_code(std::errc::invalid_argument),
                    "invalid axis (must be 0=M1 or 1=M2)");
        return false;
      }
      return true;
    };
    std::lock_guard<std::mutex> lock(mcp_mutex);
    switch (type) {
    case proto::kStart:
      mcp.start(ec) ? send_ok(type) : reply_error(type, ec, "start failed");
      break;
    case proto::kResetFaults:
      mcp.reset_faults(ec) ? send_ok(type) : reply_error(type, ec, "reset faults failed");
      break;
    case proto::kResetEstop:
      mcp.reset_estop(ec) ? send_ok(type) : reply_error(type, ec, "reset e-stop failed");
      break;
    case proto::kConfigurePositionLoop:
      if (!need_axis(13))
        break;
      mcp.configure_position_loop(axis_of(pl[0]), rd_i32(pl, 1), rd_i32(pl, 5), rd_i32(pl, 9), ec)
          ? send_ok(type)
          : reply_error(type, ec, "configure position loop failed");
      break;
    case proto::kSetPositionLimits:
      if (!need_axis(9))
        break;
      mcp.set_software_position_limits(axis_of(pl[0]), rd_i32(pl, 1), rd_i32(pl, 5), ec)
          ? send_ok(type)
          : reply_error(type, ec, "set position limits failed");
      break;
    case proto::kMoveToPosition:
      if (!need_axis(17))
        break;
      mcp.move_to_position(axis_of(pl[0]), rd_i32(pl, 1), rd_u32(pl, 5), rd_u32(pl, 9),
                           rd_u32(pl, 13), ec)
          ? send_ok(type)
          : reply_error(type, ec, "move failed");
      break;
    case proto::kDriveSpeed:
      if (!need_axis(5))
        break;
      mcp.drive_speed(axis_of(pl[0]), rd_i32(pl, 1), ec)
          ? send_ok(type)
          : reply_error(type, ec, "drive speed failed");
      break;
    case proto::kDriveDuty:
      if (!need_axis(3))
        break;
      mcp.drive_duty(axis_of(pl[0]), rd_i16(pl, 1), ec)
          ? send_ok(type)
          : reply_error(type, ec, "drive duty failed");
      break;
    case proto::kSetStatusStream:
      if (!need(3))
        break;
      {
        const uint16_t requested = rd_u16(pl, 1);
        const uint16_t period_ms =
            std::clamp<uint16_t>(requested == 0 ? kDefaultStreamPeriodMs : requested,
                                 kMinStreamPeriodMs, kMaxStreamPeriodMs);
        stream_enabled.store(pl[0] != 0);
        stream_period_ms.store(period_ms);
      }
      send_ok(type);
      break;
    case proto::kGetDeviceInfo: {
      std::string name;
      uint32_t device_type = 0;
      if (mcp.read_device_info(name, device_type, ec)) {
        std::vector<uint8_t> p;
        sf::put_u32(p, device_type);
        p.insert(p.end(), name.begin(), name.end());
        send_frame(proto::kDeviceInfo, p);
      } else {
        reply_error(type, ec, "read device info failed");
      }
      break;
    }
    default:
      reply_error(type, std::make_error_code(std::errc::not_supported), "unknown MCP266 message");
      break;
    }
  };

  // kGetStatus is handled outside the mcp_mutex-holding switch (send_status
  // locks it itself). Wrap the dispatch so GET_STATUS calls send_status().
  auto dispatch_frame = [&](const sf::Frame &frame) {
    if (!frame.is_reply() && frame.type == proto::kGetStatus) {
      send_status();
      return;
    }
    handle(frame);
  };

  espp::Dispatcher vendor_dispatcher, cdc_dispatcher;
  // Advertise the MCP266 module for capability discovery so the browser Device
  // Hub can list and link it.
  const espp::Dispatcher::ModuleInfo mcp_info{.name = "MCP266",
                                              .app = "mcp266_console.html",
                                              .description = "Configure & command MCP266 motors"};
  vendor_dispatcher.register_module(proto::kModuleId, dispatch_frame, mcp_info);
  cdc_dispatcher.register_module(proto::kModuleId, dispatch_frame, mcp_info);
  vendor_dispatcher.set_device_info(usb_cfg.product);
  cdc_dispatcher.set_device_info(usb_cfg.product);
  vendor_dispatcher.serve_discovery([&](std::span<const uint8_t> f) { usb.write_vendor(f); });
  cdc_dispatcher.serve_discovery([&](std::span<const uint8_t> f) { usb.write_cdc(f); });

  // --- USB RX plumbing: queue in the TinyUSB callback, dispatch from a worker -
  std::mutex rx_mutex;
  std::condition_variable rx_cv;
  std::deque<std::pair<Transport, std::vector<uint8_t>>> rx_queue;
  size_t rx_queued_bytes = 0;
  bool rx_overflow = false;
  static constexpr size_t kMaxQueuedRxBytes = 8 * sf::kMaxFrameSize;
  auto enqueue_rx = [&](Transport source, std::span<const uint8_t> data) {
    // NOTE: active_transport is set by the RX worker just before it feeds each
    // chunk (below), NOT here: a frame arriving on the other endpoint between
    // enqueue and dispatch must not retarget a reply for the frame being handled.
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
  const bool usb_ok = usb.initialize(usb_ec);
  if (!usb_ok)
    logger.error("Failed to initialize USB device: {} — no host transport available",
                 usb_ec.message());

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
           // Single-writer of active_transport: set it to match the chunk being
           // dispatched so replies/status generated during this feed go back on
           // the transport the request arrived on.
           active_transport.store(source);
           (source == Transport::Vendor ? vendor_dispatcher : cdc_dispatcher).feed(chunk);
         }
         return false;
       },
       .task_config = {.name = "mcp266_rx", .stack_size_bytes = 16384}});
  rx_task.start();

  if (usb_ok)
    logger.info("MCP266 console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
