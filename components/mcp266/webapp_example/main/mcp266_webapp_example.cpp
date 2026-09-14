// USB <-> MCP266 web console example.
//
// Runs the espp::Mcp266 driver on an ESP32-S3 and exposes it to a browser over
// USB: the hosted MCP266 console web app connects on the vendor (WebUSB) OR CDC
// (Web Serial) interface and can configure the position loops, command moves,
// and view live per-axis status (position / velocity / DS402 state) plus device
// telemetry (battery, temperature). All CANopen/DS402 work happens on the
// device inside espp::Mcp266Service (protocol: mcp266_protocol.hpp, dispatcher
// module 6); the web app needs no CANopen knowledge.
//
// Wiring: the ESP32-S3 is the CANopen MASTER of the MCP266 node, so connect the
// TWAI TX/RX GPIOs to a 3.3 V CAN transceiver on a terminated bus at the
// baudrate configured on the MCP266 (Basicmicro Motion Studio). Set kNodeId to
// the MCP266's configured CANopen node id. The system console/logs go to the
// separate built-in USB-Serial-JTAG.

#include <chrono>
#include <cstdint>
#include <mutex>
#include <span>
#include <thread>

#include "canopen_client.hpp"
#include "dispatcher_worker.hpp"
#include "logger.hpp"
#include "mcp266.hpp"
#include "mcp266_service.hpp"
#include "twai.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

// --- device configuration (change to match your board / MCP266) --------------
static constexpr int kCanTxGpio = 17;
static constexpr int kCanRxGpio = 16;
static constexpr uint32_t kCanBaudrate = 1000000;
static constexpr uint8_t kNodeId = 10; // the MCP266's CANopen node id (Motion Studio)

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MCP266 Console", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB<->MCP266 console example (node id {})", kNodeId);

  // --- CAN transport + CANopen client + MCP266 driver ------------------------
  // The Twai receive task feeds process_frame(); the MCP266 SDO transactions run
  // on OTHER tasks (the dispatcher workers + the services' status streamers),
  // satisfying CanopenClient's "pump RX from a different task" contract.
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
  // cppcheck-suppress danglingLifetime // app_main never returns: `client` lives forever
  client_ptr = &client;
  espp::Mcp266 mcp(client, {.log_level = espp::Logger::Verbosity::WARN});

  // A single CANopen SDO channel: every Mcp266 call -- from both services and
  // from this example -- is serialized on one mutex.
  std::mutex mcp_mutex;

  std::error_code twai_ec;
  if (!twai.initialize(twai_ec)) {
    logger.error("Failed to initialize TWAI: {}", twai_ec.message());
  } else {
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

  //! [mcp266_webapp_example]
  // Vendor (WebUSB) and CDC (Web Serial) are independent byte streams, so each
  // gets its own Mcp266Service (replies and the STATUS stream go back on the
  // stream the host is using) and its own DispatcherWorker (one bounded RX
  // queue + worker task feeding one parser, so the blocking SDO command
  // handler never runs in the TinyUSB callback context). Both services share
  // the one driver and its mutex.
  auto vendor_send = [&](std::span<const uint8_t> f) {
    if (!usb.write_vendor(f))
      logger.warn_rate_limited("dropped a {}-byte vendor frame (TX backpressure)", f.size());
  };
  auto cdc_send = [&](std::span<const uint8_t> f) {
    if (!usb.write_cdc(f))
      logger.warn_rate_limited("dropped a {}-byte CDC frame (TX backpressure)", f.size());
  };
  espp::Mcp266Service vendor_service(
      mcp, {.send = vendor_send,
            .mcp_mutex = &mcp_mutex,
            .status_task_config = {.name = "mcp266_status_v", .stack_size_bytes = 8192},
            .log_level = espp::Logger::Verbosity::INFO});
  espp::Mcp266Service cdc_service(
      mcp, {.send = cdc_send,
            .mcp_mutex = &mcp_mutex,
            .status_task_config = {.name = "mcp266_status_c", .stack_size_bytes = 8192},
            .log_level = espp::Logger::Verbosity::INFO});

  espp::DispatcherWorker vendor_link(
      {.send = vendor_send, .task_config = {.name = "mcp266_vendor", .stack_size_bytes = 16384}});
  espp::DispatcherWorker cdc_link(
      {.send = cdc_send, .task_config = {.name = "mcp266_cdc", .stack_size_bytes = 16384}});
  vendor_link.register_module(vendor_service); // module 6 + its discovery metadata
  cdc_link.register_module(cdc_service);
  vendor_link.serve_discovery(usb_cfg.product);
  cdc_link.serve_discovery(usb_cfg.product);

  // The TinyUSB callbacks just queue bytes for the workers.
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });
  //! [mcp266_webapp_example]

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {} — no host transport available",
                 usb_ec.message());
  else
    logger.info("MCP266 console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
