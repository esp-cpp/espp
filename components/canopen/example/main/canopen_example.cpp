#include <chrono>
#include <thread>

#include "canopen_client.hpp"
#include "ds402.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // static: captured by the (static) client's heartbeat callback, which must
  // outlive any early return from app_main()
  static espp::Logger logger({.tag = "CANopen Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting CANopen (CiA 301) client example!");

  //! [canopen example]
  // The CANopen node id of the device we want to talk to (e.g. a Basicmicro
  // MCP236/MCP266 motor controller). Change to match your device.
  static constexpr uint8_t node_id = 1;

  // Forward-declared handle so the Twai on_receive callback (registered at
  // Twai construction) can feed frames to the client we construct just below.
  static espp::CanopenClient *client_ptr = nullptr;

  // Bring up the TWAI (CAN 2.0) peripheral. NOTE: talking to a real CANopen
  // device requires Mode::NORMAL with a 3.3V CAN transceiver (e.g. SN65HVD230)
  // wired to the tx/rx GPIOs, a properly terminated bus, and a matching
  // baudrate (Basicmicro MCP2xx default is 250 kbit/s).
  //
  // The on_receive callback runs in the Twai receive task, i.e. NOT in the
  // task performing the (blocking) SDO transactions below -- which is exactly
  // what CanopenClient::process_frame() requires. The CanFrame struct mirrors
  // espp::Twai::Message field-for-field, so conversion is trivial.
  // NOTE: twai and client are function-local STATICS: the Twai receive task
  // and the client's send lambda (which captures &twai) reference them, and
  // app_main() has early-return error paths -- static storage guarantees they
  // outlive every callback regardless of how app_main() exits.
  static espp::Twai twai({
      .tx_gpio = 5, // GPIO5 (change to match your board / transceiver)
      .rx_gpio = 4, // GPIO4 (change to match your board / transceiver)
      .baudrate = 250000,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 5,
      .on_receive =
          [](const espp::Twai::Message &msg) {
            if (client_ptr) {
              client_ptr->process_frame(espp::CanopenClient::CanFrame{
                  .id = msg.id,
                  .extended = msg.extended,
                  .rtr = msg.rtr,
                  .dlc = msg.dlc,
                  .data = msg.data,
              });
            }
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // The CANopen client is transport-agnostic: give it a send function which
  // transmits an espp::detail::CanFrame (here: over TWAI). The CanFrame struct
  // mirrors espp::Twai::Message field-for-field, so conversion is trivial.
  static espp::CanopenClient client({
      .node_id = node_id,
      // captureless: twai has static storage duration and is referenced
      // directly (capturing a static is ill-formed under -Werror)
      .send =
          [](const espp::CanopenClient::CanFrame &frame) {
            espp::Twai::Message msg{
                .id = frame.id,
                .extended = frame.extended,
                .rtr = frame.rtr,
                .dlc = frame.dlc,
                .data = frame.data,
            };
            std::error_code tx_ec;
            return twai.transmit(msg, tx_ec);
          },
      .sdo_timeout = 100ms,
      .on_heartbeat =
          // captureless: logger has static storage duration (see above)
      [](uint8_t hb_node, espp::CanopenClient::NmtState state) {
        logger.info("Heartbeat from node {}: NMT state {}", hb_node, static_cast<int>(state));
      },
      .log_level = espp::Logger::Verbosity::INFO,
  });
  client_ptr = &client;

  std::error_code ec;
  if (!twai.initialize(ec)) {
    logger.error("Failed to initialize TWAI: {}", ec.message());
    return;
  }

  // NMT: put the node into Operational so its PDOs (if any) are active.
  if (!client.nmt_start(ec)) {
    logger.error("Failed to send NMT start: {}", ec.message());
    return;
  }
  logger.info("Sent NMT start to node {}", node_id);
  std::this_thread::sleep_for(100ms);

  // SDO: read the standard identification objects.
  espp::Ds402Drive drive(
      client,
      {.state_timeout = 1s, .poll_period = 20ms, .log_level = espp::Logger::Verbosity::INFO});

  auto device_type = drive.get_device_type(ec);
  if (ec) {
    logger.error("Failed to read device type (0x1000): {} -- is the node on the bus?",
                 ec.message());
    return;
  }
  logger.info("Device type (0x1000): 0x{:08X}", device_type);
  // device profile number is in the lower 16 bits; 402 => a CiA 402 drive
  const bool is_ds402 = (device_type & 0xFFFF) == 402;

  logger.info("Vendor id (0x1018:1): 0x{:08X}", drive.get_vendor_id(ec));
  logger.info("Product code (0x1018:2): 0x{:08X}", drive.get_product_code(ec));
  logger.info("Revision (0x1018:3): 0x{:08X}", drive.get_revision_number(ec));
  logger.info("Serial number (0x1018:4): 0x{:08X}", drive.get_serial_number(ec));
  // manufacturer device name (0x1008) is a string -> segmented SDO upload
  auto name = drive.get_device_name(ec);
  if (!ec) {
    logger.info("Device name (0x1008): '{}'", name);
  }

  if (!is_ds402) {
    logger.warn("Device does not report the CiA 402 profile; skipping motion demo");
  } else {
    // DS402: profile velocity mode, enable, gentle ramp, stop, disable.
    if (auto state = drive.get_state(ec); !ec) {
      logger.info("Drive state: {}", espp::detail::ds402::state_to_string(state));
      if (state == espp::Ds402Drive::State::Fault) {
        logger.info("Drive is in Fault; attempting fault reset");
        if (!drive.fault_reset(ec)) {
          logger.error("Fault reset failed: {}", ec.message());
          return;
        }
      }
    }

    if (!drive.set_mode(espp::Ds402Drive::OperatingMode::ProfileVelocity, ec)) {
      logger.error("Failed to set profile velocity mode: {}", ec.message());
      return;
    }
    // conservative profile accel / decel (device units)
    drive.set_profile_acceleration(1000, ec);
    drive.set_profile_deceleration(1000, ec);

    if (!drive.enable_operation(ec)) {
      logger.error("Failed to enable operation: {}", ec.message());
      return;
    }

    // gentle velocity ramp up and back down
    static constexpr int32_t max_velocity = 500; // device units, keep it gentle
    static constexpr int32_t step = 100;
    for (int32_t v = step; v <= max_velocity; v += step) {
      drive.set_target_velocity(v, ec);
      std::this_thread::sleep_for(500ms);
      logger.info("target={:4}, actual={:4}", v, drive.get_velocity_actual(ec));
    }
    for (int32_t v = max_velocity - step; v >= 0; v -= step) {
      drive.set_target_velocity(v, ec);
      std::this_thread::sleep_for(500ms);
      logger.info("target={:4}, actual={:4}", v, drive.get_velocity_actual(ec));
    }

    // stop and disable the power stage
    drive.set_target_velocity(0, ec);
    if (!drive.disable(ec)) {
      logger.error("Failed to disable drive: {}", ec.message());
    }
    logger.info("Motion demo complete");
  }
  //! [canopen example]

  logger.info("CANopen example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
