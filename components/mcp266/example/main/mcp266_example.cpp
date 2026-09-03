#include <chrono>
#include <cstdlib>
#include <thread>

#include "canopen_client.hpp"
#include "mcp266.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "MCP266 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting MCP266 CANopen example!");

  //! [mcp266 example]
  // The CANopen node id configured on the MCP266 (Motion Studio -> CAN
  // settings). Change to match your device.
  static constexpr uint8_t node_id = 10;

  // Forward-declared handle so the Twai on_receive callback (registered at
  // Twai construction) can feed frames to the client constructed below.
  static espp::CanopenClient *client_ptr = nullptr;

  // Bring up the TWAI (CAN 2.0) peripheral. Talking to a real MCP266 requires
  // Mode::NORMAL with a 3.3 V CAN transceiver on the tx/rx GPIOs, a properly
  // terminated bus, and a matching baudrate (set in Motion Studio).
  // NOTE: twai / client / mcp are function-local statics: the Twai receive
  // task and the client's send lambda reference them, and app_main() has
  // early-return paths, so static storage keeps them alive for every callback.
  static espp::Twai twai({
      .tx_gpio = 17, // change to match your board / transceiver
      .rx_gpio = 16,
      .baudrate = 1000000,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 10,
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
      .log_level = espp::Logger::Verbosity::WARN,
  });

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
      .sdo_timeout = 500ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  client_ptr = &client;

  static espp::Mcp266 mcp(client, {.log_level = espp::Logger::Verbosity::INFO});

  std::error_code ec;
  if (!twai.initialize(ec)) {
    logger.error("Failed to initialize TWAI: {}", ec.message());
    return;
  }
  // NMT-start the node and clear any latched faults on both axes.
  if (!mcp.start(ec)) {
    logger.error("Failed to start MCP266: {} -- is the node on the bus?", ec.message());
    return;
  }

  // Telemetry sanity check.
  float volts = 0.0f, temp_c = 0.0f;
  if (mcp.read_main_battery_voltage(volts, ec)) {
    logger.info("Main battery: {:.1f} V", volts);
  }
  if (mcp.read_temperature(temp_c, ec)) {
    logger.info("Board temperature: {:.1f} C", temp_c);
  }

  using Axis = espp::Mcp266::Axis;

  // One-time per-boot position-loop setup on M1: widen the MinPos/MaxPos clamp
  // (factory [0, 0] forces every target to zero) and ensure a non-zero
  // position P gain. The MCP reverts to EEPROM on power-up, so this must run
  // every boot before commanding moves. Clear any latched e-stop first.
  mcp.reset_estop(ec);
  if (!mcp.configure_position_loop(Axis::M1, -2'000'000'000, 2'000'000'000, ec)) {
    logger.error("Failed to configure M1 position loop: {}", ec.message());
    return;
  }
  mcp.set_software_position_limits(Axis::M1, -20'000, 20'000, ec);

  // Run a small profile-position sequence and report arrival.
  static constexpr int32_t targets[] = {10'000, -10'000, 0};
  static constexpr uint32_t profile_velocity = 500; // counts/s
  static constexpr uint32_t profile_accel = 500;    // counts/s^2
  static constexpr uint32_t profile_decel = 500;    // counts/s^2
  static constexpr int32_t tolerance = 100;         // counts
  for (int32_t target : targets) {
    logger.info("Moving M1 to {}", target);
    if (!mcp.move_to_position(Axis::M1, target, profile_velocity, profile_accel, profile_decel,
                              ec)) {
      logger.error("Move command rejected: {}", ec.message());
      continue;
    }
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(250ms);
      int32_t position = 0;
      if (mcp.read_encoder(Axis::M1, position, ec) && std::abs(position - target) <= tolerance) {
        logger.info("  reached {} (position={})", target, position);
        break;
      }
    }
  }
  //! [mcp266 example]

  logger.info("MCP266 example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
