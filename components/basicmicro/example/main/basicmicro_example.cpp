#include <chrono>
#include <thread>

#include "driver/uart.h"

#include "basicmicro.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

// UART wiring for the MCP: it defaults to packet serial mode at 38400 baud; S1
// (controller RX) goes to our TX pin and S2 (controller TX) goes to our RX pin.
// File-scope so the captureless transport lambdas below can reference them
// without any capture-semantics ambiguity.
static constexpr uart_port_t uart_port = UART_NUM_1;
static constexpr int uart_tx_pin = 17; // -> MCP S1
static constexpr int uart_rx_pin = 16; // <- MCP S2
static constexpr int uart_baud = 38400;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Basicmicro Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting basicmicro example");

  //! [basicmicro example]

  uart_config_t uart_config = {};
  uart_config.baud_rate = uart_baud;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(uart_port, 256, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(uart_port, uart_tx_pin, uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  espp::Basicmicro mcp({
      .address = 0x80, // default packet serial address (0x80 - 0x87)
      .write =
          [](std::span<const uint8_t> data) {
            const int written = uart_write_bytes(
                uart_port, reinterpret_cast<const char *>(data.data()), data.size());
            return written == static_cast<int>(data.size());
          },
      .read = [](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        const int read =
            uart_read_bytes(uart_port, data.data(), data.size(), pdMS_TO_TICKS(timeout.count()));
        return read < 0 ? 0 : static_cast<size_t>(read);
      },
      // must be >= 10 ms: a 10 ms quiet gap is also what clears the
      // controller's packet buffer after a communication error
      .timeout = 20ms,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;

  // identify the controller
  std::string version;
  if (mcp.read_firmware_version(version, ec)) {
    logger.info("Firmware version: '{}'", version);
  } else {
    logger.error("Could not read firmware version: {}", ec.message());
    logger.error("Is the controller connected, powered, and in packet serial mode?");
  }

  float volts{0};
  if (mcp.read_main_battery_voltage(volts, ec))
    logger.info("Main battery: {:.1f} V", volts);

  uint32_t status{0};
  if (mcp.read_status(status, ec))
    logger.info("Status: 0x{:08X}{}", status, status == 0 ? " (normal)" : "");

  // start from a known encoder state
  if (mcp.reset_encoders(ec))
    logger.info("Encoders reset");

  // gentle speed ramp on M1 (up to ~12.5% duty) with encoder readback, then
  // back down to a stop. Duty-cycle drive works without a tuned velocity PID;
  // if your encoders + PID are configured, try drive_speed(Axis::M1, ...)
  // instead. The channel is selected with the shared espp::MotorAxis enum.
  using Axis = espp::Basicmicro::Axis;
  static constexpr int16_t max_duty = 4096; // of 32767
  static constexpr int16_t step = 512;
  for (int16_t duty = 0; duty <= max_duty; duty = static_cast<int16_t>(duty + step)) {
    if (!mcp.drive_duty(Axis::M1, duty, ec)) {
      logger.error("drive_duty(M1, {}) failed: {}", duty, ec.message());
      break;
    }
    std::this_thread::sleep_for(250ms);
    int32_t count{0};
    int32_t speed{0};
    uint8_t direction{0};
    if (mcp.read_encoder(Axis::M1, count, ec) && mcp.read_speed(Axis::M1, speed, direction, ec)) {
      logger.info("duty {:5d}: encoder count = {:10d}, speed = {} pulses/s ({})", duty, count,
                  speed, direction ? "backward" : "forward");
    }
  }
  for (int16_t duty = max_duty; duty >= 0; duty = static_cast<int16_t>(duty - step)) {
    if (!mcp.drive_duty(Axis::M1, duty, ec))
      break;
    std::this_thread::sleep_for(100ms);
  }

  // make sure the motor is stopped
  if (mcp.drive_duty(Axis::M1, 0, ec))
    logger.info("Motor stopped");

  //! [basicmicro example]

  // periodically log some telemetry
  while (true) {
    float amps_m1{0}, amps_m2{0}, temperature{0};
    if (mcp.read_currents(amps_m1, amps_m2, ec))
      logger.info("Currents: M1 = {:.2f} A, M2 = {:.2f} A", amps_m1, amps_m2);
    if (mcp.read_temperature(temperature, ec))
      logger.info("Temperature: {:.1f} C", temperature);
    std::this_thread::sleep_for(5s);
  }
}
