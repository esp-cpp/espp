#include <cstdio>
#include <cstring>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#if defined(ESP_PLATFORM)
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_vfs_dev.h"
#include <sdkconfig.h>
#endif

#include "logger.hpp"
#include "odrive_ascii.hpp"

namespace {
#if defined(ESP_PLATFORM)
static void configure_blocking_console() {
  // Drain stdout before reconfiguring it
  fflush(stdout);
  fsync(fileno(stdout));

  // Disable buffering on stdin so std::cin/getc block as expected
  setvbuf(stdin, nullptr, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&cfg);
  usb_serial_jtag_vfs_use_driver();
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
#else
  // Default to UART console
  const uart_port_t port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
  const int baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE;

  // Configure UART (REF_TICK vs XTAL per target)
  uart_config_t uart_config = {};
  uart_config.baud_rate = baud_rate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
  uart_config.source_clk = UART_SCLK_REF_TICK;
#else
  uart_config.source_clk = UART_SCLK_XTAL;
#endif
  /* Install UART driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK(uart_driver_install(port, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
  /* Tell VFS to use UART driver */
  uart_vfs_dev_use_driver(port);
  /* CR for RX; CRLF for TX to be friendly with common terminals */
  uart_vfs_dev_port_set_rx_line_endings(port, ESP_LINE_ENDINGS_CR);
  uart_vfs_dev_port_set_tx_line_endings(port, ESP_LINE_ENDINGS_CRLF);
#endif

  fflush(stdout);
  fsync(fileno(stdout));
}
#endif // ESP_PLATFORM
} // namespace

extern "C" void app_main(void) {
  using namespace espp;

  Logger logger({.tag = "ODriveASCIIExample", .level = Logger::Verbosity::INFO});

  //! [odrive_ascii_basic_example]

  // Simulated motor state
  struct {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
  } state;

  OdriveAscii::Config cfg;
  cfg.log_level = Logger::Verbosity::INFO;
  OdriveAscii proto(cfg);

  // Register some read/write properties matching ODrive-style paths
  proto.register_float_property(
      "axis0.encoder.pos_estimate", [&]() { return state.position; },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  proto.register_float_property("axis0.encoder.vel_estimate", [&]() { return state.velocity; });
  proto.register_float_property(
      "axis0.controller.input_pos", [&]() { return state.position; },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });

  // High-rate command callbacks
  proto.on_position_command([&](int axis, float pos, std::optional<float> vel_ff,
                                std::optional<float> torque_ff, std::error_code &ec) {
    (void)axis;
    ec.clear();
    state.position = pos;
    if (vel_ff.has_value())
      state.velocity = *vel_ff;
    if (torque_ff.has_value())
      state.torque = *torque_ff;
    return true;
  });
  proto.on_velocity_command(
      [&](int axis, float vel, std::optional<float> torque_ff, std::error_code &ec) {
        (void)axis;
        ec.clear();
        state.velocity = vel;
        if (torque_ff.has_value())
          state.torque = *torque_ff;
        return true;
      });
  proto.on_torque_command([&](int axis, float tq, std::error_code &ec) {
    (void)axis;
    ec.clear();
    state.torque = tq;
    return true;
  });
  // Trajectory: t <axis> <goal_pos_turns>
  proto.on_trajectory_command([&](int axis, float goal, std::error_code &ec) {
    (void)axis;
    ec.clear();
    state.position = goal;
    return true;
  });
  // Feedback: f <axis> -> "pos vel\n"
  proto.on_feedback_request([&](int axis, float &pos_out, float &vel_out, std::error_code &ec) {
    (void)axis;
    ec.clear();
    pos_out = state.position;
    vel_out = state.velocity;
    return true;
  });
  // Encoder set absolute: es <axis> <abs_pos_turns>
  proto.on_encoder_set_absolute([&](int axis, float abs_pos, std::error_code &ec) {
    (void)axis;
    ec.clear();
    state.position = abs_pos;
    return true;
  });

  // ------------------------ Basic scripted self-test ------------------------
  {
    const char *script = "r axis0.encoder.pos_estimate\r\n"
                         "w axis0.controller.input_pos 12.34\n"
                         "r axis0.encoder.pos_estimate\n"
                         "p 0 1.0 0.5 0.1\r\n"
                         "v 0 2.0 0.2\r\n"
                         "t 0 0.3\n";

    std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t *>(script), strlen(script));
    auto response = proto.process_bytes(bytes);

    std::string out(reinterpret_cast<const char *>(response.data()), response.size());
    logger.info("Scripted responses:\n{}", out);
  }
  //! [odrive_ascii_basic_example]

  // ---------------------------- Interactive mode ----------------------------
  logger.info("Entering interactive ODrive ASCII mode.");
  logger.info("Send commands like: 'r axis0.encoder.pos_estimate' or 'p 0 1.0 0.5 0.1'");

  //! [odrive_ascii_console_example]
#if defined(ESP_PLATFORM)
  configure_blocking_console();
#endif

  // Read from stdin and feed the protocol; print any responses to stdout
  std::vector<uint8_t> rx;
  rx.reserve(128);

  while (true) {
    int ch = std::getc(stdin);
    if (ch == EOF) {
      // Brief yield; on ESP this shouldn't happen under normal console use
      continue;
    }
    uint8_t b = static_cast<uint8_t>(ch & 0xFF);
    rx.push_back(b);

    // Process in small chunks or on line endings for responsiveness
    if (b == '\n' || b == '\r' || rx.size() >= 64) {
      auto resp = proto.process_bytes(rx);
      rx.clear();
      if (!resp.empty()) {
        // Write raw bytes out
        (void)fwrite(resp.data(), 1, resp.size(), stdout);
        fflush(stdout);
      }
    }
  }
  //! [odrive_ascii_console_example]
}
