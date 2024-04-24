#pragma once

#define __linux__

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"

#include <sdkconfig.h>

#include <cli/cli.h>

#include "line_input.hpp"

#ifdef CONFIG_ESP_CONSOLE_USB_CDC
#error The cli component is currently incompatible with CONFIG ESP_CONSOLE_USB_CDC console.
#endif // CONFIG_ESP_CONSOLE_USB_CDC

namespace espp {
/**
 * @brief Class for implementing a basic Cli using the external cli library.
 *        Reads from the input stream and writes to the output stream. This
 *        version of the class explicitly uses a loop of std::istream::get()
 *        instead of std::getline() so that we can simultaneously read the
 *        stream and print it out.
 *
 * \section cli_ex1 Oneshot CLI Example
 * \snippet cli_example.cpp cli example
 */
class Cli : private cli::CliSession {
public:
  /**
   * @brief Enum for the different types of consoles that can be used.
   */
  enum class ConsoleType {
    UART,            ///< UART console. Requires configuration of the UART port and baud rate.
    USB_SERIAL_JTAG, ///< USB Serial JTAG console, provided by ESP ROM. No configuration required.
    CUSTOM,          ///< Custom VFS around some other driver (e.g. TinyUSB CDC)
  };

  struct ConsoleConfig {
    ConsoleType type;
    union {
      // UART configuration
      struct {
        uart_port_t port;
        int baud_rate;
      } uart;
      // no config required for USB_SERIAL_JTAG
      // CUSTOM_USB_CDC configuration
      struct {
        std::string dev_name;
        esp_vfs_t vfs;
      } custom;
    };
  };

  /**
   * @brief Configure stdin and stdout to use the specified console. This should
   *        be called before creating a Cli object if you want to use std::cin
   *        and std::cout with a console other than the one that the ESP_CONSOLE
   *        was compiled to use.
   *
   * @param config The configuration for the console to use.
   */
  static void configure_stdin_stdout(const ConsoleConfig &config) {
    if (configured_) {
      return;
    }

    switch (config.type) {
    case ConsoleType::UART:
      configure_stdin_stdout_uart(config.uart.port, config.uart.baud_rate);
      break;
    case ConsoleType::USB_SERIAL_JTAG:
      configure_stdin_stdout_usb_serial_jtag();
      break;
    case ConsoleType::CUSTOM:
      configure_stdin_stdout_vfs(config.custom.dev_name, config.custom.vfs);
      break;
    }
  }

  /**
   * @brief Configure stdin and stdout to use whatever the ESP CONSOLE was
   *       compiled to use. This will only work if the ESP_CONSOLE was
   *       configured to use one of the following:
   *       - CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
   *       - CONFIG_ESP_CONSOLE_UART
   *
   *       If you want to use a different console, you should use the other
   *       configure_stdin_stdout() function that takes a ConsoleConfig object.
   *
   * @note If you do not call a configure_stdin_stdout() function before
   *      creating a Cli object, the Cli object will call
   *      configure_stdin_stdout() with no arguments (this function), using
   *      whatever console was compiled to be the ESP primary console.
   */
  static void configure_stdin_stdout(void) {
    if (configured_) {
      return;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    configure_stdin_stdout_usb_serial_jtag();
#else
#if CONFIG_ESP_CONSOLE_UART
    configure_stdin_stdout_uart(CONFIG_ESP_CONSOLE_UART_NUM, CONFIG_ESP_CONSOLE_UART_BAUDRATE);
#else
#error "No console configured, cannot call configure_stdin_stdout() without arguments"
#endif
#endif // CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  }

  /**
   * @brief Configure the UART driver to support blocking input read, so that
   *        std::cin (which assumes a blocking read) will function. This should
   *        be primarily used when you want to use the std::cin/std::getline and
   *        other std input functions or you want to use the cli library.
   *
   *        This code was copied from
   *        https://github.com/espressif/esp-idf/blob/master/examples/common_components/protocol_examples_common/stdin_out.c,
   *        and there is some discussion here:
   *        https://github.com/espressif/esp-idf/issues/9692 and
   *        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/cplusplus.html.
   *
   * @param port The UART port to use.
   * @param baud_rate The baud rate to use.
   */
  static void configure_stdin_stdout_uart(uart_port_t port, int baud_rate) {
    if (configured_) {
      return;
    }

    // drain stdout before reconfiguring it
    fflush(stdout);
    fsync(fileno(stdout));

    // Initialize VFS & UART so we can use std::cout/cin
    // _IOFBF = full buffering
    // _IOLBF = line buffering
    // _IONBF = no buffering
    // disable buffering on stdin
    setvbuf(stdin, nullptr, _IONBF, 0);

    // Configure UART. Note that REF_TICK is used so that the baud rate remains
    // correct while APB frequency is changing in light sleep mode.
    const uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
      .source_clk = UART_SCLK_REF_TICK,
#else
      .source_clk = UART_SCLK_XTAL,
#endif
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(port);
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(port, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(port, ESP_LINE_ENDINGS_CRLF);

    fflush(stdout);
    fsync(fileno(stdout));

    configured_ = true;
  }

  /**
   * @brief Configure the USB Serial JTAG driver to support blocking input read,
   *        so that std::cin (which assumes a blocking read) will function. This
   *        should be primarily used when you want to use the std::cin/std::getline
   *        and other std input functions or you want to use the cli library.
   */
  static void configure_stdin_stdout_usb_serial_jtag(void) {
    if (configured_) {
      return;
    }

    // drain stdout before reconfiguring it
    fflush(stdout);
    fsync(fileno(stdout));

    // Initialize VFS & UART so we can use std::cout/cin
    // _IOFBF = full buffering
    // _IOLBF = line buffering
    // _IONBF = no buffering
    // disable buffering on stdin
    setvbuf(stdin, nullptr, _IONBF, 0);

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&cfg);
    esp_vfs_usb_serial_jtag_use_driver();
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    fflush(stdout);
    fsync(fileno(stdout));

    configured_ = true;
  }

  /**
   * @brief Configure stdin/stdout to use a custom VFS driver. This should be
   *        used when you have a custom VFS driver that you want to use for
   *        std::cin/std::cout, such as when using TinyUSB CDC.
   *
   * @param dev_name The name of the device to use for the VFS driver.
   * @param vfs The VFS driver configuration to use.
   *
   * @note This function must be called before creating a Cli object if you want
   *       to use std::cin/std::cout with a custom VFS driver.
   */
  static void configure_stdin_stdout_vfs(std::string_view dev_name, const esp_vfs_t &vfs) {
    if (configured_) {
      return;
    }

    // drain stdout before reconfiguring it
    fflush(stdout);
    fsync(fileno(stdout));

    // Initialize VFS & UART so we can use std::cout/cin
    // _IOFBF = full buffering
    // _IOLBF = line buffering
    // _IONBF = no buffering
    // disable buffering on stdin
    setvbuf(stdin, nullptr, _IONBF, 0);

    // Register the USB CDC interface
    auto err = esp_vfs_register(dev_name.data(), &vfs, NULL);

    // redirect stdin, stdout, stderr to the USB CDC interface
    freopen(dev_name.data(), "r", stdin);
    freopen(dev_name.data(), "w", stdout);
    freopen(dev_name.data(), "w", stderr);

    fflush(stdout);
    fsync(fileno(stdout));

    configured_ = true;
  }

  /**
   * @brief Construct a Cli object and call
   *        espp::Cli::configure_stdin_stdout() to ensure that std::cin works
   *        as needed.
   * @throw std::invalid_argument if @c _in or @c out are invalid streams
   * @param _cli the cli::Cli object containing the menus and functions to
   *        run.
   * @param _in The input stream from which to read characters.
   * @param _out The output stream to which to write characters.
   */
  explicit Cli(cli::Cli &_cli, std::istream &_in = std::cin, std::ostream &_out = std::cout)
      : CliSession(_cli, _out, 1)
      , exit(false)
      , in(_in) {
    if (!_in.good())
      throw std::invalid_argument("istream invalid");
    if (!_out.good())
      throw std::invalid_argument("ostream invalid");
    ExitAction([this](std::ostream &) { exit = true; });
    // for std::cin to work (it must be blocking), we need to configure the uart
    // driver to have it block. see
    // https://github.com/espressif/esp-idf/issues/9692 for more info
    configure_stdin_stdout();
  }

  /**
   * @brief Set the input history size for this session.
   * @param history_size new History size. Must be >= 1.
   */
  void SetInputHistorySize(size_t history_size) { line_input_.set_history_size(history_size); }

  /**
   * @brief Set the input history - replaces any existing history.
   * @param history new LineInput::History to use.
   */
  void SetInputHistory(const LineInput::History &history) { line_input_.set_history(history); }

  /**
   * @brief Set whether or not to handle resize events.
   * @param handle_resize true to handle resize events, false to ignore them.
   * @warning This is not very robust and should be used with caution.
   */
  void SetHandleResize(bool handle_resize) { line_input_.set_handle_resize(handle_resize); }

  /**
   * @brief Get the input history for this session.
   * @return The current input history for this session.
   */
  LineInput::History GetInputHistory() const { return line_input_.get_history(); }

  /**
   * @brief Start the Cli, blocking until it exits.
   */
  void Start() {
    Enter();

    while (!exit) {
      Prompt();
      if (!in.good())
        Exit();
      auto line = line_input_.get_user_input(
          in, [this]() { Prompt(); }, [this](auto line) { return GetCompletions(line); });
      if (in.eof()) {
        Exit();
      } else {
        Feed(line);
      }
    }
  }

private:
  static bool configured_;

  bool exit;
  LineInput line_input_;
  std::istream &in;
};
} // namespace espp
