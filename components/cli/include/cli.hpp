#pragma once

#if defined(ESP_PLATFORM)
#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#define __linux__

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "esp_system.h"

#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"

#include "line_input.hpp"

#ifdef CONFIG_ESP_CONSOLE_USB_CDC
#error The cli component is currently incompatible with CONFIG ESP_CONSOLE_USB_CDC console.
#endif // CONFIG_ESP_CONSOLE_USB_CDC

#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY2(s)
#define STRINGIFY2(s) #s
#endif // STRINGIFY

#include <cli/cli.h>

namespace espp {
/**
 * @brief Class for implementing a basic Cli using the external cli library.
 *        Reads from the input stream and writes to the output stream. This
 *        version of the class explicitly uses a loop of std::istream::get()
 *        instead of std::getline() so that we can simultaneously read the
 *        stream and print it out.
 *
 * @note You should call configure_stdin_stdout() before creating a Cli object
 *       to ensure that std::cin works as needed. If you do not want to use the
 *       Cli over the ESP CONSOLE (e.g. the ESP's UART, USB Serial/JTAG) and
 *       instead want to run it over a different UART port, VFS, or some other
 *       configuration, then you should call one of
 *       - configure_stdin_stdout_uart()
 *       - configure_stdin_stdout_vfs()
 *       - configure_stdin_stdout_custom()
 *       If you do not call any of these functions, the Cli will assume you
 *       intend to use the ESP CONSOLE that was configured via menuconfig, and
 *       will therefore call configure_stdin_stdout() to set it up for you.
 *
 * \section cli_ex1 Oneshot CLI Example
 * \snippet cli_example.cpp cli example
 */
class Cli : private cli::CliSession {
public:
  /**
   * @brief Configure stdin and stdout to use whatever the ESP CONSOLE was
   *       compiled to use. This will only work if the ESP_CONSOLE was
   *       configured to use one of the following:
   *       - CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
   *       - CONFIG_ESP_CONSOLE_UART
   *
   *       If you want to use a different console, you should use one of the
   *       other configure_stdin_stdout functions to configure sdin/stdout or
   *       indicate that you have already configured stdin/stdout and the vfs by
   *       calling configure_stdin_stdout_custom().
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
#elif CONFIG_ESP_CONSOLE_UART
    configure_stdin_stdout_uart((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
                                CONFIG_ESP_CONSOLE_UART_BAUDRATE);
#else
    fmt::print(fg(fmt::terminal_color::red),
               "Cannot configure stdin/stdout: ESP_CONSOLE not configured. "
               "Please call one of the other configure_stdin_stdout functions to "
               "configure stdin/stdout.\n");
#endif // CONFIG_ESP_CONSOLE_*
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
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
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
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    uart_vfs_dev_port_set_rx_line_endings(port, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    uart_vfs_dev_port_set_tx_line_endings(port, ESP_LINE_ENDINGS_CRLF);

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
    usb_serial_jtag_vfs_use_driver();
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

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
   *
   * @note You need to ensure that the read data function in the VFS driver is
   *       blocking, as std::cin assumes blocking reads. For example the
   *       espressif vfs_tinyusb implementation of tusb_read is non-blocking by
   *       default, so you will need to ensure that it blocks until data is
   *       available.
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
    [[maybe_unused]] auto err = esp_vfs_register(dev_name.data(), &vfs, NULL);

    // TODO: this function is mostly untested, so we should probably add some
    //       error handling here and store the resultant pointers for later use
    // redirect stdin, stdout, stderr to the USB CDC interface
    console_.in = freopen(dev_name.data(), "r", stdin);
    console_.out = freopen(dev_name.data(), "w", stdout);
    console_.err = freopen(dev_name.data(), "w", stderr);

    fflush(stdout);
    fsync(fileno(stdout));

    configured_ = true;
  }

  /**
   * @brief Restore the default stdin/stdout after configuring it to use a
   *        custom VFS driver. This should be used when you want to restore
   *        stdin/stdout to the default UART driver after using a custom VFS
   *        driver.
   * @note This function should be called after you are done with the Cli object
   *       and want to restore the default stdin/stdout, but only if you have
   *       called configure_stdin_stdout_vfs() before creating the Cli object.
   */
  static void restore_default_stdin_stdout() {
    if (!configured_) {
      return;
    }
    const char *default_uart_dev = "/dev/uart/" STRINGIFY(CONFIG_ESP_CONSOLE_UART_NUM);
    if (console_.in) {
      stdin = freopen(default_uart_dev, "r", console_.in);
      console_.in = nullptr;
    }
    if (console_.out) {
      stdout = freopen(default_uart_dev, "w", console_.out);
      console_.out = nullptr;
    }
    if (console_.err) {
      stderr = freopen(default_uart_dev, "w", console_.err);
      console_.err = nullptr;
    }
  }

  /**
   * @brief Configure stdin/stdout to use a custom VFS driver. This should be
   *        used when you manually configure the VFS driver and want to use
   *        std::cin/std::cout, such as when using TinyUSB CDC. This function
   *        merely sets the configured flag to true, you will need to configure
   *        the VFS driver yourself.
   *
   * @note This function must be called before creating a Cli object if you want
   *       to use std::cin/std::cout with a custom VFS driver.
   *
   * @note You need to ensure that the read data function in the VFS driver is
   *       blocking, as std::cin assumes blocking reads. For example the
   *       espressif vfs_tinyusb implementation of tusb_read is non-blocking by
   *       default, so you will need to ensure that it blocks until data is
   *       available.
   */
  static void configure_stdin_stdout_custom() { configured_ = true; }

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
   * @brief Set whether or not to send escape sequences.
   * @param send_escape_sequences true to send escape sequences, false otherwise.
   */
  void SetSendEscapeSequences(bool send_escape_sequences) {
    line_input_.set_send_escape_sequences(send_escape_sequences);
  }

  /**
   * @brief Set whether or not to handle control commands.
   * @param handle_control_commands true to handle control commands, false otherwise.
   */
  void SetHandleControlCommands(bool handle_control_commands) {
    line_input_.set_handle_control_commands(handle_control_commands);
  }

  /**
   * @brief Get the input history for this session.
   * @return The current input history for this session.
   */
  LineInput::History GetInputHistory() const { return line_input_.get_history(); }

  /**
   * @brief Start the Cli, blocking until it exits.
   * @param show_prompt true to show the prompt, false otherwise.
   * @param show_completions true to show the completions, false otherwise.
   * @note Show completions requires that SetHandleControlCommands(true) is set.
   *       By default it is on.
   */
  void Start(bool show_prompt = true, bool show_completions = true) {
    espp::LineInput::prompt_fn print_prompt = nullptr;
    if (show_prompt)
      print_prompt = [this]() { Prompt(); };
    espp::LineInput::get_completions_fn get_completions = nullptr;
    if (show_completions)
      get_completions = [this](auto line) { return GetCompletions(line); };

    Enter();
    while (!exit) {
      Prompt();
      if (!in.good())
        Exit();
      auto line = line_input_.get_user_input(in, print_prompt, get_completions);
      if (in.eof()) {
        Exit();
      } else {
        Feed(line);
      }
    }
  }

private:
  typedef struct {
    FILE *in{nullptr};
    FILE *out{nullptr};
    FILE *err{nullptr};
  } console_handle_t;

  static console_handle_t console_;

  static bool configured_;

  bool exit;
  LineInput line_input_;
  std::istream &in;
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS

#endif // ESP_PLATFORM
