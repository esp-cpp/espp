#pragma once

#define __linux__

#include "esp_err.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

#include <cli/cli.h>

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
  class Cli : public cli::CliSession {
  public:
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
     */
    static void configure_stdin_stdout(void) {
      static bool configured = false;
      if (configured) {
        return;
      }
      // Initialize VFS & UART so we can use std::cout/cin
      setvbuf(stdin, NULL, _IONBF, 0);
      /* Install UART driver for interrupt-driven reads and writes */
      ESP_ERROR_CHECK( uart_driver_install( (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
                                            256, 0, 0, NULL, 0) );
      /* Tell VFS to use UART driver */
      esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
      esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
      /* Move the caret to the beginning of the next line on '\n' */
      esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
      configured = true;
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
    explicit Cli(cli::Cli& _cli, std::istream& _in=std::cin, std::ostream& _out=std::cout)
      : CliSession(_cli, _out, 1),
        exit(false),
        in(_in) {
      if (!_in.good()) throw std::invalid_argument("istream invalid");
      if (!_out.good()) throw std::invalid_argument("ostream invalid");
      ExitAction([this](std::ostream&) {
        exit = true;
      });
      // for std::cin to work (it must be blocking), we need to configure the uart
      // driver to have it block. see
      // https://github.com/espressif/esp-idf/issues/9692 for more info
      configure_stdin_stdout();
    }

    /**
     * @brief Start the Cli, blocking until it exits.
     */
    void Start() {
      Enter();

      while(!exit) {
        Prompt();
        std::string line;
        if (!in.good())
          Exit();
        // NOTE: the original implementation (CliFileSession) uses the
        //       std::getline(in, line) impl, but that (currently) does not
        //       show the input as you are typing, which is annoying.
        char c;
        while (in.get(c)) {
          line += c;
          cli::Cli::cout() << c;
          if (c == '\n')
            break;
        }
        if (in.eof()) {
          Exit();
        } else {
          Feed(line);
        }
      }
    }

  private:
    bool exit;
    std::istream& in;
  };
} // namespace espp
