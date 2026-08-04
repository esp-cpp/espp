#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include <driver/gpio.h>
#include <driver/uart.h>

#include "base_component.hpp"
#include "nmea_parser.hpp"
#include "task.hpp"

namespace espp {
/// Driver for UART-attached GNSS receivers which output NMEA-0183 sentences,
/// such as the ATGM336H (M5Stack Cardputer-Adv LoRa+GPS Cap) or the u-blox
/// MIA-M10Q (LilyGo T-Deck Plus).
///
/// The driver installs the UART driver, then runs a task which reads the
/// NMEA stream, parses it (see espp::NmeaParser), and maintains the latest
/// fix, which can be retrieved thread-safely with fix() or delivered via the
/// fix callback.
///
/// \section gps_example Example
/// \snippet gps_example.cpp gps example
class Gps : public BaseComponent {
public:
  /// Callback invoked whenever an RMC sentence has been parsed (i.e. once
  /// per receiver update cycle), with the current fix.
  typedef std::function<void(const GpsFix &fix)> fix_callback_fn;

  /// Callback invoked for every valid NMEA sentence received (before
  /// parsing), e.g. for logging or for parsing additional sentence types.
  typedef std::function<void(std::string_view sentence)> sentence_callback_fn;

  /// Configuration for the Gps driver.
  struct Config {
    uart_port_t uart_port = UART_NUM_1; ///< The UART port to use
    gpio_num_t tx_io_num = GPIO_NUM_NC; ///< GPIO connected to the receiver's RX pin (for
                                        ///< sending configuration commands); may be NC
    gpio_num_t rx_io_num = GPIO_NUM_NC; ///< GPIO connected to the receiver's TX pin
    uint32_t baud_rate = 9600;          ///< UART baud rate (9600 for most receivers; the
                                        ///< M5Stack LoRa+GPS Cap ships configured for 115200)
    size_t rx_buffer_size = 2048;       ///< UART RX ring buffer size
    fix_callback_fn on_fix = nullptr;   ///< Optional callback invoked on each fix update
    sentence_callback_fn on_sentence = nullptr; ///< Optional callback invoked per NMEA sentence
    bool auto_start = true;                     ///< Whether to install the driver and start the
                                                ///< read task on construction
    espp::Task::BaseConfig task_config = {
        .name = "gps",
        .stack_size_bytes = 6 * 1024,
    };                                                    ///< Configuration for the reader task
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the driver
  };

  /// Constructor
  /// \param config The configuration for the driver
  explicit Gps(const Config &config);

  /// Destructor - stops the task and deletes the UART driver
  ~Gps();

  /// Install the UART driver and start the reader task.
  /// \param ec The error code to set if there is an error
  /// \return True if started successfully
  bool start(std::error_code &ec);

  /// Stop the reader task and delete the UART driver.
  /// \return True if stopped successfully
  bool stop();

  /// Whether the driver has been started
  /// \return True if the driver is running
  bool is_running() const { return running_; }

  /// Get a copy of the latest fix data.
  /// \return The latest fix
  GpsFix fix() const;

  /// Whether the receiver currently reports a valid fix
  /// \return True if the latest RMC sentence reported a valid fix
  bool has_fix() const;

  /// Write raw data to the receiver (e.g. NMEA/PCAS/UBX configuration
  /// commands). Requires tx_io_num to be set.
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  /// \return True if the data was written
  bool write(std::string_view data, std::error_code &ec);

protected:
  bool read_task(std::mutex &m, std::condition_variable &cv);
  void handle_line(std::string_view line);

  Config config_;
  std::atomic<bool> running_{false};
  bool uart_installed_{false};
  std::unique_ptr<espp::Task> task_;
  mutable std::mutex fix_mutex_;
  NmeaParser parser_;
  std::string line_buffer_;
};
} // namespace espp
