#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
/// \brief Class for interacting with the LilyGo T-Keyboard.
/// \details This class is used to interact with the LilyGo T-Keyboard. It is
///          designed as a peripheral component for use with a serial
///          interface such as I2C.
///
/// \section Example
/// \snippet t_keyboard_example.cpp tkeyboard example
class TKeyboard {
public:
  /// The default address of the keyboard.
  static constexpr uint8_t DEFAULT_ADDRESS = 0x55;

  /// \brief The function signature for the write function.
  /// \details This function is used to write data to the keyboard.
  /// \param address The address to write to.
  /// \param data The data to write.
  /// \param size The size of the data to write.
  /// \return True if the write was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> write_fn;

  /// \brief The function signature for the read function.
  /// \details This function is used to read data from the keyboard.
  /// \param address The address to read from.
  /// \param data The data to read into.
  /// \param size The size of the data to read.
  /// \return True if the read was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> read_fn;

  /// The configuration structure for the keyboard.
  struct Config {
    /// The write function to use.
    write_fn write;

    /// The read function to use.
    read_fn read;

    /// The address of the keyboard.
    uint8_t address = DEFAULT_ADDRESS;

    /// The log level to use.
    Logger::Verbosity log_level = Logger::Verbosity::WARN;
  };

  /// \brief Constructor for the TKeyboard class.
  /// \param config The configuration to use.
  explicit TKeyboard(const Config &config)
      : write_(config.write), read_(config.read), address_(config.address),
        logger_({.tag = "TKeyboard", .level = config.log_level}) {}

  /// \brief Get the key that is currently pressed.
  /// \param ec The error code to set if an error occurs.
  /// \return The key that is currently pressed.
  uint8_t get_key(std::error_code &ec) { return read_char(ec); }

protected:
  uint8_t read_char(std::error_code &ec) {
    uint8_t data = 0;
    read(&data, 1, ec);
    if (ec) {
      logger_.error("Failed to read char: {}", ec.message());
      return 0;
    }
    return data;
  }

  void write(uint8_t *data, size_t size, std::error_code &ec) {
    if (!write_(address_, data, size)) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  void read(uint8_t *data, size_t size, std::error_code &ec) {
    if (!read_(address_, data, size)) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  write_fn write_;
  read_fn read_;
  uint8_t address_;
  espp::Logger logger_;
};
} // namespace espp
