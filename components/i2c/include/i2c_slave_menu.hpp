#pragma once

#include <sdkconfig.h>

// Only include this menu if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "i2c_slave.hpp"

namespace espp {

/// @brief CLI menu for I2C Slave Device
/// @details
/// This class provides a command-line interface (CLI) menu for interacting with the I2C slave
/// device using the new ESP-IDF I2C slave API.
///
/// \section i2c_slave_menu_ex1 Example
/// \snippet i2c_example.cpp NEW SLAVE API MENU
///
/// Usage:
///   - Construct with a shared pointer to an I2cSlaveDevice
///   - Use add_to_menu() to add I2C slave commands to a CLI menu
///   - Supports reading, writing, and callback registration
///
/// \note This class is intended for use with the new ESP-IDF I2C slave API (>=5.4.0)
class I2cSlaveMenu {
public:
  /// @brief Construct I2C Slave Menu
  /// @param device Shared pointer to the I2C slave device
  explicit I2cSlaveMenu(std::shared_ptr<I2cSlaveDevice> device)
      : device_(device) {}

  /// @brief Get the I2c slave menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the I2c menu that you can use to add to a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "i2c_slave",
                                 std::string_view description = "I2c Slave menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the I2c slave device.");

    menu->Insert(
        "read", {"length"},
        [this](std::ostream &out, size_t len) -> void {
          std::vector<uint8_t> data(len);
          std::error_code ec;
          bool success = device_->read(data.data(), data.size(), ec);
          if (success) {
            out << fmt::format("Read {} bytes from slave: {::#02x}\n", len, data);
          } else {
            out << fmt::format("Error reading from slave: {}\n", ec.message());
          }
        },
        "Read bytes from the slave device.");

    menu->Insert(
        "write", {"data byte", "data byte", "..."},
        [this](std::ostream &out, const std::vector<uint8_t> &data) -> void {
          std::error_code ec;
          bool success = device_->write(data.data(), data.size(), ec);
          if (success) {
            out << fmt::format("Wrote {} bytes to slave: {::#02x}\n", data.size(), data);
          } else {
            out << fmt::format("Error writing to slave: {}\n", ec.message());
          }
        },
        "Write bytes to the slave device.");

    return menu;
  }

protected:
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      device_->set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      device_->set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      device_->set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      device_->set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      device_->set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set I2c slave log level to {}.\n", verbosity);
  }

  std::shared_ptr<I2cSlaveDevice> device_;
};

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API
