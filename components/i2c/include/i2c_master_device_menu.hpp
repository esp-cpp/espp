#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "i2c_master.hpp"

namespace espp {
/// @brief A CLI menu for interacting with a single I2c master device (new API).
/// @details This class provides a CLI menu for interacting with a single I2c master device using
/// the new ESP-IDF I2C master API. It provides options for setting the log verbosity, probing the
/// device, reading from a register, and writing to a register.
///
/// \section i2c_master_device_menu_ex1 Example
/// \snippet i2c_example.cpp i2c master menu example
///
/// Usage:
///   - Construct with a shared_ptr to an I2cMasterDevice
///   - Use get() to obtain a CLI menu for the device
///
/// \note This class is intended for use with the new ESP-IDF I2C master API (>=5.4.0)
template <typename RegisterType = uint8_t> class I2cMasterDeviceMenu {
public:
  /// @brief Construct a new I2cMasterDeviceMenu object.
  /// @param device A shared_ptr to the I2cMasterDevice to interact with.
  explicit I2cMasterDeviceMenu(std::shared_ptr<espp::I2cMasterDevice<RegisterType>> device)
      : device_(device) {}

  /// @brief Get the I2c master device menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the I2c device menu that you can use to add to a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "i2c_device",
                                 std::string_view description = "I2c Device menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // Set the log verbosity for the I2c device
    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the I2c device.");

    // Probe the device
    menu->Insert(
        "probe", {}, [this](std::ostream &out) -> void { probe_device(out); },
        "Probe for the device on the bus.");

    // Read from a register
    menu->Insert(
        "read", {"register", "length (number of bytes to read)"},
        [this](std::ostream &out, RegisterType reg, uint8_t len) -> void {
          read_register(out, reg, len);
        },
        "Read len bytes from a register.");

    // Write to a register
    menu->Insert(
        "write", {"register", "data byte (hex)", "data byte (hex)", "..."},
        [this](std::ostream &out, const std::vector<std::string> &args) -> void {
          if (args.size() < 2) {
            out << "Not enough arguments.\n";
            return;
          }
          RegisterType reg = static_cast<RegisterType>(std::stoi(args[0], nullptr, 0));
          std::vector<uint8_t> data;
          std::transform(args.begin() + 1, args.end(), std::back_inserter(data),
                         [](const std::string &s) -> uint8_t { return std::stoi(s, nullptr, 0); });
          write_register(out, reg, data);
        },
        "Write bytes to a register.");

    return menu;
  }

protected:
  /// @brief Set the log level for the I2c device.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (!device_) {
      out << "Device not set.\n";
      return;
    }
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
    out << fmt::format("Set I2c device log level to {}.\n", verbosity);
  }

  /// @brief Probe for the device on the bus.
  /// @param out The output stream to write to.
  void probe_device(std::ostream &out) {
    if (!device_) {
      out << "Device not set.\n";
      return;
    }
    std::error_code ec;
    if (device_->probe(ec)) {
      out << "Device found on the bus.\n";
    } else {
      out << fmt::format("Device not found: {}\n", ec.message());
    }
  }

  /// @brief Read from a register.
  /// @param out The output stream to write to.
  /// @param reg The register to read from.
  /// @param len The number of bytes to read.
  void read_register(std::ostream &out, RegisterType reg, uint8_t len) {
    if (!device_) {
      out << "Device not set.\n";
      return;
    }
    std::error_code ec;
    std::vector<uint8_t> data(len);
    bool ok = device_->read_register(reg, data, ec);
    if (ok) {
      out << fmt::format("Read {} bytes from register {:#x}: {::#02x}\n", data.size(), reg, data);
    } else {
      out << fmt::format("Error reading from register {:#x}: {}\n", reg, ec.message());
    }
  }

  /// @brief Write to a register.
  /// @param out The output stream to write to.
  /// @param reg The register to write to.
  /// @param data The data to write.
  void write_register(std::ostream &out, RegisterType reg, const std::vector<uint8_t> &data) {
    if (!device_) {
      out << "Device not set.\n";
      return;
    }
    std::error_code ec;
    bool ok = device_->write_register(reg, data, ec);
    if (ok) {
      out << fmt::format("Wrote {} bytes to register {:#x}: {::#02x}\n", data.size(), reg, data);
    } else {
      out << fmt::format("Error writing to register {:#x}: {}\n", reg, ec.message());
    }
  }

  std::shared_ptr<espp::I2cMasterDevice<RegisterType>> device_;
};

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API || defined(_DOXYGEN_)
