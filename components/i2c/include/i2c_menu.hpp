#pragma once

#include <sdkconfig.h>

// Only include this menu if the legacy API is selected
#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "i2c.hpp"

namespace espp {
/// @brief A CLI menu for interacting with an I2c bus.
/// @details This class provides a CLI menu for interacting with an I2c bus.
/// It provides options for setting the log verbosity, scanning the bus for
/// devices, probing for a device at a specific address, reading from a device,
/// and writing to a device.
///
/// \section i2c_menu_ex1 Example
/// \snippet i2c_example.cpp i2c menu example
class I2cMenu {
public:
  /// @brief Construct a new I2cMenu object.
  /// @param i2c A reference to the I2c bus to interact with.
  explicit I2cMenu(std::reference_wrapper<espp::I2c> i2c)
      : i2c_(i2c) {}

  /// @brief Get the I2c menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the I2c menu that you can use to add to a
  ///         CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "i2c",
                                 std::string_view description = "I2c menu") {
    auto i2c_menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // set the log verbosity for the i2c bus
    i2c_menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the I2c bus.");

    // scan the bus for devices
    //
    // NOTE: this will take a while to run, as it will probe all 128
    // possible and the hard-coded timeout on the I2C (inside ESP-IDF) is 1
    // second (I2C_CMD_ALIVE_INTERVAL_TICK within
    // esp-idf/components/driver/i2c/i2c.c).
    i2c_menu->Insert(
        "scan", [this](std::ostream &out) -> void { scan_bus(out); },
        "Scan the I2c bus for devices.");

    // probe for a device (hexadecimal address string)
    i2c_menu->Insert(
        "probe", {"address (hex)"},
        [this](std::ostream &out, const std::string &address_string) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          probe_device(out, address);
        },
        "Probe for a device at a specific address, given as a hexadecimal string.");

    // read from a device
    i2c_menu->Insert(
        "read", {"address (hex)", "register"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          read_device(out, address, reg, 1);
        },
        "Read a byte from a device at a specific address and register.");

    // read from a device
    i2c_menu->Insert(
        "read", {"address (hex)", "register", "length (number of bytes to read)"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg,
               uint8_t len) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          read_device(out, address, reg, len);
        },
        "Read len bytes from a device at a specific address and register.");

    // write to a device
    i2c_menu->Insert(
        "write", {"address (hex)", "register", "data byte"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg,
               uint8_t data) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          std::vector<uint8_t> data_vector = {reg, data};
          write_device(out, address, data_vector);
        },
        "Write a byte to a device at a specific address and register.");

    // write to a device
    i2c_menu->Insert(
        "write", {"address (hex)", "register (hex)", "data byte (hex)", "data byte (hex)", "..."},
        [this](std::ostream &out, const std::vector<std::string> &args) -> void {
          // parse the args into address, reg, and data
          if (args.size() < 3) {
            out << "Not enough arguments.\n";
            return;
          }
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(args[0], nullptr, 16);
          // remove the address byte (first element) and convert the rest of the
          // vector of strings into a vector of bytes
          std::vector<uint8_t> data;
          std::transform(args.begin() + 1, args.end(), std::back_inserter(data),
                         [](const std::string &s) -> uint8_t { return std::stoi(s, nullptr, 0); });
          write_device(out, address, data);
        },
        "Write bytes to a device at a specific address and register.");

    return i2c_menu;
  }

protected:
  /// @brief Set the log level for the I2c bus.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      i2c_.get().set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      i2c_.get().set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      i2c_.get().set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      i2c_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      i2c_.get().set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set I2c log level to {}.\n", verbosity);
  }

  /// @brief Scan the I2c bus for devices.
  /// @param out The output stream to write to.
  /// @note This will take a while to run, as it will probe all 128 possible
  ///       addresses and the hard-coded timeout on the I2C (inside ESP-IDF) is
  ///       1 second (I2C_CMD_ALIVE_INTERVAL_TICK within
  ///       esp-idf/components/driver/i2c/i2c.c).
  void scan_bus(std::ostream &out) {
    out << "Scanning I2c bus. This may take a while if you have not updated your ESP-IDF's "
           "I2C_CMD_ALIVE_INTERVAL_TICK.\n";
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 1; address < 128; address++) {
      if (i2c_.get().probe_device(address)) {
        found_addresses.push_back(address);
      }
    }
    if (found_addresses.empty()) {
      out << "No devices found.\n";
    } else {
      std::string log = fmt::format("Found devices at addresses: {::#02x}", found_addresses);
      out << log << "\n";
    }
  }

  /// @brief Probe for a device at a specific address.
  /// @param out The output stream to write to.
  /// @param address The address to probe for.
  void probe_device(std::ostream &out, uint8_t address) {
    if (i2c_.get().probe_device(address)) {
      out << fmt::format("Device found at address {:#02x}.\n", address);
    } else {
      out << fmt::format("No device found at address {:#02x}.\n", address);
    }
  }

  /// @brief Read from a device at a specific address and register.
  /// @param out The output stream to write to.
  /// @param address The address to read from.
  /// @param reg The register to read from.
  /// @param len The number of bytes to read.
  void read_device(std::ostream &out, uint8_t address, uint8_t reg, uint8_t len) {
    std::vector<uint8_t> data(len);
    std::string log;
    if (i2c_.get().read_at_register_vector(address, reg, data)) {
      log = fmt::format("Read {} bytes from address {:#02x} at register {:#02x}: {::#02x}",
                        data.size(), address, reg, data);
    } else {
      log = fmt::format("Error reading from address {:#02x} at register {:#02x}", address, reg);
    }
    out << log << "\n";
  }

  /// @brief Write to a device at a specific address and register.
  /// @param out The output stream to write to.
  /// @param address The address to write to.
  /// @param data The data to write. Should include any register address as the
  ///             first byte(s).
  void write_device(std::ostream &out, uint8_t address, std::vector<uint8_t> data) {
    std::string log;
    if (i2c_.get().write_vector(address, data)) {
      log = fmt::format("Wrote {} bytes to address {:#02x}: {::#02x}", data.size(), address, data);
    } else {
      log = fmt::format("Error writing {} bytes to address {:#02x}", data.size(), address);
    }
    out << log << "\n";
  }

  std::reference_wrapper<espp::I2c> i2c_;
};
} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_LEGACY_API
