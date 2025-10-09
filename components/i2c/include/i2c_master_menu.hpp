#pragma once

#include <sdkconfig.h>

// Only include this menu if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "i2c_master.hpp"

namespace espp {
/// @brief A CLI menu for interacting with an I2c master bus (new API).
/// @details This class provides a CLI menu for interacting with an I2c master bus using the new
/// ESP-IDF I2C master API. It provides options for setting the log verbosity, scanning the bus for
/// devices, probing for a device at a specific address, reading from a device, and writing to a
/// device.
///
/// \section i2c_master_menu_ex1 Example
/// \snippet i2c_example.cpp i2c master menu example
class I2cMasterMenu {
public:
  /// @brief Construct a new I2cMasterMenu object.
  /// @param bus A reference to the I2cMasterBus to interact with.
  explicit I2cMasterMenu(std::reference_wrapper<espp::I2cMasterBus> bus)
      : bus_(bus) {}

  /// @brief Get the I2c master menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the I2c menu that you can use to add to a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "i2c_master",
                                 std::string_view description = "I2c Master menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // Set the log verbosity for the I2c master bus
    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the I2c master bus.");

    // Scan the bus for devices
    menu->Insert(
        "scan", [this](std::ostream &out) -> void { scan_bus(out); },
        "Scan the I2c master bus for devices.");

    // Probe for a device (hexadecimal address string)
    menu->Insert(
        "probe", {"address (hex)"},
        [this](std::ostream &out, const std::string &address_string) -> void {
          uint16_t address = std::stoi(address_string, nullptr, 16);
          probe_device(out, address);
        },
        "Probe for a device at a specific address, given as a hexadecimal string.");

    // Read from a device
    menu->Insert(
        "read", {"address (hex)", "register", "length (number of bytes to read)"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg,
               uint8_t len) -> void {
          uint16_t address = std::stoi(address_string, nullptr, 16);
          read_device(out, address, reg, len);
        },
        "Read len bytes from a device at a specific address and register.");

    // Write to a device
    menu->Insert(
        "write", {"address (hex)", "register (hex)", "data byte (hex)", "data byte (hex)", "..."},
        [this](std::ostream &out, const std::vector<std::string> &args) -> void {
          if (args.size() < 3) {
            out << "Not enough arguments.\n";
            return;
          }
          uint16_t address = std::stoi(args[0], nullptr, 16);
          std::vector<uint8_t> data;
          std::transform(args.begin() + 1, args.end(), std::back_inserter(data),
                         [](const std::string &s) -> uint8_t { return std::stoi(s, nullptr, 0); });
          write_device(out, address, data);
        },
        "Write bytes to a device at a specific address and register.");

    return menu;
  }

protected:
  /// @brief Set the log level for the I2c master bus.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      bus_.get().set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      bus_.get().set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      bus_.get().set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      bus_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      bus_.get().set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set I2c master log level to {}.\n", verbosity);
  }

  /// @brief Scan the I2c master bus for devices.
  /// @param out The output stream to write to.
  void scan_bus(std::ostream &out) {
    out << "Scanning I2c master bus. This may take a while.\n";
    auto prev_log_level = bus_.get().get_log_level();
    // NOTE: we turn off logging for this so we don't spam the console
    bus_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 0; address < 128; address++) {
      std::error_code ec;
      if (bus_.get().probe(address, ec)) {
        found_addresses.push_back(address);
      }
    }
    bus_.get().set_log_level(prev_log_level);
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
  void probe_device(std::ostream &out, uint16_t address) {
    std::error_code ec;
    if (bus_.get().probe(address, ec)) {
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
  void read_device(std::ostream &out, uint16_t address, uint8_t reg, uint8_t len) {
    std::error_code ec;
    auto dev = bus_.get().add_device<uint8_t>({.device_address = address}, ec);
    if (!dev) {
      out << "Failed to create device.\n";
      return;
    }
    std::vector<uint8_t> data(len);
    bool ok = dev->read_register(reg, data, ec);
    if (ok) {
      out << fmt::format("Read {} bytes from address {:#02x} at register {:#02x}: {::#02x}\n",
                         data.size(), address, reg, data);
    } else {
      out << fmt::format("Error reading from address {:#02x} at register {:#02x}: {}\n", address,
                         reg, ec.message());
    }
  }

  /// @brief Write to a device at a specific address and register.
  /// @param out The output stream to write to.
  /// @param address The address to write to.
  /// @param data The data to write (first byte is register, rest is data).
  void write_device(std::ostream &out, uint16_t address, const std::vector<uint8_t> &data) {
    if (data.empty()) {
      out << "No register/data provided.\n";
      return;
    }
    std::error_code ec;
    auto dev = bus_.get().add_device<uint8_t>({.device_address = address}, ec);
    if (!dev) {
      out << "Failed to create device.\n";
      return;
    }
    uint8_t reg = data[0];
    std::vector<uint8_t> payload(data.begin() + 1, data.end());
    bool ok = dev->write_register(reg, payload, ec);
    if (ok) {
      out << fmt::format("Wrote {} bytes to address {:#02x} at register {:#02x}: {::#02x}\n",
                         payload.size(), address, reg, payload);
    } else {
      out << fmt::format("Error writing to address {:#02x} at register {:#02x}: {}\n", address, reg,
                         ec.message());
    }
  }

  std::reference_wrapper<espp::I2cMasterBus> bus_;
};

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API || defined(_DOXYGEN_)
