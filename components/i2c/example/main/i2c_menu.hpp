#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "i2c.hpp"

class I2cMenu {
public:
  explicit I2cMenu(std::reference_wrapper<espp::I2c> i2c) : i2c_(i2c) {}

  std::unique_ptr<cli::Menu> get(std::string_view name = "i2c",
                                 std::string_view description = "I2c menu") {
    auto i2c_menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // set the log verbosity for the i2c bus
    i2c_menu->Insert(
        "log",
        [this](std::ostream &out, const std::string &verbosity) -> void {
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
        },
        "Set the log verbosity for the I2c bus.");

    // scan the bus for devices
    i2c_menu->Insert(
        "scan",
        [this](std::ostream &out) -> void {
          out << "Scanning I2c bus...\n";
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
        },
        "Scan the I2c bus for devices.");

    // probe for a device (hexadecimal address string)
    i2c_menu->Insert(
        "probe", {"address (hex)"},
        [this](std::ostream &out, const std::string &address_string) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          if (i2c_.get().probe_device(address)) {
            out << fmt::format("Device found at address {:#02x}.\n", address);
          } else {
            out << fmt::format("No device found at address {:#02x}.\n", address);
          }
        },
        "Probe for a device at a specific address, given as a hexadecimal string.");

    // read from a device
    i2c_menu->Insert(
        "read", {"address (hex)", "register"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          uint8_t data;
          std::string log;
          if (i2c_.get().read_at_register(address, reg, &data, 1)) {
            log = fmt::format("Read from address {:#02x} at register {:#02x}: {:#02x}", address,
                              reg, data);
          } else {
            log =
                fmt::format("Error reading from address {:#02x} at register {:#02x}", address, reg);
          }
          out << log << "\n";
        },
        "Read a byte from a device at a specific address and register.");

    // read from a device
    i2c_menu->Insert(
        "read", {"address (hex)", "register", "length (number of bytes to read)"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg,
               uint8_t len) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          std::vector<uint8_t> data(len);
          std::string log;
          if (i2c_.get().read_at_register_vector(address, reg, data)) {
            log = fmt::format("Read {} bytes from address {:#02x} at register {:#02x}: {::#02x}",
                              data.size(), address, reg, data);
          } else {
            log =
                fmt::format("Error reading from address {:#02x} at register {:#02x}", address, reg);
          }
          out << log << "\n";
        },
        "Read len bytes from a device at a specific address and register.");

    // write to a device
    i2c_menu->Insert(
        "write", {"address (hex)", "register", "data byte"},
        [this](std::ostream &out, const std::string &address_string, uint8_t reg,
               uint8_t data) -> void {
          // convert address_string to a uint8_t
          uint8_t address = std::stoi(address_string, nullptr, 16);
          std::vector<uint8_t> data_vec = {reg, data};
          std::string log;
          if (i2c_.get().write_vector(address, data_vec)) {
            log = fmt::format("Wrote data {:#02x} to address {:#02x} at register {:#02x}", data,
                              address, reg);
          } else {
            log = fmt::format("Error writing data {:#02x} to address {:#02x} at register {:#02x}",
                              data, address, reg);
          }
          out << log << "\n";
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
          uint8_t reg = data[0];
          std::string log;
          if (i2c_.get().write_vector(address, data)) {
            log = fmt::format("Wrote {} bytes to address {:#02x} at register {:#02x}: {::#02x}",
                              data.size(), address, reg, data);
          } else {
            log = fmt::format("Error writing {} bytes to address {:#02x} at register {:#02x}",
                              data.size(), address, reg);
          }
          out << log << "\n";
        },
        "Write bytes to a device at a specific address and register.");

    return i2c_menu;
  }

protected:
  std::reference_wrapper<espp::I2c> i2c_;
};
