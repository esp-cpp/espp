#pragma once

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <string>

#include "NimBLEDevice.h"

#include "ble_gatt_server.hpp"
#include "cli.hpp"

namespace espp {
/// @brief A CLI menu for interacting with BleGattServer.
/// @details This class provides a CLI menu for interacting with a
/// BleGattServer. It provides options for setting the log verbosity, viewing
/// the paired devices, viewing the connected devices, disconnecting devices,
/// and more.
///
/// \section Example
/// \snippet ble_gatt_server_example.cpp ble gatt server example
class BleGattServerMenu {
public:
  /// @brief Construct a new I2cMenu object.
  /// @param i2c A reference to the I2c bus to interact with.
  explicit BleGattServerMenu(std::reference_wrapper<espp::BleGattServer> server)
      : server_(server) {}

  /// @brief Get the BleGattServer menu.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the BleGattServer menu that you can use to add
  ///         to a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "ble",
                                 std::string_view description = "BLE GATT Server menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // set the log verbosity
    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the server.");

    menu->Insert(
        "battery", [this](std::ostream &out) -> void { get_battery_level(out); },
        "Print the battery level.");
    menu->Insert(
        "battery",
        [this](std::ostream &out, int new_level) -> void { set_battery_level(out, new_level); },
        "Set the battery level.");
    menu->Insert(
        "paired", [this](std::ostream &out) -> void { print_paired_devices(out); },
        "Print the addresses of all paired BLE devices, if any.");
    menu->Insert(
        "connected", [this](std::ostream &out) -> void { print_connected_devices(out); },
        "Print the address of the connected BLE device, if any.");
    menu->Insert(
        "disconnect", [this](std::ostream &out) -> void { disconnect_all(out); },
        "disconnect from the current BLE device");
    menu->Insert(
        "unpair",
        [this](std::ostream &out) -> void {
          NimBLEDevice::deleteAllBonds();
          out << "Unpaired all devices\n";
        },
        "unpair from the current BLE device");

    menu->Insert(
        "is_adv", [this](std::ostream &out) -> void { is_advertising(out); },
        "Is the server advertising?");

    menu->Insert(
        "adv_on_disconnect", {"advertise_on_disconnect"},
        [this](std::ostream &out, bool advertise_on_disconnect) -> void {
          set_advertise_on_disconnect(out, advertise_on_disconnect);
        },
        "Set whether to advertise on disconnect.");

    menu->Insert(
        "adv_stop", [this](std::ostream &out) -> void { stop_advertising(out); },
        "Stop advertising.");

    menu->Insert(
        "adv_start", {"duration (ms)"},
        [this](std::ostream &out, uint32_t duration) -> void { start_advertising(out, duration); },
        "Start advertising.");

    menu->Insert(
        "adv_start", {"duration (ms)", "directed address"},
        [this](std::ostream &out, uint32_t duration, const std::string &directed_address) -> void {
          start_advertising(out, duration, directed_address);
        },
        "Start advertising.");

    menu->Insert(
        "adv_start", {"name", "appearance"},
        [this](std::ostream &out, const std::string &name, uint16_t appearance) -> void {
          start_advertising(out, name, appearance);
        },
        "Start advertising.");

    menu->Insert(
        "whitelist", {"address"},
        [this](std::ostream &out, const std::string &address) -> void { whitelist(out, address); },
        "Add an address to the whitelist.");

    menu->Insert(
        "whitelist", [this](std::ostream &out) -> void { show_whitelist(out); },
        "Show the whitelist.");

    return menu;
  }

protected:
  /// @brief Set the log level for the server.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      server_.get().set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      server_.get().set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      server_.get().set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      server_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      server_.get().set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set log level to {}.\n", verbosity);
  }

  /// @brief Get the battery level.
  /// @param out The output stream to write to.
  void get_battery_level(std::ostream &out) {
    int battery_level = server_.get().battery_service().get_battery_level();
    out << fmt::format("Battery level: {}%\n", battery_level);
  }

  /// @brief Set the battery level.
  /// @param out The output stream to write to.
  /// @param new_level The new battery level.
  /// @note The new battery level must be between 0 and 100, inclusive.
  void set_battery_level(std::ostream &out, int new_level) {
    if (new_level < 0 || new_level > 100) {
      out << "Battery level must be between 0 and 100, inclusive.\n";
      return;
    }
    server_.get().battery_service().set_battery_level(new_level);
    out << fmt::format("Battery set to {}%\n", new_level);
  }

  /// @brief Print the addresses of all paired BLE devices, if any.
  /// @param out The output stream to write to.
  void print_paired_devices(std::ostream &out) {
    std::string output = "";
    auto addresses = server_.get().get_paired_devices();
    output += fmt::format("Paired devices: {}\n", addresses.size());
    for (auto &addr : addresses) {
      output += fmt::format("Paired device: {}\n", addr.toString());
    }
    out << output;
  }

  /// @brief Print the address of the connected BLE device, if any.
  /// @param out The output stream to write to.
  void print_connected_devices(std::ostream &out) {
    std::string output = "";
    auto addresses = server_.get().get_connected_devices();
    output += fmt::format("Connected devices: {}\n", addresses.size());
    for (auto &addr : addresses) {
      output += fmt::format("Connected device: {}\n", addr.toString());
    }
    out << output;
  }

  /// @brief Disconnect from all devices.
  /// @param out The output stream to write to.
  void disconnect_all(std::ostream &out) {
    std::string output = "";
    auto disconnected_addresses = server_.get().disconnect_all();
    output += fmt::format("Disconnected from {} devices\n", disconnected_addresses.size());
    for (auto &addr : disconnected_addresses) {
      output += fmt::format("Disconnected from: {}\n", addr.toString());
    }
    out << output;
  }

  /// @brief Set the advertise on disconnect.
  /// @param out The output stream to write to.
  /// @param advertise_on_disconnect Whether to advertise on disconnect.
  void set_advertise_on_disconnect(std::ostream &out, bool advertise_on_disconnect) {
    server_.get().set_advertise_on_disconnect(advertise_on_disconnect);
    out << "Set advertise on disconnect to " << (advertise_on_disconnect ? "true" : "false")
        << "\n";
  }

  /// @brief Start advertising.
  /// @param out The output stream to write to.
  /// @param name The name to advertise.
  /// @param appearance The appearance to advertise.
  void start_advertising(std::ostream &out, const std::string &name = "espp",
                         uint16_t appearance = 0x0080) {
    espp::BleGattServer::AdvertisingData data;
    data.name = name;
    data.appearance = appearance;
    espp::BleGattServer::AdvertisingParameters params;
    server_.get().start_advertising(data, params);
    out << "Started advertising\n";
  }

  /// @brief Start advertising.
  /// @param out The output stream to write to.
  /// @param duration_ms The duration to advertise for.
  /// @param directed_address The address to advertise to.
  void start_advertising(std::ostream &out, uint32_t duration_ms = 0,
                         const std::string &directed_address = "") {
    NimBLEAddress *addr = nullptr;
    if (directed_address.size() > 0) {
      addr = new NimBLEAddress(directed_address);
    }
    server_.get().start_advertising(duration_ms, addr);
    out << "Started advertising\n";
  }

  /// @brief Stop advertising.
  /// @param out The output stream to write to.
  void stop_advertising(std::ostream &out) {
    server_.get().stop_advertising();
    out << "Stopped advertising\n";
  }

  /// @brief Is the server advertising?
  /// @param out The output stream to write to.
  void is_advertising(std::ostream &out) {
    auto advertising = NimBLEDevice::getAdvertising();
    out << "Advertising: " << (advertising->isAdvertising() ? "true" : "false") << "\n";
  }

  /// @brief Add an address to the whitelist.
  /// @param out The output stream to write to.
  /// @param address The address to add to the whitelist.
  void whitelist(std::ostream &out, const std::string &address) {
    NimBLEAddress addr(address);
    NimBLEDevice::whiteListAdd(addr);
    out << "Added " << address << " to the whitelist\n";
  }

  /// @brief Show the whitelist.
  /// @param out The output stream to write to.
  void show_whitelist(std::ostream &out) {
    auto count = NimBLEDevice::getWhiteListCount();
    out << "Whitelist: " << count << " addresses\n";
    for (size_t i = 0; i < count; i++) {
      auto addr = NimBLEDevice::getWhiteListAddress(i);
      out << addr.toString() << "\n";
    }
  }

  std::reference_wrapper<espp::BleGattServer> server_;
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
