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
        "rssi", [this](std::ostream &out) -> void { print_connected_device_rssi(out); },
        "Print the RSSI of all connected devices.");
    menu->Insert(
        "log_rssi", {"duration (ms)"},
        [this](std::ostream &out, int duration_ms) -> void {
          log_connected_device_rssi(out, duration_ms);
        },
        "Log the RSSI of all connected devices every 100 ms as a CSV for the specified <duration> "
        "ms.");
    menu->Insert(
        "log_rssi", {"duration (ms)", "interval (ms)"},
        [this](std::ostream &out, int duration_ms, int interval_ms) -> void {
          log_connected_device_rssi(out, duration_ms, interval_ms);
        },
        "Log the RSSI of all connected devices every <interval> ms as a CSV for the specified "
        "<duration> ms.");
    menu->Insert(
        "disconnect", [this](std::ostream &out) -> void { disconnect_all(out); },
        "disconnect from the current BLE device");
    menu->Insert(
        "unpair",
        [this](std::ostream &out) -> void {
          auto devices = server_.get().unpair_all();
          out << "Unpaired " << devices.size() << " devices individually\n";
          bool success = NimBLEDevice::deleteAllBonds();
          if (!success) {
            out << "Then failed to deleteAllBonds\n";
          } else {
            out << "Then deleted all bonds\n";
          }
        },
        "unpair from the current BLE device");

    menu->Insert(
        "is_adv", [this](std::ostream &out) -> void { is_advertising(out); },
        "Is the server advertising?");

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    menu->Insert(
        "adv_on_disconnect", {"advertise_on_disconnect"},
        [this](std::ostream &out, bool advertise_on_disconnect) -> void {
          set_advertise_on_disconnect(out, advertise_on_disconnect);
        },
        "Set whether to advertise on disconnect.");
#endif // !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

    menu->Insert(
        "adv_stop", [this](std::ostream &out) -> void { stop_advertising(out); },
        "Stop advertising.");

    menu->Insert(
        "adv_start", [this](std::ostream &out) -> void { start_advertising(out); },
        "Start advertising.");

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    menu->Insert(
        "adv_start", {"instance id", "duration (ms)"},
        [this](std::ostream &out, uint8_t instance_id, uint32_t duration) -> void {
          start_advertising(out, instance_id, duration);
        },
        "Start advertising.");

    menu->Insert(
        "adv_stop", {"instance id"},
        [this](std::ostream &out, uint8_t instance_id) -> void {
          stop_advertising(out, instance_id);
        },
        "Stop advertising.");
#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
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
#endif // !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

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
    auto infos = server_.get().get_connected_device_infos();
    if (infos.size() == 0) {
      out << "No connected devices\n";
      return;
    }
    output += fmt::format("Connected devices: {}\n", infos.size());
    for (auto &info : infos) {
      output += fmt::format(
          "Connected device: [{}] {} '{}'\n", server_.get().get_connected_device_rssi(info),
          info.getAddress().toString(), server_.get().get_connected_device_name(info));
    }
    out << output;
  }

  /// @brief Print out the RSSI of all connected devices.
  /// @param out The output stream to write to.
  void print_connected_device_rssi(std::ostream &out) {
    std::string output = "";
    auto infos = server_.get().get_connected_device_infos();
    if (infos.size() == 0) {
      out << "No connected devices\n";
      return;
    }
    output += fmt::format("Connected devices: {}\n", infos.size());
    for (auto &info : infos) {
      output +=
          fmt::format("Connected device RSSI: [{}] {}\n",
                      server_.get().get_connected_device_rssi(info), info.getAddress().toString());
    }
    out << output;
  }

  /// @brief Log the RSSI of all connected devices as a CSV until the user presses
  ///       the enter key.
  /// @param out The output stream to write to.
  /// @param duration_ms The duration to log the RSSI for.
  /// @param interval_ms The interval to log the RSSI at.
  void log_connected_device_rssi(std::ostream &out, int duration_ms = 0, int interval_ms = 100) {
    if (duration_ms <= 0) {
      out << "Duration must be greater than 0\n";
      return;
    }
    if (interval_ms <= 0) {
      out << "Interval must be greater than 0\n";
      return;
    }
    auto infos = server_.get().get_connected_device_infos();
    if (infos.size() == 0) {
      out << "No connected devices\n";
      return;
    }
    out << fmt::format("Connected devices: {}\n", infos.size());
    out << fmt::format("% time (s)");
    for (auto &info : infos) {
      out << fmt::format(", RSSI for {}", info.getAddress().toString());
    }
    out << "\n";
    uint64_t start_time = esp_timer_get_time();
    while (true) {
      uint64_t t_us = esp_timer_get_time();
      auto elapsed_ms = (t_us - start_time) / 1000;
      // see if the requested time has elapsed
      if (elapsed_ms >= duration_ms) {
        break;
      }
      out << fmt::format("{:.2f}", elapsed_ms / 1000.0f);
      for (auto &info : infos) {
        out << fmt::format(", {}", server_.get().get_connected_device_rssi(info));
      }
      out << "\n";
      // otherwise sleep for 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
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

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Set the advertise on disconnect.
  /// @param out The output stream to write to.
  /// @param advertise_on_disconnect Whether to advertise on disconnect.
  void set_advertise_on_disconnect(std::ostream &out, bool advertise_on_disconnect) {
    server_.get().set_advertise_on_disconnect(advertise_on_disconnect);
    out << "Set advertise on disconnect to " << (advertise_on_disconnect ? "true" : "false")
        << "\n";
  }
#endif // !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

  /// @brief Start advertising.
  /// @param out The output stream to write to.
  void start_advertising(std::ostream &out) {
    server_.get().start_advertising();
    out << "Started advertising\n";
  }

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Start advertising.
  /// @param out The output stream to write to.
  /// @param instance_id The instance to advertise on.
  /// @param duration_ms The duration to advertise for.
  void start_advertising(std::ostream &out, int instance_id, uint32_t duration_ms = 0) {
    server_.get().start_advertising(duration_ms, instance_id);
    out << "Started advertising on instance " << instance_id << "\n";
  }
#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Start advertising.
  /// @param out The output stream to write to.
  /// @param duration_ms The duration to advertise for.
  /// @param directed_address The address to advertise to.
  /// @param addr_type The address type to advertise to.
  void start_advertising(std::ostream &out, uint32_t duration_ms,
                         const std::string &directed_address = "",
                         uint8_t addr_type = BLE_ADDR_PUBLIC) {
    NimBLEAddress *addr = nullptr;
    if (directed_address.size() > 0) {
      addr = new NimBLEAddress(directed_address, addr_type);
    }
    server_.get().start_advertising(duration_ms, addr);
    out << "Started advertising\n";
  }
#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

  /// @brief Stop advertising.
  /// @param out The output stream to write to.
  void stop_advertising(std::ostream &out) {
    server_.get().stop_advertising();
    out << "Stopped advertising\n";
  }

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Stop advertising.
  /// @param out The output stream to write to.
  /// @param instance_id The instance to stop advertising on.
  void stop_advertising(std::ostream &out, int instance_id) {
    server_.get().stop_advertising(instance_id);
    out << "Stopped advertising on instance " << instance_id << "\n";
  }
#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

  /// @brief Is the server advertising?
  /// @param out The output stream to write to.
  void is_advertising(std::ostream &out) {
    auto advertising = NimBLEDevice::getAdvertising();
    out << "Advertising: " << (advertising->isAdvertising() ? "true" : "false") << "\n";
  }

  /// @brief Add an address to the whitelist.
  /// @param out The output stream to write to.
  /// @param address The address to add to the whitelist.
  /// @param addr_type The address type to add to the whitelist.
  void whitelist(std::ostream &out, const std::string &address,
                 uint8_t addr_type = BLE_ADDR_PUBLIC) {
    NimBLEAddress addr(address, addr_type);
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
