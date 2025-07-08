#pragma once

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "wifi_ap.hpp"

namespace espp {
/// @brief A CLI menu for interacting with a WiFi Access Point (AP) mode.
/// @details This class provides a CLI menu for interacting with a WiFi
///          access point. It provides options for setting the log verbosity,
///          connecting to a WiFi network, disconnecting from a WiFi network,
///          getting the current RSSI (Received Signal Strength Indicator),
///          getting the current IP address, checking if the WiFi is
///          connected, getting the current MAC address, and getting the
///          current WiFi configuration.
///
/// \section wifi_ap_menu_ex1 Example
/// \snippet wifi_example.cpp wifi ap menu example
class WifiApMenu {
public:
  /// @brief Construct a new WifiApMenu object.
  /// @param wifi_ap A reference to the WifiAp object to interact with.
  explicit WifiApMenu(std::reference_wrapper<espp::WifiAp> wifi_ap)
      : wifi_ap_(wifi_ap) {}

  /// @brief Get the CLI menu for the WiFi access point.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the WiFi AP menu that you can use to add to
  ///         a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "ap",
                                 std::string_view description = "WiFi AP menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // set the log verbosity
    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the wifi ap.");

    menu->Insert(
        "hostname",
        [this](std::ostream &out) -> void {
          auto hostname = wifi_ap_.get().get_hostname();
          if (!hostname.empty()) {
            out << fmt::format("Current hostname: {}\n", hostname);
          } else {
            out << "No hostname is currently set.\n";
          }
        },
        "Get the current hostname of the WiFi access point.");
    menu->Insert(
        "hostname", {"hostname"},
        [this](std::ostream &out, const std::string &hostname) -> void {
          if (wifi_ap_.get().set_hostname(hostname)) {
            out << fmt::format("Hostname set to '{}'.\n", hostname);
          } else {
            out << "Failed to set hostname.\n";
          }
        },
        "Set the hostname for the WiFi access point.");

    menu->Insert(
        "ip",
        [this](std::ostream &out) -> void {
          auto ip = wifi_ap_.get().get_ip_address();
          out << fmt::format("Current IP address: {}\n", ip);
        },
        "Get the current IP address of the WiFi connection.");
    menu->Insert(
        "mac",
        [this](std::ostream &out) -> void {
          auto mac = wifi_ap_.get().get_mac_address_string();
          out << fmt::format("Current MAC address: {}\n", mac);
        },
        "Get the current MAC address of the WiFi connection.");
    menu->Insert(
        "ssid",
        [this](std::ostream &out) -> void {
          auto ssid = wifi_ap_.get().get_ssid();
          if (!ssid.empty()) {
            out << fmt::format("Current SSID: {}\n", ssid);
          } else {
            out << "No SSID is currently set.\n";
          }
        },
        "Get the current SSID (Service Set Identifier) of the WiFi connection.");

    menu->Insert(
        "connected",
        [this](std::ostream &out) -> void {
          auto stations = wifi_ap_.get().get_connected_stations();
          out << fmt::format("Number of connected stations: {}\n", stations.size());
          out << fmt::format("Connected stations info:\n");
          if (stations.empty()) {
            out << "No connected stations found.\n";
          } else {
            out << fmt::format("\t{}\n", stations);
          }
        },
        "Print the number and info for connected stations.");

    menu->Insert(
        "rssis",
        [this](std::ostream &out) -> void {
          std::vector<int> rssis = wifi_ap_.get().get_connected_station_rssis();
          if (rssis.empty()) {
            out << "No connected stations found.\n";
          } else {
            out << "RSSI values of connected stations:\n";
            for (const auto &rssi : rssis) {
              out << fmt::format("RSSI: {} dBm\n", rssi);
            }
          }
        },
        "Get the RSSI (Received Signal Strength Indicator) of connected stations.");

    menu->Insert(
        "config",
        [this](std::ostream &out) -> void {
          wifi_config_t config;
          if (wifi_ap_.get().get_saved_config(config)) {
            out << fmt::format("Current WiFi configuration:\n"
                               "SSID: '{}'\n"
                               "Password: '{}'\n",
                               std::string_view(reinterpret_cast<char *>(config.ap.ssid)),
                               std::string_view(reinterpret_cast<char *>(config.ap.password)));
          } else {
            out << "No saved WiFi configuration found.\n";
          }
        },
        "Get the current WiFi configuration.");

    menu->Insert(
        "config", {"ssid"},
        [this](std::ostream &out, const std::string &ssid) -> void {
          std::string password = "";
          if (wifi_ap_.get().set_ssid_and_password(ssid, password)) {
            out << fmt::format("WiFi configuration set successfully:\n"
                               "SSID: '{}'\n"
                               "Password: '{}'\n",
                               ssid, password);
          } else {
            out << "Failed to set WiFi configuration.\n";
          }
        },
        "Set the WiFi configuration with SSID and password.");

    menu->Insert(
        "config", {"ssid", "password"},
        [this](std::ostream &out, const std::string &ssid, const std::string &password) -> void {
          if (wifi_ap_.get().set_ssid_and_password(ssid, password)) {
            out << fmt::format("WiFi configuration set successfully:\n"
                               "SSID: '{}'\n"
                               "Password: '{}'\n",
                               ssid, password);
          } else {
            out << "Failed to set WiFi configuration.\n";
          }
        },
        "Set the WiFi configuration with SSID and password.");

    menu->Insert(
        "start",
        [this](std::ostream &out) -> void {
          if (wifi_ap_.get().start()) {
            out << "WiFi access point started successfully.\n";
          } else {
            out << "Failed to start WiFi access point.\n";
          }
        },
        "Start the WiFi access point.");

    menu->Insert(
        "stop",
        [this](std::ostream &out) -> void {
          if (wifi_ap_.get().stop()) {
            out << "WiFi access point stopped successfully.\n";
          } else {
            out << "Failed to stop WiFi access point.\n";
          }
        },
        "Stop the WiFi access point.");

    return menu;
  }

protected:
  /// @brief Set the log level for the WiFi access point.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      wifi_ap_.get().set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      wifi_ap_.get().set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      wifi_ap_.get().set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      wifi_ap_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      wifi_ap_.get().set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set wifi ap log level to {}.\n", verbosity);
  }

  std::reference_wrapper<espp::WifiAp> wifi_ap_;
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
