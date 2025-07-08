#pragma once

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"
#include "wifi_sta.hpp"

namespace espp {
/// @brief A CLI menu for interacting with a WiFi station (STA) mode.
/// @details This class provides a CLI menu for interacting with a WiFi
///          station. It provides options for setting the log verbosity,
///          connecting to a WiFi network, disconnecting from a WiFi network,
///          getting the current RSSI (Received Signal Strength Indicator),
///          getting the current IP address, checking if the WiFi is
///          connected, getting the current MAC address, and getting the
///          current WiFi configuration.
///
/// \section wifi_sta_menu_ex1 Example
/// \snippet wifi_example.cpp wifi sta menu example
class WifiStaMenu {
public:
  /// @brief Construct a new WifiStaMenu object.
  /// @param wifi_sta A reference to the WifiSta object to interact with.
  explicit WifiStaMenu(std::reference_wrapper<espp::WifiSta> wifi_sta)
      : wifi_sta_(wifi_sta) {}

  /// @brief Get the CLI menu for the WiFi station.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the WiFi STA menu that you can use to add to
  ///         a CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "sta",
                                 std::string_view description = "WiFi STA menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // set the log verbosity
    menu->Insert(
        "log", {"verbosity"},
        [this](std::ostream &out, const std::string &verbosity) -> void {
          set_log_level(out, verbosity);
        },
        "Set the log verbosity for the wifi sta.");

    menu->Insert(
        "hostname",
        [this](std::ostream &out) -> void {
          auto hostname = wifi_sta_.get().get_hostname();
          if (!hostname.empty()) {
            out << fmt::format("Current hostname: {}\n", hostname);
          } else {
            out << "No hostname is currently set.\n";
          }
        },
        "Get the current hostname of the WiFi station.");
    menu->Insert(
        "hostname", {"hostname"},
        [this](std::ostream &out, const std::string &hostname) -> void {
          if (wifi_sta_.get().set_hostname(hostname)) {
            out << fmt::format("Hostname set to '{}'\n", hostname);
          } else {
            out << "Failed to set hostname.\n";
          }
        },
        "Set the hostname for the WiFi station.");

    menu->Insert(
        "connect",
        [this](std::ostream &out) -> void {
          if (wifi_sta_.get().connect()) {
            out << "Connected to WiFi network " << wifi_sta_.get().get_ssid() << ".\n";
          } else {
            out << "Failed to connect to saved WiFi network.\n";
          }
        },
        "Connect to a WiFi network using the saved configuration (if any).");
    menu->Insert(
        "connect", {"ssid", "password"},
        [this](std::ostream &out, const std::string &ssid, const std::string &password) -> void {
          if (wifi_sta_.get().connect(ssid, password)) {
            out << "Connected to WiFi network " << ssid << ".\n";
          } else {
            out << "Failed to connect to WiFi network " << ssid << ".\n";
          }
        },
        "Connect to a WiFi network with the given SSID and password.");
    menu->Insert(
        "disconnect",
        [this](std::ostream &out) -> void {
          if (wifi_sta_.get().disconnect()) {
            out << "Disconnected from WiFi network.\n";
          } else {
            out << "Failed to disconnect from WiFi network.\n";
          }
        },
        "Disconnect from the current WiFi network.");

    menu->Insert(
        "restart_dhcp_client",
        [this](std::ostream &out) -> void {
          if (wifi_sta_.get().restart_dhcp_client()) {
            out << "DHCP client restarted successfully.\n";
          } else {
            out << "Failed to restart DHCP client.\n";
          }
        },
        "Restart the DHCP client to renew the IP address from the DHCP server.");

    menu->Insert(
        "ssid",
        [this](std::ostream &out) -> void {
          auto ssid = wifi_sta_.get().get_ssid();
          if (!ssid.empty()) {
            out << fmt::format("Current SSID: {}\n", ssid);
          } else {
            out << "No SSID is currently set.\n";
          }
        },
        "Get the current SSID (Service Set Identifier) of the WiFi connection.");

    menu->Insert(
        "rssi",
        [this](std::ostream &out) -> void {
          int32_t rssi = wifi_sta_.get().get_rssi();
          if (rssi != INT32_MIN) {
            out << fmt::format("Current RSSI: {} dBm\n", rssi);
          } else {
            out << "Failed to get RSSI.\n";
          }
        },
        "Get the current RSSI (Received Signal Strength Indicator) of the WiFi connection.");

    menu->Insert(
        "ip",
        [this](std::ostream &out) -> void {
          auto ip = wifi_sta_.get().get_ip();
          out << fmt::format("Current IP address: {}\n", ip);
        },
        "Get the current IP address of the WiFi connection.");

    menu->Insert(
        "connected",
        [this](std::ostream &out) -> void {
          if (wifi_sta_.get().is_connected()) {
            out << "WiFi is connected.\n";
          } else {
            out << "WiFi is not connected.\n";
          }
        },
        "Check if the WiFi is connected.");

    menu->Insert(
        "mac",
        [this](std::ostream &out) -> void {
          auto mac = wifi_sta_.get().get_mac();
          out << fmt::format("Current MAC address: {}\n", mac);
        },
        "Get the current MAC address of the WiFi connection.");
    menu->Insert(
        "bssid",
        [this](std::ostream &out) -> void {
          auto bssid = wifi_sta_.get().get_bssid();
          if (!bssid.empty()) {
            out << fmt::format("Current BSSID: {}\n", bssid);
          } else {
            out << "No BSSID is currently set.\n";
          }
        },
        "Get the current BSSID (MAC addressof the access point) of the WiFi connection.");

    menu->Insert(
        "channel",
        [this](std::ostream &out) -> void {
          int channel = wifi_sta_.get().get_channel();
          if (channel != -1) {
            out << fmt::format("Current WiFi channel: {}\n", channel);
          } else {
            out << "Failed to get WiFi channel.\n";
          }
        },
        "Get the current WiFi channel of the connection.");

    menu->Insert(
        "config",
        [this](std::ostream &out) -> void {
          wifi_config_t config;
          if (wifi_sta_.get().get_saved_config(config)) {
            out << fmt::format("Current WiFi configuration:\n"
                               "SSID: '{}'\n"
                               "Password: '{}'\n",
                               std::string_view(reinterpret_cast<char *>(config.sta.ssid)),
                               std::string_view(reinterpret_cast<char *>(config.sta.password)));
          } else {
            out << "No saved WiFi configuration found.\n";
          }
        },
        "Get the current WiFi configuration.");

    menu->Insert(
        "scan",
        [this](std::ostream &out, int count) -> void {
          wifi_ap_record_t ap_records[count];
          int num_records = wifi_sta_.get().scan(ap_records, count);
          if (num_records > 0) {
            out << fmt::format("Found {} WiFi networks:\n", num_records);
            for (int i = 0; i < num_records; ++i) {
              out << fmt::format("SSID: '{}', RSSI: {}, BSSID: {}\n",
                                 std::string_view(reinterpret_cast<char *>(ap_records[i].ssid)),
                                 ap_records[i].rssi, ap_records[i].bssid);
            }
          } else {
            out << "No WiFi networks found.\n";
          }
        },
        "Scan for available WiFi networks.");

    return menu;
  }

protected:
  /// @brief Set the log level for the WiFi station.
  /// @param out The output stream to write to.
  /// @param verbosity The verbosity level to set.
  void set_log_level(std::ostream &out, const std::string &verbosity) {
    if (verbosity == "debug") {
      wifi_sta_.get().set_log_level(espp::Logger::Verbosity::DEBUG);
    } else if (verbosity == "info") {
      wifi_sta_.get().set_log_level(espp::Logger::Verbosity::INFO);
    } else if (verbosity == "warn") {
      wifi_sta_.get().set_log_level(espp::Logger::Verbosity::WARN);
    } else if (verbosity == "error") {
      wifi_sta_.get().set_log_level(espp::Logger::Verbosity::ERROR);
    } else if (verbosity == "none") {
      wifi_sta_.get().set_log_level(espp::Logger::Verbosity::NONE);
    } else {
      out << "Invalid log level.\n";
      return;
    }
    out << fmt::format("Set wifi sta log level to {}.\n", verbosity);
  }

  std::reference_wrapper<espp::WifiSta> wifi_sta_;
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
