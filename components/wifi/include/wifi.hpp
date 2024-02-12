#pragma once

#include "esp_wifi.h"

#include "wifi_ap.hpp"
#include "wifi_sta.hpp"

namespace espp {

/// @brief The Wifi class provides access to the ESP32 Wifi functionality.
/// @details The Wifi class is a singleton class that provides access to the
/// ESP32 Wifi functionality. The Wifi class is a wrapper around the ESP32
/// Wifi API. The Wifi class provides access to the Wifi AP and Wifi STA
/// functionality.
class Wifi {
public:
  /// @brief Access the singleton instance of the Wifi class.
  /// @return The singleton instance of the Wifi class.
  static Wifi &instance() {
    static Wifi wifi;
    return wifi;
  }

  Wifi(const Wifi &) = delete;
  Wifi &operator=(const Wifi &) = delete;
  Wifi(Wifi &&) = delete;
  Wifi &operator=(Wifi &&) = delete;

  /// @brief Get the IP address of the Wifi AP or Wifi STA interface.
  /// @param ip_address The IP address of the Wifi AP or Wifi STA interface.
  /// @return True if the IP address was retrieved, false otherwise.
  bool get_ip_address(std::string &ip_address) {
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(get_esp_interface_netif(ESP_IF_WIFI_AP), &ip) != ESP_OK) {
      if (esp_netif_get_ip_info(get_esp_interface_netif(ESP_IF_WIFI_STA), &ip) != ESP_OK) {
        return false;
      }
    }

    ip_address = ip4addr_ntoa(&ip.ip);
    return true;
  }

protected:
  /// @brief Construct a new Wifi object.
  /// @details The Wifi object is a singleton object and can only be
  /// constructed once.
  Wifi() {
    /*
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_start();
    */
  }

  std::unique_ptr<WifiAp> ap_;
  std::unique_ptr<WifiSta> sta_;
};

} // namespace espp
