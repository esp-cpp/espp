#pragma once

#include <string>

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_wifi_ap_get_sta_list.h"
#include "nvs_flash.h"

#include "base_component.hpp"
#include "wifi_base.hpp"
#include "wifi_format_helpers.hpp"

namespace espp {
/**
 *  WiFi Access Point (AP)
 *
 * see
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-ap-general-scenario
 *
 * @note If CONFIG_ESP32_WIFI_NVS_ENABLED is set to `y` (which is the
 *       default), then you must ensure that you call `nvs_flash_init()`
 *       prior to creating the WiFi Access Point.
 *
 * \section wifiap_ex1 WiFi Access Point Example
 * \snippet wifi_example.cpp wifi ap example
 */
class WifiAp : public WifiBase {
public:
  /**
   * @brief Configuration structure for the WiFi Access Point (AP)
   */
  struct Config {
    std::string ssid;     /**< SSID for the access point. */
    std::string password; /**< Password for the access point. If empty, the AP will be open / have
                             no security. */
    wifi_phy_rate_t phy_rate{
        WIFI_PHY_RATE_MCS0_LGI}; /**< PHY rate to use for the access point. Default is MCS0_LGI (6.5
                                    - 13.5 Mbps Long Guard Interval). */
    uint8_t channel{1};          /**< WiFi channel, range [1,13]. */
    uint8_t max_number_of_stations{4}; /**< Max number of connected stations to this AP. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity of WifiAp logger. */
  };

  /**
   * @brief Structure representing a connected station
   */
  struct Station {
    uint8_t mac[6]; /**< MAC address of the connected station. */
    int rssi;       /**< RSSI (Received Signal Strength Indicator) of the connected station. */
    std::string ip; /**< IP address of the connected station. */
  };

  /**
   * @brief Initialize the WiFi Access Point (AP)
   * @param config WifiAp::Config structure with initialization information.
   */
  explicit WifiAp(const Config &config)
      : WifiBase("WifiAp", config.log_level) {
    init(config);
  }

  /**
   * @brief Initialize the WiFi Access Point (AP)
   * @param config WifiAp::Config structure with initialization information.
   * @param netif Pointer to the AP network interface (obtained from Wifi singleton)
   */
  explicit WifiAp(const Config &config, esp_netif_t *netif)
      : WifiBase("WifiAp", config.log_level, netif) {
    init(config);
  }

  /**
   * @brief Destructor for the WiFi Access Point (AP)
   *
   * This will stop the WiFi AP and deinitialize the WiFi subsystem.
   */
  ~WifiAp() override {
    if (!unregister_event_handlers()) {
      logger_.error("Could not unregister event handlers");
    }
    if (!stop()) {
      logger_.error("Could not stop WiFi AP");
    }
    // Note: WiFi deinit and netif destruction are handled by Wifi singleton
  }

  /**
   * @brief Get the saved WiFi configuration.
   * @param wifi_config Reference to a wifi_config_t structure to store the configuration.
   * @return true if the configuration was retrieved successfully, false otherwise.
   */
  bool get_saved_config(wifi_config_t &wifi_config) {
    esp_err_t err = esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi AP config: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /**
   * @brief Get the MAC address of the WiFi Access Point (AP)
   * @return MAC address of the AP as a vector of bytes.
   */
  std::vector<uint8_t> get_mac_address() {
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK) {
      logger_.error("Could not get MAC address: {}", esp_err_to_name(err));
      return {};
    }
    return std::vector<uint8_t>(mac, mac + sizeof(mac));
  }

  /**
   * @brief Get the MAC address of the WiFi Access Point (AP)
   * @return MAC address of the AP as a string.
   */
  std::string get_mac_address_string() override {
    std::vector<uint8_t> mac = get_mac_address();
    if (mac.empty()) {
      return "";
    }
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2],
             mac[3], mac[4], mac[5]);
    return std::string(mac_str);
  }

  /**
   * @brief Check if any stations are connected to the WiFi Access Point (AP)
   * @return True if at least one station is connected, false otherwise.
   */
  bool is_connected() const override { return get_num_connected_stations() > 0; }

  /**
   * @brief Get the SSID of the WiFi Access Point (AP)
   * @return SSID of the AP as a string.
   */
  std::string get_ssid() override {
    wifi_config_t wifi_config;
    if (!get_saved_config(wifi_config)) {
      return "";
    }
    return std::string(reinterpret_cast<const char *>(wifi_config.ap.ssid),
                       wifi_config.ap.ssid_len);
  }

  /**
   * @brief Get the number of connected stations to this AP
   * @return Number of connected stations.
   */
  uint8_t get_num_connected_stations() const {
    wifi_sta_list_t sta_list;
    esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
    if (err != ESP_OK) {
      logger_.error("Could not get station list: {}", esp_err_to_name(err));
      return 0;
    }
    logger_.debug("Number of connected stations: {}", sta_list.num);
    return sta_list.num;
  }

  /**
   * @brief Get the RSSI (Received Signal Strength Indicator) of connected stations
   * @return Vector of RSSI values of connected stations.
   */
  std::vector<int> get_connected_station_rssis() {
    std::vector<int> rssis;
    wifi_sta_list_t sta_list;
    esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
    if (err != ESP_OK) {
      logger_.error("Could not get station list: {}", esp_err_to_name(err));
      return rssis;
    }

    for (int i = 0; i < sta_list.num; ++i) {
      rssis.push_back(sta_list.sta[i].rssi);
    }
    return rssis;
  }

  /**
   * @brief Get the IP addresses of connected stations to this AP
   * @return Vector of IP addresses of connected stations.
   */
  std::vector<std::string> get_connected_station_ips() {
    std::vector<std::string> ips;
    wifi_sta_list_t sta_list;
    esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
    if (err != ESP_OK) {
      logger_.error("Could not get station list: {}", esp_err_to_name(err));
      return ips;
    }

    // now that we have the sta list, we need to call the esp_wifi_ap_get_sta_list_with_ip
    // to get the IP addresses of the connected stations
    wifi_sta_mac_ip_list_t sta_list_with_ip;
    err = esp_wifi_ap_get_sta_list_with_ip(&sta_list, &sta_list_with_ip);
    if (err != ESP_OK) {
      logger_.error("Could not get station list with IPs: {}", esp_err_to_name(err));
      return ips;
    }

    for (int i = 0; i < sta_list_with_ip.num; ++i) {
      char ip_str[16];
      snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&sta_list_with_ip.sta[i].ip));
      ips.push_back(std::string(ip_str));
    }

    return ips;
  }

  /**
   * @brief Get the connected stations to this AP
   * @return Vector of connected stations with their MAC addresses, RSSI and IPs.
   */
  std::vector<Station> get_connected_stations() {
    std::vector<Station> stations;
    wifi_sta_list_t sta_list;
    esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
    if (err != ESP_OK) {
      logger_.error("Could not get station list: {}", esp_err_to_name(err));
      return stations;
    }

    // also get the IP addresses of the connected stations
    wifi_sta_mac_ip_list_t sta_list_with_ip;
    err = esp_wifi_ap_get_sta_list_with_ip(&sta_list, &sta_list_with_ip);
    if (err != ESP_OK) {
      logger_.error("Could not get station list with IPs: {}", esp_err_to_name(err));
      return stations;
    }

    // now populate the stations vector with the appropriate mac addresses, RSSI and IPs
    for (int i = 0; i < sta_list.num; ++i) {
      Station station;
      memcpy(station.mac, sta_list.sta[i].mac, sizeof(station.mac));
      station.rssi = sta_list.sta[i].rssi;
      char ip_str[16];
      snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&sta_list_with_ip.sta[i].ip));
      station.ip = std::string(ip_str);
      stations.push_back(station);
    }

    return stations;
  }

  /**
   * @brief Reconfigure the WiFi Access Point with a new configuration.
   * @param config WifiAp::Config structure with new AP configuration.
   * @return True if reconfiguration was successful, false otherwise.
   * @note This will stop the AP, apply the new configuration, and restart it.
   */
  bool reconfigure(const Config &config) {
    logger_.info("Reconfiguring WiFi AP");

    // Stop the current AP
    bool was_running = false;
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err == ESP_OK && (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA)) {
      was_running = true;
      stop();
    }

    // Update configuration
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    wifi_config.ap.ssid_len = (uint8_t)config.ssid.size();
    wifi_config.ap.channel = config.channel;
    wifi_config.ap.max_connection = config.max_number_of_stations;
    memcpy(wifi_config.ap.ssid, config.ssid.data(), config.ssid.size());
    memcpy(wifi_config.ap.password, config.password.data(), config.password.size());
    if (config.password.size() == 0) {
      wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    logger_.debug("Setting WiFi mode to AP");
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi to AP: {}", err);
      return false;
    }

    logger_.debug("Setting WiFi config");
    err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi config: {}", err);
      return false;
    }

    logger_.debug("Setting WiFi phy rate to {}", config.phy_rate);
    err = esp_wifi_config_80211_tx_rate(WIFI_IF_AP, config.phy_rate);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi PHY rate: {}", err);
      return false;
    }

    // Restart if it was running
    if (was_running) {
      return start();
    }

    logger_.info("WiFi AP reconfigured successfully");
    return true;
  }

  /**
   * @brief Start the WiFi Access Point (AP)
   * @return True if the operation was successful, false otherwise.
   */
  bool start() override {
    logger_.debug("Starting WiFi");
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
      logger_.error("Could not start WiFi AP: {}", esp_err_to_name(err));
      return false;
    }
    logger_.info("WiFi AP started");
    return true;
  }

  /**
   * @brief Stop the WiFi Access Point (AP)
   * @return True if the operation was successful, false otherwise.
   */
  bool stop() override {
    logger_.debug("Stopping WiFi");
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
      logger_.error("Could not stop WiFi AP: {}", esp_err_to_name(err));
      return false;
    }
    logger_.info("WiFi AP stopped");
    return true;
  }

  /**
   * @brief Set the SSID and password of the WiFi Access Point (AP)
   * @param ssid New SSID for the access point.
   * @param password New password for the access point. If empty, the AP will be open / have no
   *                 security.
   * @return True if the operation was successful, false otherwise.
   */
  bool set_ssid_and_password(const std::string &ssid, const std::string &password) {
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    // get the current configuration
    esp_err_t err = esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not get current WiFi config: {}", esp_err_to_name(err));
      return false;
    }
    memset(wifi_config.ap.ssid, 0, sizeof(wifi_config.ap.ssid));
    memset(wifi_config.ap.password, 0, sizeof(wifi_config.ap.password));
    // now update the SSID and password
    wifi_config.ap.ssid_len = (uint8_t)ssid.size();
    memcpy(wifi_config.ap.ssid, ssid.data(), ssid.size());
    // update the password / authmode
    if (password.empty()) {
      wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
      wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
      memcpy(wifi_config.ap.password, password.data(), password.size());
    }
    // set the new configuration
    err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set SSID and password: {}", esp_err_to_name(err));
      return false;
    }
    logger_.info("SSID and password updated to '{}'", ssid);
    return true;
  }

protected:
  static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                            void *event_data) {
    auto wifi_ap = static_cast<WifiAp *>(arg);
    if (wifi_ap) {
      wifi_ap->event_handler(event_base, event_id, event_data);
    }
  }

  void event_handler(esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
      wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
      logger_.info("Station join, AID={}", event->aid); // MAC2STR(event->mac) MACSTR
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
      wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
      logger_.info("Station leave, AID={}", event->aid); // MAC2STR(event->mac) MACSTR
    }
  }

  bool register_event_handlers() {
    logger_.debug("Registering event handler");
    esp_err_t err = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiAp::event_handler, this, &event_handler_instance_);
    if (err != ESP_OK) {
      logger_.error("Could not register wifi event handler: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  bool unregister_event_handlers() {
    logger_.debug("Unregistering event handler");
    esp_err_t err = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                          event_handler_instance_);
    if (err != ESP_OK) {
      logger_.error("Could not unregister wifi event handler: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  void init(const Config &config);

  esp_event_handler_instance_t event_handler_instance_;
};
} // namespace espp

// libfmt printing of WifiAp::Station
namespace fmt {
template <> struct formatter<espp::WifiAp::Station> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::WifiAp::Station &station, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "{{MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, RSSI: {}, IP: {}}}",
                          station.mac[0], station.mac[1], station.mac[2], station.mac[3],
                          station.mac[4], station.mac[5], station.rssi, station.ip);
  }
};
} // namespace fmt

namespace fmt {
// libfmt printing of WifiAp::Config
template <> struct formatter<espp::WifiAp::Config> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::WifiAp::Config &config, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "WifiAp::Config {{ ssid: '{}', password length: {}, phy_rate: {}, channel: "
        "{}, max_number_of_stations: {} }}",
        config.ssid, config.password.size(), config.phy_rate, config.channel,
        config.max_number_of_stations);
  }
};
} // namespace fmt
