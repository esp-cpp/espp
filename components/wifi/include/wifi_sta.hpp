#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "base_component.hpp"
#include "wifi_format_helpers.hpp"

namespace espp {
/**
 *  WiFi Station (STA)
 *
 * see
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-station-general-scenario
 *
 * @note If CONFIG_ESP32_WIFI_NVS_ENABLED is set to `y` (which is the
 *       default), then you must ensure that you call `nvs_flash_init()`
 *       prior to creating the WiFi Station.
 *
 * \section wifista_ex1 WiFi Station Example
 * \snippet wifi_example.cpp wifi sta example
 */
class WifiSta : public BaseComponent {
public:
  /**
   * @brief called when the WiFi station connects to an access point.
   */
  typedef std::function<void(void)> connect_callback;
  /**
   * @brief Called when the WiFi station is disconnected from the access point
   *        and has exceeded the configured Config::num_connect_retries.
   */
  typedef std::function<void(void)> disconnect_callback;
  /**
   * @brief Called whe nthe WiFi station has gotten an IP from the access
   *        point.
   * @param ip_evt Pointer to IP Event data structure (contains ip address).
   */
  typedef std::function<void(ip_event_got_ip_t *ip_evt)> ip_callback;

  /**
   * @brief Configuration structure for the WiFi Station.
   */
  struct Config {
    std::string ssid;     /**< SSID for the access point. */
    std::string password; /**< Password for the access point. If empty, the AP will be open / have
                             no security. */
    wifi_phy_rate_t phy_rate{
        WIFI_PHY_RATE_MCS0_LGI}; /**< PHY rate to use for the station. Default is MCS0_LGI (6.5
                                    - 13.5 Mbps Long Guard Interval). */
    size_t num_connect_retries{
        0}; /**< Number of times to retry connecting to the AP before stopping. After this many
               retries, on_disconnected will be called. */
    connect_callback on_connected{
        nullptr}; /**< Called when the station connects, or fails to connect. */
    disconnect_callback on_disconnected{nullptr}; /**< Called when the station disconnects. */
    ip_callback on_got_ip{nullptr}; /**< Called when the station gets an IP address. */
    uint8_t channel{0};             /**< Channel of target AP; set to 0 for unknown. */
    bool set_ap_mac{false}; /**< Whether to check MAC address of the AP (generally no). If yes,
                               provide ap_mac. */
    uint8_t ap_mac[6]{0};   /**< MAC address of the AP to check if set_ap_mac is set to true. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity of WifiSta logger. */
  };

  /**
   * @brief Initialize the WiFi Station (STA)
   * @param config WifiSta::Config structure with initialization information.
   */
  explicit WifiSta(const Config &config)
      : BaseComponent("WifiSta", config.log_level)
      , num_retries_(config.num_connect_retries)
      , connect_callback_(config.on_connected)
      , disconnect_callback_(config.on_disconnected)
      , ip_callback_(config.on_got_ip) {
    // Code below is modified from:
    // https://github.com/espressif/esp-idf/blob/1c84cfde14dcffdc77d086a5204ce8a548dce935/examples/wifi/getting_started/station/main/station_example_main.c
    esp_err_t err;
    logger_.debug("Initializing WiFiSta");
    err = esp_netif_init();
    if (err != ESP_OK) {
      logger_.error("Could not initialize netif: {}", err);
    }

    logger_.debug("Creating default event loop");
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
      logger_.error("Could not create default event loop: {}", err);
    }

    // Create default WiFi STA
    logger_.debug("Creating default WiFi STA netif");
    netif_ = esp_netif_create_default_wifi_sta();
    if (netif_ == nullptr) {
      logger_.error("Could not create default WiFi STA");
    }

    logger_.debug("Wifi init...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
      logger_.error("Could not init wifi subsystem: {}", err);
    }

    // register our event handlers
    logger_.debug("Adding event handler for WIFI_EVENT(s)");
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiSta::event_handler,
                                              this, &event_handler_instance_any_id_);
    if (err != ESP_OK) {
      logger_.error("Could not add wifi ANY event handler: {}", err);
    }
    logger_.debug("Adding IP event handler for IP_EVENT_STA_GOT_IP");
    err =
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiSta::event_handler,
                                            this, &event_handler_instance_got_ip_);
    if (err != ESP_OK) {
      logger_.error("Could not add ip GOT_IP event handler: {}", err);
    }

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    // if the ssid is empty, then load the saved configuration
    if (config.ssid.empty()) {
      logger_.debug("SSID is empty, trying to load saved WiFi configuration");
      if (!get_saved_config(wifi_config)) {
        logger_.error("Could not get saved WiFi configuration, SSID must be set");
        return;
      }
      logger_.info("Loaded saved WiFi configuration: SSID = '{}'",
                   std::string_view(reinterpret_cast<char *>(wifi_config.sta.ssid)));
    } else {
      logger_.debug("Setting WiFi SSID to '{}'", config.ssid);
      memcpy(wifi_config.sta.ssid, config.ssid.data(), config.ssid.size());
      memcpy(wifi_config.sta.password, config.password.data(), config.password.size());
    }

    wifi_config.sta.channel = config.channel;
    wifi_config.sta.bssid_set = config.set_ap_mac;
    if (config.set_ap_mac) {
      memcpy(wifi_config.sta.bssid, config.ap_mac, 6);
    }

    logger_.debug("Setting WiFi mode to STA");
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi mode STA: {}", err);
    }

    logger_.debug("Setting Wifi config");
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi config: {}", err);
    }

    logger_.debug("Setting WiFi PHY rate to {}", config.phy_rate);
    err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, config.phy_rate);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi PHY rate: {}", err);
    }

    logger_.debug("Starting WiFi");
    err = esp_wifi_start();
    if (err != ESP_OK) {
      logger_.error("Could not start WiFi: {}", err);
    }
  }

  /**
   * @brief Stop the WiFi station and deinit the wifi subystem.
   */
  ~WifiSta() {
    // remove any callbacks
    logger_.debug("Destroying WiFiSta");
    connected_ = false;
    disconnecting_ = true;
    attempts_ = 0;
    connect_callback_ = nullptr;
    disconnect_callback_ = nullptr;
    ip_callback_ = nullptr;

    // unregister our event handlers
    logger_.debug("Unregistering event handlers");
    esp_err_t err;
    // unregister our any event handler
    logger_.debug("Unregistering any wifi event handler");
    err = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                event_handler_instance_any_id_);
    if (err != ESP_OK) {
      logger_.error("Could not unregister any wifi event handler: {}", err);
    }
    // unregister our ip event handler
    logger_.debug("Unregistering got ip event handler");
    err = esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                event_handler_instance_got_ip_);
    if (err != ESP_OK) {
      logger_.error("Could not unregister got ip event handler: {}", err);
    }

    // NOTE: Deinit phase
    // stop the wifi
    logger_.debug("Stopping WiFi");
    err = esp_wifi_stop();
    if (err != ESP_OK) {
      logger_.error("Could not stop WiFiSta: {}", err);
    }
    // deinit the subsystem
    logger_.debug("Deinit WiFi subsystem");
    err = esp_wifi_deinit();
    if (err != ESP_OK) {
      logger_.error("Could not deinit WiFiSta: {}", err);
    }
    logger_.info("WiFi stopped");
    // destroy (free the memory)
    logger_.debug("Destroying default WiFi STA");
    esp_netif_destroy_default_wifi(netif_);
  }

  /**
   * @brief Whether the station is connected to an access point.
   * @return true if it is currently connected, false otherwise.
   */
  bool is_connected() const { return connected_; }

  /**
   * @brief Get the current hostname of the WiFi Station (STA).
   * @return Current hostname of the station as a string.
   */
  std::string get_hostname() {
    const char *hostname;
    esp_err_t err = esp_netif_get_hostname(netif_, &hostname);
    if (err != ESP_OK) {
      logger_.error("Could not get hostname: {}", err);
      return "";
    }
    return std::string(hostname);
  }

  /**
   * @brief Set the hostname for the WiFi Station (STA).
   * @param hostname New hostname for the station.
   * @param restart_dhcp Whether to restart the DHCP client to apply the new hostname.
   * @return True if the operation was successful, false otherwise.
   * @note The hostname is set for the station interface and will take effect
   *       after the DHCP client is stopped and restarted.
   */
  bool set_hostname(const std::string &hostname, bool restart_dhcp = true) {
    esp_err_t err = esp_netif_set_hostname(netif_, hostname.c_str());
    if (err != ESP_OK) {
      logger_.error("Could not set hostname: {}", err);
      return false;
    }
    logger_.info("Hostname set to '{}'", hostname);
    // return early if not restarting DHCP client
    if (!restart_dhcp) {
      logger_.info("Not restarting DHCP client. Make sure to restart it later (or reconnect to AP) "
                   "to apply the new hostname.");
      return true;
    }
    // restart DHCP client to apply the new hostname
    logger_.info("Restarting the DHCP client for the new hostname to take effect.");
    if (!restart_dhcp_client()) {
      logger_.error("Failed to restart DHCP client after setting hostname");
      return false;
    }
    return true;
  }

  /**
   * @brief Restart the DHCP client for the WiFi Station (STA).
   * @return True if the DHCP client was restarted successfully, false otherwise.
   * @note This is useful if you want to refresh the IP address, hostname, or
   *       apply new settings.
   */
  bool restart_dhcp_client() {
    esp_err_t err = esp_netif_dhcpc_stop(netif_);
    if (err != ESP_OK) {
      logger_.error("Could not stop DHCP client: {}", err);
      return false;
    }
    err = esp_netif_dhcpc_start(netif_);
    if (err != ESP_OK) {
      logger_.error("Could not start DHCP client: {}", err);
      return false;
    }
    logger_.info("DHCP client restarted successfully");
    return true;
  }

  /**
   * @brief Get the MAC address of the station.
   * @return MAC address of the station.
   */
  std::string get_mac() {
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err != ESP_OK) {
      logger_.error("Could not get MAC address: {}", err);
      return "";
    }
    return fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", mac[0], mac[1], mac[2], mac[3],
                       mac[4], mac[5]);
  }

  /**
   * @brief Get the IP address of the station.
   * @return IP address of the station.
   */
  std::string get_ip() {
    esp_netif_ip_info_t ip_info;
    esp_err_t err = esp_netif_get_ip_info(netif_, &ip_info);
    if (err != ESP_OK) {
      logger_.error("Could not get IP address: {}", err);
      return "";
    }
    return fmt::format("{}.{}.{}.{}", IP2STR(&ip_info.ip));
  }

  /**
   * @brief Get the SSID of the access point the station is connected to.
   * @return SSID of the access point.
   */
  std::string get_ssid() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get SSID: {}", err);
      return "";
    }
    return std::string(reinterpret_cast<const char *>(ap_info.ssid),
                       strlen((const char *)ap_info.ssid));
  }

  /**
   * @brief Get the RSSI (Received Signal Strength Indicator) of the access point.
   * @return RSSI of the access point.
   */
  int32_t get_rssi() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get RSSI: {}", err);
      return 0;
    }
    return ap_info.rssi;
  }

  /**
   * @brief Get the channel of the access point the station is connected to.
   * @return Channel of the access point.
   */
  uint8_t get_channel() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get channel: {}", err);
      return 0;
    }
    return ap_info.primary;
  }

  /**
   * @brief Get the BSSID (MAC address) of the access point the station is connected to.
   * @return BSSID of the access point.
   */
  std::string get_bssid() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get BSSID: {}", err);
      return "";
    }
    return fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", ap_info.bssid[0],
                       ap_info.bssid[1], ap_info.bssid[2], ap_info.bssid[3], ap_info.bssid[4],
                       ap_info.bssid[5]);
  }

  /**
   * @brief Set the number of retries to connect to the access point.
   * @param num_retries Number of retries to connect to the access point.
   */
  void set_num_retries(size_t num_retries) {
    if (num_retries > 0) {
      num_retries_ = num_retries;
    } else {
      logger_.warn("Number of retries must be greater than 0, not setting it");
    }
  }

  /**
   * @brief Set the callback to be called when the station connects to the access point.
   * @param callback Callback to be called when the station connects to the access point.
   */
  void set_connect_callback(connect_callback callback) { connect_callback_ = callback; }

  /**
   * @brief Set the callback to be called when the station disconnects from the access point.
   * @param callback Callback to be called when the station disconnects from the access point.
   */
  void set_disconnect_callback(disconnect_callback callback) { disconnect_callback_ = callback; }

  /**
   * @brief Set the callback to be called when the station gets an IP address.
   * @param callback Callback to be called when the station gets an IP address.
   */
  void set_ip_callback(ip_callback callback) { ip_callback_ = callback; }

  /**
   * @brief Get the netif associated with this WiFi station.
   * @return Pointer to the esp_netif_t associated with this WiFi station.
   */
  esp_netif_t *get_netif() const { return netif_; }

  /**
   * @brief Get the saved WiFi configuration.
   * @param wifi_config Reference to a wifi_config_t structure to store the configuration.
   * @return true if the configuration was retrieved successfully, false otherwise.
   */
  bool get_saved_config(wifi_config_t &wifi_config) {
    esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi STA config: {}", err);
      return false;
    }
    return true;
  }

  /**
   * @brief Connect to the access point with the saved SSID and password.
   * @return true if the connection was initiated successfully, false otherwise.
   */
  bool connect() {
    wifi_config_t wifi_config;
    if (!get_saved_config(wifi_config)) {
      logger_.error("Could not get saved WiFi configuration, SSID must be set");
      return false;
    }
    std::string ssid(reinterpret_cast<const char *>(wifi_config.sta.ssid),
                     strlen((const char *)wifi_config.sta.ssid));
    std::string password(reinterpret_cast<const char *>(wifi_config.sta.password),
                         strlen((const char *)wifi_config.sta.password));
    return connect(ssid, password);
  }

  /**
   * @brief Connect to the access point with the given SSID and password.
   * @param ssid SSID of the access point.
   * @param password Password of the access point. If empty, the AP will be open / have no security.
   * @return true if the connection was initiated successfully, false otherwise.
   */
  bool connect(std::string_view ssid, std::string_view password) {
    if (ssid.empty()) {
      logger_.error("SSID cannot be empty");
      return false;
    }
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    memcpy(wifi_config.sta.ssid, ssid.data(), ssid.size());
    if (!password.empty()) {
      memcpy(wifi_config.sta.password, password.data(), password.size());
    }
    // ensure retries are reset
    attempts_ = 0;
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi config: {}", err);
      return false;
    }
    attempts_ = 0;
    connected_ = false;
    return esp_wifi_connect() == ESP_OK;
  }

  /**
   * @brief Disconnect from the access point.
   * @return true if the disconnection was initiated successfully, false otherwise.
   */
  bool disconnect() {
    connected_ = false;
    attempts_ = 0;
    disconnecting_ = true;
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
      logger_.error("Could not disconnect from WiFi: {}", err);
      return false;
    }
    return true;
  }

  /**
   * @brief Scan for available access points.
   * @param ap_records Pointer to an array of wifi_ap_record_t to store the results.
   * @param max_records Maximum number of access points to scan for.
   * @return Number of access points found, or negative error code on failure.
   * @note This is a blocking call that will wait for the scan to complete.
   */
  int scan(wifi_ap_record_t *ap_records, size_t max_records) {
    if (ap_records == nullptr || max_records == 0) {
      logger_.error("Invalid parameters for scan");
      return -1;
    }
    uint16_t number = max_records;
    uint16_t ap_count = 0;
    static constexpr bool blocking = true; // blocking scan
    esp_err_t err = esp_wifi_scan_start(nullptr, blocking);
    if (err != ESP_OK) {
      logger_.error("Could not start WiFi scan: {}", err);
      return -1;
    }
    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi scan AP num: {}", err);
      return -1;
    }
    err = esp_wifi_scan_get_ap_records(&number, ap_records);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi scan results: {}", err);
      return -1;
    }
    logger_.debug("Scanned {} APs, found {} access points", ap_count, number);
    if (ap_count > max_records) {
      logger_.warn("Found {} access points, but only {} can be stored", ap_count, max_records);
      ap_count = max_records;
    }
    return number;
  }

protected:
  static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                            void *event_data) {
    auto wifi_sta = static_cast<WifiSta *>(arg);
    if (wifi_sta) {
      wifi_sta->event_handler(event_base, event_id, event_data);
    }
  }

  void event_handler(esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      connected_ = false;
      esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      connected_ = false;
      if (disconnecting_) {
        disconnecting_ = false;
      } else {
        if (attempts_ < num_retries_) {
          esp_wifi_connect();
          attempts_++;
          logger_.info("Retrying to connect to the AP");
          // return early, don't call disconnect callback
          return;
        }
      }
      logger_.info("Failed to connect to the AP after {} attempts", attempts_.load());
      if (disconnect_callback_) {
        disconnect_callback_();
      }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      logger_.info("got ip: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
      attempts_ = 0;
      connected_ = true;
      if (connect_callback_) {
        connect_callback_();
      }
      if (ip_callback_) {
        ip_callback_(event);
      }
    }
  }

  std::atomic<size_t> attempts_{0};
  std::atomic<size_t> num_retries_{0};
  esp_netif_t *netif_{nullptr};
  connect_callback connect_callback_{nullptr};
  disconnect_callback disconnect_callback_{nullptr};
  ip_callback ip_callback_{nullptr};
  std::atomic<bool> connected_{false};
  std::atomic<bool> disconnecting_{false};
  esp_event_handler_instance_t event_handler_instance_any_id_;
  esp_event_handler_instance_t event_handler_instance_got_ip_;
};
} // namespace espp
