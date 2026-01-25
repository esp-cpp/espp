#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "base_component.hpp"
#include "wifi_base.hpp"
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
class WifiSta : public WifiBase {
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
   * @brief Scan configuration for WiFi networks
   */
  struct ScanConfig {
    uint16_t max_results{20}; ///< Maximum number of scan results to return
    bool show_hidden{false};  ///< Whether to show hidden SSIDs
  };

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
    bool auto_connect{true}; /**< Whether to automatically connect when started. If false, call
                                connect() manually. */
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
      : WifiBase("WifiSta", config.log_level) {
    init(config);
  }

  /**
   * @brief Initialize the WiFi Station (STA)
   * @param config WifiSta::Config structure with initialization information.
   * @param netif Pointer to the STA network interface (obtained from Wifi singleton)
   */
  explicit WifiSta(const Config &config, esp_netif_t *netif)
      : WifiBase("WifiSta", config.log_level, netif) {
    init(config);
  }

  /**
   * @brief Stop the WiFi station and deinit the wifi subystem.
   */
  ~WifiSta() override {
    // remove any callbacks
    logger_.debug("Destroying WiFiSta");
    connected_ = false;
    disconnecting_ = true;
    attempts_ = 0;
    config_.on_connected = nullptr;
    config_.on_disconnected = nullptr;
    config_.on_got_ip = nullptr;

    // unregister our event handlers
    logger_.debug("Unregistering event handlers");
    if (!unregister_event_handlers()) {
      logger_.error("Could not unregister event handlers");
    }

    // NOTE: Deinit phase
    // Check if we're in APSTA mode - if so, just disconnect STA, don't stop WiFi entirely
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err == ESP_OK && mode == WIFI_MODE_APSTA) {
      // In APSTA mode, just disconnect the STA interface
      logger_.debug("In APSTA mode, disconnecting STA interface");
      esp_wifi_disconnect();
      // Switch to AP-only mode
      err = esp_wifi_set_mode(WIFI_MODE_AP);
      if (err != ESP_OK) {
        logger_.error("Could not switch to AP mode: {}", esp_err_to_name(err));
      }
      logger_.info("WiFi STA disconnected, AP still running");
    } else {
      // In STA-only mode, stop WiFi entirely
      logger_.debug("Stopping WiFi");
      err = esp_wifi_stop();
      if (err != ESP_OK) {
        logger_.error("Could not stop WiFiSta: {}", esp_err_to_name(err));
      }
      logger_.info("WiFi STA stopped");
    }
    // Note: WiFi deinit and netif destruction are handled by Wifi singleton
  }

  /**
   * @brief Whether the station is connected to an access point.
   * @return true if it is currently connected, false otherwise.
   */
  bool is_connected() const override { return connected_; }

  /**
   * @brief Set the hostname for the WiFi Station (STA).
   * @param hostname New hostname for the station.
   * @return True if the operation was successful, false otherwise.
   */
  bool set_hostname(const std::string &hostname) override { return set_hostname(hostname, true); }

  /**
   * @brief Set the hostname for the WiFi Station (STA).
   * @param hostname New hostname for the station.
   * @param restart_dhcp Whether to restart the DHCP client to apply the new hostname.
   * @return True if the operation was successful, false otherwise.
   * @note The hostname is set for the station interface and will take effect
   *       after the DHCP client is stopped and restarted.
   */
  bool set_hostname(const std::string &hostname, bool restart_dhcp) {
    esp_err_t err = esp_netif_set_hostname(netif_, hostname.c_str());
    if (err != ESP_OK) {
      logger_.error("Could not set hostname: {}", esp_err_to_name(err));
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
      logger_.error("Could not stop DHCP client: {}", esp_err_to_name(err));
      return false;
    }
    err = esp_netif_dhcpc_start(netif_);
    if (err != ESP_OK) {
      logger_.error("Could not start DHCP client: {}", esp_err_to_name(err));
      return false;
    }
    logger_.info("DHCP client restarted successfully");
    return true;
  }

  /**
   * @brief Get the MAC address of the WiFi Station (STA) as a string
   * @return MAC address as a string in format "XX:XX:XX:XX:XX:XX"
   */
  std::string get_mac_address_string() override { return get_mac(); }

  /**
   * @brief Get the MAC address of the station.
   * @return MAC address of the station.
   */
  std::string get_mac() {
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err != ESP_OK) {
      logger_.error("Could not get MAC address: {}", esp_err_to_name(err));
      return "";
    }
    return fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", mac[0], mac[1], mac[2], mac[3],
                       mac[4], mac[5]);
  }

  /**
   * @brief Get the IP address of the station.
   * @return IP address of the station.
   */
  std::string get_ip() { return get_ip_address(); }

  /**
   * @brief Get the SSID of the access point the station is connected to.
   * @return SSID of the access point.
   */
  std::string get_ssid() override {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get SSID: {}", esp_err_to_name(err));
      return "";
    }
    return std::string(reinterpret_cast<const char *>(ap_info.ssid),
                       strlen((const char *)ap_info.ssid));
  }

  /**
   * @brief Scan for available WiFi networks
   * @param max_results Maximum number of scan results to return
   * @param show_hidden Whether to show hidden SSIDs
   * @return Vector of AP records found during scan (empty on error)
   * @note This method requires WiFi to be in STA or APSTA mode
   * @note Scanning will disconnect any active STA connection
   * @note WiFi will be restarted after scan if auto_connect is enabled
   */
  std::vector<wifi_ap_record_t> scan(uint16_t max_results = 20, bool show_hidden = false) {
    ScanConfig config;
    config.max_results = max_results;
    config.show_hidden = show_hidden;

    std::vector<wifi_ap_record_t> results;

    logger_.info("Scanning for WiFi networks (max: {}, show_hidden: {})", config.max_results,
                 config.show_hidden);

    // Remember if we were connected
    bool was_connected = is_connected();
    bool saved_auto_connect;
    {
      std::lock_guard<std::recursive_mutex> lock(config_mutex_);
      saved_auto_connect = config_.auto_connect;
    }

    // Disconnect if currently connected (scan requires disconnection)
    esp_wifi_disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Configure scan parameters
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = config.show_hidden;

    // Start the scan
    esp_err_t err = esp_wifi_scan_start(&scan_config, true); // true = blocking
    if (err != ESP_OK) {
      logger_.error("WiFi scan failed: {}", esp_err_to_name(err));
      if (was_connected && saved_auto_connect) {
        start();
      }
      return results;
    }

    // Get scan results
    uint16_t ap_count = config.max_results;
    results.resize(ap_count);
    err = esp_wifi_scan_get_ap_records(&ap_count, results.data());
    if (err != ESP_OK) {
      logger_.error("Failed to get scan results: {}", esp_err_to_name(err));
      results.clear();
      if (was_connected && saved_auto_connect) {
        start();
      }
      return results;
    }

    results.resize(ap_count);
    logger_.info("Found {} networks", ap_count);

    // Reconnect/restart if needed
    if (was_connected) {
      // We were connected before the scan, reconnect
      if (saved_auto_connect) {
        start();
      }
    } else if (!WifiBase::is_initialized()) {
      // We were not connected, but WiFi was stopped during scan restoration
      // Restart WiFi so the STA interface is in a proper state for future connection attempts
      start();
    }

    return results;
  }

  /**
   * @brief Get the RSSI (Received Signal Strength Indicator) of the access point.
   * @return RSSI of the access point.
   */
  int32_t get_rssi() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
      logger_.error("Could not get RSSI: {}", esp_err_to_name(err));
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
      logger_.error("Could not get channel: {}", esp_err_to_name(err));
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
      logger_.error("Could not get BSSID: {}", esp_err_to_name(err));
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
      std::lock_guard<std::recursive_mutex> lock(config_mutex_);
      config_.num_connect_retries = num_retries;
    } else {
      logger_.warn("Number of retries must be greater than 0, not setting it");
    }
  }

  /**
   * @brief Set the callback to be called when the station connects to the access point.
   * @param callback Callback to be called when the station connects to the access point.
   */
  void set_connect_callback(connect_callback callback) {
    std::lock_guard<std::recursive_mutex> lock(config_mutex_);
    config_.on_connected = callback;
  }

  /**
   * @brief Set the callback to be called when the station disconnects from the access point.
   * @param callback Callback to be called when the station disconnects from the access point.
   */
  void set_disconnect_callback(disconnect_callback callback) {
    std::lock_guard<std::recursive_mutex> lock(config_mutex_);
    config_.on_disconnected = callback;
  }

  /**
   * @brief Set the callback to be called when the station gets an IP address.
   * @param callback Callback to be called when the station gets an IP address.
   */
  void set_ip_callback(ip_callback callback) {
    std::lock_guard<std::recursive_mutex> lock(config_mutex_);
    config_.on_got_ip = callback;
  }

  /**
   * @brief Get the saved WiFi configuration.
   * @param wifi_config Reference to a wifi_config_t structure to store the configuration.
   * @return true if the configuration was retrieved successfully, false otherwise.
   */
  bool get_saved_config(wifi_config_t &wifi_config) {
    esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi STA config: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /**
   * @brief Reconfigure the WiFi Station with a new configuration.
   * @param config WifiSta::Config structure with new STA configuration.
   * @return True if reconfiguration was successful, false otherwise.
   * @note This will disconnect, apply the new configuration, and reconnect.
   */
  bool reconfigure(const Config &config) {
    logger_.info("Reconfiguring WiFi STA");

    // Disconnect if connected
    bool was_connected = connected_;
    if (was_connected) {
      disconnect();
      // Wait for disconnect to complete
      int attempts = 0;
      while (connected_ && attempts < 10) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempts++;
      }
    }

    // Update stored configuration using assignment operator
    {
      std::lock_guard<std::recursive_mutex> lock(config_mutex_);
      config_ = config;
    }

    // Update WiFi configuration
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    // if the ssid is empty, then load the saved configuration
    if (config.ssid.empty()) {
      logger_.debug("SSID is empty, trying to load saved WiFi configuration");
      if (!get_saved_config(wifi_config)) {
        logger_.error("Could not get saved WiFi configuration, SSID must be set");
        return false;
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

    // Get current mode and add STA to it if needed
    wifi_mode_t current_mode;
    esp_err_t err = esp_wifi_get_mode(&current_mode);
    wifi_mode_t new_mode = WIFI_MODE_STA;
    if (err == ESP_OK) {
      if (current_mode == WIFI_MODE_AP) {
        new_mode = WIFI_MODE_APSTA;
        logger_.debug("AP mode already active, setting mode to APSTA");
      } else if (current_mode == WIFI_MODE_APSTA) {
        new_mode = WIFI_MODE_APSTA;
        logger_.debug("APSTA mode already set");
      }
    }

    if (current_mode != new_mode) {
      logger_.debug("Setting WiFi mode to {}", new_mode);
      err = esp_wifi_set_mode(new_mode);
      if (err != ESP_OK) {
        logger_.error("Could not set WiFi mode STA: {}", err);
        return false;
      }
    } else {
      logger_.debug("WiFi mode already set to {}", new_mode);
    }

    logger_.debug("Setting WiFi config");
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi config: {}", err);
      return false;
    }

    logger_.debug("Setting WiFi PHY rate to {}", config.phy_rate);
    err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, config.phy_rate);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi PHY rate: {}", esp_err_to_name(err));
    }

    // Reconnect if it was connected
    if (was_connected) {
      logger_.info("Was connected before reconfiguration, reconnecting...");
      return connect();
    }

    logger_.info("WiFi STA reconfigured successfully");
    return true;
  }

  /**
   * @brief Start the WiFi Station (STA) interface
   * @return true if started successfully, false otherwise.
   * @note This is an alias for connect() to match the WifiBase interface
   */
  bool start() override {
    esp_err_t err;
    logger_.debug("Starting WiFi");
    err = esp_wifi_start();
    if (err != ESP_OK) {
      logger_.error("Could not start WiFi: {}", esp_err_to_name(err));
      return false;
    }
    logger_.info("WiFi started");
    return true;
  }

  /**
   * @brief Stop the WiFi Station (STA) interface
   * @return true if stopped successfully, false otherwise.
   * @note This is an alias for disconnect() to match the WifiBase interface
   */
  bool stop() override { return disconnect(); }

  /**
   * @brief Connect to the access point with the saved SSID and password.
   * @return true if the connection was initiated successfully, false otherwise.
   */
  bool connect() {
    esp_err_t err;

    // Ensure WiFi is properly configured before connecting
    if (!ensure_wifi_ready()) {
      logger_.error("Could not ensure WiFi is ready for connection");
      return false;
    }

    // ensure retries are reset
    attempts_ = 0;
    connected_ = false;
    err = esp_wifi_connect();
    if (err != ESP_OK) {
      logger_.error("Could not connect to WiFi: {}", esp_err_to_name(err));
      return false;
    }
    return true;
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
    logger_.info("Connecting to SSID '{}'", ssid);
    wifi_config_t wifi_config;
    if (!get_saved_config(wifi_config)) {
      logger_.error("Could not get saved WiFi configuration");
      return false;
    }
    memcpy(wifi_config.sta.ssid, ssid.data(), ssid.size());
    if (!password.empty()) {
      memcpy(wifi_config.sta.password, password.data(), password.size());
    }
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi config: {}", esp_err_to_name(err));
      return false;
    }

    return connect();
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
      logger_.error("Could not disconnect from WiFi: {}", esp_err_to_name(err));
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
      logger_.error("Could not start WiFi scan: {}", esp_err_to_name(err));
      return -1;
    }
    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi scan AP num: {}", esp_err_to_name(err));
      return -1;
    }
    err = esp_wifi_scan_get_ap_records(&number, ap_records);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi scan results: {}", esp_err_to_name(err));
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
      logger_.debug("WIFI_EVENT_STA_START");
      connected_ = false;
      bool should_auto_connect;
      {
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);
        should_auto_connect = config_.auto_connect;
      }
      if (should_auto_connect) {
        logger_.debug("auto_connect enabled - initiating connection");
        esp_wifi_connect();
      } else {
        logger_.debug("auto_connect disabled - waiting for manual connect()");
      }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      logger_.debug("WIFI_EVENT_STA_DISCONNECTED");
      connected_ = false;
      if (disconnecting_) {
        logger_.debug("Intentional disconnect, not retrying");
        disconnecting_ = false;
      } else {
        size_t max_retries;
        {
          std::lock_guard<std::recursive_mutex> lock(config_mutex_);
          max_retries = config_.num_connect_retries;
        }
        if (attempts_ < max_retries) {
          esp_wifi_connect();
          attempts_++;
          logger_.info("Retrying to connect to the AP (attempt {}/{})", attempts_.load(),
                       max_retries);
          // return early, don't call disconnect callback
          return;
        }
      }
      logger_.info("Failed to connect to the AP after {} attempts", attempts_.load());
      disconnect_callback cb;
      {
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);
        cb = config_.on_disconnected;
      }
      if (cb) {
        cb();
      }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
      logger_.debug("WIFI_EVENT_STA_CONNECTED - waiting for IP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      logger_.info("got ip: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
      attempts_ = 0;
      connected_ = true;
      connect_callback on_conn;
      ip_callback on_ip;
      {
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);
        on_conn = config_.on_connected;
        on_ip = config_.on_got_ip;
      }
      if (on_conn) {
        on_conn();
      }
      if (on_ip) {
        on_ip(event);
      }
    }
  }

  bool register_event_handlers() {
    esp_err_t err;
    // register our event handlers
    logger_.debug("Adding event handler for WIFI_EVENT(s)");
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiSta::event_handler,
                                              this, &event_handler_instance_any_id_);
    if (err != ESP_OK) {
      logger_.error("Could not add wifi ANY event handler: {}", esp_err_to_name(err));
      return false;
    }
    logger_.debug("Adding IP event handler for IP_EVENT_STA_GOT_IP");
    err =
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiSta::event_handler,
                                            this, &event_handler_instance_got_ip_);
    if (err != ESP_OK) {
      logger_.error("Could not add ip GOT_IP event handler: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  bool unregister_event_handlers() {
    esp_err_t err;
    // unregister our any event handler
    logger_.debug("Unregistering any wifi event handler");
    err = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                event_handler_instance_any_id_);
    if (err != ESP_OK) {
      logger_.error("Could not unregister any wifi event handler: {}", esp_err_to_name(err));
      return false;
    }
    // unregister our ip event handler
    logger_.debug("Unregistering got ip event handler");
    err = esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                event_handler_instance_got_ip_);
    if (err != ESP_OK) {
      logger_.error("Could not unregister got ip event handler: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  void init(const Config &config);

  /**
   * @brief Ensure WiFi is initialized, configured, and started for this STA instance.
   * @return true if WiFi is ready, false otherwise.
   * @note This method will reconfigure and restart WiFi if needed.
   */
  bool ensure_wifi_ready() {
    // Check if WiFi is initialized
    if (!is_initialized()) {
      logger_.error("WiFi not initialized - cannot auto-recover. Please reinitialize WiFi.");
      return false;
    }

    // Check if WiFi is started
    wifi_mode_t current_mode;
    esp_err_t err = esp_wifi_get_mode(&current_mode);
    if (err != ESP_OK || current_mode == WIFI_MODE_NULL) {
      logger_.warn("WiFi not started - reconfiguring and starting");
      // Reconfigure using stored config (without changing callbacks)
      if (!reconfigure(config_)) {
        logger_.error("Could not reconfigure WiFi STA");
        return false;
      }
      if (!start()) {
        logger_.error("Could not start WiFi");
        return false;
      }
    } else {
      // WiFi is started, just make sure it's started (handles ESP_ERR_WIFI_STATE gracefully)
      err = esp_wifi_start();
      if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        logger_.error("Could not start WiFi: {}", esp_err_to_name(err));
        return false;
      }
    }

    return true;
  }

  Config config_;                             /**< Stored configuration for state recovery */
  mutable std::recursive_mutex config_mutex_; /**< Mutex for protecting config_ access */
  std::atomic<size_t> attempts_{0};
  std::atomic<bool> connected_{false};
  std::atomic<bool> disconnecting_{false};
  esp_event_handler_instance_t event_handler_instance_any_id_;
  esp_event_handler_instance_t event_handler_instance_got_ip_;
};
} // namespace espp

namespace fmt {
// libfmt printing of WifiSta::Config
template <> struct formatter<espp::WifiSta::Config> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::WifiSta::Config &config, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "WifiSta::Config {{ ssid: '{}', password length: {}, phy_rate: {}, num_connect_retries: "
        "{}, auto_connect: {}, "
        "channel: {}, set_ap_mac: {}, ap_mac: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} }}",
        config.ssid, config.password.size(), config.phy_rate, config.num_connect_retries,
        config.auto_connect, config.channel, config.set_ap_mac, config.ap_mac[0], config.ap_mac[1],
        config.ap_mac[2], config.ap_mac[3], config.ap_mac[4], config.ap_mac[5]);
  }
};

} // namespace fmt
