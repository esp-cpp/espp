#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "nvs.hpp"
#include "wifi_ap.hpp"
#include "wifi_sta.hpp"

#include "esp_http_server.h"

namespace espp {

/**
 * @brief WiFi Provisioning Component
 *
 * Provides a web-based WiFi provisioning interface. Creates a WiFi access point
 * with an embedded web server that allows users to scan for and connect to WiFi networks.
 *
 * Features:
 * - Automatic AP creation with customizable SSID/password
 * - HTML web interface for WiFi scanning and configuration
 * - Network scanning and signal strength display
 * - Credential validation before saving
 * - Callback notification on successful provisioning
 * - Automatic AP shutdown after provisioning (optional)
 *
 * \section provisioning_ex1 Provisioning Example
 * \snippet provisioning_example.cpp provisioning example
 */
class Provisioning : public BaseComponent {
public:
  /**
   * @brief Callback when provisioning completes successfully
   * @param ssid The SSID that was provisioned
   * @param password The password that was configured
   */
  typedef std::function<void(const std::string &ssid, const std::string &password)>
      provisioned_callback;

  /**
   * @brief Configuration for the provisioning component
   */
  struct Config {
    std::string ap_ssid{"ESP32-Setup"};                   ///< SSID for the provisioning AP
    std::string ap_password{""};                          ///< Password for AP (empty = open)
    std::string device_name{"ESP32 Device"};              ///< Device name shown in UI
    uint16_t server_port{80};                             ///< HTTP server port
    bool auto_shutdown_ap{true};                          ///< Shutdown AP after provisioning
    std::chrono::seconds ap_timeout{300};                 ///< AP timeout (0 = no timeout)
    provisioned_callback on_provisioned{nullptr};         ///< Called on successful provisioning
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /**
   * @brief Construct provisioning component
   * @param config Configuration structure
   */
  explicit Provisioning(const Config &config);

  /**
   * @brief Destructor - stops server and AP
   */
  ~Provisioning();

  /**
   * @brief Start the provisioning process
   * @return true if started successfully
   */
  bool start();

  /**
   * @brief Stop the provisioning process
   */
  void stop();

  /**
   * @brief Check if provisioning is active
   * @return true if the AP and server are running
   */
  bool is_active() const { return is_active_; }

  /**
   * @brief Check if device has been provisioned
   * @return true if provisioning callback has been called
   */
  bool is_provisioned() const { return is_provisioned_; }

  /**
   * @brief Get the IP address of the provisioning AP
   * @return IP address string (e.g., "192.168.4.1")
   */
  std::string get_ip_address() const;

protected:
  void init(const Config &config);
  bool start_ap();
  bool start_server();
  void stop_server();
  void stop_ap();

  // HTTP handlers
  static esp_err_t root_handler(httpd_req_t *req);
  static esp_err_t scan_handler(httpd_req_t *req);
  static esp_err_t connect_handler(httpd_req_t *req);
  static esp_err_t complete_handler(httpd_req_t *req);
  static esp_err_t status_handler(httpd_req_t *req);
  static esp_err_t saved_handler(httpd_req_t *req);
  static esp_err_t delete_handler(httpd_req_t *req);

  // Helper methods
  std::string generate_html() const;
  std::string scan_networks();
  bool test_connection(const std::string &ssid, const std::string &password);

  // Credential storage methods
  std::vector<std::string> get_saved_ssids();
  std::string get_saved_password(const std::string &ssid);
  void save_credentials(const std::string &ssid, const std::string &password);
  void delete_credentials(const std::string &ssid);
  std::string get_saved_networks_json();

  Config config_;
  std::unique_ptr<WifiAp> wifi_ap_;
  std::unique_ptr<WifiSta> test_sta_;
  httpd_handle_t server_{nullptr};
  std::atomic<bool> is_active_{false};
  std::atomic<bool> is_provisioned_{false};
  std::string provisioned_ssid_;
  std::string provisioned_password_;
  std::chrono::time_point<std::chrono::steady_clock> ap_start_time_;
};
} // namespace espp
