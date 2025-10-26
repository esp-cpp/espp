#pragma once

#include <string>

#include "esp_netif.h"
#include "esp_wifi.h"

#include "base_component.hpp"

namespace espp {

/**
 * @brief Base class for WiFi interfaces (AP and STA)
 * @details Provides common functionality shared between WiFi Access Point
 *          and WiFi Station implementations.
 */
class WifiBase : public BaseComponent {
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~WifiBase() = default;

  /**
   * @brief Get the hostname of the WiFi interface
   * @return Current hostname as a string
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
   * @brief Set the hostname of the WiFi interface
   * @param hostname New hostname to set
   * @return True if successful, false otherwise
   */
  virtual bool set_hostname(const std::string &hostname) {
    esp_err_t err = esp_netif_set_hostname(netif_, hostname.c_str());
    if (err != ESP_OK) {
      logger_.error("Could not set hostname: {}", err);
      return false;
    }
    logger_.info("Hostname set to '{}'", hostname);
    return true;
  }

  /**
   * @brief Get the MAC address as a string
   * @return MAC address in format "XX:XX:XX:XX:XX:XX"
   */
  virtual std::string get_mac_address_string() = 0;

  /**
   * @brief Get the IP address as a string
   * @return IP address as a string
   */
  virtual std::string get_ip_address() {
    if (netif_ == nullptr) {
      return "";
    }
    esp_netif_ip_info_t ip_info;
    logger_.info("Getting IP address...");
    esp_err_t err = esp_netif_get_ip_info(netif_, &ip_info);
    if (err != ESP_OK) {
      logger_.error("Could not get IP address: {}", err);
      return "";
    }
    return fmt::format("{}.{}.{}.{}", IP2STR(&ip_info.ip));
  }

  /**
   * @brief Check if the interface is connected/active
   * @return True if connected/active, false otherwise
   */
  virtual bool is_connected() const = 0;

  /**
   * @brief Get the SSID
   * @return Current SSID as a string
   */
  virtual std::string get_ssid() = 0;

  /**
   * @brief Start the WiFi interface
   * @return True if successful, false otherwise
   */
  virtual bool start() = 0;

  /**
   * @brief Stop the WiFi interface
   * @return True if successful, false otherwise
   */
  virtual bool stop() = 0;

  /**
   * @brief Get the network interface pointer
   * @return Pointer to the esp_netif_t structure
   */
  esp_netif_t *get_netif() const { return netif_; }

protected:
  /**
   * @brief Constructor for WifiBase
   * @param tag Logger tag
   * @param log_level Logger verbosity level
   */
  explicit WifiBase(const std::string &tag, Logger::Verbosity log_level)
      : BaseComponent(tag, log_level) {}

  /**
   * @brief Constructor for WifiBase
   * @param tag Logger tag
   * @param log_level Logger verbosity level
   * @param netif Pointer to the network interface
   */
  explicit WifiBase(const std::string &tag, Logger::Verbosity log_level, esp_netif_t *netif)
      : BaseComponent(tag, log_level)
      , netif_(netif) {}

  esp_netif_t *netif_{nullptr};
};

} // namespace espp
