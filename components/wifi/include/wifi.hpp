#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "esp_wifi.h"

#include "base_component.hpp"
#include "wifi_ap.hpp"
#include "wifi_base.hpp"
#include "wifi_format_helpers.hpp"
#include "wifi_sta.hpp"

namespace espp {

/// @brief The Wifi class provides access to the ESP32 Wifi functionality.
/// @details The Wifi class is a singleton class that provides access to the
/// ESP32 Wifi functionality. The Wifi class is a wrapper around the ESP32
/// Wifi API. The Wifi class provides access to the Wifi AP and Wifi STA
/// functionality, allowing registration and switching between multiple
/// access points and stations.
///
/// @note This class manages the WiFi stack initialization. Call init() before
///       registering any interfaces, and deinit() when done.
///
/// \section wifi_ex1 WiFi Example
/// \snippet wifi_example.cpp wifi example
class Wifi : public espp::BaseComponent {
public:
  /// @brief Access the singleton instance of the Wifi class.
  /// @return The singleton instance of the Wifi class.
  static Wifi &get() {
    static Wifi wifi;
    return wifi;
  }

  Wifi(const Wifi &) = delete;
  Wifi &operator=(const Wifi &) = delete;
  Wifi(Wifi &&) = delete;
  Wifi &operator=(Wifi &&) = delete;

  /// @brief Initialize the WiFi stack.
  /// @return True if initialization was successful, false otherwise.
  /// @note This must be called before registering any WiFi interfaces.
  ///       It's safe to call multiple times - subsequent calls are no-ops.
  bool init() {
    if (initialized_) {
      return true; // Already initialized
    }

    logger_.info("Initializing WiFi stack...");

    esp_err_t err;

    // Initialize network interface
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      logger_.error("Could not initialize netif: {}", esp_err_to_name(err));
      return false;
    }

    // Create default event loop
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      logger_.error("Could not create default event loop: {}", esp_err_to_name(err));
      return false;
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
      return false;
    }

    initialized_ = true;
    return true;
  }

  /// @brief Deinitialize the WiFi stack.
  /// @return True if deinitialization was successful, false otherwise.
  /// @note This will stop and remove all registered interfaces.
  bool deinit() {
    if (!initialized_) {
      return true; // Already deinitialized
    }

    logger_.info("Deinitializing WiFi stack...");

    // Remove all interfaces
    interfaces_.clear();
    active_ = nullptr;
    active_name_.clear();

    // Destroy network interfaces
    if (sta_netif_) {
      esp_netif_destroy_default_wifi(sta_netif_);
      sta_netif_ = nullptr;
    }
    if (ap_netif_) {
      esp_netif_destroy_default_wifi(ap_netif_);
      ap_netif_ = nullptr;
    }

    // Deinit WiFi
    esp_err_t err = esp_wifi_deinit();
    if (err != ESP_OK) {
      logger_.error("Could not deinitialize WiFi subsystem: {}", esp_err_to_name(err));
      return false;
    }

    initialized_ = false;
    return true;
  }

  /// @brief Check if the WiFi stack is initialized.
  /// @return True if initialized, false otherwise.
  bool is_initialized() const { return initialized_; }

  /// @brief Get or create the STA network interface.
  /// @return Pointer to the STA netif, or nullptr on error.
  /// @note If the WiFi subsystem is not initialized, logs an error and returns
  ///       nullptr.
  esp_netif_t *get_sta_netif() {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return nullptr;
    }
    if (!sta_netif_) {
      sta_netif_ = esp_netif_create_default_wifi_sta();
    }
    return sta_netif_;
  }

  /// @brief Get or create the AP network interface.
  /// @return Pointer to the AP netif, or nullptr on error.
  /// @note If the WiFi subsystem is not initialized, logs an error and returns
  ///       nullptr.
  esp_netif_t *get_ap_netif() {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return nullptr;
    }
    if (!ap_netif_) {
      ap_netif_ = esp_netif_create_default_wifi_ap();
    }
    return ap_netif_;
  }

  /// @brief Get the current WiFi mode.
  /// @param mode Reference to a wifi_mode_t variable to store the result.
  /// @return True if retrieval was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool get_mode(wifi_mode_t &mode) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi mode: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Set the WiFi storage type.
  /// @param storage The wifi_storage_t value to set. Can be WIFI_STORAGE_RAM or
  ///                WIFI_STORAGE_FLASH. The default value is
  ///                WIFI_STORAGE_FLASH.
  /// @return True if setting was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool set_storage(wifi_storage_t storage) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_set_storage(storage);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi storage: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Get the current WiFi power save type.
  /// @param ps_type Reference to a wifi_ps_type_t variable to store the result.
  /// @return True if retrieval was successful, false otherwise.
  bool get_power_save(wifi_ps_type_t &ps_type) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_get_ps(&ps_type);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi power save type: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Set the WiFi power save type.
  /// @param ps_type The wifi_ps_type_t value to set.
  /// @return True if setting was successful, false otherwise.
  bool set_power_save(wifi_ps_type_t ps_type) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_set_ps(ps_type);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi power save type: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Get the maximum WiFi transmit power.
  /// @param power_raw Reference to an int8_t variable to store the result.
  /// @return True if retrieval was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  /// @note Returned value is a range [8, 84] representing power in 0.25 dBm
  ///       units, i.e., 8 = 2 dBm, 84 = 21 dBm.
  bool get_max_tx_power_raw(int8_t &power_raw) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_get_max_tx_power(&power_raw);
    if (err != ESP_OK) {
      logger_.error("Could not get WiFi max TX power: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Get the maximum WiFi transmit power in dBm.
  /// @param power_dbm Reference to a float variable to store the result.
  /// @return True if retrieval was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool get_max_tx_power_dbm(float &power_dbm) {
    int8_t power_quarter_dbm;
    if (!get_max_tx_power_raw(power_quarter_dbm)) {
      return false;
    }
    power_dbm = static_cast<float>(power_quarter_dbm) / 4.0f;
    return true;
  }

  /// @brief Set the maximum WiFi transmit power.
  /// @param power_raw The maximum transmit power in 0.25 dBm units.
  /// @return True if setting was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool set_max_tx_power_raw(int8_t power_raw) {
    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    esp_err_t err = esp_wifi_set_max_tx_power(power_raw);
    if (err != ESP_OK) {
      logger_.error("Could not set WiFi max TX power: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// @brief Set the maximum WiFi transmit power in dBm.
  /// @param power_dbm The maximum transmit power in dBm.
  /// @return True if setting was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool set_max_tx_power_dbm(float power_dbm) {
    int8_t power_quarter_dbm = static_cast<int8_t>(power_dbm * 4.0f);
    return set_max_tx_power_raw(power_quarter_dbm);
  }

  /// @brief Register a new WiFi Access Point configuration.
  /// @param name Unique identifier for this AP configuration.
  /// @param config WifiAp::Config structure with AP configuration.
  /// @return True if registration was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool register_ap(const std::string &name, const WifiAp::Config &config) {
    if (!initialized_) {
      return false; // WiFi stack not initialized
    }
    if (interfaces_.find(name) != interfaces_.end()) {
      return false; // Interface with this name already exists
    }
    auto ap = std::make_unique<WifiAp>(config, get_ap_netif());
    // if there is an active interface, stop it
    if (active_) {
      logger_.info("Stopping currently active interface '{}'", active_name_);
      active_->stop();
    }
    // set the active
    active_ = ap.get();
    active_name_ = name;
    // store the interface
    interfaces_[name] = std::move(ap);
    return true;
  }

  /// @brief Register a new WiFi Station configuration.
  /// @param name Unique identifier for this STA configuration.
  /// @param config WifiSta::Config structure with STA configuration.
  /// @return True if registration was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  bool register_sta(const std::string &name, const WifiSta::Config &config) {
    if (!initialized_) {
      return false; // WiFi stack not initialized
    }
    if (interfaces_.find(name) != interfaces_.end()) {
      return false; // Interface with this name already exists
    }
    auto sta = std::make_unique<WifiSta>(config, get_sta_netif());
    // if there is an active interface, stop it
    if (active_) {
      logger_.info("Stopping currently active interface '{}'", active_name_);
      active_->stop();
    }
    // set the active
    active_ = sta.get();
    active_name_ = name;
    // store the interface
    interfaces_[name] = std::move(sta);
    return true;
  }

  /// @brief Switch to a different registered interface.
  /// @param name The name of the interface to switch to.
  /// @return True if the switch was successful, false otherwise.
  bool switch_to(const std::string &name) {
    auto it = interfaces_.find(name);
    if (it == interfaces_.end()) {
      return false; // Interface not found
    }
    if (active_ && active_name_ != name) {
      active_->stop();
    }
    active_ = it->second.get();
    active_name_ = name;
    return active_->start();
  }

  /// @brief Get pointer to the active WiFi interface.
  /// @return Pointer to the active WifiBase instance, or nullptr if none active.
  WifiBase *get_active() { return active_; }

  /// @brief Get pointer to a specific WiFi interface by name.
  /// @param name The name of the interface to retrieve.
  /// @return Pointer to the WifiBase instance, or nullptr if not found.
  WifiBase *get(const std::string &name) {
    auto it = interfaces_.find(name);
    return (it != interfaces_.end()) ? it->second.get() : nullptr;
  }

  /// @brief Get pointer to a specific WiFi AP by name.
  /// @param name The name of the AP to retrieve.
  /// @return Pointer to the WifiAp instance, or nullptr if not found.
  /// @note The caller must ensure the named interface is actually an AP.
  WifiAp *get_ap(const std::string &name) {
    auto it = interfaces_.find(name);
    if (it == interfaces_.end()) {
      return nullptr;
    }
    return static_cast<WifiAp *>(it->second.get());
  }

  /// @brief Get pointer to a specific WiFi STA by name.
  /// @param name The name of the STA to retrieve.
  /// @return Pointer to the WifiSta instance, or nullptr if not found.
  /// @note The caller must ensure the named interface is actually a STA.
  WifiSta *get_sta(const std::string &name) {
    auto it = interfaces_.find(name);
    if (it == interfaces_.end()) {
      return nullptr;
    }
    return static_cast<WifiSta *>(it->second.get());
  }

  /// @brief Get the name of the currently active interface.
  /// @return Name of the active interface, or empty string if none active.
  const std::string &get_active_name() const { return active_name_; }

  /// @brief Remove a registered interface.
  /// @param name The name of the interface to remove.
  /// @return True if the interface was removed, false if not found.
  bool remove(const std::string &name) {
    auto it = interfaces_.find(name);
    if (it == interfaces_.end()) {
      return false;
    }
    if (active_ == it->second.get()) {
      active_->stop();
      active_ = nullptr;
      active_name_.clear();
    }
    interfaces_.erase(it);
    return true;
  }

  /// @brief Get the IP address of the active WiFi interface.
  /// @param ip_address The IP address of the active WiFi interface.
  /// @return True if the IP address was retrieved, false otherwise.
  bool get_ip_address(std::string &ip_address) {
    if (!active_) {
      return false;
    }
    std::string ip = active_->get_ip_address();
    if (ip.empty()) {
      return false;
    }
    ip_address = ip;
    return true;
  }

protected:
  /// @brief Construct a new Wifi object.
  /// @details The Wifi object is a singleton object and can only be
  /// constructed once. WiFi stack initialization is deferred to init().
  Wifi()
      : BaseComponent("Wifi") {}

  /// @brief Destructor - cleans up WiFi stack.
  ~Wifi() {
    if (initialized_) {
      deinit();
    }
  }

  std::unordered_map<std::string, std::unique_ptr<WifiBase>> interfaces_;
  WifiBase *active_{nullptr};
  std::string active_name_;
  bool initialized_{false};
  esp_netif_t *sta_netif_{nullptr};
  esp_netif_t *ap_netif_{nullptr};
};

} // namespace espp
