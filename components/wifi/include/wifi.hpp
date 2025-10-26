#pragma once

#include <memory>
#include <mutex>
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
  /// @note This will stop all interfaces and clear all configurations.
  bool deinit() {
    if (!initialized_) {
      return true; // Already deinitialized
    }

    logger_.info("Deinitializing WiFi stack...");

    // Clear all configs
    ap_configs_.clear();
    sta_configs_.clear();
    active_ap_name_.clear();
    active_sta_name_.clear();

    // Destroy AP and STA instances
    ap_.reset();
    sta_.reset();

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

  /// @brief Get the esp_netif_t of the currently active interface.
  /// @return Pointer to the esp_netif_t of the active interface, or nullptr if
  ///         no interface is active.
  esp_netif_t *get_active_netif() {
    auto *active = get_active();
    if (!active) {
      return nullptr;
    }
    return active->get_netif();
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
  /// @param set_active If true, immediately activate this AP configuration.
  /// @return True if registration was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  ///       Only one interface (AP or STA) can be active at a time.
  bool register_ap(const std::string &name, const WifiAp::Config &config, bool set_active = false) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    if (ap_configs_.find(name) != ap_configs_.end()) {
      logger_.warn("AP config '{}' already exists", name);
      return false;
    }

    ap_configs_[name] = config;

    if (set_active) {
      return switch_to_ap_internal(name);
    }

    return true;
  }

  /// @brief Register a new WiFi Station configuration.
  /// @param name Unique identifier for this STA configuration.
  /// @param config WifiSta::Config structure with STA configuration.
  /// @param set_active If true, immediately activate this STA configuration.
  /// @return True if registration was successful, false otherwise.
  /// @note WiFi stack must be initialized via init() before calling this.
  ///       Only one interface (AP or STA) can be active at a time.
  bool register_sta(const std::string &name, const WifiSta::Config &config,
                    bool set_active = false) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_) {
      logger_.error("WiFi subsystem not initialized");
      return false;
    }
    if (sta_configs_.find(name) != sta_configs_.end()) {
      logger_.warn("STA config '{}' already exists", name);
      return false;
    }

    sta_configs_[name] = config;

    if (set_active) {
      return switch_to_sta_internal(name);
    }

    return true;
  }

  /// @brief Get all registered STA configuration names.
  /// @return Vector of registered STA configuration names.
  std::vector<std::string> get_registered_sta_names() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> names;
    for (const auto &pair : sta_configs_) {
      names.push_back(pair.first);
    }
    return names;
  }

  /// @brief Get all registered AP configuration names.
  /// @return Vector of registered AP configuration names.
  std::vector<std::string> get_registered_ap_names() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> names;
    for (const auto &pair : ap_configs_) {
      names.push_back(pair.first);
    }
    return names;
  }

  /// @brief Get all registered STA configurations.
  /// @return Unordered map of registered STA configurations.
  std::unordered_map<std::string, WifiSta::Config> get_registered_sta_configs() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return sta_configs_;
  }

  /// @brief Get all registered AP configurations.
  /// @return Unordered map of registered AP configurations.
  std::unordered_map<std::string, WifiAp::Config> get_registered_ap_configs() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return ap_configs_;
  }

  /// @brief Switch to a different registered AP configuration.
  /// @param name The name of the AP configuration to switch to.
  /// @return True if the switch was successful, false otherwise.
  /// @note This will stop any active STA interface.
  bool switch_to_ap(const std::string &name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return switch_to_ap_internal(name);
  }

  /// @brief Switch to a different registered STA configuration.
  /// @param name The name of the STA configuration to switch to.
  /// @return True if the switch was successful, false otherwise.
  /// @note This will stop any active AP interface.
  bool switch_to_sta(const std::string &name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return switch_to_sta_internal(name);
  }

  /// @brief Get pointer to the WiFi AP instance.
  /// @return Pointer to the WifiAp instance, or nullptr if not created.
  WifiAp *get_ap() { return ap_.get(); }

  /// @brief Get pointer to the WiFi STA instance.
  /// @return Pointer to the WifiSta instance, or nullptr if not created.
  WifiSta *get_sta() { return sta_.get(); }

  /// @brief Get pointer to the currently active interface.
  /// @return Pointer to the active WifiBase instance, or nullptr if none active.
  WifiBase *get_active() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!active_sta_name_.empty() && sta_) {
      return sta_.get();
    }
    if (!active_ap_name_.empty() && ap_) {
      return ap_.get();
    }
    return nullptr;
  }

  /// @brief Get the name of the currently active interface (AP or STA).
  /// @return Name of the active interface, or empty string if none active.
  std::string get_active_name() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!active_sta_name_.empty()) {
      return active_sta_name_;
    }
    if (!active_ap_name_.empty()) {
      return active_ap_name_;
    }
    return "";
  }

  /// @brief Check if the active interface is an AP.
  /// @return True if active interface is AP, false otherwise.
  bool is_active_ap() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return !active_ap_name_.empty();
  }

  /// @brief Check if the active interface is a STA.
  /// @return True if active interface is STA, false otherwise.
  bool is_active_sta() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return !active_sta_name_.empty();
  }

  /// @brief Get the name of the currently active AP configuration.
  /// @return Name of the active AP config, or empty string if none active.
  std::string get_active_ap_name() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return active_ap_name_;
  }

  /// @brief Get the name of the currently active STA configuration.
  /// @return Name of the active STA config, or empty string if none active.
  std::string get_active_sta_name() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return active_sta_name_;
  }

  /// @brief Remove a registered AP configuration.
  /// @param name The name of the AP config to remove.
  /// @return True if the config was removed, false if not found.
  bool remove_ap(const std::string &name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto it = ap_configs_.find(name);
    if (it == ap_configs_.end()) {
      return false;
    }

    // Stop if this is the active AP
    if (active_ap_name_ == name) {
      stop_internal();
    }

    ap_configs_.erase(it);
    return true;
  }

  /// @brief Remove a registered STA configuration.
  /// @param name The name of the STA config to remove.
  /// @return True if the config was removed, false if not found.
  bool remove_sta(const std::string &name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto it = sta_configs_.find(name);
    if (it == sta_configs_.end()) {
      return false;
    }

    // Stop if this is the active STA
    if (active_sta_name_ == name) {
      stop_internal();
    }

    sta_configs_.erase(it);
    return true;
  }

  /// @brief Stop the currently active interface (AP or STA).
  /// @return True if an interface was stopped, false if none was active.
  bool stop() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stop_internal();
  }

  /// @brief Get the IP address of the active WiFi interface.
  /// @param ip_address The IP address of the active WiFi interface.
  /// @return True if the IP address was retrieved, false otherwise.
  bool get_ip_address(std::string &ip_address) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto *active = get_active();
    if (!active) {
      logger_.error("No active WiFi interface");
      return false;
    }
    ip_address = active->get_ip_address();
    if (ip_address.empty()) {
      logger_.error("Active WiFi interface has no IP address");
      return false;
    }
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

  /// @brief Internal method to stop active interface without mutex.
  /// @return True if an interface was stopped, false if none was active.
  bool stop_internal() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    bool stopped = false;

    if (!active_sta_name_.empty() && sta_) {
      logger_.info("Stopping active STA '{}'", active_sta_name_);
      sta_->disconnect();
      active_sta_name_.clear();
      stopped = true;
    }

    if (!active_ap_name_.empty() && ap_) {
      logger_.info("Stopping active AP '{}'", active_ap_name_);
      ap_->stop();
      active_ap_name_.clear();
      stopped = true;
    }

    return stopped;
  }

  /// @brief Internal AP switch without mutex (already locked by caller).
  /// @param name The name of the AP configuration to switch to.
  /// @return True if the switch was successful, false otherwise.
  bool switch_to_ap_internal(const std::string &name) {
    logger_.debug("Switching to AP config for name '{}'", name);

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = ap_configs_.find(name);
    if (it == ap_configs_.end()) {
      logger_.error("AP config '{}' not found", name);
      return false;
    }

    // Stop any active interface (mutual exclusivity)
    stop_internal();

    const WifiAp::Config &config = it->second;
    // Create or reconfigure AP
    if (!ap_) {
      ap_ = std::make_unique<WifiAp>(config, get_ap_netif());
      if (!ap_) {
        logger_.error("Failed to create WifiAp");
        return false;
      }
      // Don't need to start here since it's auto-started in constructor
    } else {
      // Reconfigure existing AP with full config
      if (!ap_->reconfigure(config)) {
        logger_.error("Failed to reconfigure WifiAp");
        return false;
      }
      // since we're reconfiguring an existing AP, we need to start it again.
      logger_.info("Starting AP '{}'", name);
      if (!ap_->start()) {
        logger_.error("Failed to start WifiAp after reconfiguration");
        return false;
      }
    }
    logger_.debug("Switched to AP config: {}", config);
    active_ap_name_ = name;
    return true;
  }

  /// @brief Internal STA switch without mutex (already locked by caller).
  /// @param name The name of the STA configuration to switch to.
  /// @return True if the switch was successful, false otherwise.
  bool switch_to_sta_internal(const std::string &name) {
    logger_.debug("Switching to STA config for name '{}'", name);

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = sta_configs_.find(name);
    if (it == sta_configs_.end()) {
      logger_.error("STA config '{}' not found", name);
      return false;
    }

    // Stop any active interface (mutual exclusivity)
    stop_internal();

    const WifiSta::Config &config = it->second;
    // Create or reconfigure STA
    if (!sta_) {
      sta_ = std::make_unique<WifiSta>(config, get_sta_netif());
      if (!sta_) {
        logger_.error("Failed to create WifiSta");
        return false;
      }
      // Don't need to start here since it's auto-started in constructor
    } else {
      // Reconfigure existing STA with full config
      if (!sta_->reconfigure(config)) {
        logger_.error("Failed to reconfigure WifiSta");
        return false;
      }
      // since we're reconfiguring an existing STA, we need to start it again.
      logger_.info("Starting STA '{}'", name);
      if (!sta_->start()) {
        logger_.error("Failed to start WifiSta after reconfiguration");
        return false;
      }
    }
    active_sta_name_ = name;
    logger_.debug("Switched to STA config {}", config);
    return true;
  }

  mutable std::recursive_mutex mutex_; ///< Mutex for thread-safe access to configs and state
  std::unordered_map<std::string, WifiAp::Config> ap_configs_;
  std::unordered_map<std::string, WifiSta::Config> sta_configs_;
  std::unique_ptr<WifiAp> ap_;
  std::unique_ptr<WifiSta> sta_;
  std::string active_ap_name_;
  std::string active_sta_name_;
  bool initialized_{false};
  esp_netif_t *sta_netif_{nullptr};
  esp_netif_t *ap_netif_{nullptr};
};

} // namespace espp
