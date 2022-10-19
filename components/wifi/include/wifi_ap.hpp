#pragma once

#include <string>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "logger.hpp"

namespace espp {
  /**
    *  WiFi Access Point (AP)
    *
    * see
    * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-ap-general-scenario
    *
    * NOTE: if CONFIG_ESP32_WIFI_NVS_ENABLED is set to `y` (which is the
    * default), then you must ensure that you call `nvs_flash_init()` prior to
    * creating the WiFi Access Point.
    *
    * \section wifiap_ex1 WiFi Access Point Example
    * \snippet wifi_example.cpp wifi ap example
    */
  class WifiAp {
  public:
    struct Config {
      std::string ssid; /**< SSID for the access point. */
      std::string password; /**< Password for the access point. If empty, the AP will be open / have no security. */
      uint8_t channel{1}; /**< WiFi channel, range [1,13]. */
      uint8_t max_number_of_stations{4}; /**< Max number of connected stations to this AP. */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity of WifiAp logger. */
    };

    /**
      * @brief Initialize the WiFi Access Point (AP)
      * @param config WifiAp::Config structure with initialization information.
      */
    WifiAp(const Config& config) : logger_({.tag = "WifiAp", .level = config.log_level}) {
      // Code below is modified from:
      // https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/softAP/main/softap_example_main.c
      // NOTE: Init phase
      esp_err_t err;
      logger_.debug("Initializing network interfaces");
      err = esp_netif_init();
      if (err != ESP_OK) {
        logger_.error("Could not initialize esp_netif: {}", err);
      }
      logger_.debug("Creating event loop");
      err = esp_event_loop_create_default();
      if (err != ESP_OK) {
        logger_.error("Could not create default event loop: {}", err);
      }
      logger_.debug("Creating default WiFi AP");
      esp_netif_create_default_wifi_ap();

      // NOTE: Configure phase
      wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
      logger_.debug("Initializing WiFi");
      err = esp_wifi_init(&cfg);
      if (err != ESP_OK) {
        logger_.error("Could not init  wifi: {}", err);
      }

      logger_.debug("Registering event handler");
      err = esp_event_handler_instance_register(WIFI_EVENT,
                                                ESP_EVENT_ANY_ID,
                                                &WifiAp::event_handler,
                                                this,
                                                event_handler_instance_);
      if (err != ESP_OK) {
        logger_.error("Could not register wifi event handler: {}", err);
      }

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

      logger_.debug("Setting mode to AP");
      err = esp_wifi_set_mode(WIFI_MODE_AP);
      if (err != ESP_OK) {
        logger_.error("Could not set WiFi to AP: {}", err);
      }

      logger_.debug("Setting WiFi config");
      err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
      if (err != ESP_OK) {
        logger_.error("Could not create default event loop: {}", err);
      }

      // NOTE: Start phase
      logger_.debug("Starting WiFi");
      err = esp_wifi_start();
      if (err != ESP_OK) {
        logger_.error("Could not create default event loop: {}", err);
      }
      logger_.info("WiFi AP started, SSID: '{}'", config.ssid);
    }

    ~WifiAp() {
      esp_err_t err;
      if (event_handler_instance_) {
        // unregister our event handler
        logger_.debug("Unregistering event handler");
        err = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    *event_handler_instance_);
        if (err != ESP_OK) {
          logger_.error("Could not unregister event handler: {}", err);
        }
      }
      // NOTE: Deinit phase
      // stop the wifi
      logger_.debug("Stopping WiFi");
      err = esp_wifi_stop();
      if (err != ESP_OK) {
        logger_.error("Could not stop WiFiAp: {}", err);
      }
      // deinit the subsystem
      logger_.debug("Deinit WiFi subsystem");
      err = esp_wifi_deinit();
      if (err != ESP_OK) {
        logger_.error("Could not deinit WiFiAp: {}", err);
      }
      logger_.info("WiFi stopped");
    }

  protected:
    static void event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
      auto wifi_ap = static_cast<WifiAp*>(arg);
      if (wifi_ap) {
        wifi_ap->event_handler(event_base, event_id, event_data);
      }
    }

    void event_handler(esp_event_base_t event_base,
                       int32_t event_id, void* event_data) {
      if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        logger_.info("Station join, AID={}", event->aid); // MAC2STR(event->mac) MACSTR
      } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        logger_.info("Station leave, AID={}", event->aid); // MAC2STR(event->mac) MACSTR
      }
    }

    Logger logger_;
    esp_event_handler_instance_t *event_handler_instance_{nullptr};
  };
}
