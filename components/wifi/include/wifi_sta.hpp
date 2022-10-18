#pragma once

#include <atomic>
#include <string>
#include <functional>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "logger.hpp"

namespace espp {
  /**
    *  WiFi Station (STA)
    *
    * see
    * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-station-general-scenario
    *
    * NOTE: if CONFIG_ESP32_WIFI_NVS_ENABLED is set to `y` (which is the
    * default), then you must ensure that you call `nvs_flash_init()` prior to
    * creating the WiFi Access Point.
    */
  class WifiSta {
  public:
    typedef std::function<void(void)> connect_callback;
    typedef std::function<void(void)> disconnect_callback;
    typedef std::function<void(ip_event_got_ip_t*)> ip_callback;

    struct Config {
      std::string ssid; /**< SSID for the access point. */
      std::string password; /**< Password for the access point. If empty, the AP will be open / have no security. */
      size_t num_connect_retries{0}; /**< Number of times to retry connecting to the AP before stopping. After this many retries, on_disconnected will be called. */
      connect_callback on_connected{nullptr}; /**< Called when the station connects, or fails to connect. */
      disconnect_callback on_disconnected{nullptr}; /**< Called when the station disconnects. */
      ip_callback on_got_ip{nullptr}; /**< Called when the station gets an IP address. */
      uint8_t channel{0}; /**< Channel of target AP; set to 0 for unknown. */
      bool set_ap_mac{false}; /**< Whether to check MAC address of the AP (generally no). If yes, provide ap_mac. */
      uint8_t ap_mac[6]{0}; /**< MAC address of the AP to check if set_ap_mac is set to true. */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity of WifiAp logger. */
    };

    /**
     * @brief Initialize the WiFi Station (STA)
     * @param config WifiAp::Config structure with initialization information.
     */
    WifiSta(const Config& config)
      : num_retries_(config.num_connect_retries),
        connect_callback_(config.on_connected),
        disconnect_callback_(config.on_disconnected),
        ip_callback_(config.on_got_ip),
        logger_({.tag = "WifiAp", .level = config.log_level}) {
      // Code below is modified from:
      // https://github.com/espressif/esp-idf/blob/1c84cfde14dcffdc77d086a5204ce8a548dce935/examples/wifi/getting_started/station/main/station_example_main.c
      esp_err_t err;
      err = esp_netif_init();
      if (err != ESP_OK) {
        logger_.error("Could not initialize netif: {}", err);
      }

      err = esp_event_loop_create_default();
      if (err != ESP_OK) {
        logger_.error("Could not create default event loop: {}", err);
      }
      esp_netif_create_default_wifi_sta();

      wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
      err = esp_wifi_init(&cfg);
      if (err != ESP_OK) {
        logger_.error("Could not init wifi subsystem: {}", err);
      }

      err = esp_event_handler_instance_register(WIFI_EVENT,
                                                ESP_EVENT_ANY_ID,
                                                &WifiSta::event_handler,
                                                this,
                                                event_handler_instance_any_id_);
      if (err != ESP_OK) {
        logger_.error("Could not add wifi ANY event handler: {}", err);
      }
      err = esp_event_handler_instance_register(IP_EVENT,
                                                IP_EVENT_STA_GOT_IP,
                                                &WifiSta::event_handler,
                                                this,
                                                event_handler_instance_got_ip_);
      if (err != ESP_OK) {
        logger_.error("Could not add ip GOT_IP event handler: {}", err);
      }

      wifi_config_t wifi_config;
      memset(&wifi_config, 0, sizeof(wifi_config));
      wifi_config.sta.channel = config.channel;
      wifi_config.sta.bssid_set = config.set_ap_mac;
      if (config.set_ap_mac) {
        memcpy(wifi_config.sta.bssid, config.ap_mac, 6);
      }
      memcpy(wifi_config.sta.ssid, config.ssid.data(), config.ssid.size());
      memcpy(wifi_config.sta.password, config.password.data(), config.password.size());

      err = esp_wifi_set_mode(WIFI_MODE_STA);
      if (err != ESP_OK) {
        logger_.error("Could not set WiFi mode STA: {}", err);
      }
      err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
      if (err != ESP_OK) {
        logger_.error("Could not set WiFi config: {}", err);
      }
      err = esp_wifi_start();
      if (err != ESP_OK) {
        logger_.error("Could not start WiFi: {}", err);
      }
    }

    ~WifiSta() {
      esp_err_t err;
      if (event_handler_instance_any_id_) {
        // unregister our event handler
        logger_.debug("Unregistering any wifi event handler");
        err = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    *event_handler_instance_any_id_);
        if (err != ESP_OK) {
          logger_.error("Could not unregister any wifi event handler: {}", err);
        }
      }
      if (event_handler_instance_got_ip_) {
        // unregister our event handler
        logger_.debug("Unregistering got ip event handler");
        err = esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                    *event_handler_instance_got_ip_);
        if (err != ESP_OK) {
          logger_.error("Could not unregister got ip event handler: {}", err);
        }
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
    }

    bool is_connected() {
      return connected_;
    }

  protected:
    static void event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
      auto wifi_sta = static_cast<WifiSta*>(arg);
      if (wifi_sta) {
        wifi_sta->event_handler(event_base, event_id, event_data);
      }
    }

    void event_handler(esp_event_base_t event_base,
                       int32_t event_id, void* event_data) {
      if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        connected_ = false;
        esp_wifi_connect();
      } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        connected_ = false;
        if (attempts_ < num_retries_) {
          esp_wifi_connect();
          attempts_++;
          logger_.info("retry to connect to the AP");
        } else {
          if (disconnect_callback_) {
            disconnect_callback_();
          }
        }
        logger_.info("connect to the AP fail");
      } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
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

    size_t attempts_{0};
    size_t num_retries_{0};
    connect_callback connect_callback_{nullptr};
    disconnect_callback disconnect_callback_{nullptr};
    ip_callback ip_callback_{nullptr};
    std::atomic<bool> connected_{false};
    Logger logger_;
    esp_event_handler_instance_t *event_handler_instance_any_id_{nullptr};
    esp_event_handler_instance_t *event_handler_instance_got_ip_{nullptr};
  };
}
