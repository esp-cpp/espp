#include <chrono>
#include <vector>

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "sdkconfig.h"
#include "wifi_sta.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting WiFi example!\n");

  size_t num_seconds_to_run = 2;

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  {
    espp::WifiSta wifi_sta({
        .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
        .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
        .on_connected = nullptr,
        .on_disconnected = nullptr,
        .on_got_ip = [](ip_event_got_ip_t* eventdata) {
          fmt::print("got IP: {}.{}.{}.{}\n", IP2STR(&eventdata->ip_info.ip));
        }
      });

    while (!wifi_sta.is_connected()) {
      std::this_thread::sleep_for(100ms);
    }

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("WiFi example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
