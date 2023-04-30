#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <thread>

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#include "file_system.hpp"
#include "ftp_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "main", .level = espp::Logger::Verbosity::INFO});

  logger.info("Stating socket example!");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  std::string ip_address;
  espp::WifiSta wifi_sta({.ssid = CONFIG_ESP_WIFI_SSID,
                          .password = CONFIG_ESP_WIFI_PASSWORD,
                          .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                          .on_connected = nullptr,
                          .on_disconnected = nullptr,
                          .on_got_ip = [&ip_address](ip_event_got_ip_t *eventdata) {
                            ip_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
                            fmt::print("got IP: {}\n", ip_address);
                          }});

  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }

  // NOTE: the call to FileSystem::get() is required to initialize the file system
  espp::FtpServer ftp_server(ip_address, CONFIG_FTP_SERVER_PORT,
                             espp::FileSystem::get().get_root_path());
  ftp_server.start();

  // sleep forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
