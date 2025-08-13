#include <chrono>
#include <vector>

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi_ap.hpp"
#include "wifi_sta.hpp"

#if defined(CONFIG_COMPILER_CXX_EXCEPTIONS)
#include "cli.hpp"
#include "wifi_ap_menu.hpp"
#include "wifi_sta_menu.hpp"
#endif

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "wifi_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting WiFi example...");

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

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
  {
    //! [wifi sta menu example]
    espp::WifiSta wifi_sta({.ssid = "",     // CONFIG_ESP_WIFI_SSID,
                            .password = "", // CONFIG_ESP_WIFI_PASSWORD,
                            .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                            .on_connected = nullptr,
                            .on_disconnected = nullptr,
                            .on_got_ip =
                                [&](ip_event_got_ip_t *eventdata) {
                                  logger.info("got IP: {}.{}.{}.{}",
                                              IP2STR(&eventdata->ip_info.ip));
                                },
                            .log_level = espp::Logger::Verbosity::DEBUG});
    auto sta_menu = espp::WifiStaMenu(wifi_sta);
    cli::Cli cli(sta_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [wifi sta menu example]
  }
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

  {
    logger.info("Starting WiFi STA example...");
    //! [wifi sta example]
    espp::WifiSta wifi_sta({.ssid = CONFIG_ESP_WIFI_SSID,
                            .password = CONFIG_ESP_WIFI_PASSWORD,
                            .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                            .on_connected = nullptr,
                            .on_disconnected = nullptr,
                            .on_got_ip =
                                [&](ip_event_got_ip_t *eventdata) {
                                  logger.info("got IP: {}.{}.{}.{}",
                                              IP2STR(&eventdata->ip_info.ip));
                                },
                            .log_level = espp::Logger::Verbosity::DEBUG});

    while (!wifi_sta.is_connected()) {
      std::this_thread::sleep_for(100ms);
    }
    //! [wifi sta example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    logger.info("WiFi STA example complete!");
  }

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
  {
    //! [wifi ap menu example]
    espp::WifiAp wifi_ap({.ssid = "", .password = ""});
    auto ap_menu = espp::WifiApMenu(wifi_ap);
    cli::Cli cli(ap_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [wifi ap menu example]
  }
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

  {
    logger.info("Starting WiFi AP example...");
    //! [wifi ap example]
    espp::WifiAp wifi_ap({.ssid = CONFIG_ESP_WIFI_SSID,
                          .password = CONFIG_ESP_WIFI_PASSWORD,
                          .log_level = espp::Logger::Verbosity::DEBUG});
    //! [wifi ap example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    logger.info("WiFi AP example complete!");
  }

  logger.info("WiFi example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
