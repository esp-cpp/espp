#include <chrono>
#include <vector>

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi.hpp"
#include "wifi_ap.hpp"
#include "wifi_sta.hpp"

#include "cli.hpp"
#include "wifi_ap_menu.hpp"
#include "wifi_sta_menu.hpp"

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

  {
    //! [wifi sta menu example]
    auto &wifi = espp::Wifi::get();
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    wifi.register_sta("menu_sta", {.ssid = "",     // use whatever was saved to NVS (if any)
                                   .password = "", // use whatever was saved to NVS (if any)
                                   .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                   .on_connected = nullptr,
                                   .on_disconnected = nullptr,
                                   .on_got_ip =
                                       [&](ip_event_got_ip_t *eventdata) {
                                         logger.info("got IP: {}.{}.{}.{}",
                                                     IP2STR(&eventdata->ip_info.ip));
                                       },
                                   .log_level = espp::Logger::Verbosity::DEBUG});

    auto *wifi_sta = wifi.get_sta("menu_sta");
    if (!wifi_sta) {
      logger.error("Failed to get STA");
      return;
    }

    auto sta_menu = espp::WifiStaMenu(*wifi_sta);
    cli::Cli cli(sta_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();

    wifi.deinit();
    //! [wifi sta menu example]
  }

  {
    logger.info("Starting WiFi STA example...");
    //! [wifi sta example]
    auto &wifi = espp::Wifi::get();
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    wifi.register_sta("example_sta", {.ssid = "",     // use whatever was saved to NVS (if any)
                                      .password = "", // use whatever was saved to NVS (if any)
                                      .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                      .on_connected = nullptr,
                                      .on_disconnected = nullptr,
                                      .on_got_ip =
                                          [&](ip_event_got_ip_t *eventdata) {
                                            logger.info("got IP: {}.{}.{}.{}",
                                                        IP2STR(&eventdata->ip_info.ip));
                                          },
                                      .log_level = espp::Logger::Verbosity::DEBUG});

    auto *wifi_sta = wifi.get_sta("example_sta");
    if (!wifi_sta) {
      logger.error("Failed to get STA");
      return;
    }

    while (!wifi_sta->is_connected()) {
      std::this_thread::sleep_for(100ms);
    }
    //! [wifi sta example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    wifi.deinit();
    logger.info("WiFi STA example complete!");
  }

  {
    //! [wifi ap menu example]
    auto &wifi = espp::Wifi::get();
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    wifi.register_ap("menu_ap", {.ssid = "", .password = ""});

    auto *wifi_ap = wifi.get_ap("menu_ap");
    if (!wifi_ap) {
      logger.error("Failed to get AP");
      return;
    }

    auto ap_menu = espp::WifiApMenu(*wifi_ap);
    cli::Cli cli(ap_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();

    wifi.deinit();
    //! [wifi ap menu example]
  }

  {
    logger.info("Starting WiFi AP example...");
    //! [wifi ap example]
    auto &wifi = espp::Wifi::get();
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    wifi.register_ap("example_ap", {.ssid = CONFIG_ESP_WIFI_SSID,
                                    .password = CONFIG_ESP_WIFI_PASSWORD,
                                    .log_level = espp::Logger::Verbosity::DEBUG});
    //! [wifi ap example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    wifi.deinit();
    logger.info("WiFi AP example complete!");
  }

  {
    logger.info("Starting WiFi singleton example...");
    //! [wifi example]
    auto &wifi = espp::Wifi::get();
    wifi.set_log_level(espp::Logger::Verbosity::INFO);

    // Initialize the WiFi stack
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    // Register multiple station configurations
    wifi.register_sta("home", {.ssid = "",     // use whatever was saved to NVS (if any)
                               .password = "", // use whatever was saved to NVS (if any)
                               .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                               .on_got_ip =
                                   [&](ip_event_got_ip_t *eventdata) {
                                     logger.info("Home network - got IP: {}.{}.{}.{}",
                                                 IP2STR(&eventdata->ip_info.ip));
                                   },
                               .log_level = espp::Logger::Verbosity::INFO});

    wifi.register_sta("backup", {.ssid = "BackupNetwork",
                                 .password = "backuppassword",
                                 .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                 .on_got_ip =
                                     [&](ip_event_got_ip_t *eventdata) {
                                       logger.info("Backup network - got IP: {}.{}.{}.{}",
                                                   IP2STR(&eventdata->ip_info.ip));
                                     },
                                 .log_level = espp::Logger::Verbosity::INFO});

    // Register an AP configuration
    wifi.register_ap("device_ap", {.ssid = "MyESP32-AP",
                                   .password = "esp32password",
                                   .channel = 6,
                                   .max_number_of_stations = 4,
                                   .log_level = espp::Logger::Verbosity::INFO});

    // The first registered interface is automatically active
    logger.info("Active interface: {}", wifi.get_active_name());

    // Wait for connection
    auto *active = wifi.get_active();
    if (active) {
      logger.info("Waiting for connection to {}", wifi.get_active_name());
      while (!active->is_connected()) {
        std::this_thread::sleep_for(100ms);
      }
      logger.info("Connected to {}", wifi.get_active_name());
    }

    std::this_thread::sleep_for(num_seconds_to_run * 1s);

    // Access specific STA by name
    auto *home_sta = wifi.get_sta("home");
    if (home_sta) {
      logger.info("Home STA is available and connected: {}", home_sta->is_connected());
    }

    // Get AP and check connected stations
    auto *device_ap = wifi.get_ap("device_ap");
    if (device_ap) {
      auto stations = device_ap->get_connected_stations();
      logger.info("AP 'device_ap' has {} connected stations", stations.size());
    }

    // Demonstrate switching
    wifi.switch_to("backup");
    //! [wifi example]

    logger.info("WiFi singleton example complete!");
  }

  logger.info("WiFi example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
