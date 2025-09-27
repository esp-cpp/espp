#include <sdkconfig.h>
#include <string>

#include "cli.hpp"
#include "ping.hpp"
#include "wifi_sta.hpp"
#include "wifi_sta_menu.hpp"

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ping_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Ping example...");

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
    //! [ping_simple_example]
    std::atomic<bool> got_ip{false};
    // Simple WiFi STA bring-up assumed configured via menu
    espp::WifiSta wifi_sta({.ssid = "",
                            .password = "",
                            .num_connect_retries = 5,
                            .on_connected = nullptr,
                            .on_disconnected = nullptr,
                            .on_got_ip =
                                [&](ip_event_got_ip_t *eventdata) {
                                  logger.info("got IP: {}.{}.{}.{}",
                                              IP2STR(&eventdata->ip_info.ip));
                                  got_ip = true;
                                },
                            .log_level = espp::Logger::Verbosity::INFO});
    // create the ping instance
    espp::Ping ping({.session =
                         {
                             .target_host = "google.com",
                         },
                     .callbacks = {
                         .on_session_start = [&]() { logger.info("Ping session started"); },
                         .on_reply =
                             [&](uint32_t seq, uint32_t ttl, uint32_t time_ms, uint32_t bytes) {
                               logger.info("Reply: seq={} ttl={} time={}ms bytes={}", seq, ttl,
                                           time_ms, bytes);
                             },
                         .on_timeout = [&]() { logger.warn("Request timed out"); },
                         .on_end =
                             [&](const espp::Ping::Stats &stats) {
                               logger.info("Ping session ended: {}", stats);
                             },
                     },
                     .log_level = espp::Logger::Verbosity::DEBUG});
    // wait for wifi connection
    while (!got_ip) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // run the ping session (synchronously)
    std::error_code ec;
    if (!ping.run(ec)) {
      logger.error("Ping failed to start: {}", ec.message());
    }
    //! [ping_simple_example]
  }

  {
    //! [ping_cli_example]
    // Simple WiFi STA bring-up assumed configured via menu
    espp::WifiSta wifi_sta({.ssid = "",
                            .password = "",
                            .num_connect_retries = 5,
                            .on_connected = nullptr,
                            .on_disconnected = nullptr,
                            .on_got_ip =
                                [&](ip_event_got_ip_t *eventdata) {
                                  logger.info("got IP: {}.{}.{}.{}",
                                              IP2STR(&eventdata->ip_info.ip));
                                },
                            .log_level = espp::Logger::Verbosity::INFO});
    // create the wifi menu to allow connecting to APs
    espp::WifiStaMenu wifi_menu(wifi_sta);

    // create the ping instance and menu
    espp::Ping ping({.session =
                         {
                             .target_host = "1.1.1.1",
                             .task_stack_size = 8192,
                         },
                     .callbacks = {
                         .on_session_start = [&]() { logger.info("Ping session started"); },
                         .on_reply =
                             [&](uint32_t seq, uint32_t ttl, uint32_t time_ms, uint32_t bytes) {
                               logger.info("Reply: seq={} ttl={} time={}ms bytes={}", seq, ttl,
                                           time_ms, bytes);
                             },
                         .on_timeout = [&]() { logger.warn("Request timed out"); },
                         .on_end =
                             [&](const espp::Ping::Stats &stats) {
                               logger.info("Ping session ended: {}", stats);
                             },
                     },
                     .log_level = espp::Logger::Verbosity::DEBUG});
    espp::Ping::Menu ping_menu(ping);

    auto root = std::make_unique<cli::Menu>("root", "root menu");
    root->Insert(wifi_menu.get());
    root->Insert(ping_menu.get());

    cli::Cli cli(std::move(root));
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [ping_cli_example]
  }
}
