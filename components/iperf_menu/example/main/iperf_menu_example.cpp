#include <chrono>
#include <vector>

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#if defined(CONFIG_COMPILER_CXX_EXCEPTIONS)
#include "cli.hpp"
#include "iperf_menu.hpp"
#include "wifi_sta_menu.hpp"
#endif

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "iperf_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example...");

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
    //! [iperf menu example]
    espp::WifiSta wifi_sta({.ssid = "",               // CONFIG_ESP_WIFI_SSID,
                            .password = "",           // CONFIG_ESP_WIFI_PASSWORD,
                            .num_connect_retries = 5, // CONFIG_ESP_MAXIMUM_RETRY,
                            .on_connected = nullptr,
                            .on_disconnected = nullptr,
                            .on_got_ip =
                                [&](ip_event_got_ip_t *eventdata) {
                                  logger.info("got IP: {}.{}.{}.{}",
                                              IP2STR(&eventdata->ip_info.ip));
                                },
                            .log_level = espp::Logger::Verbosity::DEBUG});
    auto sta_menu = espp::WifiStaMenu(wifi_sta);

    auto iperf_menu = espp::IperfMenu();

    // Now make the cli and start it
    auto root_menu = std::make_unique<cli::Menu>("example");
    root_menu->Insert(sta_menu.get());
    root_menu->Insert(iperf_menu.get());
    cli::Cli cli(std::move(root_menu));
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [iperf menu example]
  }
#else
  logger.error("C++ exceptions are not enabled. Please enable them in the menuconfig.");
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
