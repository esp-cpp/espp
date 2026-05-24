#include <chrono>
#include <vector>

#include "sdkconfig.h"

#include "logger.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#include "iperf_menu.hpp"
#include "wifi_sta_menu.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "iperf_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example...");

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

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
