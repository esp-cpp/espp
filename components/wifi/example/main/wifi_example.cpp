#include <chrono>
#include <string>
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

// Test result tracking
struct TestResult {
  std::string name;
  bool passed;
  std::string details;
};

std::vector<TestResult> test_results;

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
    espp::WifiSta::Config config{.ssid = "",     // use whatever was saved to NVS (if any)
                                 .password = "", // use whatever was saved to NVS (if any)
                                 .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                 .on_connected = nullptr,
                                 .on_disconnected = nullptr,
                                 .on_got_ip =
                                     [&](ip_event_got_ip_t *eventdata) {
                                       logger.info("got IP: {}.{}.{}.{}",
                                                   IP2STR(&eventdata->ip_info.ip));
                                     },
                                 .log_level = espp::Logger::Verbosity::DEBUG};

    espp::WifiSta wifi_sta(config);
    auto sta_menu = espp::WifiStaMenu(wifi_sta);
    cli::Cli cli(sta_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [wifi sta menu example]
  }

  {
    logger.info("Starting WiFi STA example...");
    //! [wifi sta example]
    espp::WifiSta::Config config{.ssid = "",     // use whatever was saved to NVS (if any)
                                 .password = "", // use whatever was saved to NVS (if any)
                                 .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                 .on_connected = nullptr,
                                 .on_disconnected = nullptr,
                                 .on_got_ip =
                                     [&](ip_event_got_ip_t *eventdata) {
                                       logger.info("got IP: {}.{}.{}.{}",
                                                   IP2STR(&eventdata->ip_info.ip));
                                     },
                                 .log_level = espp::Logger::Verbosity::DEBUG};

    espp::WifiSta wifi_sta(config);
    while (!wifi_sta.is_connected()) {
      std::this_thread::sleep_for(100ms);
    }
    //! [wifi sta example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    logger.info("WiFi STA example complete!");
  }

  {
    //! [wifi ap menu example]
    espp::WifiAp::Config config{.ssid = "ESP++ WiFi AP", .password = ""};

    espp::WifiAp wifi_ap(config);
    auto ap_menu = espp::WifiApMenu(wifi_ap);
    cli::Cli cli(ap_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [wifi ap menu example]
  }

  {
    logger.info("Starting WiFi AP example...");
    //! [wifi ap example]
    espp::WifiAp::Config config{.ssid = CONFIG_ESP_WIFI_SSID,
                                .password = CONFIG_ESP_WIFI_PASSWORD,
                                .log_level = espp::Logger::Verbosity::DEBUG};
    espp::WifiAp wifi_ap(config);
    //! [wifi ap example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    logger.info("WiFi AP example complete!");
  }

  {
#if CONFIG_TEST_WIFI_SCAN && CONFIG_TEST_SCAN_WITH_NO_CONFIG
    logger.info("Starting WiFi scan test (no config)...");
    //! [wifi scan no config example]
    auto &wifi = espp::Wifi::get();
    wifi.set_log_level(espp::Logger::Verbosity::DEBUG);

    // Initialize the WiFi stack
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    // Test scanning with no active configuration
    logger.info("Scanning for networks (no STA/AP active)...");
    auto scan_results = wifi.scan(10);
    logger.info("Found {} networks:", scan_results.size());
    for (const auto &ap : scan_results) {
      logger.info("  SSID: {}, RSSI: {}, Channel: {}", (char *)ap.ssid, ap.rssi, ap.primary);
    }

    wifi.deinit();
    logger.info("WiFi scan test (no config) complete!");
    std::this_thread::sleep_for(2s);
    //! [wifi scan no config example]
#endif
  }

  {
    logger.info("Starting WiFi singleton example...");
    //! [wifi example]
    auto &wifi = espp::Wifi::get();
    wifi.set_log_level(espp::Logger::Verbosity::DEBUG);

    // Initialize the WiFi stack
    if (!wifi.init()) {
      logger.error("Failed to initialize WiFi stack");
      return;
    }

    // Register multiple station configurations
    wifi.register_sta(
        "home",
        {.ssid = "",     // use whatever was saved to NVS (if any)
         .password = "", // use whatever was saved to NVS (if any)
         .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
         .auto_connect =
             !CONFIG_TEST_SCAN_BEFORE_STA, // Don't auto-connect if we're going to scan first
         .on_got_ip =
             [&](ip_event_got_ip_t *eventdata) {
               logger.info("Home network - got IP: {}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
             },
         .log_level = espp::Logger::Verbosity::INFO});

    // set the 'home' network to be active and ensure that the backup
    // registration (when added) doesn't override it
    wifi.switch_to_sta("home");

#if CONFIG_TEST_WIFI_SCAN && CONFIG_TEST_SCAN_BEFORE_STA
    logger.info("\n=== Testing scan before STA connection ===");
    //! [wifi scan before sta example]

    bool test_passed = false;
    std::string test_details;

    // STA was created with auto_connect=false, so it won't connect automatically
    // Scan for networks first
    logger.info("Scanning for networks before connecting...");
    auto scan_results = wifi.scan(10);
    logger.info("Found {} networks:", scan_results.size());
    for (const auto &ap : scan_results) {
      logger.info("  SSID: {}, RSSI: {}, Channel: {}", (char *)ap.ssid, ap.rssi, ap.primary);
    }

    // Now manually initiate the connection
    logger.info("Attempting to connect to STA after scan...");
    auto *sta = wifi.get_sta();
    if (sta) {
      sta->connect(); // Manually trigger connection

      int wait_count = 0;
      while (!sta->is_connected() && wait_count < 150) { // 15 second timeout
        std::this_thread::sleep_for(100ms);
        wait_count++;
      }
      if (sta->is_connected()) {
        logger.info("STA connected successfully after scan!");
        std::string ip;
        if (wifi.get_ip_address(ip)) {
          logger.info("IP address: {}", ip);
          test_passed = true;
          test_details = fmt::format("Connected, IP: {}", ip);
        }
      } else {
        logger.warn("STA failed to connect after scan within timeout");
        test_details = "Connection timeout";
      }
    } else {
      test_details = "No STA instance";
    }

    test_results.push_back(
        {.name = "Scan before STA connect", .passed = test_passed, .details = test_details});
    std::this_thread::sleep_for(2s);
    //! [wifi scan before sta example]
#endif

    wifi.register_sta("backup", {.ssid = "BackupNetwork",
                                 .password = "backuppassword",
                                 .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                                 .on_got_ip =
                                     [&](ip_event_got_ip_t *eventdata) {
                                       logger.info("Backup network - got IP: {}.{}.{}.{}",
                                                   IP2STR(&eventdata->ip_info.ip));
                                     },
                                 .log_level = espp::Logger::Verbosity::INFO});

    // ensure that the active interface is still 'home'
    if (wifi.get_active_name() != "home") {
      logger.error("Active interface changed unexpectedly after registering backup STA");
      return;
    }

    // Register an AP configuration
    wifi.register_ap("device_ap", {.ssid = "MyESP32-AP",
                                   .password = "esp32password",
                                   .channel = 6,
                                   .max_number_of_stations = 4,
                                   .log_level = espp::Logger::Verbosity::INFO});

    // ensure that the active interface is still 'home' (STA)
    if (wifi.get_active_name() != "home") {
      logger.error("Active interface changed unexpectedly after registering AP");
      return;
    }

    logger.info("Registered STA configurations: ");
    auto sta_names = wifi.get_registered_sta_names();
    for (const auto &name : sta_names) {
      logger.info(" - '{}'", name);
    }
    logger.info("Registered AP configurations: ");
    auto ap_names = wifi.get_registered_ap_names();
    for (const auto &name : ap_names) {
      logger.info(" - '{}'", name);
    }

    // now switch to AP
    logger.info("\n=== Testing switch to AP ===");
    wifi.switch_to_ap("device_ap");

    // Check what's currently active
    std::string active_name = wifi.get_active_name();
    bool ap_test_passed = (active_name == "device_ap");
    std::string ap_test_details;

    if (!ap_test_passed) {
      logger.error("Active interface is not 'device_ap' after switch_to_ap");
      ap_test_details = fmt::format("Wrong active: {}", active_name);
    } else {
      logger.info("Active interface: '{}' (is_ap={}, is_sta={})", active_name, wifi.is_active_ap(),
                  wifi.is_active_sta());

      // Wait for the active interface to be ready/connected
      auto *active = wifi.get_active();
      if (active) {
        logger.info("Waiting for '{}' to be connected...", active_name);
        while (!active->is_connected()) {
          std::this_thread::sleep_for(100ms);
        }
        logger.info("'{}' is now connected!", active_name);

        logger.info("Checking IP address...");
        // Get and display IP
        std::string ip;
        if (wifi.get_ip_address(ip)) {
          logger.info("IP address: {}", ip);
          ap_test_details = fmt::format("AP ready, IP: {}", ip);
        }
      }
    }

    test_results.push_back(
        {.name = "Switch to AP", .passed = ap_test_passed, .details = ap_test_details});
    std::this_thread::sleep_for(num_seconds_to_run * 1s);

    // Access STA instance (single instance, manages all configs)
    auto *home_sta = wifi.get_sta();
    if (home_sta) {
      logger.info("STA is connected: {}", home_sta->is_connected());
    }

    // Get AP and check connected stations
    auto *device_ap = wifi.get_ap();
    if (device_ap) {
      auto stations = device_ap->get_connected_stations();
      logger.info("AP has {} connected stations", stations.size());
    }

    // Demonstrate switching to STA
    logger.info("\n=== Testing switch to STA ===");
    wifi.switch_to_sta("home");

    // If auto_connect was disabled (e.g., from scan test), manually connect
    sta = wifi.get_sta();
    if (sta && !sta->is_connected()) {
      logger.info("STA not auto-connected, manually connecting...");
      sta->connect();
    }

    active_name = wifi.get_active_name();
    logger.info("Active interface: '{}' (is_ap={}, is_sta={})", active_name, wifi.is_active_ap(),
                wifi.is_active_sta());

    bool sta_test_passed = false;
    std::string sta_test_details;

    // Wait for STA to connect
    auto active = wifi.get_active();
    active_name = wifi.get_active_name();
    if (active) {
      logger.info("Waiting for '{}' to be connected...", active_name);
      int wait_count = 0;
      while (!active->is_connected() && wait_count < 100) {
        std::this_thread::sleep_for(100ms);
        wait_count++;
      }
      if (active->is_connected()) {
        logger.info("'{}' is now connected!", active_name);
        std::string ip;
        if (wifi.get_ip_address(ip)) {
          logger.info("IP address: {}", ip);
          sta_test_passed = true;
          sta_test_details = fmt::format("Connected, IP: {}", ip);
        }
      } else {
        logger.warn("'{}' failed to connect within timeout", active_name);
        sta_test_details = "Connection timeout";
      }
    } else {
      sta_test_details = "No active interface";
    }

    test_results.push_back(
        {.name = "Switch to STA", .passed = sta_test_passed, .details = sta_test_details});
    std::this_thread::sleep_for(2s);

    // Demonstrate switching back to AP
    logger.info("\n=== Testing switch back to AP ===");
    wifi.switch_to_ap("device_ap");
    active_name = wifi.get_active_name();
    logger.info("Active interface: '{}' (is_ap={}, is_sta={})", active_name, wifi.is_active_ap(),
                wifi.is_active_sta());

    bool ap_switch_back_passed = (active_name == "device_ap");
    std::string ap_switch_back_details;

    // Wait for AP to be ready
    active = wifi.get_active();
    if (active) {
      logger.info("Waiting for '{}' to be ready...", active_name);
      std::this_thread::sleep_for(1s); // AP starts quickly
      if (active->is_connected()) {    // For AP, this checks if stations are connected
        logger.info("'{}' has stations connected", active_name);
      } else {
        logger.info("'{}' is ready (no stations connected yet)", active_name);
      }

      std::string ip;
      if (wifi.get_ip_address(ip)) {
        logger.info("IP address: {}", ip);
        ap_switch_back_details = fmt::format("AP ready, IP: {}", ip);
      }
    } else {
      ap_switch_back_passed = false;
      ap_switch_back_details = "No active interface";
    }

    test_results.push_back({.name = "Switch back to AP",
                            .passed = ap_switch_back_passed,
                            .details = ap_switch_back_details});
    std::this_thread::sleep_for(2s);

    // Demonstrate stop
    logger.info("\n=== Testing stop ===");
    wifi.stop();
    active_name = wifi.get_active_name();
    logger.info("Active interface after stop: '{}' (empty=stopped)", active_name);

    bool stop_test_passed = active_name.empty();
    std::string stop_test_details =
        stop_test_passed ? "WiFi stopped successfully" : "WiFi still active";
    test_results.push_back(
        {.name = "Stop WiFi", .passed = stop_test_passed, .details = stop_test_details});

    //! [wifi example]

    logger.info("WiFi singleton example complete!");
  }

  // now that we're done with the examples, deinit the wifi stack
  auto &wifi = espp::Wifi::get();
  if (!wifi.deinit()) {
    logger.error("Failed to deinitialize WiFi stack");
    return;
  }
  logger.info("WiFi stack deinitialized");

  // Print test summary
  logger.info("\n==========================================================");
  logger.info("                    TEST SUMMARY                          ");
  logger.info("==========================================================");

  int passed = 0;
  int failed = 0;

  for (const auto &result : test_results) {
    std::string status = result.passed ? "✓ PASS" : "✗ FAIL";
    logger.info("{}: {} - {}", status, result.name, result.details);

    if (result.passed) {
      passed++;
    } else {
      failed++;
    }
  }

  logger.info("==========================================================");
  logger.info("Total: {} | Passed: {} | Failed: {}", test_results.size(), passed, failed);
  logger.info("==========================================================\n");

  logger.info("WiFi example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
