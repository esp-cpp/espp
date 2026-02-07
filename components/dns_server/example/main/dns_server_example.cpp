#include <chrono>
#include <system_error>
#include <vector>

#include "dns_server.hpp"
#include "logger.hpp"
#include "wifi.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "DNS Server Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting DNS Server Example");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  // Initialize WiFi in AP mode
  std::string ap_ssid = "ESP-DNS-Test";
  std::string ap_password = "testpassword";

  logger.info("Starting WiFi AP: {}", ap_ssid);

  espp::WifiAp ap{espp::WifiAp::Config{
      .ssid = ap_ssid,
      .password = ap_password,
      .channel = 1,
      .max_number_of_stations = 4,
  }};

  logger.info("WiFi AP started successfully");
  logger.info("Connect to SSID: {} with password: {}", ap_ssid, ap_password);

  // Get the AP IP address
  std::string ap_ip = ap.get_ip_address();
  logger.info("AP IP Address: {}", ap_ip);

  // Create and start DNS server
  logger.info("Starting DNS server on {}:53", ap_ip);

  espp::DnsServer::Config dns_config{.ip_address = ap_ip,
                                     .log_level = espp::Logger::Verbosity::INFO};

  espp::DnsServer dns_server(dns_config);
  std::error_code ec;
  if (!dns_server.start(ec)) {
    logger.error("Failed to start DNS server: {}", ec.message());
    return;
  }

  logger.info("DNS server started successfully");
  logger.info("All DNS queries will resolve to: {}", ap_ip);
  logger.info("");
  logger.info("To test:");
  logger.info("1. Connect your device to WiFi network '{}'", ap_ssid);
  logger.info("2. Try pinging any domain (e.g., ping google.com)");
  logger.info("3. All domains should resolve to {}", ap_ip);

  // Run forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
