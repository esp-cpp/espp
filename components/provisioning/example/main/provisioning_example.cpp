#include <chrono>
#include <thread>

#include "logger.hpp"
#include "provisioning.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Provisioning Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting WiFi Provisioning Example");

  // Initialize NVS (required for WiFi)
  logger.info("Initializing NVS...");
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger.error("Failed to initialize NVS: {}", ec.message());
    return;
  }

  // Generate unique AP SSID
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  std::string ap_ssid = fmt::format("ESP-Prov-{:02X}{:02X}", mac[4], mac[5]);
  logger.info("AP SSID: {}", ap_ssid);

  // Create provisioning configuration
  espp::Provisioning::Config config{
      .ap_ssid = ap_ssid,
      .ap_password = "", // Open network for simplicity
      .device_name = "ESP32 Device",
      .server_port = 80,
      .auto_shutdown_ap = false, // Keep AP running for this example
      .ap_timeout = 0s,          // No timeout
      .on_provisioned =
          [&logger](const std::string &ssid, const std::string &password) {
            logger.info("Provisioned successfully!");
            logger.info("  SSID: {}", ssid);
            logger.info("  Password: {}", password.empty() ? "(none)" : "********");

            // Save credentials to NVS
            std::error_code ec;
            espp::NvsHandle handle("wifi_config", ec);
            if (!ec) {
              handle.set("ssid", ssid, ec);
              handle.set("password", password, ec);
              if (!ec) {
                logger.info("Credentials saved to NVS");
              } else {
                logger.error("Failed to save credentials: {}", ec.message());
              }
            }
          },
      .log_level = espp::Logger::Verbosity::INFO};

  {
    // Create provisioning
    espp::Provisioning provisioning(config);

    // print any existing network ssids that have been provisioned
    {
      espp::NvsHandle handle("wifi_config", ec);
      if (!ec) {
        std::string saved_ssid;
        handle.get("ssid", saved_ssid, ec);
        if (!ec) {
          logger.info("Existing provisioned network SSID: {}", saved_ssid);
        } else {
          logger.info("No existing provisioned network found in NVS");
        }
      }
    }

    // Start provisioning
    if (!provisioning.start()) {
      logger.error("Failed to start provisioning: {}", ec.message());
      return;
    }

    logger.info("Provisioning started");
    logger.info("Connect to WiFi network: {}", ap_ssid);
    logger.info("Open browser to: http://192.168.4.1");

    // Keep running
    while (!provisioning.is_provisioned()) {
      std::this_thread::sleep_for(1s);
    }

    logger.info("Device has been provisioned, stopping provisioning service");
    provisioning.stop();
  }

  // destructor should clean up everything, this tests to ensure no issues

  logger.info("Provisioning example finished");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
