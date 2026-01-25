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

  // Create provisioning configuration
  espp::Provisioning::Config config{
      .ap_ssid = "ESP-Prov",
      .append_mac_to_ssid = true, // Will append MAC to make unique
      .ap_password = "",          // Open network for simplicity
      .device_name = "Provisioning Example",
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
    logger.error("Failed to start provisioning");
    return;
  }

  logger.info("Provisioning started");
  logger.info("Connect to WiFi network: {}", provisioning.get_ap_ssid());
  logger.info("Open browser to: http://192.168.4.1");

  // Keep running until user completes provisioning
  while (!provisioning.is_completed()) {
    std::this_thread::sleep_for(1s);
  }

  logger.info("Provisioning completed by user, stopping service");
  provisioning.stop();

  logger.info("Provisioning example finished");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
