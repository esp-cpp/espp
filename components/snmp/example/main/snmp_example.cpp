#include <atomic>
#include <chrono>
#include <thread>

#include "sdkconfig.h"

#include "logger.hpp"
#include "snmp.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "snmp_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting SNMP example");

  std::atomic<bool> got_ip{false};
  espp::WifiSta wifi({
      .ssid = CONFIG_SNMP_EXAMPLE_WIFI_SSID,
      .password = CONFIG_SNMP_EXAMPLE_WIFI_PASSWORD,
      .num_connect_retries = 5,
      .on_got_ip =
          [&](ip_event_got_ip_t *event) {
            logger.info("Got IP: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
            got_ip = true;
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  for (int i = 0; i < 200 && !got_ip; i++) {
    std::this_thread::sleep_for(100ms);
  }
  if (!got_ip) {
    logger.error("WiFi connection timed out");
    return;
  }

#if CONFIG_SNMP_EXAMPLE_USE_V3
  constexpr auto snmp_version = espp::Snmp::Version::V3;
  espp::Snmp::V3Config v3_config{
      .username = CONFIG_SNMP_EXAMPLE_V3_USERNAME,
      .auth_password = CONFIG_SNMP_EXAMPLE_V3_AUTH_PASSWORD,
      .privacy_password = CONFIG_SNMP_EXAMPLE_V3_PRIV_PASSWORD,
  };
#else
  constexpr auto snmp_version = espp::Snmp::Version::V2C;
  espp::Snmp::V3Config v3_config{};
#endif

  espp::Snmp::Config config{
      .endpoint = {.host = CONFIG_SNMP_EXAMPLE_TARGET_HOST, .port = 161},
      .version = snmp_version,
      .v2c = {.community = CONFIG_SNMP_EXAMPLE_COMMUNITY},
      .v3 = v3_config,
      .timeout = std::chrono::milliseconds(CONFIG_SNMP_EXAMPLE_TIMEOUT_MS),
      .retries = 1,
      .max_message_size = 1500,
      .log_level = espp::Logger::Verbosity::INFO,
  };

  espp::Snmp snmp(config);
  espp::Snmp::SystemGroup system;
  std::error_code ec;
  if (!snmp.get_system_group(system, ec)) {
    logger.error("Failed to read system group: {}", ec.message());
    return;
  }

  logger.info("sysDescr: {}", system.sys_descr);
  logger.info("sysObjectID: {}", system.sys_object_id.to_string());
  logger.info("sysUpTime: {}", system.sys_up_time);
  logger.info("sysContact: {}", system.sys_contact);
  logger.info("sysName: {}", system.sys_name);
  logger.info("sysLocation: {}", system.sys_location);

  std::vector<espp::Snmp::InterfaceInfo> interfaces;
  if (!snmp.walk_interfaces(interfaces, ec)) {
    logger.warn("Failed to walk interfaces: {}", ec.message());
    return;
  }

  logger.info("Discovered {} interfaces", interfaces.size());
  for (const auto &iface : interfaces) {
    logger.info("ifIndex={} descr='{}' admin={} oper={} alias='{}'", iface.if_index, iface.if_descr,
                static_cast<int>(iface.if_admin_status), static_cast<int>(iface.if_oper_status),
                iface.if_alias);
  }
}
