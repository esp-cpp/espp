#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "sdkconfig.h"

#include "android_tv_remote.hpp"
#include "logger.hpp"
#include "nvs.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  //! [android tv remote example]
  espp::Logger logger({.tag = "android_tv_remote_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Android TV Remote example");

  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger.error("Failed to initialize NVS: {}", ec.message());
    return;
  }

  std::atomic<bool> got_ip{false};
  espp::WifiSta wifi({
      .ssid = CONFIG_ANDROID_TV_REMOTE_EXAMPLE_WIFI_SSID,
      .password = CONFIG_ANDROID_TV_REMOTE_EXAMPLE_WIFI_PASSWORD,
      .num_connect_retries = 5,
      .on_got_ip =
          [&](ip_event_got_ip_t *event) {
            logger.info("Got IP: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
            got_ip = true;
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  for (int i = 0; i < 150 && !got_ip; i++) {
    std::this_thread::sleep_for(100ms);
  }

  if (!got_ip) {
    logger.warn("WiFi connection timed out, skipping remote discovery and control");
    return;
  }

  espp::AndroidTvRemote remote({
      .pairing =
          {
              .client_name = CONFIG_ANDROID_TV_REMOTE_EXAMPLE_CLIENT_NAME,
          },
      .transport =
          {
              .enable_ime =
#ifdef CONFIG_ANDROID_TV_REMOTE_EXAMPLE_ENABLE_IME
                  true,
#else
                  false,
#endif
          },
      .persistence =
          {
              .nvs_namespace = CONFIG_ANDROID_TV_REMOTE_EXAMPLE_PERSISTENCE_NAMESPACE,
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::string target_host = CONFIG_ANDROID_TV_REMOTE_EXAMPLE_TARGET_HOST;
  std::vector<espp::AndroidTvRemote::DeviceInfo> devices;
#ifdef CONFIG_ANDROID_TV_REMOTE_EXAMPLE_ENABLE_DISCOVERY
  if (remote.discover(devices, ec)) {
    logger.info("Discovery returned {} device(s)", devices.size());
    for (const auto &device : devices) {
      logger.info("  {} -> {} ({})", device.name, device.host, device.hostname);
    }
    if (target_host.empty() && !devices.empty()) {
      target_host = devices.front().host;
      logger.info("Using discovered host: {}", target_host);
    }
  } else {
    logger.warn("Discovery failed: {}", ec.message());
    ec.clear();
  }
#endif

  if (target_host.empty()) {
    logger.warn("No target host configured or discovered, ending example");
    return;
  }

#ifdef CONFIG_ANDROID_TV_REMOTE_EXAMPLE_PAIR_IF_CONFIGURED
  if (strlen(CONFIG_ANDROID_TV_REMOTE_EXAMPLE_PAIRING_CODE) > 0) {
    logger.info("Attempting pairing with {}", target_host);
    if (!remote.pair(
            target_host,
            []() -> std::optional<std::string> {
              return std::string(CONFIG_ANDROID_TV_REMOTE_EXAMPLE_PAIRING_CODE);
            },
            ec)) {
      logger.warn("Pairing failed: {}", ec.message());
      ec.clear();
    } else {
      logger.info("Pairing completed");
    }
  } else {
    logger.info("Pairing enabled but no pairing code configured, skipping");
  }
#endif

  if (!remote.connect(target_host, ec)) {
    logger.warn("Connect failed: {}", ec.message());
    return;
  }

  logger.info("Connected, sending a few demo commands");
  remote.home(ec);
  if (ec)
    logger.warn("HOME failed: {}", ec.message());
  ec.clear();
  std::this_thread::sleep_for(250ms);

  remote.send_key(espp::AndroidTvRemote::Key::DpadDown, espp::AndroidTvRemote::Action::Short, ec);
  if (ec)
    logger.warn("DPAD_DOWN failed: {}", ec.message());
  ec.clear();
  std::this_thread::sleep_for(250ms);

  remote.send_key(espp::AndroidTvRemote::Key::Enter, espp::AndroidTvRemote::Action::Short, ec);
  if (ec)
    logger.warn("ENTER failed: {}", ec.message());
  ec.clear();
  std::this_thread::sleep_for(250ms);

#ifdef CONFIG_ANDROID_TV_REMOTE_EXAMPLE_ENABLE_IME
  remote.send_text("hello from espp", ec);
  if (ec)
    logger.warn("Text input failed: {}", ec.message());
  ec.clear();
  std::this_thread::sleep_for(250ms);
#endif

  remote.media_play_pause(ec);
  if (ec)
    logger.warn("Play/pause failed: {}", ec.message());
  ec.clear();

  remote.disconnect();
  logger.info("Android TV Remote example complete");
  //! [android tv remote example]
}
