#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "sdkconfig.h"

#include "android_tv_remote.hpp"
#include "logger.hpp"
#include "nvs.hpp"
#include "wifi_sta.hpp"

#include "remote_ui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  //! [android tv remote example]
  espp::Logger logger({.tag = "android_tv_remote_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Android TV Remote example");

  // NVS must be initialized before the UI (which may read saved WiFi
  // credentials from it) and before WiFi.
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger.error("Failed to initialize NVS: {}", ec.message());
    return;
  }

  // Select and bring up the UI (serial console by default, or a board screen +
  // keyboard/touch when built with -DATV_BOARD=...). See remote_ui.hpp.
  std::unique_ptr<RemoteUi> ui = atv_make_remote_ui();
  if (!ui->begin(logger)) {
    logger.error("Failed to initialize the UI");
    return;
  }

  std::atomic<bool> got_ip{false};
  auto make_wifi_config = [&](const WifiCreds &creds) {
    return espp::WifiSta::Config{
        .ssid = creds.ssid,
        .password = creds.password,
        .num_connect_retries = 5,
        .on_got_ip =
            [&](ip_event_got_ip_t *event) {
              logger.info("Got IP: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
              got_ip = true;
            },
        .log_level = espp::Logger::Verbosity::INFO,
    };
  };

  WifiCreds creds = ui->get_wifi_credentials();
  ui->status(creds.ssid.empty() ? "Connecting to saved WiFi..."
                                : "Connecting to WiFi: " + creds.ssid);
  espp::WifiSta wifi(make_wifi_config(creds));

  auto wait_for_ip = [&]() {
    got_ip = false;
    for (int i = 0; i < 150 && !got_ip; i++)
      std::this_thread::sleep_for(100ms);
    return got_ip.load();
  };

  bool connected = wait_for_ip();
#if defined(ATV_UI_BOARD)
  // On a board the user can re-enter credentials, so keep prompting until we
  // connect instead of giving up.
  while (!connected) {
    ui->status("WiFi failed for '" + creds.ssid + "'. Re-enter the network.");
    creds = ui->get_wifi_credentials();
    ui->status("Connecting to WiFi: " + creds.ssid);
    wifi.reconfigure(make_wifi_config(creds));
    connected = wait_for_ip();
  }
#endif
  if (!connected) {
    ui->status("WiFi connection timed out");
    logger.warn("WiFi connection timed out, skipping remote discovery and control");
    return;
  }
  ui->status("WiFi connected");

  espp::AndroidTvRemote remote({
      .discovery = {},
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
              // Give the TLS handshake generous headroom: on a board that is
              // also running a display + input tasks, mbedtls gets less CPU, and
              // the very first connect may follow a multi-second RSA keygen.
              .connect_timeout = std::chrono::seconds(15),
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
  ui->status("Discovering devices...");
  if (remote.discover(devices, ec)) {
    logger.info("Discovery returned {} device(s)", devices.size());
    for (size_t i = 0; i < devices.size(); i++) {
      logger.info("  [{}] {} -> {} ({})", i, devices[i].name, devices[i].host, devices[i].hostname);
    }
    // A configured target host always wins; otherwise pick from discovery.
    if (target_host.empty() && !devices.empty()) {
      size_t index = 0;
#if !defined(ATV_UI_BOARD)
      // On the serial console, let the tester choose when several remote-capable
      // devices are found. Board UIs auto-select the first (set TARGET_HOST to
      // override), since arbitrary on-screen list selection is out of scope.
      if (devices.size() > 1)
        index = atv_read_index_from_console(logger, devices.size());
#endif
      target_host = devices[index].host;
      logger.info("Using discovered device [{}]: {} -> {}", index, devices[index].name, target_host);
    }
  } else {
    logger.warn("Discovery failed: {}", ec.message());
    ec.clear();
  }
#endif

  if (target_host.empty()) {
    ui->status("No target host found");
    logger.warn("No target host configured or discovered, ending example");
    return;
  }
  ui->status("Connecting to " + target_host);

  // The control port (6466) only accepts a client certificate the TV has
  // already authorized through pairing (6467). Try to connect first: if this
  // device is already paired, the stored certificate is reused and no code
  // prompt appears. If it is not yet paired the TV resets the control
  // handshake (mbedtls NET_CONN_RESET), so fall back to the interactive
  // pairing exchange -- the TV shows a code once the pairing connection is up,
  // which the UI reads from the user -- and then connect again.
  if (!remote.connect(target_host, ec)) {
    logger.info("Control connect failed ({}); this device is likely not paired yet, "
                "starting pairing with {}",
                ec.message(), target_host);
    ec.clear();
    ui->status("Pairing with " + target_host);
    if (!remote.pair(
            target_host, [&ui]() -> std::optional<std::string> { return ui->read_code(); }, ec)) {
      logger.error("Pairing failed: {}", ec.message());
      ui->status(std::string("Pairing failed: ") + ec.message());
      return;
    }
    logger.info("Pairing completed; connecting");
    ui->status("Paired; connecting");
    if (!remote.connect(target_host, ec)) {
      logger.error("Connect after pairing failed: {}", ec.message());
      ui->status(std::string("Connect failed: ") + ec.message());
      return;
    }
  }

  logger.info("Connected to {}", target_host);
  ui->status("Connected");

  // Hand off to the UI to drive the remote (interactive on a board, or a short
  // scripted demo on the serial console).
  ui->control_loop(remote);

  remote.disconnect();
  logger.info("Android TV Remote example complete");
  ui->status("Disconnected");
  //! [android tv remote example]
}
