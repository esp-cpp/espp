#pragma once

// Network: the Wi-Fi station (espp::WifiSta -- status, scan, connect /
// disconnect with the credentials kept in NVS) and, under its own Kconfig
// gate, the RMII Ethernet link (espp::Ethernet). The interfaces are brought
// up on the first launch and kept across windows (closing the window does not
// disconnect); the window's 1 s timer renders the state the driver callbacks
// record. A Wi-Fi scan blocks (and disconnects first), so it runs on its own
// task.

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sdkconfig.h"

#include "desktop.hpp"
#include "nvs_handle_espp.hpp"
#include "task.hpp"
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
#include "esp_wifi.h"
#include "wifi.hpp"
#include "wifi_sta.hpp"
#endif
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
#include "esp_netif.h"
#include "ethernet.hpp"
#endif

namespace desktop_example {

/// The interfaces, shared by every Network window (created on first launch).
struct NetworkState {
  std::mutex mutex;
  std::string wifi_status{"idle"};
  std::string wifi_ip;
  std::string wifi_mac;
  std::vector<std::string> scan_ssids; // in AP list order
  std::vector<std::string> scan_rows;  // the AP list's items (shown again by a new window)
  /// A scan is in flight on the scan task: it disconnects and drives the
  /// station, so every other station action is refused until it is done.
  std::atomic<bool> scanning{false};
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
  std::unique_ptr<espp::WifiSta> wifi;
  std::unique_ptr<espp::Task> scan_task; // after wifi: joined before it goes away
#endif
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
  std::unique_ptr<espp::Ethernet> eth;
#endif

  void set_status(std::string_view status, std::string_view ip = "") {
    std::lock_guard<std::mutex> lock(mutex);
    wifi_status = std::string(status);
    wifi_ip = std::string(ip);
  }

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
  static const char *auth_name(wifi_auth_mode_t mode) {
    switch (mode) {
    case WIFI_AUTH_OPEN:
      return "open";
    case WIFI_AUTH_WEP:
      return "WEP";
    case WIFI_AUTH_WPA_PSK:
      return "WPA";
    case WIFI_AUTH_WPA2_PSK:
      return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:
      return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "WPA2/WPA3";
    default:
      return "other";
    }
  }

  /// The station config for `ssid` / `password`: the callbacks record the
  /// state this struct holds (they run on the event-loop task).
  espp::WifiSta::Config wifi_config(std::string ssid, std::string password) {
    return {.ssid = std::move(ssid),
            .password = std::move(password),
            .num_connect_retries = 3,
            .auto_connect = true,
            .on_connected = [this]() { set_status("connected, waiting for an IP"); },
            .on_disconnected = [this]() { set_status("disconnected"); },
            .on_got_ip =
                [this](ip_event_got_ip_t *e) {
                  set_status("connected", fmt::format("{}.{}.{}.{}", IP2STR(&e->ip_info.ip)));
                },
            .log_level = espp::Logger::Verbosity::INFO};
  }

  /// A string from NVS ("" when unset). NvsHandle::get() sizes the string to
  /// the stored length INCLUDING the terminating NUL, so strip trailing NULs
  /// (a 32-byte SSID would otherwise come back as 33 bytes).
  static std::string nvs_string(espp::NvsHandle &nvs, const char *key) {
    std::error_code ec;
    std::string s;
    nvs.get(key, s, std::string(""), ec);
    if (ec)
      return "";
    while (!s.empty() && s.back() == '\0')
      s.pop_back();
    return s;
  }

  /// Bring the station up once, with the saved credentials (none = idle).
  void ensure_wifi() {
    if (wifi)
      return;
    std::error_code ec;
    espp::NvsHandle nvs("desktop", ec);
    std::string ssid, pass;
    if (!ec) {
      ssid = nvs_string(nvs, "wifi_ssid");
      pass = nvs_string(nvs, "wifi_pass");
    }
    // the same bounds Connect enforces before saving (the driver fields are
    // 32 / 64 bytes); anything else in flash is treated as no credentials
    if (ssid.size() > 32 || pass.size() > 63) {
      ssid.clear();
      pass.clear();
    }
    // The app's NVS keys are the only persisted credentials: keep the
    // driver's own copy in RAM so esp_wifi_set_config() does not write a
    // second one to flash (which Forget could not clear).
    auto &stack = espp::Wifi::get();
    if (stack.init())
      stack.set_storage(WIFI_STORAGE_RAM);
    auto cfg = wifi_config(ssid, pass);
    cfg.auto_connect = !ssid.empty();
    set_status(ssid.empty() ? "idle (no saved network)" : "connecting");
    wifi = std::make_unique<espp::WifiSta>(cfg);
    wifi_mac = wifi->get_mac();
  }

  /// Drop the saved credentials everywhere: the app's NVS keys, the driver's
  /// station config and WifiSta's stored config (reconfigured with empty
  /// credentials and auto-connect off, so nothing reconnects).
  bool forget() {
    std::error_code ec;
    espp::NvsHandle nvs("desktop", ec);
    bool erased = !ec;
    if (erased) {
      // a missing key is fine (nothing was saved); any other failure is not
      std::error_code e1, e2;
      nvs.erase("wifi_ssid", e1);
      nvs.erase("wifi_pass", e2);
      nvs.commit(ec); // erase_item() only stages the change
      erased = !ec;
    }
    wifi->disconnect();
    wifi_config_t empty{};
    const bool cleared = esp_wifi_set_config(WIFI_IF_STA, &empty) == ESP_OK;
    auto cfg =
        wifi_config("", ""); // empty ssid: reconfigure() adopts the (now empty) driver config
    cfg.auto_connect = false;
    const bool ok = wifi->reconfigure(cfg) && cleared && erased;
    set_status("idle (no saved network)");
    return ok;
  }

  /// Save the credentials (committed, so they survive a reboot).
  bool save_credentials(const std::string &ssid, const std::string &pass) {
    std::error_code ec;
    espp::NvsHandle nvs("desktop", ec);
    if (ec)
      return false;
    nvs.set("wifi_ssid", ssid, ec);
    if (!ec)
      nvs.set("wifi_pass", pass, ec);
    if (!ec)
      nvs.commit(ec);
    return !ec;
  }
#endif

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
  /// Bring the RMII link up once (ESP32-Ethernet-Kit pins: IP101 PHY, 50 MHz
  /// reference clock in on GPIO0; adjust for your board).
  void ensure_ethernet() {
    if (eth)
      return;
    eth = std::make_unique<espp::Ethernet>(
        espp::Ethernet::Config{.interface = espp::Ethernet::RmiiConfig{.mdc_gpio = 23,
                                                                       .mdio_gpio = 18,
                                                                       .phy_addr = 1,
                                                                       .phy_reset_gpio = 5,
                                                                       .clock_ext_in = true,
                                                                       .clock_gpio = 0},
                               .hostname = "espp-desktop",
                               .log_level = espp::Logger::Verbosity::INFO});
    std::error_code ec;
    eth->initialize(ec);
  }
#endif
};

} // namespace desktop_example

inline void register_network_app(espp::Desktop &desktop) {
  auto net = std::make_shared<desktop_example::NetworkState>();
  desktop.register_app({
      .name = "Network",
      .icon = "\xF0\x9F\x93\xA1", // satellite antenna
      .description = "Wi-Fi station and Ethernet status",
      .launch =
          [net](espp::Desktop &d, espp::Desktop::AppId app) {
            using namespace std::chrono_literals;
            using D = espp::Desktop;
            auto win = d.create_window({.title = "Network", .app = app, .w = 520, .h = 0});
            std::vector<std::function<void()>> refreshers;

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
            net->ensure_wifi();
            auto wifi = win.group("Wi-Fi station");
            auto status = win.label("Status: -", wifi.id(), D::kLabelBold);
            auto ssid_label = win.label("SSID: -", wifi.id(), D::kLabelMonospace);
            auto ip_label = win.label("IP: -", wifi.id(), D::kLabelMonospace);
            auto rssi_label = win.label("RSSI: -", wifi.id(), D::kLabelMonospace);
            win.label(fmt::format("MAC: {}", net->wifi_mac), wifi.id(), D::kLabelMonospace);
            auto scan_bar = win.row(wifi.id());
            auto scan_btn = win.button("Scan", nullptr, scan_bar.id());
            auto scan_label = win.label("", scan_bar.id());
            std::vector<std::string> rows_now;
            {
              std::lock_guard<std::mutex> lock(net->mutex);
              rows_now = net->scan_rows; // the last scan, if any
            }
            auto ap_list = win.list(rows_now, nullptr, wifi.id());
            ap_list.set_size(0, 140);
            auto pass_row = win.row(wifi.id());
            win.label("Password", pass_row.id());
            auto pass_box = win.textbox("", nullptr, pass_row.id(),
                                        "leave empty for an open network", D::kTextBoxPassword);
            auto actions = win.row(wifi.id());
            auto connect_btn = win.button("Connect", nullptr, actions.id(), D::kButtonPrimary);
            auto disconnect_btn = win.button("Disconnect", nullptr, actions.id());
            auto forget_btn = win.button("Forget", nullptr, actions.id(), D::kButtonDanger);
            // every station action is disabled while a scan drives the
            // station from the scan task (and refused if clicked anyway)
            auto set_busy = [=](bool busy) mutable {
              scan_btn.set_enabled(!busy);
              connect_btn.set_enabled(!busy);
              disconnect_btn.set_enabled(!busy);
              forget_btn.set_enabled(!busy);
            };
            auto refuse_if_scanning = [=, &d]() {
              if (!net->scanning)
                return false;
              d.notify({.title = "Wi-Fi",
                        .text = "a scan is in progress; try again when it is done",
                        .level = D::NotifyLevel::Warn});
              return true;
            };
            set_busy(net->scanning);

            refreshers.push_back([=]() mutable {
              std::string st, ip;
              {
                std::lock_guard<std::mutex> lock(net->mutex);
                st = net->wifi_status;
                ip = net->wifi_ip;
              }
              // also re-syncs a window opened while a scan started elsewhere
              set_busy(net->scanning);
              const bool connected = net->wifi->is_connected();
              status.set_text("Status: {}", st);
              ssid_label.set_text("SSID: {}", connected ? net->wifi->get_ssid() : "-");
              ip_label.set_text("IP: {}", ip.empty() ? "-" : ip);
              if (connected)
                rssi_label.set_text("RSSI: {} dBm", net->wifi->get_rssi());
              else
                rssi_label.set_text("RSSI: -");
            });

            scan_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click || refuse_if_scanning())
                return;
              net->scanning = true;
              net->scan_task.reset(); // the previous (finished) scan
              set_busy(true);
              scan_label.set_text("scanning\xE2\x80\xA6 (disconnects first)");
              auto *state = net.get(); // outlives the task (joined by its destructor)
              net->scan_task = std::make_unique<espp::Task>(espp::Task::Config{
                  .callback =
                      [=]() mutable {
                        // WifiSta::scan() drops the link with a bare
                        // esp_wifi_disconnect(), which WifiSta's DISCONNECTED
                        // handler treats as unintentional and answers with a
                        // reconnect attempt (num_connect_retries > 0) while the
                        // scan is starting. Disconnect intentionally first (that
                        // suppresses the retries), give the event time to land,
                        // then scan and reconnect ourselves afterwards.
                        const bool was_connected = state->wifi->is_connected();
                        if (was_connected) {
                          state->wifi->disconnect();
                          std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        }
                        state->set_status("scanning");
                        const auto aps = state->wifi->scan(20);
                        if (was_connected) {
                          state->set_status("reconnecting");
                          state->wifi->connect();
                        } else {
                          state->set_status("disconnected");
                        }
                        std::vector<std::string> rows, ssids;
                        for (const auto &ap : aps) {
                          const std::string ssid(reinterpret_cast<const char *>(ap.ssid));
                          ssids.push_back(ssid);
                          rows.push_back(fmt::format(
                              "{}  ({} dBm, {}, ch {})", ssid.empty() ? "<hidden>" : ssid,
                              static_cast<int>(ap.rssi), state->auth_name(ap.authmode),
                              static_cast<int>(ap.primary)));
                        }
                        {
                          std::lock_guard<std::mutex> lock(state->mutex);
                          state->scan_ssids = ssids;
                          state->scan_rows = rows;
                        }
                        ap_list.set_items(rows);
                        scan_label.set_text("{} networks", rows.size());
                        state->scanning = false;
                        set_busy(false);
                        return true; // one shot
                      },
                  .task_config = {.name = "wifi_scan", .stack_size_bytes = 6 * 1024}});
              net->scan_task->start();
            });

            connect_btn.on_event([=, &d](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click || refuse_if_scanning())
                return;
              std::string ssid;
              {
                std::lock_guard<std::mutex> lock(net->mutex);
                const int32_t i = ap_list.selected();
                if (i >= 0 && static_cast<size_t>(i) < net->scan_ssids.size())
                  ssid = net->scan_ssids[static_cast<size_t>(i)];
              }
              if (ssid.empty()) {
                d.notify({.title = "Wi-Fi",
                          .text = "scan, then select a network",
                          .level = D::NotifyLevel::Warn});
                return;
              }
              const std::string pass = pass_box.text();
              // the driver fields are 32 / 64 bytes (WifiSta::reconfigure
              // copies the strings as given); the text boxes are unbounded
              if (ssid.size() > 32 || pass.size() > 63) {
                d.notify({.title = "Wi-Fi",
                          .text = "SSID must be at most 32 bytes and the password at "
                                  "most 63 bytes",
                          .level = D::NotifyLevel::Error});
                return;
              }
              if (!net->save_credentials(ssid, pass))
                d.notify({.title = "Wi-Fi",
                          .text = "could not save the credentials to NVS (connecting anyway)",
                          .level = D::NotifyLevel::Warn});
              net->set_status(fmt::format("connecting to {}", ssid));
              const bool was_connected = net->wifi->is_connected();
              // reconfigure() reconnects when it was connected; else connect()
              bool ok = net->wifi->reconfigure(net->wifi_config(ssid, pass));
              if (ok && !was_connected)
                ok = net->wifi->connect();
              if (!ok) {
                net->set_status("connect failed");
                d.notify({.title = "Wi-Fi",
                          .text = fmt::format("could not start connecting to {}", ssid),
                          .level = D::NotifyLevel::Error});
              }
            });
            disconnect_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click || refuse_if_scanning())
                return;
              net->wifi->disconnect();
              net->set_status("disconnected");
            });
            forget_btn.on_event([=, &d](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click || refuse_if_scanning())
                return;
              const bool ok = net->forget();
              d.notify({.title = "Wi-Fi",
                        .text = ok ? "saved network forgotten"
                                   : "credentials erased, but the station could not be reset",
                        .level = ok ? D::NotifyLevel::Ok : D::NotifyLevel::Warn});
            });
#endif

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
            net->ensure_ethernet();
            auto eth = win.group("Ethernet (RMII)");
            auto link = win.label("Link: -", eth.id(), D::kLabelBold);
            auto eth_ip = win.label("IP: -", eth.id(), D::kLabelMonospace);
            win.label(fmt::format("MAC: {}", net->eth->get_mac_address()), eth.id(),
                      D::kLabelMonospace);
            auto speed = win.label("Speed: -", eth.id(), D::kLabelMonospace);
            refreshers.push_back([=]() mutable {
              const bool up = net->eth->link_up();
              link.set_text("Link: {}", !net->eth->is_initialized() ? "not initialized"
                                        : up                        ? "up"
                                                                    : "down");
              eth_ip.set_text("IP: {}",
                              net->eth->is_connected() ? net->eth->get_ip_address() : "-");
              if (const auto sd = net->eth->link_speed_duplex(); sd)
                speed.set_text("Speed: {} Mbit/s {}-duplex", sd->first,
                               sd->second ? "full" : "half");
              else
                speed.set_text("Speed: -");
            });
#endif

            // (the app is only registered when at least one group is enabled)
            auto refresh = [refreshers]() {
              for (const auto &fn : refreshers)
                fn();
            };
            refresh();
            win.add_timer(1s, refresh);
          },
  });
}
