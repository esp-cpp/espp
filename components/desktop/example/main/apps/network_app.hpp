#pragma once

// Network: the Wi-Fi station (espp::WifiSta -- status, scan, connect /
// disconnect with the credentials kept in NVS) and, under its own Kconfig
// gate, the RMII Ethernet link (espp::Ethernet). The interfaces are brought
// up on the first launch and kept across windows (closing the window does not
// disconnect); the window's 1 s timer renders the state the driver callbacks
// record. Every station transition (scan, connect, disconnect, forget) runs
// on one worker task as a small explicit state machine (Idle -> Stopping ->
// Configuring -> Associating -> Connected), never on the desktop task, with
// the controls disabled until it is done.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "sdkconfig.h"

#include "nvs.h"

#include "desktop.hpp"
#include "logger.hpp"
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
  using D = espp::Desktop;

  explicit NetworkState(D &desktop)
      : desktop(desktop) {}

  D &desktop; ///< outlives every task (toasts from the worker)
  espp::Logger logger{{.tag = "Network app", .level = espp::Logger::Verbosity::INFO}};
  std::mutex mutex;
  std::string wifi_status{"idle"};
  std::string wifi_ip;
  std::string wifi_mac;
  std::vector<std::string> scan_ssids; // in AP list order
  std::vector<std::string> scan_rows;  // the AP list's items (shown again by a new window)
  uint32_t scan_generation{0};         // bumped per completed scan (windows re-sync on change)

  /// The station state machine, driven by the jobs below on the worker task
  /// and by the driver callbacks (got-ip -> Connected, retries exhausted ->
  /// Idle). `Stopping` is the explicit wait for the DISCONNECTED event.
  enum class Phase : uint8_t { Idle, Stopping, Configuring, Associating, Connected };
  std::atomic<Phase> phase{Phase::Idle};
  /// A job (scan / connect / disconnect / forget) is running on the worker:
  /// every station control is disabled, and another job is refused.
  std::atomic<bool> busy{false};
  /// DESIRED connectivity: set by Connect and by the saved-credentials
  /// auto-connect at start, cleared only by Disconnect / Forget -- never by
  /// the driver callbacks. A scan stops the station and restores it afterwards
  /// when this is set, whatever the transient phase.
  std::atomic<bool> want_connected{false};
  /// Connection-attempt GENERATION, a belt-and-braces check only: the driver
  /// events are asynchronous and WifiSta fetches `config_.on_disconnected`
  /// at DELIVERY time, so the installed callback cannot tell which attempt an
  /// event belongs to. Ownership is therefore never inferred from the
  /// callback: before a new attempt the worker actually WAITS for the
  /// DISCONNECTED event of the stopped station (stop_and_wait, bounded). The
  /// generation only filters the residue (an event that still arrives after
  /// the wait timed out).
  std::atomic<uint32_t> attempt{0};
  std::condition_variable disconnected_cv; // with `mutex`: a DISCONNECTED event arrived
  uint32_t disconnected_events{0};         // under `mutex`; any generation
  std::string cur_ssid, cur_pass;          // the credentials of the current attempt (under `mutex`)
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
  std::unique_ptr<espp::WifiSta> wifi;
  std::unique_ptr<espp::Task> worker; // after wifi: joined before it goes away
#endif
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
  std::unique_ptr<espp::Ethernet> eth;
#endif

  void set_status(std::string_view status, std::string_view ip = "") {
    std::lock_guard<std::mutex> lock(mutex);
    wifi_status = std::string(status);
    wifi_ip = std::string(ip);
  }

  void toast(std::string text, D::NotifyLevel level) {
    desktop.notify({.title = "Wi-Fi", .text = std::move(text), .level = level});
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

  /// The station config for `ssid` / `password` of attempt `gen`: the
  /// callbacks drive the phase (they run on the event-loop task) and ignore
  /// the residue of an older generation (see `attempt`).
  espp::WifiSta::Config wifi_config(std::string ssid, std::string password, uint32_t gen) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      cur_ssid = ssid;
      cur_pass = password;
    }
    return {.ssid = std::move(ssid),
            .password = std::move(password),
            .num_connect_retries = 3,
            .auto_connect = true,
            .on_connected =
                [this, gen]() {
                  if (gen == attempt)
                    set_status("connected, waiting for an IP");
                },
            .on_disconnected =
                [this, gen]() {
                  // every DISCONNECTED (intentional or retries exhausted) wakes
                  // a stop_and_wait() in progress, whatever its generation
                  {
                    std::lock_guard<std::mutex> lock(mutex);
                    ++disconnected_events;
                  }
                  disconnected_cv.notify_all();
                  if (gen != attempt)
                    return; // residue of a previous attempt
                  if (phase == Phase::Stopping)
                    return;            // the worker owns this transition
                  phase = Phase::Idle; // retries exhausted
                  set_status(want_connected ? "disconnected (retries exhausted; Connect or a "
                                              "scan retries)"
                                            : "disconnected");
                },
            .on_got_ip =
                [this, gen](ip_event_got_ip_t *e) {
                  if (gen != attempt)
                    return;
                  phase = Phase::Connected;
                  set_status("connected", fmt::format("{}.{}.{}.{}", IP2STR(&e->ip_info.ip)));
                },
            .log_level = espp::Logger::Verbosity::INFO};
  }

  /// Start a new attempt generation: the residue of the previous one is
  /// ignored from here on.
  uint32_t new_attempt() { return ++attempt; }

  /// Whether a string key exists in the app's namespace: ESP_OK (it does),
  /// ESP_ERR_NVS_NOT_FOUND (it does not) or another error (a genuine NVS
  /// failure). NvsHandle maps every string read failure to one error code,
  /// so the raw nvs_get_str() size query is used to tell the two apart.
  static esp_err_t probe_key(const char *key) {
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open("desktop", NVS_READONLY, &h);
    if (err != ESP_OK)
      return err; // ESP_ERR_NVS_NOT_FOUND when the namespace was never written
    size_t len = 0;
    err = nvs_get_str(h, key, nullptr, &len);
    nvs_close(h);
    return err;
  }

  /// A string from NVS: "" when the key is not stored, nullopt on a genuine
  /// NVS failure. NvsHandle::get() sizes the string to the stored length
  /// INCLUDING the terminating NUL, so trailing NULs are stripped (a 32-byte
  /// SSID would otherwise come back as 33 bytes).
  static std::optional<std::string> nvs_string(espp::NvsHandle &nvs, const char *key) {
    const esp_err_t probed = probe_key(key);
    if (probed == ESP_ERR_NVS_NOT_FOUND)
      return std::string{};
    if (probed != ESP_OK)
      return std::nullopt;
    std::error_code ec;
    std::string s;
    nvs.get(key, s, ec); // no default: never writes
    if (ec)
      return std::nullopt;
    while (!s.empty() && s.back() == '\0')
      s.pop_back();
    return s;
  }

  /// Erase one key; true when it is gone (it was erased, or was never
  /// stored), false on a genuine NVS failure (probe or erase).
  static bool erase_key(espp::NvsHandle &nvs, const char *key) {
    const esp_err_t probed = probe_key(key);
    if (probed == ESP_ERR_NVS_NOT_FOUND)
      return true;
    if (probed != ESP_OK)
      return false;
    std::error_code ec;
    return nvs.erase(key, ec);
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

  /// Bring the station up once, with the saved credentials (none = idle).
  /// Desktop task, first launch.
  void ensure_wifi() {
    if (wifi)
      return;
    std::error_code ec;
    espp::NvsHandle nvs("desktop", ec);
    std::string ssid, pass;
    if (!ec) {
      const auto saved_ssid = nvs_string(nvs, "wifi_ssid");
      const auto saved_pass = nvs_string(nvs, "wifi_pass");
      if (saved_ssid && saved_pass) {
        ssid = *saved_ssid;
        pass = *saved_pass;
      } else {
        // a genuine NVS failure (not "nothing saved"): start idle, say why
        logger.error("could not read the saved Wi-Fi credentials from NVS; starting idle");
      }
    } else {
      logger.error("could not open the NVS namespace: {}", ec.message());
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
    auto cfg = wifi_config(ssid, pass, new_attempt());
    cfg.auto_connect = !ssid.empty();
    want_connected = cfg.auto_connect;
    // WifiSta connects from its STA_START event: that is an association
    phase = cfg.auto_connect ? Phase::Associating : Phase::Idle;
    set_status(ssid.empty() ? "idle (no saved network)" : "connecting");
    wifi = std::make_unique<espp::WifiSta>(cfg);
    wifi_mac = wifi->get_mac();
  }

  // ---- the worker: one job at a time, every station control disabled ----

  /// Run `job` on the worker task; refused (with a toast) while another job
  /// runs or when the task cannot be started. Desktop task.
  bool run_job(const char *name, std::function<void()> job) {
    if (busy.exchange(true)) {
      toast("the station is busy; try again when it is done", D::NotifyLevel::Warn);
      return false;
    }
    worker.reset(); // the previous (finished) job
    worker = std::make_unique<espp::Task>(
        espp::Task::Config{.callback =
                               [this, job = std::move(job)]() {
                                 job();
                                 busy = false;
                                 return true; // one shot
                               },
                           .task_config = {.name = name, .stack_size_bytes = 6 * 1024}});
    if (!worker->start()) {
      worker.reset();
      busy = false;
      toast("could not start the Wi-Fi worker task (out of memory?)", D::NotifyLevel::Error);
      return false;
    }
    return true;
  }

  /// Stopping: if the station is connected or associating, disconnect
  /// intentionally (that suppresses WifiSta's retries) and WAIT for its
  /// DISCONNECTED event (bounded), so a later attempt can never be confused
  /// with the stopped one. On an idle station WifiSta::disconnect() is not
  /// called at all: no event would come and its private `disconnecting_`
  /// would stay set, costing the next attempt its retries. Worker task.
  /// Returns true once the station is idle.
  bool stop_and_wait(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
    const Phase p = phase.load();
    if (p != Phase::Connected && p != Phase::Associating)
      return true;
    phase = Phase::Stopping;
    set_status("stopping");
    uint32_t seen = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      seen = disconnected_events;
    }
    if (!wifi->disconnect()) {
      // the driver rejected it (nothing was associated): no event will come
      phase = Phase::Idle;
      return true;
    }
    bool got = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      got = disconnected_cv.wait_for(lock, timeout, [&] { return disconnected_events != seen; });
    }
    if (!got)
      logger.warn("no DISCONNECTED event within {} ms; continuing (the generation check drops "
                  "it if it still arrives)",
                  timeout.count());
    phase = Phase::Idle;
    return got;
  }

  /// Configuring -> Associating: a new attempt with `ssid` / `pass` (the
  /// station must be idle: call stop_and_wait() first). Worker task.
  bool start_attempt(const std::string &ssid, const std::string &pass, const char *what) {
    phase = Phase::Configuring;
    set_status(fmt::format("configuring {}", ssid));
    const uint32_t gen = new_attempt();
    // after stop_and_wait() WifiSta's connected_ is false, so reconfigure()
    // does not connect by itself: connect() explicitly
    const bool ok = wifi->reconfigure(wifi_config(ssid, pass, gen)) && wifi->connect();
    if (!ok) {
      phase = Phase::Idle;
      set_status(fmt::format("{} failed: use Connect", what));
      toast(fmt::format("could not start the {} to {}", what, ssid), D::NotifyLevel::Error);
      return false;
    }
    phase = Phase::Associating;
    set_status(fmt::format("connecting to {}", ssid));
    return true;
  }

  /// Connect: stop whatever is active (waiting for its event), then attempt.
  void connect_job(const std::string &ssid, const std::string &pass) {
    want_connected = true;
    stop_and_wait();
    start_attempt(ssid, pass, "connection");
  }

  /// Disconnect: give up the desired state and stop the station.
  void disconnect_job() {
    want_connected = false;
    stop_and_wait();
    set_status("disconnected");
  }

  /// Scan: stop the station first (WifiSta::scan() would otherwise drop the
  /// link with a bare esp_wifi_disconnect() that its DISCONNECTED handler
  /// answers with a retry while the scan starts), scan, publish the rows for
  /// every window, then restore the desired state with a fresh attempt.
  void scan_job() {
    const bool intent = want_connected;
    stop_and_wait();
    set_status("scanning");
    const auto aps = wifi->scan(20);
    std::vector<std::string> rows, ssids;
    for (const auto &ap : aps) {
      const std::string ssid(reinterpret_cast<const char *>(ap.ssid));
      ssids.push_back(ssid);
      rows.push_back(fmt::format("{}  ({} dBm, {}, ch {})", ssid.empty() ? "<hidden>" : ssid,
                                 static_cast<int>(ap.rssi), auth_name(ap.authmode),
                                 static_cast<int>(ap.primary)));
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      scan_ssids = ssids;
      scan_rows = rows;
      ++scan_generation;
    }
    if (intent) {
      std::string ssid, pass;
      {
        std::lock_guard<std::mutex> lock(mutex);
        ssid = cur_ssid;
        pass = cur_pass;
      }
      start_attempt(ssid, pass, "reconnect after the scan");
    } else {
      set_status("disconnected");
    }
  }

  /// Forget: drop the saved credentials everywhere -- the app's NVS keys,
  /// the driver's station config and WifiSta's stored config (reconfigured
  /// with empty credentials and auto-connect off, so nothing reconnects).
  void forget_job() {
    std::error_code ec;
    espp::NvsHandle nvs("desktop", ec);
    bool erased = !ec;
    if (erased) {
      // both erasures must succeed (a key that was never stored counts as
      // already forgotten), and the staged change must be committed
      erased = erase_key(nvs, "wifi_ssid");
      erased = erase_key(nvs, "wifi_pass") && erased;
      nvs.commit(ec); // erase_item() only stages the change
      erased = erased && !ec;
    }
    want_connected = false;
    stop_and_wait();
    wifi_config_t empty{};
    const bool cleared = esp_wifi_set_config(WIFI_IF_STA, &empty) == ESP_OK;
    // empty ssid: reconfigure() adopts the (now empty) driver config
    auto cfg = wifi_config("", "", new_attempt());
    cfg.auto_connect = false;
    const bool station_reset = wifi->reconfigure(cfg) && cleared;
    phase = Phase::Idle;
    // the persistent status must not claim the credentials are gone when the
    // NVS erase / commit failed
    set_status(erased ? "idle (no saved network)"
                      : "Forget failed: the credentials may still be saved");
    if (!erased)
      toast("Forget failed: the credentials may still be saved in NVS", D::NotifyLevel::Error);
    else if (!station_reset)
      toast("credentials erased, but the station could not be reset", D::NotifyLevel::Warn);
    else
      toast("saved network forgotten", D::NotifyLevel::Ok);
  }
#endif

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
  /// The RMII wiring of the board Kconfig selects (IP101 PHY at address 1,
  /// 50 MHz reference clock in). The ESP32's EMAC data pins are fixed by the
  /// IO_MUX; the ESP32-P4's are routable and must be given (the pins of the
  /// ESP32-P4-Function-EV-Board, as in components/esp32-p4-function-ev-board).
  static espp::Ethernet::RmiiConfig rmii_config() {
#if CONFIG_DESKTOP_EXAMPLE_ETHERNET_BOARD_P4_FUNCTION_EV
    return {.mdc_gpio = 31,
            .mdio_gpio = 52,
            .phy_addr = 1,
            .phy_reset_gpio = 51,
            .clock_ext_in = true,
            .clock_gpio = 50,
            .data_pins = espp::Ethernet::RmiiConfig::DataPins{
                .tx_en = 49, .txd0 = 34, .txd1 = 35, .crs_dv = 28, .rxd0 = 29, .rxd1 = 30}};
#else // ESP32-Ethernet-Kit
    return {.mdc_gpio = 23,
            .mdio_gpio = 18,
            .phy_addr = 1,
            .phy_reset_gpio = 5,
            .clock_ext_in = true,
            .clock_gpio = 0};
#endif
  }

  /// Bring the RMII link up once.
  void ensure_ethernet() {
    if (eth)
      return;
    eth = std::make_unique<espp::Ethernet>(
        espp::Ethernet::Config{.interface = rmii_config(),
                               .hostname = "espp-desktop",
                               .log_level = espp::Logger::Verbosity::INFO});
    std::error_code ec;
    eth->initialize(ec);
  }
#endif
};

} // namespace desktop_example

inline void register_network_app(espp::Desktop &desktop) {
  auto net = std::make_shared<desktop_example::NetworkState>(desktop);
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

    // Ethernet FIRST: espp::Ethernet::initialize() treats
    // esp_netif_init()'s ESP_ERR_INVALID_STATE (already initialized by
    // the Wi-Fi stack) as fatal, while Wifi::init() tolerates it.
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
            net->ensure_ethernet();
#endif
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
            net->ensure_wifi();
            auto *state = net.get(); // outlives every window and task
            auto wifi = win.group("Wi-Fi station");
            auto status = win.label("Status: -", wifi.id(), D::kLabelBold);
            auto ssid_label = win.label("SSID: -", wifi.id(), D::kLabelMonospace);
            auto ip_label = win.label("IP: -", wifi.id(), D::kLabelMonospace);
            auto rssi_label = win.label("RSSI: -", wifi.id(), D::kLabelMonospace);
            win.label(fmt::format("MAC: {}", net->wifi_mac), wifi.id(), D::kLabelMonospace);
            auto scan_bar = win.row(wifi.id());
            auto scan_btn = win.button("Scan", nullptr, scan_bar.id());
            auto scan_label = win.label("", scan_bar.id());
            // THIS window's AP list and the SSIDs behind its rows: snapshotted
            // together whenever the rows are replaced, so a selection always
            // maps to the SSID of the row the user sees (another window's scan
            // may replace the shared rows at any time)
            auto ssids = std::make_shared<std::vector<std::string>>();
            auto seen_generation = std::make_shared<uint32_t>(0);
            std::vector<std::string> rows_now;
            {
              std::lock_guard<std::mutex> lock(net->mutex);
              rows_now = net->scan_rows; // the last scan, if any
              *ssids = net->scan_ssids;
              *seen_generation = net->scan_generation;
            }
            auto ap_list = win.list(rows_now, nullptr, wifi.id());
            ap_list.set_size(0, 140);
            if (!rows_now.empty())
              scan_label.set_text("{} networks", rows_now.size());
            auto pass_row = win.row(wifi.id());
            win.label("Password", pass_row.id());
            auto pass_box = win.textbox("", nullptr, pass_row.id(),
                                        "leave empty for an open network", D::kTextBoxPassword);
            auto actions = win.row(wifi.id());
            auto connect_btn = win.button("Connect", nullptr, actions.id(), D::kButtonPrimary);
            auto disconnect_btn = win.button("Disconnect", nullptr, actions.id());
            auto forget_btn = win.button("Forget", nullptr, actions.id(), D::kButtonDanger);
            // every station control is disabled while a job runs on the
            // worker (and a click is refused by run_job() anyway)
            auto set_busy = [=](bool busy) mutable {
              scan_btn.set_enabled(!busy);
              connect_btn.set_enabled(!busy);
              disconnect_btn.set_enabled(!busy);
              forget_btn.set_enabled(!busy);
            };
            set_busy(net->busy);

            refreshers.push_back([=]() mutable {
              std::string st, ip;
              std::optional<std::vector<std::string>> new_rows;
              {
                std::lock_guard<std::mutex> lock(net->mutex);
                st = net->wifi_status;
                ip = net->wifi_ip;
                // a scan completed since this window last synced (from any
                // window): replace the rows AND the SSIDs behind them together,
                // and drop the selection, which pointed at the old rows
                if (net->scan_generation != *seen_generation) {
                  *seen_generation = net->scan_generation;
                  new_rows = net->scan_rows;
                  *ssids = net->scan_ssids;
                }
              }
              if (new_rows) {
                ap_list.set_items(*new_rows);
                ap_list.set_selected(D::kNoSelection);
                scan_label.set_text("{} networks", new_rows->size());
              }
              // also re-syncs a window opened while a job started elsewhere
              set_busy(net->busy);
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
              if (e.kind != D::WidgetEventKind::Click)
                return;
              if (net->run_job("wifi_scan", [state]() { state->scan_job(); })) {
                set_busy(true);
                scan_label.set_text("scanning\xE2\x80\xA6 (stops the station first)");
              }
            });

            connect_btn.on_event([=, &d](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              std::string ssid;
              const int32_t i = ap_list.selected();
              if (i >= 0 && static_cast<size_t>(i) < ssids->size())
                ssid = (*ssids)[static_cast<size_t>(i)];
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
              if (net->run_job("wifi_connect",
                               [state, ssid, pass]() { state->connect_job(ssid, pass); }))
                set_busy(true);
            });
            disconnect_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              if (net->run_job("wifi_disconnect", [state]() { state->disconnect_job(); }))
                set_busy(true);
            });
            forget_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              if (net->run_job("wifi_forget", [state]() { state->forget_job(); }))
                set_busy(true);
            });
#endif

#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
            // (the link was brought up above, before the Wi-Fi stack)
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
