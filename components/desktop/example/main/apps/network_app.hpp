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
#include <set>
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
  NetworkState(const NetworkState &) = delete;
  NetworkState &operator=(const NetworkState &) = delete;

  /// In practice never reached: the state is owned by the app's launch
  /// callback and lives as long as the desktop. Still correct: the worker is
  /// joined first (nothing drives the station any more), the STA_STOP
  /// handler is withdrawn (registry entry, then the esp_event registration)
  /// before the station is destroyed -- ~WifiSta stops the station, which
  /// queues a STA_STOP that must not reach a freed `this` -- and the handler
  /// itself only touches instances still in the registry.
  ~NetworkState() {
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
    worker.reset();
    {
      std::lock_guard<std::mutex> lock(registry_mutex());
      live_instances().erase(this);
    }
    if (sta_stop_handler) {
      esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_STOP, sta_stop_handler);
      sta_stop_handler = nullptr;
    }
    wifi.reset();
#endif
  }

  /// The live instances a WIFI_EVENT_STA_STOP delivery may touch (a late
  /// delivery after an instance withdrew is dropped, whatever the event
  /// loop's unregister ordering).
  static std::mutex &registry_mutex() {
    static std::mutex m;
    return m;
  }
  static std::set<NetworkState *> &live_instances() {
    static std::set<NetworkState *> s;
    return s;
  }

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
  /// `Recovering` is the deterministic stop / start of the whole station
  /// (recover()) after a DISCONNECTED wait timed out. `Failed` is the
  /// recoverable error state a transition aborts into when the station could
  /// not be stopped (the disconnect was rejected, or the recovery itself
  /// failed): nothing is reconfigured, the station is left as it is, the user
  /// retries. A later event resolves it (the callbacks still own the phase),
  /// and the next job starts with a fresh stop_and_wait().
  enum class Phase : uint8_t {
    Idle,
    Stopping,
    Recovering,
    Configuring,
    Associating,
    Connected,
    Failed
  };
  std::atomic<Phase> phase{Phase::Idle};
  /// A job (scan / connect / disconnect / forget) is running on the worker:
  /// every station control is disabled, and another job is refused.
  std::atomic<bool> busy{false};
  /// DESIRED connectivity: set by Connect and by the saved-credentials
  /// auto-connect at start, cleared only by Disconnect / Forget -- never by
  /// the driver callbacks. A scan stops the station and restores it afterwards
  /// when this is set, whatever the transient phase.
  std::atomic<bool> want_connected{false};
  /// Correlating the asynchronous DISCONNECTED events with the worker's
  /// requests. WifiSta fetches its callbacks at DELIVERY time, so "which
  /// callback is installed" can never identify an event: the three callbacks
  /// are therefore installed ONCE (ensure_wifi) and never replaced -- every
  /// reconfigure() passes the same long-lived std::function objects -- and
  /// they consult this state under `mutex`. `disconnected_events` counts the
  /// deliveries stop_and_wait() waits on. When that wait times out the
  /// station is RECOVERED deterministically (recover(): esp_wifi_stop(),
  /// wait for WIFI_EVENT_STA_STOP, esp_wifi_start()) -- the stop tears down
  /// any in-flight association, every event of the old session is delivered
  /// before STA_STOP, and nothing trails after it -- so the worker never
  /// resumes with an unconfirmed disconnect outstanding and no credit /
  /// drain bookkeeping is needed.
  std::condition_variable disconnected_cv; // with `mutex`: a DISCONNECTED event arrived
  uint32_t disconnected_events{0};         // under `mutex`
  std::condition_variable stopped_cv;      // with `mutex`: WIFI_EVENT_STA_STOP arrived
  uint32_t stopped_events{0};              // under `mutex`
  std::string cur_ssid, cur_pass;          // the credentials of the current attempt (under `mutex`)
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_WIFI
  std::unique_ptr<espp::WifiSta> wifi;
  std::unique_ptr<espp::Task> worker;                     // after wifi: joined before it goes away
  esp_event_handler_instance_t sta_stop_handler{nullptr}; // WIFI_EVENT_STA_STOP -> stopped_cv
  // the long-lived callbacks (see above); they run on the event-loop task
  espp::WifiSta::connect_callback on_connected_fn = [this]() {
    set_status("connected, waiting for an IP");
  };
  espp::WifiSta::disconnect_callback on_disconnected_fn = [this]() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++disconnected_events; // a confirmation stop_and_wait() may be waiting on
    }
    disconnected_cv.notify_all();
    const Phase p = phase.load();
    if (p == Phase::Stopping || p == Phase::Recovering)
      return;            // the worker owns this transition
    phase = Phase::Idle; // retries exhausted
    set_status(want_connected ? "disconnected (retries exhausted; Connect or a scan retries)"
                              : "disconnected");
  };
  espp::WifiSta::ip_callback on_got_ip_fn = [this](ip_event_got_ip_t *e) {
    const Phase p = phase.load();
    if (p == Phase::Stopping || p == Phase::Recovering)
      return; // being stopped: the DISCONNECTED / STA_STOP that follows settles it
    phase = Phase::Connected;
    set_status("connected", fmt::format("{}.{}.{}.{}", IP2STR(&e->ip_info.ip)));
  };
  /// WIFI_EVENT_STA_STOP (event-loop task): wakes recover(). `arg` is only
  /// dereferenced while it is a live instance (see the registry): a delivery
  /// that outlives its NetworkState is dropped.
  static void on_sta_stop(void *arg, esp_event_base_t, int32_t, void *) {
    auto *self = static_cast<NetworkState *>(arg);
    std::lock_guard<std::mutex> registry(registry_mutex());
    if (!live_instances().count(self))
      return; // stale: the instance withdrew (or is being destroyed)
    {
      std::lock_guard<std::mutex> lock(self->mutex);
      ++self->stopped_events;
    }
    self->stopped_cv.notify_all();
  }
#endif
#if CONFIG_DESKTOP_EXAMPLE_ENABLE_ETHERNET
  std::unique_ptr<espp::Ethernet> eth;
  std::string eth_init_error{}; ///< why initialize() failed (empty = ok / not attempted)
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

  /// The station config for `ssid` / `password`: only the credentials change
  /// between attempts, the callbacks are always the same long-lived objects
  /// (WifiSta::reconfigure() replaces its whole config, so they are passed
  /// again each time).
  espp::WifiSta::Config wifi_config(std::string ssid, std::string password) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      cur_ssid = ssid;
      cur_pass = password;
    }
    return {.ssid = std::move(ssid),
            .password = std::move(password),
            .num_connect_retries = 3,
            // never auto-connect from STA_START: every association is issued
            // explicitly by the worker (and once at start-up), so a recovery's
            // esp_wifi_start() cannot connect to stale credentials by itself
            .auto_connect = false,
            .on_connected = on_connected_fn,
            .on_disconnected = on_disconnected_fn,
            .on_got_ip = on_got_ip_fn,
            .log_level = espp::Logger::Verbosity::INFO};
  }

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
    wifi = std::make_unique<espp::WifiSta>(wifi_config(ssid, pass));
    wifi_mac = wifi->get_mac();
    // recover() waits for the station's STOP event, which WifiSta does not
    // expose: observe it directly
    {
      std::lock_guard<std::mutex> lock(registry_mutex());
      live_instances().insert(this);
    }
    const esp_err_t reg = esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_STOP, &on_sta_stop, this, &sta_stop_handler);
    if (reg != ESP_OK) {
      // recover() then cannot observe STA_STOP (its wait times out -> Failed)
      sta_stop_handler = nullptr;
      logger.error("could not register the STA_STOP handler: {}; station recovery will not work",
                   esp_err_to_name(reg));
    }
    // the start-up association is issued explicitly (auto_connect is off)
    want_connected = !ssid.empty();
    if (ssid.empty()) {
      phase = Phase::Idle;
      set_status("idle (no saved network)");
    } else {
      phase = Phase::Associating;
      set_status(fmt::format("connecting to {}", ssid));
      if (!wifi->connect()) {
        phase = Phase::Idle;
        set_status("connect failed: use Connect");
      }
    }
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
  ///
  /// The rule on failure: a rejected disconnect ABORTS the transition (phase
  /// Failed, a toast, false returned) -- the caller must not reconfigure. A
  /// missing DISCONNECTED (the wait timed out) is never resumed from with the
  /// confirmation outstanding -- it may never come, and resuming would re-open
  /// the stale-event race this wait exists to close: the station is RECOVERED
  /// (recover(): stop the whole station, wait for STA_STOP, start it again),
  /// after which it is known idle and the transition continues; only a failed
  /// recovery aborts into Failed. From Failed, a later event has usually
  /// resolved the phase (Idle / Connected); if none came and the driver
  /// reports no association, the station is taken as idle.
  /// Returns true once the station is idle.
  bool stop_and_wait(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
    Phase p = phase.load();
    if (p == Phase::Failed) {
      wifi_ap_record_t ap{};
      if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        p = Phase::Connected; // still associated: stop it like a connected station
      else
        return true; // nothing associated: idle
    }
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
      // the driver rejected it (WifiSta rolls its flag back; nothing was
      // initiated): a failed transition, not an idle station
      phase = Phase::Failed;
      set_status("stopping failed: try again");
      toast("the station could not be stopped; try again", D::NotifyLevel::Error);
      return false;
    }
    bool got = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      got = disconnected_cv.wait_for(lock, timeout, [&] { return disconnected_events != seen; });
    }
    if (!got) {
      logger.warn("no DISCONNECTED event within {} ms; recovering the station", timeout.count());
      return recover();
    }
    phase = Phase::Idle;
    return true;
  }

  /// Recovering: stop the whole station (esp_wifi_stop() tears down any
  /// in-flight association; every event of the old session -- a DISCONNECTED
  /// included -- is delivered before WIFI_EVENT_STA_STOP and nothing trails
  /// after it), wait for STA_STOP (bounded), then start it again. With
  /// auto_connect off nothing associates by itself, so the station is known
  /// idle afterwards and the interrupted transition continues from there.
  /// espp::Wifi::stop() only serves the registry's active interface (this
  /// station is standalone), hence the driver calls. Worker task.
  bool recover(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    phase = Phase::Recovering;
    set_status("recovering the station");
    uint32_t seen = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      seen = stopped_events;
    }
    const esp_err_t stop_err = esp_wifi_stop();
    bool stopped = stop_err == ESP_OK;
    if (stopped) {
      std::unique_lock<std::mutex> lock(mutex);
      stopped = stopped_cv.wait_for(lock, timeout, [&] { return stopped_events != seen; });
    }
    // start again even after a failed stop (the station may already be
    // stopped); WifiSta::start() is esp_wifi_start() with its logging
    const bool started = wifi->start();
    if (!stopped || !started) {
      logger.error("station recovery failed (stop: {}, STA_STOP: {}, start: {})",
                   esp_err_to_name(stop_err), stopped ? "seen" : "missing", started);
      phase = Phase::Failed;
      set_status("recovery failed: try again");
      toast("the station could not be recovered; try again", D::NotifyLevel::Error);
      return false;
    }
    phase = Phase::Idle;
    return true;
  }

  /// Configuring -> Associating: a new attempt with `ssid` / `pass` (the
  /// station must be idle: call stop_and_wait() first). Worker task.
  bool start_attempt(const std::string &ssid, const std::string &pass, const char *what) {
    phase = Phase::Configuring;
    set_status(fmt::format("configuring {}", ssid));
    // after stop_and_wait() the DISCONNECTED event has cleared WifiSta's
    // connected_, so reconfigure() does not connect by itself: connect()
    // explicitly
    if (!wifi->reconfigure(wifi_config(ssid, pass))) {
      phase = Phase::Idle;
      set_status(fmt::format("{} failed: use Connect", what));
      toast(fmt::format("could not configure the {} to {}", what, ssid), D::NotifyLevel::Error);
      return false;
    }
    // Associating is entered BEFORE connect(): GOT_IP can fire on the
    // event-loop task before connect() returns, and the callbacks own the
    // phase from here on (Connected / Idle). Nothing below writes the phase
    // unconditionally.
    phase = Phase::Associating;
    set_status(fmt::format("connecting to {}", ssid));
    if (!wifi->connect()) {
      // only back to Idle if no callback moved the phase on meanwhile
      Phase expected = Phase::Associating;
      phase.compare_exchange_strong(expected, Phase::Idle);
      set_status(fmt::format("{} failed: use Connect", what));
      toast(fmt::format("could not start the {} to {}", what, ssid), D::NotifyLevel::Error);
      return false;
    }
    return true;
  }

  /// Connect: stop whatever is active (waiting for its event), then attempt.
  void connect_job(const std::string &ssid, const std::string &pass) {
    want_connected = true;
    if (!stop_and_wait())
      return; // aborted (Failed): nothing is reconfigured, the user retries
    start_attempt(ssid, pass, "connection");
  }

  /// Disconnect: give up the desired state and stop the station.
  void disconnect_job() {
    want_connected = false;
    if (stop_and_wait())
      set_status("disconnected");
  }

  /// Scan: stop the station first (WifiSta::scan() would otherwise drop the
  /// link with a bare esp_wifi_disconnect() that its DISCONNECTED handler
  /// answers with a retry while the scan starts), scan, publish the rows for
  /// every window, then restore the desired state with a fresh attempt.
  void scan_job() {
    const bool intent = want_connected;
    if (!stop_and_wait())
      return; // aborted: scan() would drop the link with a bare disconnect
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
    if (!stop_and_wait()) {
      // the credentials are gone (or not: say so), the station is not reset
      toast(erased ? "credentials erased, but the station could not be stopped; Forget again "
                     "to reset it"
                   : "Forget failed: the credentials may still be saved in NVS",
            D::NotifyLevel::Error);
      return;
    }
    wifi_config_t empty{};
    const bool cleared = esp_wifi_set_config(WIFI_IF_STA, &empty) == ESP_OK;
    // empty ssid: reconfigure() adopts the (now empty) driver config
    auto cfg = wifi_config("", "");
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
    if (!eth->initialize(ec)) {
      eth_init_error = ec ? ec.message() : "initialize failed";
      toast("RMII Ethernet init failed: " + eth_init_error +
                " (check the PHY wiring / board choice)",
            D::NotifyLevel::Error);
    }
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
              // the credentials are saved by the ACCEPTED job (after the busy
              // check and a started worker): a refused Connect leaves NVS alone
              if (net->run_job("wifi_connect", [state, ssid, pass]() {
                    if (!state->save_credentials(ssid, pass))
                      state->toast("could not save the credentials to NVS (connecting anyway)",
                                   D::NotifyLevel::Warn);
                    state->connect_job(ssid, pass);
                  }))
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
              if (!net->eth->is_initialized())
                link.set_text("Link: not initialized{}",
                              net->eth_init_error.empty()
                                  ? ""
                                  : " (init failed: " + net->eth_init_error + ")");
              else
                link.set_text("Link: {}", up ? "up" : "down");
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
