#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "NimBLEDevice.h"
#include "ble_gatt_server.hpp"

#include "base_component.hpp"
#include "timer.hpp"

#include "switch2_pro_pairing.hpp"
#include "switch2_pro_protocol.hpp"
#include "switch2_pro_report.hpp"

namespace espp {

/// @brief Emulates a Nintendo Switch 2 Pro Controller as a BLE peripheral.
///
/// The Switch 2 uses a proprietary BLE GATT interface (not HID-over-GATT) with
/// a custom pairing handshake (not BLE SMP). This class stands up that GATT
/// tree on top of espp::BleGattServer, advertises with Nintendo manufacturer
/// data, and answers the console's command channel — including the reverse-
/// engineered pairing handshake so a real console will bond with it.
///
/// Status: works on ESP32-C6 (and the other open-NimBLE-controller chips) and on
/// ESP32-S3 (ESP-IDF >= v6.1) — advertising, the custom GATT tree, the 0x15
/// pairing handshake, the full init/calibration command sequence, LL encryption,
/// bond persistence, continuous input-report streaming, reconnect, and
/// wake-from-sleep are all implemented and verified against a real console. The
/// console drives the link at a sub-spec 5 ms interval for sustained input,
/// reconnect, and wake; the S3/C3 get that from ESP-IDF's default-on
/// CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE (>= v6.1), the open-NimBLE chips
/// from the opt-in tools/patch_nimble_5ms.py (see the component README).
///
/// \section switch2_pro_ex1 Example
/// \snippet switch2_pro_example.cpp switch2_pro example
class Switch2Pro : public BaseComponent {
public:
  /// Configuration for the controller.
  struct Config {
    std::string device_name{"Pro Controller"}; ///< BLE advertised name.
    Logger::Verbosity log_level{Logger::Verbosity::INFO};
    /// If we boot with a saved bond, broadcast the *wake* advertisement (and
    /// re-issue it every wake_interval) until the console connects, so a sleeping
    /// console is woken and reconnects without re-pairing. When false we use the
    /// plain reconnection advertisement (only reconnects an already-awake console).
    bool wake_console_on_boot{true};
    std::chrono::duration<float> wake_interval{std::chrono::seconds(5)};
    /// Replay captured IMU motion frames in the input reports' motion block. Once
    /// the console enables the IMU feature (it does during standard init), every
    /// report carries a 40-byte motion block. **Off (default): the block is sent
    /// all-zero**, which the console accepts (verified on hardware, and what the
    /// zhantss emulator ships) — fine for games that do not use motion. On: replay
    /// a captured 128-frame resting sequence — but it loops (~2 s at 62 Hz) so its
    /// embedded timestamps jump backwards at the wrap. The driver owns the motion
    /// block, so there is no per-report motion input today; a live-IMU path (a
    /// motion setter on the report) is future work.
    bool stream_imu_motion{false};
    /// Streaming model. **On (default) = continuous:** send one report every
    /// connection interval with the counter incrementing each time, exactly like
    /// a real controller — verified stable and lag-free on the C6-class chips and
    /// on the ESP32-S3 (ESP-IDF >= v6.1). This is the recommended mode on every
    /// supported target. **Off = on-change:** notify only when the app's
    /// button/stick state changes, plus a low-rate keepalive — an optional
    /// reduced-traffic mode (it was also a workaround for the pre-v6.1 S3 BTDM
    /// tx-servicing stall, now fixed; see README "Known issues").
    bool continuous_streaming{true};
    /// Continuous-mode send divisor: send one report every Nth connection interval
    /// (1 = every interval, 2 = every other, ...). The resulting rate depends on the
    /// live interval the console chose: at the initial 15 ms, N=1 is ~62 Hz and N=2
    /// ~31 Hz; once the console moves the link to 5 ms, N=1 is ~200 Hz and N=2
    /// ~100 Hz. Diagnostic knob to separate a time-based stall (console
    /// deprioritisation — stalls at the same wall-clock regardless of N) from a
    /// packet-count-based one (our-side tx-credit accumulation — survives ~N×
    /// longer). Ignored in on-change mode.
    uint32_t continuous_stream_divisor{1};
  };

  explicit Switch2Pro(const Config &config)
      : BaseComponent("Switch2Pro", config.log_level)
      , device_name_(config.device_name)
      , wake_console_on_boot_(config.wake_console_on_boot)
      , wake_interval_(config.wake_interval)
      , stream_imu_motion_(config.stream_imu_motion)
      , continuous_streaming_(config.continuous_streaming)
      , continuous_stream_divisor_(
            config.continuous_stream_divisor ? config.continuous_stream_divisor : 1)
      , ble_gatt_server_({.callbacks = {}, .log_level = Logger::Verbosity::WARN}) {}

  /// Stop the input-streaming task on teardown.
  ~Switch2Pro();

  /// Initialize NimBLE, build the custom GATT services, configure security so
  /// the console (not standard SMP) drives pairing, and start advertising.
  /// @return true on success.
  bool init();

  /// Whether the pairing handshake has completed with a console.
  bool is_paired() const { return paired_; }

  /// Whether a console is currently connected (link established; init/input
  /// subscription may still be in progress — see is_input_streaming()).
  bool is_connected() const { return active_conn_handle_ != 0xffff; }

  /// Broadcast the wake advertisement now (e.g. from a button press, matching a
  /// real controller's press-a-button-to-wake-the-console behaviour): embeds the
  /// bonded console's identity address with the wake flag so a sleeping console
  /// powers on and reconnects. Requires a stored bond (from a completed pairing,
  /// this boot or restored from NVS) and no active connection. Returns true if
  /// the advertisement was issued.
  bool wake_console();

  /// Whether the console has subscribed to the input characteristic (0x000e) and
  /// we are actively streaming input reports. Goes true near the end of init and
  /// false on disconnect; useful for driving post-connect behaviour (e.g. the
  /// L+R "select this controller" prompt) from the application.
  bool is_input_streaming() const { return input_subscribed_; }

  /// Store the latest controller state. This does NOT send — a driver-owned
  /// streaming task notifies the newest stored report once per connection
  /// interval (continuously, like a real controller), so you can call this as
  /// often as you like (e.g. on every button/stick change) without flooding the
  /// link. Thread-safe.
  void set_input_report(const switch2::Pro2InputReport &report) {
    std::lock_guard<std::mutex> lk(input_mutex_);
    input_report_ = report;
  }

  /// Advertisement variant. Discovery = fresh pairing (zero host addr). Reconnect
  /// = we already have a bond; the paired console's address is embedded so it
  /// recognises us and reconnects (skipping the 0x15 pairing). Wake = like
  /// Reconnect but sets the wake flag to bring a sleeping console back up.
  enum class AdvMode { Discovery, Reconnect, Wake };

protected:
  // --- setup ---
  bool build_gatt();
  void configure_security();
  void configure_callbacks();
  /// `host_addr_le` is the paired console's BD_ADDR in wire (little-endian) order,
  /// embedded verbatim for Reconnect/Wake; ignored for Discovery.
  /// @return true if advertising actually started.
  bool start_advertising(AdvMode mode, const std::array<uint8_t, 6> &host_addr_le = {});
  /// Advertise in the mode appropriate to the current state: Wake (with the stored
  /// console address) if bonded and wake-on-boot is enabled, else Reconnect if
  /// bonded, else Discovery.
  /// @return true if advertising actually started.
  bool advertise();
  /// Start a periodic timer that re-issues the wake advertisement (via advertise())
  /// every wake_interval_ while disconnected, so a sleeping console keeps getting
  /// nudged until it wakes and reconnects. No-op if already running.
  void start_wake_timer();

  /// Log a byte buffer as hex at debug level (command/response tracing).
  void log_hex(const char *prefix, const uint8_t *data, size_t len);
  /// Log the assigned GATT handles (call after the server has started).
  void log_handle_map();

  // --- command channel ---
  /// Handle a write on a command characteristic. `via_vibration_command` is
  /// true for writes on 0x0016 (pairing/init) which reply on 0x001e, false for
  /// writes on 0x0014 which reply on 0x001a. Parses the 8-byte header and
  /// dispatches, notifying a response.
  void on_command_write(bool via_vibration_command, const uint8_t *data, size_t len);
  void handle_pairing(bool via_vibration_command, uint8_t transport, switch2::PairingSub sub,
                      const uint8_t *payload, size_t len);
  void handle_command(bool via_vibration_command, switch2::Command cmd, uint8_t transport,
                      uint8_t sub, const uint8_t *payload, size_t len);

  /// Build an 8-byte device->host response header + payload and notify it on
  /// the response characteristic matching the request source.
  void send_response(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub,
                     uint8_t byte4, uint8_t byte5, const uint8_t *payload, size_t payload_len);
  /// Header-only BLE ACK (byte4=0x10, byte5=0x78, no payload) — matches a real
  /// Pro Controller 2's init-sequence ACKs.
  void send_ack(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub);

  /// Inject the current LTK (ltk_) into NimBLE's security store for `peer` so the
  /// controller can satisfy the console's link-layer encryption request (the
  /// Switch 2 uses standard LL encryption with the app-derived LTK, not SMP).
  /// @return true if the LTK was written to the store; false on failure (LL
  ///         encryption will then fail and the console will drop the link).
  bool inject_ltk(uint8_t peer_type, const uint8_t *peer_val_le);
  /// Inject ltk_ for the currently-connected peer (used right after finalise).
  /// @return true on success; false if there is no active connection or the
  ///         store write failed.
  bool inject_pairing_ltk();
  /// Persist the bond {console address, LTK} to NVS so it survives reboots and
  /// the controller can reconnect/wake without re-pairing.
  void save_bond();
  /// Load a persisted bond into bond_peer_* / ltk_. Returns true if one exists.
  bool load_bond();
  /// Our own BT address (6 bytes) for the exchange-addresses reply.
  std::array<uint8_t, 6> local_bt_address() const;

  /// Driver-owned streaming task: while the console is subscribed, notify the
  /// input report on 0x000e. In continuous mode (default, Config::continuous_streaming
  /// = true) it sends one report every connection interval like a real controller;
  /// in on-change mode it sends only when the app state changes plus a low-rate
  /// keepalive. Started in init(), stopped in the destructor.
  void input_stream_loop();
  /// Send the given input-report snapshot now (the caller passes the exact bytes it
  /// snapshotted under input_mutex_; this adds the counter/rumble/motion fields it
  /// manages), honoring the mbuf backpressure cap. Returns true iff a notification
  /// was actually queued (rc==0); false on a backpressure skip or ENOMEM. Called by
  /// input_stream_loop().
  bool send_input_report(const std::array<uint8_t, switch2::Pro2InputReport::SIZE> &report_data);
  /// On-change keepalive: send a report at least this often (in connection
  /// intervals) even when the app state is unchanged, so the console keeps seeing
  /// the controller as active. ~10 intervals ≈ 150 ms at 15 ms.
  static constexpr uint32_t kKeepaliveIntervals = 10;
  /// Compact one-line dump of every NimBLE mempool's free/total(low-water) — the
  /// authoritative "is the tx pool actually draining back?" signal for the wedge.
  std::string pool_stats();
  /// Real backpressure: true iff the host msys_1 mbuf pool has fewer than
  /// kMaxOutstandingMbufs blocks currently un-drained (outstanding = total-free).
  /// This is the TRUE over-air-completion signal — unlike notify_in_flight_, which
  /// is decremented at host→controller handoff and so never reflects the backlog.
  bool msys1_headroom();
  /// Max input-report mbufs allowed un-drained at once. Healthy streaming holds
  /// ~2 outstanding, so this only ever bites during a backlog — capping latency
  /// (~Nx interval) and guaranteeing the pool never reaches 0 (the ENOMEM wedge).
  static constexpr int kMaxOutstandingMbufs = 8;
  /// Read the live connection interval/latency/PHY and log a line whenever any of
  /// them changes (diagnostic for the pairing->active LL renegotiation).
  void poll_conn_state();
  /// Track CCCD subscribe/unsubscribe so we only stream input when the console
  /// has asked for it (updates input_subscribed_ for the 0x000e characteristic).
  void on_subscribe(NimBLECharacteristic *characteristic, uint16_t sub_value);
  /// Notification tx-complete for `characteristic` (frees a tx buffer). Decrements
  /// the in-flight count for the input characteristic so notify_input_report can
  /// flow-control the stream and never overrun the link's tx pool.
  void on_notify_tx(NimBLECharacteristic *characteristic);

  friend class ChannelCallbacks;

  std::string device_name_;
  bool wake_console_on_boot_;
  std::chrono::duration<float> wake_interval_;
  bool stream_imu_motion_;
  bool continuous_streaming_; ///< see Config::continuous_streaming (on-change vs per-interval)
  uint32_t
      continuous_stream_divisor_; ///< see Config::continuous_stream_divisor (rate-halving probe)
  std::shared_ptr<espp::Timer> wake_timer_; ///< re-issues the wake advertisement until connected
  BleGattServer ble_gatt_server_;

  // Proprietary GATT characteristics (owned by NimBLE once created).
  NimBLECharacteristic *common_input_{nullptr};
  NimBLECharacteristic *pro2_input_{nullptr};
  NimBLECharacteristic *command_{nullptr};
  NimBLECharacteristic *vibration_command_{nullptr};
  NimBLECharacteristic *command_response1_{nullptr};
  NimBLECharacteristic *command_response2_{nullptr};

  // Pairing state.
  /// Whether the pairing handshake has completed / a bond exists. Written by the
  /// FINALISE callback (host task), read by the public is_paired() getter (app
  /// task) — atomic.
  std::atomic<bool> paired_{false};
  /// Highest completed 0x15 pairing step this connection: 0=none, 1=exchange
  /// addresses, 2=exchange keys, 3=confirm LTK. FINALISE (step 4) is only accepted
  /// when this is 3, so an out-of-order/malformed peer cannot persist a bad bond.
  /// Reset to 0 on each new connection.
  uint8_t pairing_stage_{0};
  /// Booted with a stored bond (reconnect, not fresh pair). Written by FINALISE
  /// (host task) / init, read by advertise() and wake_console() (app task) — atomic.
  std::atomic<bool> reconnect_mode_{false};
  /// wake_console() latched: keep the WAKE adv variant on the air until connected.
  /// Written from the app task (wake_console) and the connect callback, read by
  /// advertise() — atomic.
  std::atomic<bool> wake_pending_{false};
  /// One-shot wake-on-boot state: true from boot (when wake_console_on_boot_ and
  /// bonded) until the FIRST successful connection, then cleared so we do NOT keep
  /// waking a console the user later puts to sleep. Read by the wake-timer task and
  /// advertise(), written by the connect callback — atomic. (User-requested wake is
  /// separate: wake_pending_.)
  std::atomic<bool> boot_wake_pending_{false};
  /// Console has enabled input-report notifications (0x000e). Written from the
  /// NimBLE callback thread, read by the streaming thread — atomic to avoid a race.
  std::atomic<bool> input_subscribed_{false};
  uint8_t report_counter_{0}; ///< input-report sequence (byte 0); +1 per delivered report
  std::atomic<int> notify_in_flight_{0}; ///< queued-but-not-yet-transmitted input notifications
  std::atomic<uint32_t> tx_completions_{
      0};                                 ///< count of NOTIFY_TX completions (flow-control signal)
  std::atomic<uint32_t> enomem_count_{0}; ///< diagnostic: notifies deferred because the tx pool was
                                          ///< full (read cross-thread at disconnect)
  uint32_t motion_idx_{0};                ///< index into kMotionSequence for the replayed IMU block
  std::array<uint8_t, switch2::Pro2InputReport::SIZE>
      last_streamed_{};        ///< exact snapshot we last notified (on-change dedup)
  bool have_streamed_{false};  ///< false until the first report goes out (forces initial send)
  uint32_t idle_intervals_{0}; ///< connection intervals since last send (on-change keepalive)
  uint32_t interval_tick_{0};  ///< continuous-mode interval counter (for the rate divisor)
  // --- tx-wedge diagnostics: localize the ENOMEM stall (our tx drain vs the console) ---
  std::atomic<int64_t> last_tx_complete_us_{0}; ///< esp_timer time of the last NOTIFY_TX completion
  std::atomic<int64_t> stream_start_us_{0};     ///< when the current streaming run began (0 = not
                                                ///< started; read cross-thread at disconnect)
  int64_t hb_last_us_{0};                       ///< last heartbeat timestamp
  uint32_t hb_last_completions_{
      0};                      ///< tx_completions_ snapshot at last heartbeat (drain-rate delta)
  uint32_t hb_last_enomem_{0}; ///< enomem_count_ snapshot at last heartbeat
  uint32_t send_attempts_{0};  ///< send_input_report() calls this streaming run
  uint32_t backpressure_skips_{0}; ///< sends deferred because msys_1 had no headroom
  std::atomic<bool> wedge_reported_{
      false}; ///< one-shot guard for the wedge-onset log (read cross-thread at disconnect)
  std::mutex input_mutex_; ///< guards input_report_ (set from app task, read by stream task)
  std::thread input_stream_thread_;      ///< streams input reports once per connection interval
  std::atomic<bool> stream_stop_{false}; ///< signals input_stream_thread_ to exit
  // Last-observed link state, logged whenever it changes so we can see exactly
  // what the console renegotiates at the pairing->active transition.
  uint16_t last_itvl_{0};
  uint16_t last_latency_{0xffff};
  uint8_t last_tx_phy_{0};
  uint8_t last_rx_phy_{0};
  /// Current connection handle (0xffff = BLE_HS_CONN_HANDLE_NONE). Written from the
  /// NimBLE connect/disconnect callbacks, read by the streaming/timer threads — atomic.
  std::atomic<uint16_t> active_conn_handle_{0xffff};
  std::array<uint8_t, 16> ltk_{};          ///< derived during key exchange (A1 ^ B1)
  std::array<uint8_t, 6> host_addr_{};     ///< console BD_ADDR (from exchange-addresses)
  uint8_t bond_peer_type_{0};              ///< persisted console address type
  std::array<uint8_t, 6> bond_peer_val_{}; ///< persisted console address (wire/little-endian order)
  uint8_t feature_mask_{switch2::PRO2_FEATURE_MASK};
  /// Features the console has actually enabled via FEATURE_SELECT (0x0c). The
  /// input report must reflect these: rumble (bit 5) sets report byte 0x0B to
  /// 0x38, and IMU (bit 2) makes us stream the 40-byte motion block — the
  /// console enables both (mask 0x2f) and discards reports that omit them.
  /// Written from the FEATURE_SELECT command handler (callback thread), read by the
  /// streaming thread when building each report — atomic.
  std::atomic<uint8_t> enabled_features_{0};

  switch2::Pro2InputReport input_report_{};
};

} // namespace espp
