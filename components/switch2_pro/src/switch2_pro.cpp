#include "switch2_pro.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_pthread.h" // esp_pthread_set_cfg — size the streaming task's stack
#include "esp_timer.h"   // esp_timer_get_time — µs timestamps for tx-wedge telemetry
#include "nvs.h"
#include "os/os_mempool.h" // os_mempool_info_get_next — mbuf pool free/low-water telemetry

#include "host/ble_gatt.h"    // ble_gatts_notify_custom — low-level notify (exposes rc)
#include "host/ble_hs.h"      // ble_hs_id_infer_auto / ble_hs_id_copy_addr
#include "host/ble_hs_mbuf.h" // ble_hs_mbuf_from_flat
#include "host/ble_store.h"   // ble_store_write_our_sec — inject the pairing LTK

#include "switch2_pro_flash.hpp"
#include "switch2_pro_motion.hpp"

// The Switch 2 console filters controllers on a 31-byte LEGACY advertisement
// carrying Nintendo manufacturer data, and this component builds that via the
// legacy BleGattServer::AdvertisingParameters path (which only exists when NimBLE
// extended advertising is disabled). Fail fast with a clear message instead of a
// confusing template error if a consumer enables extended advertising.
#if defined(CONFIG_BT_NIMBLE_EXT_ADV) && CONFIG_BT_NIMBLE_EXT_ADV
#error                                                                                             \
    "switch2_pro requires legacy advertising; disable CONFIG_BT_NIMBLE_EXT_ADV (NimBLE extended advertising)."
#endif

namespace espp {

using namespace switch2;

/// Characteristic callbacks that (a) trace everything the console does — reads,
/// writes, notification subscriptions — for debugging bring-up, and (b) for the
/// two command channels, dispatch writes into the owner. role: 0 = passive
/// (log only), 1 = command channel 0x0014, 2 = vibration+command 0x0016.
class ChannelCallbacks : public NimBLECharacteristicCallbacks {
public:
  ChannelCallbacks(Switch2Pro *owner, const char *name, int role)
      : owner_(owner)
      , name_(name)
      , role_(role) {}

  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo & /*conn*/) override {
    auto value = characteristic->getValue();
    // Command channels log their own decoded hex in on_command_write; only log
    // a raw dump here for the passive channels (role 0) to avoid duplication.
    if (role_ == 0) {
      owner_->logger_.debug("WRITE {} ({} bytes)", name_, value.size());
      owner_->log_hex(name_, value.data(), value.size());
    }
    if (role_ == 1)
      owner_->on_command_write(/*via_vibration_command=*/false, value.data(), value.size());
    else if (role_ == 2)
      owner_->on_command_write(/*via_vibration_command=*/true, value.data(), value.size());
  }
  void onRead(NimBLECharacteristic * /*c*/, NimBLEConnInfo & /*conn*/) override {
    owner_->logger_.info("READ  {}", name_);
  }
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo & /*conn*/,
                   uint16_t sub_value) override {
    owner_->logger_.info("SUBSCRIBE {} value=0x{:04x} ({})", name_, sub_value,
                         sub_value ? "on" : "off");
    owner_->on_subscribe(c, sub_value);
  }
  // Fires (BLE_GAP_EVENT_NOTIFY_TX) once a notification we sent has been
  // transmitted, freeing its tx buffer. Used to flow-control the input stream so
  // we never queue faster than the link drains (which otherwise saturates the
  // tx pool and makes every subsequent notify fail).
  void onStatus(NimBLECharacteristic *c, NimBLEConnInfo & /*conn*/, int /*code*/) override {
    owner_->on_notify_tx(c);
  }

private:
  Switch2Pro *owner_;
  const char *name_;
  int role_;
};

bool Switch2Pro::init() {
  // Keep the NimBLE host log quiet — our own Switch2Pro trace carries the
  // protocol flow. Bump these to ESP_LOG_DEBUG when the raw stack-level view
  // (every ATT/ACL byte) is needed.
  esp_log_level_set("NimBLE", ESP_LOG_WARN);
  esp_log_level_set("NimBLEGATTS", ESP_LOG_WARN);

  // The pairing crypto is the load-bearing part; verify it against the golden
  // vector up front so a broken build fails loudly rather than at the console.
  if (PairingCrypto::self_test()) {
    logger_.info("pairing crypto self-test passed");
  } else {
    logger_.error("pairing crypto self-test FAILED — pairing will be rejected");
    return false;
  }

  configure_callbacks();
  // A real Pro Controller 2 exposes ONLY its two vendor services (plus GAP/GATT)
  // — no Device Information or Battery service. Suppress BleGattServer's built-in
  // DIS/BAS so the console's GATT discovery sees the same attribute set; the
  // extra services (and the handle shift they cause) make the console reject us
  // after discovery.
  ble_gatt_server_.set_builtin_info_services_enabled(false);
  if (!ble_gatt_server_.init(device_name_)) {
    logger_.error("failed to init BLE GATT server");
    return false;
  }
  // A real Pro Controller 2 advertises with a FIXED address; esp-nimble-cpp
  // defaults to a random address that also changes every boot. The console
  // stores the controller's address during exchange-addresses (0x15/0x01) and
  // rejects an unstable one. Prefer the public address; if the S3 controller
  // exposes none, derive a STABLE static-random address from the factory MAC so
  // it never changes between boots. local_bt_address() reports whatever we set,
  // so the exchange always matches our advertisement.
  if (NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC)) {
    logger_.info("BLE address: using PUBLIC");
  } else {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BT); // stable factory MAC, big-endian (display order)
    // ble_hs_id_set_rnd wants little-endian; a static-random address needs the
    // two most-significant bits of the MSB set.
    std::array<uint8_t, 6> rnd = {mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]};
    rnd[5] |= 0xC0;
    NimBLEDevice::setOwnAddr(rnd.data());
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    logger_.warn("BLE address: PUBLIC unavailable; using STABLE static-random {:02x}:{:02x}:{:02x}:"
                 "{:02x}:{:02x}:{:02x}",
                 rnd[5], rnd[4], rnd[3], rnd[2], rnd[1], rnd[0]);
  }
  // Load any persisted bond BEFORE configure_security (which otherwise clears
  // bonds): if present we reconnect instead of re-pairing.
  reconnect_mode_ = load_bond();
  if (reconnect_mode_) {
    paired_ = true;
    logger_.info("loaded stored bond — reconnection mode (console address persisted)");
  }
  configure_security();
  if (!build_gatt()) {
    logger_.error("failed to build GATT services");
    return false;
  }
  ble_gatt_server_.start_services();
  ble_gatt_server_.start();
  log_handle_map(); // after start(), so handles are assigned
  // On reconnect the console skips the 0x15 pairing and jumps straight to LL
  // encryption, so the LTK must already be in NimBLE's store before it connects.
  if (reconnect_mode_) {
    inject_ltk(bond_peer_type_, bond_peer_val_.data());
    if (wake_console_on_boot_) {
      logger_.info(
          "wake-on-boot: broadcasting the wake advertisement every {:.0f}s until connected",
          wake_interval_.count());
      start_wake_timer();
    }
  }
  advertise();
  logger_.info("Switch2Pro advertising as '{}'", device_name_);

  // Start the driver-owned input-streaming task. It notifies the latest report
  // once per connection interval while the console is subscribed. Give it a
  // generous stack (ble_gatts_notify_custom is a deep call) and pin it to core 0,
  // away from the BLE controller/host on core 1.
  esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
  cfg.stack_size = 8192;
  cfg.prio = 5;
  cfg.pin_to_core = 0;
  cfg.thread_name = "s2p_stream";
  if (esp_pthread_set_cfg(&cfg) != ESP_OK)
    logger_.warn("esp_pthread_set_cfg failed; streaming thread will use default stack/prio/core");
  input_stream_thread_ = std::thread(&Switch2Pro::input_stream_loop, this);
  return true;
}

Switch2Pro::~Switch2Pro() {
  // Tear down everything that can call back into `this` BEFORE the members those
  // callbacks touch are destroyed. The streaming thread, the wake timer, and the
  // NimBLE GAP/GATT callbacks all capture `this`; members declared after
  // ble_gatt_server_/wake_timer_ are destroyed first, so a late callback would
  // otherwise access already-destroyed state.
  stream_stop_.store(true);
  if (input_stream_thread_.joinable())
    input_stream_thread_.join();
  if (wake_timer_) {
    wake_timer_->cancel(); // stop + join the wake-advertisement timer task
    wake_timer_.reset();
  }
  ble_gatt_server_.stop_advertising();
  ble_gatt_server_.set_callbacks({}); // detach the this-capturing GAP/GATT callbacks
}

void Switch2Pro::configure_security() {
  // The Switch 2 does its own app-level pairing over the command channel (the
  // 0x15 exchange), NOT BLE SMP. BLE-level bonding here just creates a bond the
  // console then uses for GATT caching, which makes it skip service discovery
  // on reconnect and get stuck. So: no bonding, no SMP-initiated security.
  ble_gatt_server_.set_security(/*bonding=*/false, /*mitm=*/false, /*secure=*/false);
  ble_gatt_server_.set_io_capabilities(BLE_HS_IO_NO_INPUT_OUTPUT);
  // Only clear bonds on a FRESH start. If we have a persisted bond we are in
  // reconnection mode and must KEEP the injected LTK so the console can re-encrypt
  // without re-pairing.
  if (!reconnect_mode_) {
    size_t cleared = ble_gatt_server_.unpair_all().size();
    if (cleared)
      logger_.info("cleared {} stale BLE bond(s)", cleared);
  }
}

void Switch2Pro::configure_callbacks() {
  BleGattServer::Callbacks callbacks;
  callbacks.connect_callback = [this](NimBLEConnInfo &info) {
    // Remember the connection so the pairing exchange can report the exact
    // over-the-air address the console connected to (see local_bt_address()).
    active_conn_handle_ = info.getConnHandle();
    wake_pending_ = false; // wake accomplished — subsequent advertising can be passive
    pairing_stage_ = 0;    // a fresh connection restarts the 0x15 handshake sequence
    // The connection interval right after connect is the key diagnostic: the
    // Switch 2 drives 5 ms (interval == 4 units). If a controller can't hold
    // that, the console typically disconnects with a supervision timeout.
    // Connection interval is logged for reference, but note: a real console
    // pairs entirely at the initial 15 ms interval (verified against the
    // procon2 pairing capture) — it does NOT move to 5 ms until after pairing.
    // So this value is not a pairing gate; it matters only for post-pairing
    // low-latency input streaming.
    logger_.info("connected: peer={} interval={:.2f}ms supervision={}ms latency={}",
                 info.getAddress().toString(), info.getConnInterval() * 1.25f,
                 info.getConnTimeout() * 10, info.getConnLatency());
    // NOTE: we deliberately do NOT initiate a connection-parameter update here.
    // It cannot lower us below 7.5 ms anyway (NimBLE floors ble_gap_update_params
    // at the spec minimum), the console keeps 15 ms regardless, and the real 5 ms
    // arrives in the console's CONNECT_IND on reconnect (needs the controller
    // patch), not via an update. On the ESP32-S3 BTDM controller, kicking off an
    // update procedure right after connect correlated with a degraded/limping tx
    // link, so it is removed. See request_fast_interval() history in git if you
    // want to re-test it.
  };
  callbacks.disconnect_callback = [this](NimBLEConnInfo &info, BleGattServer::DisconnectReason r) {
    logger_.warn("disconnected: peer={} reason={} (paired={})", info.getAddress().toString(), r,
                 paired_);
    // Tie the disconnect to the tx-wedge timeline: how long we streamed, whether
    // we had wedged, and how stale the last completion was. A disconnect ~1 SVN
    // timeout after the wedge with a large since_last_tx = over-air exchange
    // stopped at the wedge; staying up long after = the link outlived our tx stall.
    if (stream_start_us_ != 0) {
      const int64_t now = esp_timer_get_time();
      logger_.warn("  @disconnect: streamed {:.1f}s, {} completions, {} enomem, wedged={}, "
                   "since_last_tx={:.0f}ms | {}",
                   (now - stream_start_us_) / 1e6f, tx_completions_.load(), enomem_count_,
                   wedge_reported_, (now - last_tx_complete_us_.load()) / 1000.0f, pool_stats());
    }
    // NOTE: paired_ is intentionally NOT cleared here. is_paired() reports whether
    // the pairing handshake has completed / a bond exists — which survives a
    // disconnect (the bond is persisted in NVS and a bonded reconnect does not
    // re-run the 0x15 handshake). Use is_connected()/is_input_streaming() for
    // live-session state.
    input_subscribed_ = false;
    active_conn_handle_ = 0xffff; // so the wake timer knows we're disconnected
    advertise();
  };
  callbacks.conn_params_update_callback = [this](NimBLEConnInfo &info) {
    // Fires when ANY connection-parameter-update procedure completes — including
    // the console's answer to our at-connect offer, and any console-initiated
    // update. If the interval here is still 15 ms, the console REJECTED (or
    // no-op'd) the procedure; if it moved (5/7.5 ms), it accepted. Before this
    // callback existed we were blind to the difference between "rejected" and
    // "console never responded".
    logger_.info("CONN PARAMS UPDATE: itvl={:.2f}ms latency={} timeout={}ms",
                 info.getConnInterval() * 1.25f, info.getConnLatency(), info.getConnTimeout() * 10);
  };
  callbacks.authentication_complete_callback = [this](const NimBLEConnInfo &info) {
    // If this fires, the console ran BLE SMP (which the research says it should
    // NOT do). Encrypted={}, bonded={} tells us what security state it reached.
    logger_.info("AUTH complete: encrypted={} bonded={} authenticated={}", info.isEncrypted(),
                 info.isBonded(), info.isAuthenticated());
  };
  ble_gatt_server_.set_callbacks(callbacks);
}

void Switch2Pro::log_hex(const char *prefix, const uint8_t *data, size_t len) {
  std::string hex;
  hex.reserve(len * 3);
  char tmp[4];
  for (size_t i = 0; i < len; ++i) {
    std::snprintf(tmp, sizeof(tmp), "%02x ", data[i]);
    hex += tmp;
  }
  // DEBUG: the raw command/response bytes are verbose (and, streamed over serial
  // during the rapid init sequence, can saturate the UART). Set the component
  // log level to DEBUG to see them.
  logger_.debug("{} [{}]: {}", prefix, len, hex);
}

bool Switch2Pro::build_gatt() {
  auto *server = ble_gatt_server_.server();
  if (server == nullptr)
    return false;

  // Register our Nintendo services BEFORE NimBLE's GAP/GATT so they occupy the
  // low attribute handles (0x0001+) with GAP/GATT last — matching a real Pro
  // Controller 2's exact handle layout. A real console addresses the controller
  // by fixed handles (0x0016 command, 0x001e response, …) and never discovers;
  // with our services shifted to 0x0022+ the console is forced into a discovery
  // + firmware-probe fallback path that rejects at the pairing commit. Must be
  // set before start_services() (below) starts the GATT server.
  server->registerServicesFirst(true);

  // Attach a tracing callback to every characteristic so bring-up logs show
  // exactly what the console does. Roles: 1 = command 0x0014, 2 = vibration+
  // command 0x0016, 0 = passive.
  auto attach = [this](NimBLECharacteristic *c, const char *name, int role) {
    c->setCallbacks(new ChannelCallbacks(this, name, role));
  };

  // Service 1 (purpose not fully understood; created so the handle map matches
  // what the console observed from a real controller).
  auto *svc1 = server->createService(NimBLEUUID(SERVICE1_UUID));
  attach(svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_281_UUID), NIMBLE_PROPERTY::READ),
         "svc1.281", 0);
  attach(svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_282_UUID), NIMBLE_PROPERTY::WRITE),
         "svc1.282", 0);
  attach(svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_283_UUID), NIMBLE_PROPERTY::READ),
         "svc1.283", 0);

  // Service 2 — the main HID-like service. The full characteristic + descriptor
  // set is replicated from a real Pro Controller 2 (bluetooth_interface.md GATT
  // table): the NOTIFY characteristics auto-get a 0x2902 CCCD from NimBLE, and a
  // real controller additionally hangs a vendor descriptor off each report /
  // response characteristic (0x679d5510 "report rate" on inputs, 0xb746df8c on
  // responses). The console reads the whole table during discovery, so a missing
  // characteristic or descriptor makes it reject us.
  auto add_desc = [](NimBLECharacteristic *c, const char *uuid) {
    c->createDescriptor(NimBLEUUID(uuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, 32);
  };
  auto *svc2 = server->createService(NimBLEUUID(SERVICE2_UUID));

  common_input_ = svc2->createCharacteristic(NimBLEUUID(COMMON_INPUT_UUID),
                                             NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  attach(common_input_, "common_input(0x000a)", 0);
  add_desc(common_input_, REPORT_RATE_DESC_UUID);
  pro2_input_ = svc2->createCharacteristic(NimBLEUUID(PRO2_INPUT_UUID),
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  attach(pro2_input_, "pro2_input(0x000e)", 0);
  add_desc(pro2_input_, REPORT_RATE_DESC_UUID);
  attach(svc2->createCharacteristic(NimBLEUUID(VIBRATION_UUID), NIMBLE_PROPERTY::WRITE_NR),
         "vibration(0x0012)", 0);
  command_ = svc2->createCharacteristic(NimBLEUUID(COMMAND_UUID), NIMBLE_PROPERTY::WRITE_NR);
  attach(command_, "command(0x0014)", 1);
  vibration_command_ =
      svc2->createCharacteristic(NimBLEUUID(VIBRATION_COMMAND_UUID), NIMBLE_PROPERTY::WRITE_NR);
  attach(vibration_command_, "vib_command(0x0016)", 2);
  // Firmware-update output is WRITE-NO-RESPONSE on a real controller, not WRITE.
  attach(svc2->createCharacteristic(NimBLEUUID(FIRMWARE_UPDATE_UUID), NIMBLE_PROPERTY::WRITE_NR),
         "firmware(0x0018)", 0);
  command_response1_ =
      svc2->createCharacteristic(NimBLEUUID(COMMAND_RESPONSE1_UUID), NIMBLE_PROPERTY::NOTIFY);
  attach(command_response1_, "resp1(0x001a)", 0);
  add_desc(command_response1_, CMD_RESPONSE_DESC_UUID);
  command_response2_ =
      svc2->createCharacteristic(NimBLEUUID(COMMAND_RESPONSE2_UUID), NIMBLE_PROPERTY::NOTIFY);
  attach(command_response2_, "resp2(0x001e)", 0);
  add_desc(command_response2_, CMD_RESPONSE_DESC_UUID);

  // Additional attributes a real Pro Controller 2 exposes (purpose unknown);
  // replicated so the console's discovery sees the full characteristic set.
  auto *unknown_input1 =
      svc2->createCharacteristic(NimBLEUUID(UNKNOWN_INPUT1_UUID), NIMBLE_PROPERTY::NOTIFY);
  attach(unknown_input1, "unk_input1(0x0022)", 0);
  add_desc(unknown_input1, CMD_RESPONSE_DESC_UUID);
  auto *unknown_input2 = svc2->createCharacteristic(
      NimBLEUUID(UNKNOWN_INPUT2_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  attach(unknown_input2, "unk_input2(0x0026)", 0);
  add_desc(unknown_input2, REPORT_RATE_DESC_UUID);
  attach(svc2->createCharacteristic(NimBLEUUID(UNKNOWN_OUTPUT_UUID), NIMBLE_PROPERTY::WRITE_NR),
         "unk_output(0x002a)", 0);

  // Headset-audio attributes of an updated Pro Controller 2 — presence signals
  // fully-updated firmware so the console treats us as a genuine (not factory)
  // controller.
  attach(svc2->createCharacteristic(NimBLEUUID(AUDIO_OUTPUT_UUID), NIMBLE_PROPERTY::WRITE_NR),
         "audio_output(0x002c)", 0);
  auto *audio_input = svc2->createCharacteristic(NimBLEUUID(AUDIO_INPUT_UUID),
                                                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  attach(audio_input, "audio_input(0x002e)", 0);
  add_desc(audio_input, REPORT_RATE_DESC_UUID);
  attach(svc2->createCharacteristic(NimBLEUUID(AUDIO_COMMAND_UUID), NIMBLE_PROPERTY::WRITE_NR),
         "audio_command(0x0032)", 0);

  svc1->start();
  svc2->start();
  return true;
}

void Switch2Pro::log_handle_map() {
  // Handles are only assigned once the server has started, so this must run
  // after ble_gatt_server_.start(). A real Pro Controller 2 has these
  // characteristics at fixed handles (parenthesized); if BleGattServer's
  // GAP/GATT/DeviceInfo/Battery services shifted ours off those and the console
  // keys off them, that explains connect-but-no-command-channel.
  logger_.info("GATT handle map (actual vs real-controller):");
  logger_.info("  common_input  = 0x{:04x} (0x000a)", common_input_->getHandle());
  logger_.info("  pro2_input    = 0x{:04x} (0x000e)", pro2_input_->getHandle());
  logger_.info("  command       = 0x{:04x} (0x0014)", command_->getHandle());
  logger_.info("  vib_command   = 0x{:04x} (0x0016)", vibration_command_->getHandle());
  logger_.info("  resp1         = 0x{:04x} (0x001a)", command_response1_->getHandle());
  logger_.info("  resp2         = 0x{:04x} (0x001e)", command_response2_->getHandle());
}

void Switch2Pro::start_advertising(AdvMode mode, const std::array<uint8_t, 6> &host_addr_le) {
  auto mfr = MANUFACTURER_DATA_DISCOVERY;
  if (mode != AdvMode::Discovery) {
    if (mode == AdvMode::Wake)
      mfr[MANUFACTURER_WAKE_FLAG_OFFSET] = WAKE_FLAG;
    // The paired console's address is embedded verbatim (already wire order);
    // this is how the console recognises a known controller on reconnect/wake.
    for (size_t i = 0; i < 6; ++i)
      mfr[MANUFACTURER_HOST_ADDR_OFFSET + i] = host_addr_le[i];
  }

  // The console filters on the Nintendo manufacturer data, so it MUST be in the
  // primary advertisement. Flags (3) + manufacturer data (2 + 26 = 28) = 31
  // bytes, exactly the 31-byte legacy limit; the name goes in the scan response
  // so the whole thing doesn't overflow (which would silently drop the manufacturer
  // data and make the controller invisible to the console).
  // Stop any active advertising FIRST: NimBLE's start() early-returns when
  // already advertising, and updating adv data mid-advertising (HCI Set
  // Advertising Data while enabled) is not honoured by every controller — so a
  // variant switch (e.g. Reconnect -> Wake on a button press) is only
  // guaranteed to air after a clean stop/set/start cycle.
  ble_gatt_server_.stop_advertising();

  BleGattServer::AdvertisedData adv_data;
  adv_data.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  if (!adv_data.setManufacturerData(mfr.data(), mfr.size()))
    logger_.error("manufacturer data did not fit the advertisement!");
  ble_gatt_server_.set_advertisement_data(adv_data);

  BleGattServer::AdvertisedData scan_response;
  scan_response.setName(device_name_);
  ble_gatt_server_.set_scan_response_data(scan_response);

  BleGattServer::AdvertisingParameters params{};
  params.connectable = true;
  params.scan_response = true;
  ble_gatt_server_.start_advertising(params);
  logger_.info("advertising ({}): flags+mfr({} B) in adv, name in scan response",
               mode == AdvMode::Discovery   ? "discovery"
               : mode == AdvMode::Reconnect ? "reconnect"
                                            : "wake",
               mfr.size());
}

void Switch2Pro::advertise() {
  // Embed the console's STABLE IDENTITY address (from the 0x15 exchange, persisted
  // in the bond) — that is what the real controller advertises on reconnect, and
  // what the console matches to recognise us and grant the fast 5 ms interval. Do
  // NOT use bond_peer_val_ (the NimBLE connection address), which can be a
  // rotating private address we cannot resolve without an SMP bond.
  //
  // Use the WAKE variant (0x81 flag = "user pressed a button, connect to me")
  // while a wake is pending: an awake console ignores the flag-less Reconnect
  // variant from its idle screens, and a waking console may transiently
  // connect/drop (which re-enters here) — the latch keeps the wake variant on
  // the air until a connection actually completes.
  if (reconnect_mode_ && (wake_console_on_boot_ || wake_pending_))
    start_advertising(AdvMode::Wake, host_addr_);
  else if (reconnect_mode_)
    start_advertising(AdvMode::Reconnect, host_addr_);
  else
    start_advertising(AdvMode::Discovery);
}

bool Switch2Pro::wake_console() {
  if (active_conn_handle_ != 0xffff)
    return false; // already connected — nothing to wake
  static constexpr std::array<uint8_t, 6> kZeroAddr{};
  if (host_addr_ == kZeroAddr)
    return false; // no bonded console identity to address the wake to
  logger_.info("wake: broadcasting wake advertisement (user-requested)");
  wake_pending_ = true; // keep the wake variant on the air (across any transient
                        // connect/drop while the console boots) until connected
  start_advertising(AdvMode::Wake, host_addr_);
  return true;
}

void Switch2Pro::start_wake_timer() {
  if (wake_timer_)
    return;
  wake_timer_ = std::make_shared<espp::Timer>(espp::Timer::Config{
      .name = "switch2 wake",
      .period = wake_interval_,
      .callback = [this]() -> bool {
        // While disconnected, keep re-issuing the wake advertisement so a
        // sleeping console is repeatedly nudged; do nothing once connected.
        if (active_conn_handle_ == 0xffff) {
          logger_.info("wake: re-broadcasting wake advertisement (waiting for console)");
          advertise();
        }
        return false; // never cancel — resume nudging after any disconnect
      },
      .auto_start = true,
      .stack_size_bytes = 8192, // advertise() → NimBLE is a deep call; 4096 can overflow
  });
}

std::array<uint8_t, 6> Switch2Pro::local_bt_address() const {
  // Return the exact BLE address the console connected to, in on-air
  // little-endian order (LSB first) — this is what the pairing exchange expects
  // (the console sends its own addresses byte-reversed too). ble_hs_id_copy_addr
  // gives NimBLE's address in that order and accounts for public-vs-random, so it
  // always matches our advertisement. esp_read_mac (display/big-endian order, and
  // not necessarily the advertised address) is only a fallback.
  std::array<uint8_t, 6> addr{};
  // Best source: the exact over-the-air address this connection was established
  // with (our_ota_addr, already little-endian). This is precisely what the
  // console connected to, so the exchange can never disagree with our
  // advertisement regardless of public-vs-random.
  struct ble_gap_conn_desc desc;
  if (active_conn_handle_ != BLE_HS_CONN_HANDLE_NONE &&
      ble_gap_conn_find(active_conn_handle_, &desc) == 0) {
    std::copy(std::begin(desc.our_ota_addr.val), std::end(desc.our_ota_addr.val), addr.begin());
    return addr;
  }
  // Fallbacks (not in a connection): whatever address id NimBLE holds, else MAC.
  if (ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.data(), nullptr) == 0)
    return addr;
  if (ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr.data(), nullptr) == 0)
    return addr;
  esp_read_mac(addr.data(), ESP_MAC_BT);
  std::reverse(addr.begin(), addr.end()); // esp_read_mac is big-endian; exchange is little-endian
  return addr;
}

// ---------------------------------------------------------------------------
// Response framing
// ---------------------------------------------------------------------------

void Switch2Pro::send_response(bool via_vibration_command, uint8_t cmd, uint8_t transport,
                               uint8_t sub, uint8_t byte4, uint8_t byte5, const uint8_t *payload,
                               size_t payload_len) {
  auto *response_char = via_vibration_command ? command_response2_ : command_response1_;
  if (response_char == nullptr)
    return;
  std::vector<uint8_t> out;
  out.reserve(RESPONSE_PREFIX_SIZE + COMMAND_HEADER_SIZE + payload_len);
  // Responses on the 0x001e channel are prefixed with a fixed 14-byte (zero)
  // report header, mirroring the command channel's vibration prefix; the console
  // reads the 8-byte response header at that offset.
  if (via_vibration_command)
    out.resize(RESPONSE_PREFIX_SIZE, 0x00);
  // Device->host header: [cmd, 0x01, transport, sub, byte4, byte5, 0x00, 0x00].
  out.insert(out.end(), {cmd, DIR_DEVICE_TO_HOST, transport, sub, byte4, byte5, 0x00, 0x00});
  if (payload != nullptr && payload_len > 0)
    out.insert(out.end(), payload, payload + payload_len);
  log_hex(via_vibration_command ? "rsp->0x001e" : "rsp->0x001a", out.data(), out.size());
  response_char->setValue(out.data(), out.size());
  response_char->notify();
}

void Switch2Pro::send_ack(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub) {
  // Bare BLE ACK: header only, byte4=0x10, byte5=0x78, no payload. Every Pro
  // Controller 2 Bluetooth response uses 0x10/0x78 (the 0x00/0xf8 form is the USB
  // transport); the captured init-sequence ACKs (e.g. 0x0a/0x02, 0x09/0x07) are
  // header-only with no trailing data.
  send_response(via_vibration_command, cmd, transport, sub, RSP_BYTE4_BT, RSP_BYTE5_BT, nullptr, 0);
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void Switch2Pro::on_command_write(bool via_vibration_command, const uint8_t *data, size_t len) {
  log_hex(via_vibration_command ? "cmd<-0x0016" : "cmd<-0x0014", data, len);
  // On the vibration+command channel (0x0016) the 8-byte command header follows a
  // fixed 33-byte vibration payload; skip it so the command id/subcommand parse
  // from the right offset. The command-only channel (0x0014) has no such prefix.
  if (via_vibration_command) {
    if (len < VIBRATION_COMMAND_PREFIX_SIZE + COMMAND_HEADER_SIZE) {
      logger_.warn("short vibration+command write ({} bytes)", len);
      return;
    }
    data += VIBRATION_COMMAND_PREFIX_SIZE;
    len -= VIBRATION_COMMAND_PREFIX_SIZE;
  }
  if (len < COMMAND_HEADER_SIZE) {
    logger_.warn("short command write ({} bytes)", len);
    return;
  }
  const auto cmd = static_cast<Command>(data[0]);
  const uint8_t transport = data[2];
  const uint8_t sub = data[3];
  const uint8_t *payload = data + COMMAND_HEADER_SIZE;
  const size_t payload_len = len - COMMAND_HEADER_SIZE;

  // Concise trace of the init/command flow (the full byte dump is also at DEBUG).
  logger_.debug("cmd 0x{:02x}/0x{:02x} ({}B data)", static_cast<uint8_t>(cmd), sub, payload_len);

  if (cmd == Command::PAIRING) {
    handle_pairing(via_vibration_command, transport, static_cast<PairingSub>(sub), payload,
                   payload_len);
  } else {
    handle_command(via_vibration_command, cmd, transport, sub, payload, payload_len);
  }
}

namespace {
// NVS-persisted bond: the paired console's address and the negotiated LTK, so
// the controller can reconnect / wake without re-running the 0x15 pairing.
constexpr const char *kNvsNamespace = "switch2pro";
constexpr const char *kNvsBondKey = "bond";
constexpr uint8_t kBondMagic = 0xB2;
struct StoredBond {
  uint8_t magic;
  uint8_t peer_type;
  uint8_t peer_val[6];  // NimBLE peer_id_addr (wire/little-endian order)
  uint8_t ltk[16];      // ltk_ (= A1 ^ B1), natural order
  uint8_t host_addr[6]; // console identity addr from 0x15/01 (embedded in reconnect adv)
};
} // namespace

void Switch2Pro::on_subscribe(NimBLECharacteristic *characteristic, uint16_t sub_value) {
  // The console enables input-report notifications on the Pro Controller 2 input
  // characteristic (0x000e) near the end of init; only then do we stream.
  if (characteristic == pro2_input_) {
    // Just flip the flag (atomic). The per-session counters/link-baseline are
    // reset by the streaming thread itself at the start of each streaming run
    // (see input_stream_loop) so they stay single-writer — no cross-thread race.
    const bool subscribed = (sub_value != 0);
    input_subscribed_.store(subscribed);
    logger_.info("input-report streaming {}", subscribed ? "ENABLED (0x000e)" : "disabled");
  }
}

// notify_in_flight_ / tx_completions_ remain as telemetry only (NOTIFY_TX count).
// Effective backpressure is msys1_headroom() — see send_input_report().

void Switch2Pro::on_notify_tx(NimBLECharacteristic *characteristic) {
  // A notification we queued has been transmitted; free its flow-control slot.
  if (characteristic == pro2_input_) {
    tx_completions_.fetch_add(1);
    last_tx_complete_us_.store(esp_timer_get_time());
    if (notify_in_flight_.load() > 0)
      notify_in_flight_.fetch_sub(1);
  }
}

std::string Switch2Pro::pool_stats() {
  // Walk every NimBLE mempool. The host mbuf (MSYS) pools are what a notify draws
  // from; if their free count trends to 0 (min_free==0), the host ran out of
  // buffers → ENOMEM originates host-side. If they stay healthy while we still
  // ENOMEM, the stall is downstream at the controller's ACL tx buffers.
  std::string s;
  struct os_mempool *mp = nullptr;
  struct os_mempool_info info;
  char line[80];
  while ((mp = os_mempool_info_get_next(mp, &info)) != nullptr) {
    if (info.omi_num_blocks <= 1) // skip tiny 1-block control pools — noise
      continue;
    snprintf(line, sizeof(line), "%s=%d/%d(min%d) ", info.omi_name[0] ? info.omi_name : "?",
             info.omi_num_free, info.omi_num_blocks, info.omi_min_free);
    s += line;
  }
  return s;
}

bool Switch2Pro::msys1_headroom() {
  struct os_mempool *mp = nullptr;
  struct os_mempool_info info;
  while ((mp = os_mempool_info_get_next(mp, &info)) != nullptr) {
    if (std::strcmp(info.omi_name, "msys_1") == 0)
      return (info.omi_num_blocks - info.omi_num_free) < kMaxOutstandingMbufs;
  }
  return true; // pool not found (shouldn't happen) — fail open, don't block the stream
}

void Switch2Pro::poll_conn_state() {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(active_conn_handle_, &desc) != 0)
    return;
  uint8_t tx_phy = 0, rx_phy = 0;
  ble_gap_read_le_phy(active_conn_handle_, &tx_phy, &rx_phy);
  if (desc.conn_itvl == last_itvl_ && desc.conn_latency == last_latency_ &&
      tx_phy == last_tx_phy_ && rx_phy == last_rx_phy_)
    return;
  last_itvl_ = desc.conn_itvl;
  last_latency_ = desc.conn_latency;
  last_tx_phy_ = tx_phy;
  last_rx_phy_ = rx_phy;
  // PHY: 1 = 1M, 2 = 2M, 3 = coded. Interval in 1.25 ms units, timeout in 10 ms.
  logger_.info("LINK CHANGE: itvl={:.2f}ms latency={} timeout={}ms tx_phy={} rx_phy={}",
               desc.conn_itvl * 1.25f, desc.conn_latency, desc.supervision_timeout * 10, tx_phy,
               rx_phy);
}

void Switch2Pro::input_stream_loop() {
  // Two streaming models (Config::continuous_streaming):
  //
  //  * continuous (default): send one report every connection interval with the
  //    counter incrementing every time, exactly like a real controller (the
  //    fresh-pair capture shows a real device streaming 62 Hz at 15 ms). Verified
  //    stable and lag-free on the C6-class chips.
  //  * on-change: notify only when the app's button/stick state changed since the
  //    last delivered report, plus a keepalive every kKeepaliveIntervals. A
  //    reduced-traffic fallback that partially masks the ESP32-S3 BTDM
  //    controller's tx-servicing bug (it stops draining tx ~3 s into any
  //    sustained encrypted stream — see README "Known issues").
  while (!stream_stop_.load()) {
    if (!input_subscribed_ || active_conn_handle_ == 0xffff || pro2_input_ == nullptr) {
      have_streamed_ = false; // (re)subscribe forces a fresh initial send
      idle_intervals_ = 0;
      stream_start_us_ = 0; // reset the wedge diagnostics for the next run
      wedge_reported_ = false;
      send_attempts_ = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    // NOTE: poll_conn_state() is NOT called here — it issues an HCI LE-Read-PHY
    // command, and running that at the 62 Hz stream rate floods the HCI path and
    // wedges the host's data-tx draining after ~3 s. It runs in the 500 ms
    // heartbeat below instead. ble_gap_conn_find() is local (no HCI) so it's cheap.
    uint32_t itvl_us = 15000;
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(active_conn_handle_, &desc) == 0 && desc.conn_itvl > 0)
      itvl_us = static_cast<uint32_t>(desc.conn_itvl) * 1250; // 1.25 ms units -> us

    // --- tx-wedge telemetry ---
    const int64_t now_us = esp_timer_get_time();
    if (stream_start_us_ == 0) { // first live tick of this streaming run
      // Reset the per-session state here (in the streaming thread) rather than in
      // the on_subscribe callback, so these stay single-writer.
      report_counter_ = 0; // fresh byte-0 sequence for the console to track
      motion_idx_ = 0;
      enomem_count_ = 0;
      backpressure_skips_ = 0;
      notify_in_flight_.store(0);
      tx_completions_.store(0);
      last_itvl_ = 0; // force a fresh LINK baseline log from poll_conn_state
      last_latency_ = 0xffff;
      last_tx_phy_ = 0;
      last_rx_phy_ = 0;
      stream_start_us_ = hb_last_us_ = now_us;
      last_tx_complete_us_.store(now_us);
      hb_last_completions_ = tx_completions_.load();
      hb_last_enomem_ = enomem_count_;
    }
    if (now_us - hb_last_us_ >= 500000) { // 500 ms heartbeat
      poll_conn_state();                  // interval/PHY-change log — 2 Hz, off the hot path
      const uint32_t c = tx_completions_.load(), e = enomem_count_;
      const float dt = (now_us - hb_last_us_) / 1e6f;
      const int64_t since_tx = now_us - last_tx_complete_us_.load();
      logger_.debug(
          "stream@{:.1f}s drain={:.0f}Hz(Δ{}) attempts={} skips={} enomemΔ={} inflight={} "
          "since_tx={:.0f}ms itvl={:.1f}ms | {}",
          (now_us - stream_start_us_) / 1e6f, (c - hb_last_completions_) / dt,
          c - hb_last_completions_, send_attempts_, backpressure_skips_, e - hb_last_enomem_,
          notify_in_flight_.load(), since_tx / 1000.0f, itvl_us / 1000.0f, pool_stats());
      hb_last_us_ = now_us;
      hb_last_completions_ = c;
      hb_last_enomem_ = e;
    }

    bool should_send = true;
    if (continuous_streaming_) {
      // Rate-halving probe: send only every Nth interval (N=1 → every interval).
      should_send = (interval_tick_++ % continuous_stream_divisor_) == 0;
    } else {
      bool changed;
      {
        std::lock_guard<std::mutex> lk(input_mutex_);
        changed = !have_streamed_ || input_report_.data() != last_streamed_.data();
      }
      if (changed || ++idle_intervals_ >= kKeepaliveIntervals)
        should_send = true, idle_intervals_ = 0;
      else
        should_send = false;
    }

    if (should_send && send_input_report() && !continuous_streaming_) {
      // Only advance the on-change baseline on an ACTUAL send, so a flow-control
      // skip retries the change next interval instead of dropping the input.
      std::lock_guard<std::mutex> lk(input_mutex_);
      last_streamed_ = input_report_;
      have_streamed_ = true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(itvl_us));
  }
}

bool Switch2Pro::send_input_report() {
  ++send_attempts_; // telemetry: every call the loop wanted to send
  // Real backpressure. The old notify_in_flight_/NOTIFY_TX cap is INERT here:
  // NOTIFY_TX fires at host->controller handoff, not over-air completion, so the
  // counter reads ~1 while mbufs actually pile up in the host tx queue until the
  // msys_1 pool hits 0 and every notify ENOMEMs (confirmed on-HW). Gate on the
  // pool's true un-drained count instead: only queue another report while the
  // backlog is under kMaxOutstandingMbufs. This rate-matches the link (like a real
  // controller sending one packet per connection event) and the pool never empties.
  if (!msys1_headroom()) {
    ++backpressure_skips_;
    return false;
  }

  // Latest stored app state + the protocol fields the app doesn't manage: byte 0
  // counter, byte 0x0B rumble flag, and the 40-byte IMU motion block (replayed
  // from a captured monotonic sequence when the console has enabled IMU).
  std::array<uint8_t, switch2::Pro2InputReport::SIZE> buf;
  {
    std::lock_guard<std::mutex> lk(input_mutex_);
    buf = input_report_.data();
  }
  buf[0] = report_counter_;
  buf[0x0b] = 0x38; // constant on a real controller (and the known-working emulator)
  if (enabled_features_ & switch2::FEATURE_IMU) {
    buf[0x0e] = 0x28; // motion data length (40) — always present once IMU is enabled
    if (stream_imu_motion_) {
      // Replay captured resting-motion frames. NOTE: the sequence loops (128
      // frames ≈ 2 s at 62 Hz), so its embedded timestamps jump backwards at the
      // wrap; the known-working emulator streams ALL-ZERO motion instead, which
      // the console accepts. Disable stream_imu_motion for zero-motion parity.
      const auto &blk = switch2::kMotionSequence[motion_idx_++ % switch2::kMotionSequence.size()];
      std::copy(blk.begin(), blk.end(), buf.begin() + 0x0f);
    } // else: motion block stays zeroed, like the known-working emulator
  }

  // Low-level notify so the exact rc is visible (esp-nimble-cpp's notify() hides it).
  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf.data(), buf.size());
  const bool mbuf_alloc_failed = (om == nullptr); // host MSYS pool exhausted vs downstream
  int rc = om ? ble_gatts_notify_custom(active_conn_handle_, pro2_input_->getHandle(), om)
              : BLE_HS_ENOMEM;
  if (rc == 0) {
    ++report_counter_; // +1 per delivered report, matching the real device
    notify_in_flight_.fetch_add(1);
  } else if (rc == BLE_HS_ENOMEM) {
    ++enomem_count_;
    if (!wedge_reported_) { // one-shot snapshot at the exact moment the stall begins
      wedge_reported_ = true;
      const int64_t now = esp_timer_get_time();
      const int64_t since_tx = now - last_tx_complete_us_.load();
      logger_.warn(
          "TX WEDGE: first ENOMEM at {:.1f}s after {} completions / {} attempts; "
          "source={} inflight={} since_last_tx={:.0f}ms → {}",
          stream_start_us_ ? (now - stream_start_us_) / 1e6f : 0.f, tx_completions_.load(),
          send_attempts_, mbuf_alloc_failed ? "HOST-mbuf-alloc" : "notify_custom(downstream)",
          notify_in_flight_.load(), since_tx / 1000.0f,
          since_tx < 50000 ? "completions still recent → console polling, our pool/pacing bug"
                           : "completions STALLED → tx drain stopped");
      logger_.warn("  pools @wedge: {}", pool_stats());
    }
  }
  const bool sent = (rc == 0);

  // Per-second stream health at DEBUG: reports delivered (txdone), tx-pool
  // deferrals (enomem), the counter, and the buttons on the wire.
  static uint32_t dbg_tick = 0;
  if ((dbg_tick++ % 66) == 0)
    logger_.debug("input stream: inflight={} txdone={} enomem={} ctr=0x{:02x} btn=[{:02x} {:02x} "
                  "{:02x}] 0x0b={:02x} feat={:02x}",
                  notify_in_flight_.load(), tx_completions_.load(), enomem_count_, buf[0], buf[2],
                  buf[3], buf[4], buf[0x0b], enabled_features_.load());
  return sent;
}

void Switch2Pro::inject_ltk(uint8_t peer_type, const uint8_t *peer_val_le) {
  struct ble_store_value_sec sec = {};
  sec.peer_addr.type = peer_type;
  std::copy(peer_val_le, peer_val_le + 6, sec.peer_addr.val);
  sec.key_size = 16;
  sec.ediv = 0; // no SMP key distribution — the console uses the LTK directly
  sec.rand_num = 0;
  // NimBLE hands ltk[] straight to the controller with no byte-swap, so it must
  // be in the same order the console's controller uses: ltk_ (= A1 ^ B1) as
  // computed. (The 0x03/0x07 "send pairing info" blob is this value reversed,
  // but that is just the on-wire transmission form, not the key order.)
  std::copy(ltk_.begin(), ltk_.end(), sec.ltk);
  sec.ltk_present = 1;
  sec.authenticated = 1;
  int rc = ble_store_write_our_sec(&sec);
  logger_.info("injected LTK into NimBLE store (rc={}) — ready for LL encryption", rc);
}

void Switch2Pro::inject_pairing_ltk() {
  struct ble_gap_conn_desc desc;
  if (active_conn_handle_ == 0xffff || ble_gap_conn_find(active_conn_handle_, &desc) != 0) {
    logger_.warn("cannot inject LTK: no active connection");
    return;
  }
  inject_ltk(desc.peer_id_addr.type, desc.peer_id_addr.val);
}

void Switch2Pro::save_bond() {
  struct ble_gap_conn_desc desc;
  if (active_conn_handle_ == 0xffff || ble_gap_conn_find(active_conn_handle_, &desc) != 0)
    return;
  StoredBond b{};
  b.magic = kBondMagic;
  b.peer_type = desc.peer_id_addr.type;
  std::copy(std::begin(desc.peer_id_addr.val), std::end(desc.peer_id_addr.val), b.peer_val);
  std::copy(ltk_.begin(), ltk_.end(), b.ltk);
  std::copy(host_addr_.begin(), host_addr_.end(), b.host_addr);
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) {
    logger_.error("save_bond: nvs_open failed");
    return;
  }
  nvs_set_blob(h, kNvsBondKey, &b, sizeof(b));
  nvs_commit(h);
  nvs_close(h);
  bond_peer_type_ = b.peer_type;
  std::copy(std::begin(b.peer_val), std::end(b.peer_val), bond_peer_val_.begin());
  logger_.info("saved bond to NVS (console addr + LTK)");
}

bool Switch2Pro::load_bond() {
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK)
    return false;
  StoredBond b{};
  size_t sz = sizeof(b);
  esp_err_t err = nvs_get_blob(h, kNvsBondKey, &b, &sz);
  nvs_close(h);
  if (err != ESP_OK || sz != sizeof(b) || b.magic != kBondMagic)
    return false;
  bond_peer_type_ = b.peer_type;
  std::copy(std::begin(b.peer_val), std::end(b.peer_val), bond_peer_val_.begin());
  std::copy(std::begin(b.ltk), std::end(b.ltk), ltk_.begin());
  std::copy(std::begin(b.host_addr), std::end(b.host_addr), host_addr_.begin());
  return true;
}

void Switch2Pro::handle_pairing(bool via_vibration_command, uint8_t transport, PairingSub sub,
                                const uint8_t *payload, size_t len) {
  // Pairing responses use byte4=0x10, byte5=0x78, and a payload that begins
  // with a 0x01 status byte (exact framing from ndeadly's captures).
  switch (sub) {
  case PairingSub::EXCHANGE_ADDRESSES: {
    // Request data: [0x00][count][addr1 (6, LE wire order)][addr2 (6)...]. addr1
    // is the console's STABLE IDENTITY address. Store it VERBATIM — the real
    // controller embeds exactly this in its reconnect/wake advertisement so the
    // console recognises the reconnect and grants the fast 5 ms interval. (We run
    // bonding=false, so NimBLE can't resolve the console's rotating private
    // connection address to its identity; this app-level exchange is where we get
    // the stable address.) Previously we byte-reversed payload[0..5], which read
    // the 0x00/count prefix as the address — garbage the console never recognises.
    if (len >= 8) {
      std::copy(payload + 2, payload + 8, host_addr_.begin());
      pairing_stage_ = 1;
      logger_.info(
          "pairing: stored console identity addr {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
          host_addr_[5], host_addr_[4], host_addr_[3], host_addr_[2], host_addr_[1], host_addr_[0]);
    } else {
      logger_.warn("pairing: exchange-addresses payload too short ({} bytes)", len);
    }
    // Reply: {0x01, 0x04, 0x01} + our BT address. The 0x04/0x01 prefix bytes
    // are as observed in captures; address byte order to be confirmed on HW.
    const auto addr = local_bt_address();
    std::array<uint8_t, 9> reply{0x01,    0x04,    0x01,    addr[0], addr[1],
                                 addr[2], addr[3], addr[4], addr[5]};
    send_response(via_vibration_command, 0x15, transport, 0x01, 0x10, 0x78, reply.data(),
                  reply.size());
    logger_.info("pairing: exchange addresses -> replied with our address {:02x} {:02x} {:02x} "
                 "{:02x} {:02x} {:02x} (little-endian)",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    break;
  }
  case PairingSub::EXCHANGE_KEYS: {
    // Request data is [0x00][A1 (16 bytes)] — skip the leading 0x00.
    if (pairing_stage_ >= 1 && len >= 17) {
      std::array<uint8_t, 16> a1{};
      std::copy(payload + 1, payload + 17, a1.begin());
      ltk_ = PairingCrypto::derive_ltk(a1);
      pairing_stage_ = 2;
    } else {
      logger_.warn("pairing: exchange-keys out of order or short (stage={}, len={})",
                   pairing_stage_, len);
    }
    // Reply: {0x01} + fixed controller key B1.
    std::array<uint8_t, 17> reply{0x01};
    std::copy(CONTROLLER_KEY_B1.begin(), CONTROLLER_KEY_B1.end(), reply.begin() + 1);
    send_response(via_vibration_command, 0x15, transport, 0x04, 0x10, 0x78, reply.data(),
                  reply.size());
    logger_.info("pairing: exchange keys -> LTK derived, replied B1");
    break;
  }
  case PairingSub::CONFIRM_LTK: {
    // Request data is [0x00][A2 challenge (16 bytes)] — skip the leading 0x00.
    std::array<uint8_t, 16> b2{};
    if (pairing_stage_ >= 2 && len >= 17) {
      std::array<uint8_t, 16> a2{};
      std::copy(payload + 1, payload + 17, a2.begin());
      b2 = PairingCrypto::confirm(ltk_, a2);
      pairing_stage_ = 3;
    } else {
      logger_.warn("pairing: confirm out of order or short (stage={}, len={})", pairing_stage_,
                   len);
    }
    // Reply: {0x01} + B2 = AES-128-ECB(rev(LTK), rev(A2)).
    std::array<uint8_t, 17> reply{0x01};
    std::copy(b2.begin(), b2.end(), reply.begin() + 1);
    send_response(via_vibration_command, 0x15, transport, 0x02, 0x10, 0x78, reply.data(),
                  reply.size());
    logger_.info("pairing: confirm -> replied B2");
    break;
  }
  case PairingSub::FINALISE: {
    // Only finalise if address exchange, key derivation, and LTK confirmation all
    // completed in order (stage 3). Otherwise an out-of-order/malformed peer could
    // mark us paired and persist an all-zero/partial bond, sending future boots
    // into reconnect mode with a useless bond. Reject without replying/persisting.
    if (pairing_stage_ < 3) {
      logger_.warn("pairing: FINALISE rejected — handshake incomplete (stage={})", pairing_stage_);
      break;
    }
    static constexpr std::array<uint8_t, 1> reply{0x01};
    send_response(via_vibration_command, 0x15, transport, 0x03, 0x10, 0x78, reply.data(),
                  reply.size());
    paired_ = true;
    logger_.info("pairing: finalised — bonded");
    // Right after finalise the console starts standard BLE link-layer encryption
    // using the LTK we just negotiated (there is no SMP key distribution). Inject
    // the LTK into NimBLE's security store so the controller can answer the
    // console's LTK request; without it encryption fails and the console drops us.
    inject_pairing_ltk();
    // Persist {console address, LTK} so we can reconnect/wake after a reboot
    // without re-running the pairing exchange.
    save_bond();
    reconnect_mode_ = true;
    break;
  }
  default:
    logger_.debug("pairing: unhandled subcommand 0x{:02x}", static_cast<uint8_t>(sub));
    break;
  }
}

void Switch2Pro::handle_command(bool via_vibration_command, Command cmd, uint8_t transport,
                                uint8_t sub, const uint8_t *payload, size_t len) {
  switch (cmd) {
  case Command::FLASH_READ: {
    // Request payload: [len, 0x7e, 0x00, 0x00, addr(4 LE)]. Reply echoes
    // len+addr then the data from the simulated flash.
    if (len < 8) {
      send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
      break;
    }
    const uint8_t read_len = payload[0];
    const uint32_t addr =
        static_cast<uint32_t>(payload[4]) | (static_cast<uint32_t>(payload[5]) << 8) |
        (static_cast<uint32_t>(payload[6]) << 16) | (static_cast<uint32_t>(payload[7]) << 24);
    // Response payload: [len(4 LE)][addr(4 LE)][data]. There is NO status byte —
    // the flash contents follow the address directly (the leading 0x01 seen at
    // 0x13000 is real flash data, not a status).
    std::vector<uint8_t> reply(8u + read_len, 0);
    reply[0] = read_len;
    reply[4] = payload[4];
    reply[5] = payload[5];
    reply[6] = payload[6];
    reply[7] = payload[7];
    simulated_flash_read(addr, read_len, reply.data() + 8);
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                  RSP_BYTE5_BT, reply.data(), reply.size());
    logger_.debug("flash read {} bytes @ 0x{:06x}", read_len, addr);
    break;
  }
  case Command::UNKNOWN_07: {
    // Init handshake: response is the header plus a single zero data byte.
    static constexpr std::array<uint8_t, 1> d = {0x00};
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                  RSP_BYTE5_BT, d.data(), d.size());
    break;
  }
  case Command::UNKNOWN_16: {
    // Init handshake: response is the header plus 24 zero data bytes.
    static constexpr std::array<uint8_t, 24> d = {};
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                  RSP_BYTE5_BT, d.data(), d.size());
    break;
  }
  case Command::UNKNOWN_11: {
    // Late-init handshake. The response is subcommand-specific and each must
    // match a real Pro Controller 2 exactly, or the console keeps re-probing and
    // never enables input streaming:
    //   0x11/0x03 -> a fixed 29-byte blob (looks like report/sensor config).
    //   0x11/0x01 -> {0x01,0,0,0}.
    // A header-only ACK (or the wrong subcommand's blob) stalls the init.
    static constexpr std::array<uint8_t, 29> blob03 = {
        0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8,
        0x9c, 0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c, 0x41, 0x58, 0xa0, 0x0b, 0x41};
    static constexpr std::array<uint8_t, 4> blob01 = {0x01, 0x00, 0x00, 0x00};
    if (sub == 0x01)
      send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                    RSP_BYTE5_BT, blob01.data(), blob01.size());
    else
      send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                    RSP_BYTE5_BT, blob03.data(), blob03.size());
    break;
  }
  case Command::UNKNOWN_18: {
    // Late-init probe. 0x18/0x01 expects a fixed 8-byte device blob; a header-only
    // ACK leaves the console unsatisfied and it keeps probing instead of
    // activating input.
    if (sub == 0x01) {
      static constexpr std::array<uint8_t, 8> d = {0x00, 0x00, 0x40, 0xf0, 0x00, 0x00, 0x60, 0x00};
      send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                    RSP_BYTE5_BT, d.data(), d.size());
    } else {
      send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    }
    break;
  }
  case Command::FEATURE_SELECT: {
    // Track which features the console enables so our input report can reflect
    // them (see notify_input_report). 0x02 = set mask, 0x04 = enable (within the
    // mask), 0x05 = disable. The console typically enables mask 0x2f
    // (buttons+sticks+IMU+rumble).
    if (len >= 1) {
      if (sub == 0x02) {
        feature_mask_ = payload[0];
        // On RECONNECT the console only sets the mask (0x0c/02) and never sends
        // 0x0c/04 (enable) — it expects the controller to have persisted its
        // enabled features. So treat set-mask as enabling those features; on a
        // fresh pair the 0x0c/04 that follows is then just idempotent.
        enabled_features_ = payload[0];
      } else if (sub == 0x04)
        enabled_features_ |= static_cast<uint8_t>(payload[0] & feature_mask_);
      else if (sub == 0x05)
        enabled_features_ &= static_cast<uint8_t>(~payload[0]);
    }
    // Response is the header plus 4 zero data bytes (both 0x0c/0x02 and 0x0c/0x04).
    static constexpr std::array<uint8_t, 4> d = {};
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                  RSP_BYTE5_BT, d.data(), d.size());
    break;
  }
  case Command::FIRMWARE_INFO: {
    // 0x10/0x01 response: [fw ver major.minor.micro (3)][controller type (1)]
    // [BT patch ver (3)][pad][DSP ver (3, updated Pro only)]. Byte 3 is the
    // controller type: 0x02 = Pro Controller (the doc's example uses 0x01 =
    // JoyCon (R), which must NOT be used here — the console cross-checks this
    // against the VID/PID and GATT and rejects a controller whose firmware type
    // disagrees with the rest of its identity). Bytes 8-10 are the DSP (audio)
    // firmware version: a real un-updated controller reports ff ff ff (no DSP),
    // but since we expose the headset-audio characteristics we report a valid
    // DSP version so the identity is consistent (updated firmware). Bytes match
    // the known-working zhantss emulator (fw 2.1.4 | Pro | BT 12.0.0 | pad |
    // DSP 2.3.0) — a controller identity the console demonstrably accepts for
    // sustained streaming, and current enough not to trigger the update path.
    static constexpr std::array<uint8_t, 12> fw = {0x02, 0x01, 0x04, 0x02, 0x0c, 0x00,
                                                   0x00, 0x00, 0x00, 0x02, 0x03, 0x00};
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, 0x10, 0x78,
                  fw.data(), fw.size());
    break;
  }
  case Command::FIRMWARE_UPDATE:
    // ACK without offering an update, to suppress the console's update prompt.
    // TODO(hw): confirm the exact bytes the console needs to skip the prompt.
    send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    break;
  case Command::NFC:
    // Command 0x01 (NFC). During init the console probes 0x01/0x0c and expects a
    // fixed 4-byte reply; other subcommands are ACKed for now.
    if (sub == 0x0c) {
      static constexpr std::array<uint8_t, 4> d = {0x61, 0x12, 0x50, 0x0d};
      send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, RSP_BYTE4_BT,
                    RSP_BYTE5_BT, d.data(), d.size());
    } else {
      send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    }
    break;
  case Command::INIT:
  case Command::PLAYER_LEDS:
  case Command::VIBRATION:
  case Command::BATTERY:
  default:
    // Acknowledge so the console's init state machine advances. Command-specific
    // payloads (battery level, etc.) are refined in later work.
    send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    break;
  }
}

} // namespace espp
