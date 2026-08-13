#include "switch2_pro.hpp"

#include <cstdio>
#include <string>

#include "esp_mac.h"

#include "switch2_pro_flash.hpp"

namespace espp {

using namespace switch2;

/// Forwards NimBLE characteristic writes on a command channel into the owner,
/// tagged with which channel (0x0014 vs the vibration+command 0x0016) so the
/// response goes out on the matching notify characteristic.
class CommandCallbacks : public NimBLECharacteristicCallbacks {
public:
  CommandCallbacks(Switch2Pro *owner, bool via_vibration_command)
      : owner_(owner)
      , via_vibration_command_(via_vibration_command) {}
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo & /*conn*/) override {
    auto value = characteristic->getValue();
    owner_->on_command_write(via_vibration_command_, value.data(), value.size());
  }

private:
  Switch2Pro *owner_;
  bool via_vibration_command_;
};

namespace {
// Per-channel callbacks instances. Live for the process (NimBLE keeps raw
// pointers to them).
CommandCallbacks *g_command_cb = nullptr;           // 0x0014 -> response 0x001a
CommandCallbacks *g_vibration_command_cb = nullptr; // 0x0016 -> response 0x001e
} // namespace

bool Switch2Pro::init() {
  // The pairing crypto is the load-bearing part; verify it against the golden
  // vector up front so a broken build fails loudly rather than at the console.
  if (PairingCrypto::self_test()) {
    logger_.info("pairing crypto self-test passed");
  } else {
    logger_.error("pairing crypto self-test FAILED — pairing will be rejected");
    return false;
  }

  configure_callbacks();
  if (!ble_gatt_server_.init(device_name_)) {
    logger_.error("failed to init BLE GATT server");
    return false;
  }
  configure_security();
  if (!build_gatt()) {
    logger_.error("failed to build GATT services");
    return false;
  }
  ble_gatt_server_.start_services();
  ble_gatt_server_.start();
  start_advertising(/*wake=*/false);
  logger_.info("Switch2Pro advertising as '{}'", device_name_);
  return true;
}

void Switch2Pro::configure_security() {
  // The console performs its own app-level pairing over the command channel and
  // will drop a peer that initiates BLE SMP. We enable bonding + legacy (not LE
  // Secure Connections) and never initiate security ourselves.
  ble_gatt_server_.set_security(/*bonding=*/true, /*mitm=*/false, /*secure=*/false);
  ble_gatt_server_.set_io_capabilities(BLE_HS_IO_NO_INPUT_OUTPUT);
}

void Switch2Pro::configure_callbacks() {
  BleGattServer::Callbacks callbacks;
  callbacks.connect_callback = [this](NimBLEConnInfo &info) {
    // The connection interval right after connect is the key diagnostic: the
    // Switch 2 drives 5 ms (interval == 4 units). If a controller can't hold
    // that, the console typically disconnects with a supervision timeout.
    logger_.info("connected: peer={} interval={:.2f}ms supervision={}ms latency={}",
                 info.getAddress().toString(), info.getConnInterval() * 1.25f,
                 info.getConnTimeout() * 10, info.getConnLatency());
  };
  callbacks.disconnect_callback = [this](NimBLEConnInfo &info, BleGattServer::DisconnectReason r) {
    logger_.warn("disconnected: peer={} reason={} (paired={})", info.getAddress().toString(), r,
                 paired_);
    paired_ = false;
    start_advertising(/*wake=*/false);
  };
  ble_gatt_server_.set_callbacks(callbacks);
}

void Switch2Pro::log_hex(const char *prefix, const uint8_t *data, size_t len) {
  if (logger_.get_verbosity() > Logger::Verbosity::DEBUG)
    return;
  std::string hex;
  hex.reserve(len * 3);
  char tmp[4];
  for (size_t i = 0; i < len; ++i) {
    std::snprintf(tmp, sizeof(tmp), "%02x ", data[i]);
    hex += tmp;
  }
  logger_.debug("{} [{}]: {}", prefix, len, hex);
}

bool Switch2Pro::build_gatt() {
  auto *server = ble_gatt_server_.server();
  if (server == nullptr)
    return false;

  // Service 1 (purpose not fully understood; created so the handle map matches
  // what the console observed from a real controller).
  auto *svc1 = server->createService(NimBLEUUID(SERVICE1_UUID));
  svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_281_UUID), NIMBLE_PROPERTY::READ);
  svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_282_UUID), NIMBLE_PROPERTY::WRITE);
  svc1->createCharacteristic(NimBLEUUID(SERVICE1_CHR_283_UUID), NIMBLE_PROPERTY::READ);

  // Service 2 — the main HID-like service.
  auto *svc2 = server->createService(NimBLEUUID(SERVICE2_UUID));
  common_input_ = svc2->createCharacteristic(NimBLEUUID(COMMON_INPUT_UUID),
                                             NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pro2_input_ = svc2->createCharacteristic(NimBLEUUID(PRO2_INPUT_UUID),
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  svc2->createCharacteristic(NimBLEUUID(VIBRATION_UUID), NIMBLE_PROPERTY::WRITE_NR);
  command_ = svc2->createCharacteristic(NimBLEUUID(COMMAND_UUID), NIMBLE_PROPERTY::WRITE_NR);
  vibration_command_ =
      svc2->createCharacteristic(NimBLEUUID(VIBRATION_COMMAND_UUID), NIMBLE_PROPERTY::WRITE_NR);
  svc2->createCharacteristic(NimBLEUUID(FIRMWARE_UPDATE_UUID), NIMBLE_PROPERTY::WRITE);
  command_response1_ =
      svc2->createCharacteristic(NimBLEUUID(COMMAND_RESPONSE1_UUID), NIMBLE_PROPERTY::NOTIFY);
  command_response2_ =
      svc2->createCharacteristic(NimBLEUUID(COMMAND_RESPONSE2_UUID), NIMBLE_PROPERTY::NOTIFY);

  if (g_command_cb == nullptr)
    g_command_cb = new CommandCallbacks(this, /*via_vibration_command=*/false);
  if (g_vibration_command_cb == nullptr)
    g_vibration_command_cb = new CommandCallbacks(this, /*via_vibration_command=*/true);
  command_->setCallbacks(g_command_cb);
  vibration_command_->setCallbacks(g_vibration_command_cb);

  svc1->start();
  svc2->start();
  return true;
}

void Switch2Pro::start_advertising(bool wake, const std::array<uint8_t, 6> &host_addr) {
  auto mfr = MANUFACTURER_DATA_DISCOVERY;
  if (wake) {
    mfr[MANUFACTURER_WAKE_FLAG_OFFSET] = WAKE_FLAG;
    for (size_t i = 0; i < 6; ++i) // host address is byte-reversed on the wire
      mfr[MANUFACTURER_HOST_ADDR_OFFSET + i] = host_addr[5 - i];
  }

  BleGattServer::AdvertisedData adv_data;
  adv_data.setFlags(BLE_HS_ADV_F_DISC_GEN);
  adv_data.setName(device_name_);
  adv_data.setManufacturerData(mfr.data(), mfr.size());
  ble_gatt_server_.set_advertisement_data(adv_data);

  BleGattServer::AdvertisingParameters params{};
  params.connectable = true;
  ble_gatt_server_.start_advertising(params);
}

std::array<uint8_t, 6> Switch2Pro::local_bt_address() const {
  std::array<uint8_t, 6> addr{};
  esp_read_mac(addr.data(), ESP_MAC_BT);
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
  out.reserve(COMMAND_HEADER_SIZE + payload_len);
  // Device->host header: [cmd, 0x01, transport, sub, byte4, byte5, 0x00, 0x00].
  out.insert(out.end(), {cmd, DIR_DEVICE_TO_HOST, transport, sub, byte4, byte5, 0x00, 0x00});
  if (payload != nullptr && payload_len > 0)
    out.insert(out.end(), payload, payload + payload_len);
  log_hex(via_vibration_command ? "rsp->0x001e" : "rsp->0x001a", out.data(), out.size());
  response_char->setValue(out.data(), out.size());
  response_char->notify();
}

void Switch2Pro::send_ack(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub) {
  // Header-only ACK: byte4=0x00, byte5=0xf8, payload {0x01,0,0,0} (matches the
  // captured 0x03/0x0d init ACK).
  static constexpr std::array<uint8_t, 4> kAckPayload = {0x01, 0x00, 0x00, 0x00};
  send_response(via_vibration_command, cmd, transport, sub, 0x00, 0xf8, kAckPayload.data(),
                kAckPayload.size());
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void Switch2Pro::on_command_write(bool via_vibration_command, const uint8_t *data, size_t len) {
  log_hex(via_vibration_command ? "cmd<-0x0016" : "cmd<-0x0014", data, len);
  if (len < COMMAND_HEADER_SIZE) {
    logger_.warn("short command write ({} bytes)", len);
    return;
  }
  const auto cmd = static_cast<Command>(data[0]);
  const uint8_t transport = data[2];
  const uint8_t sub = data[3];
  const uint8_t *payload = data + COMMAND_HEADER_SIZE;
  const size_t payload_len = len - COMMAND_HEADER_SIZE;

  if (cmd == Command::PAIRING) {
    handle_pairing(via_vibration_command, transport, static_cast<PairingSub>(sub), payload,
                   payload_len);
  } else {
    handle_command(via_vibration_command, cmd, transport, sub, payload, payload_len);
  }
}

void Switch2Pro::handle_pairing(bool via_vibration_command, uint8_t transport, PairingSub sub,
                                const uint8_t *payload, size_t len) {
  // Pairing responses use byte4=0x10, byte5=0x78, and a payload that begins
  // with a 0x01 status byte (exact framing from ndeadly's captures).
  switch (sub) {
  case PairingSub::EXCHANGE_ADDRESSES: {
    if (len >= 6) {
      for (size_t i = 0; i < 6; ++i) // console host address, byte-reversed
        host_addr_[i] = payload[5 - i];
    }
    // Reply: {0x01, 0x04, 0x01} + our BT address. The 0x04/0x01 prefix bytes
    // are as observed in captures; address byte order to be confirmed on HW.
    const auto addr = local_bt_address();
    std::array<uint8_t, 9> reply{0x01,    0x04,    0x01,    addr[0], addr[1],
                                 addr[2], addr[3], addr[4], addr[5]};
    send_response(via_vibration_command, 0x15, transport, 0x01, 0x10, 0x78, reply.data(),
                  reply.size());
    logger_.info("pairing: exchange addresses -> replied with our address");
    break;
  }
  case PairingSub::EXCHANGE_KEYS: {
    if (len >= 16) {
      std::array<uint8_t, 16> a1{};
      std::copy(payload, payload + 16, a1.begin());
      ltk_ = PairingCrypto::derive_ltk(a1);
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
    std::array<uint8_t, 16> b2{};
    if (len >= 16) {
      std::array<uint8_t, 16> a2{};
      std::copy(payload, payload + 16, a2.begin());
      b2 = PairingCrypto::confirm(ltk_, a2);
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
    static constexpr std::array<uint8_t, 1> reply{0x01};
    send_response(via_vibration_command, 0x15, transport, 0x03, 0x10, 0x78, reply.data(),
                  reply.size());
    paired_ = true;
    logger_.info("pairing: finalised — bonded");
    // TODO(milestone-4): persist {host_addr_, ltk_} to NVS for reconnect/wake.
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
    std::vector<uint8_t> reply(8u + read_len, 0);
    reply[0] = read_len;
    reply[4] = payload[4];
    reply[5] = payload[5];
    reply[6] = payload[6];
    reply[7] = payload[7];
    simulated_flash_read(addr, read_len, reply.data() + 8);
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, 0x10, 0x78,
                  reply.data(), reply.size());
    logger_.debug("flash read {} bytes @ 0x{:06x}", read_len, addr);
    break;
  }
  case Command::FEATURE_SELECT:
    if (sub == 0x02 && len >= 1)
      feature_mask_ = payload[0]; // set feature mask
    send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    break;
  case Command::FIRMWARE_INFO: {
    // Captured reply for 0x10/0x01.
    static constexpr std::array<uint8_t, 12> fw = {0x01, 0x00, 0x0e, 0x01, 0x0c, 0x00,
                                                   0x00, 0x00, 0xff, 0xff, 0xff, 0xff};
    send_response(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub, 0x10, 0x78,
                  fw.data(), fw.size());
    break;
  }
  case Command::FIRMWARE_UPDATE:
    // ACK without offering an update, to suppress the console's update prompt.
    // TODO(hw): confirm the exact bytes the console needs to skip the prompt.
    send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    break;
  case Command::INIT:
  case Command::PLAYER_LEDS:
  case Command::VIBRATION:
  case Command::BATTERY:
  case Command::NFC:
  default:
    // Acknowledge so the console's init state machine advances. Command-specific
    // payloads (battery level, etc.) are refined in later work.
    send_ack(via_vibration_command, static_cast<uint8_t>(cmd), transport, sub);
    break;
  }
}

} // namespace espp
