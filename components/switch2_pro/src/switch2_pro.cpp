#include "switch2_pro.hpp"

namespace espp {

using namespace switch2;

/// Forwards NimBLE characteristic writes on the command channels into the owner.
class CommandCallbacks : public NimBLECharacteristicCallbacks {
public:
  explicit CommandCallbacks(Switch2Pro *owner)
      : owner_(owner) {}
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo & /*conn*/) override {
    auto value = characteristic->getValue();
    owner_->on_command_write(value.data(), value.size());
  }

private:
  Switch2Pro *owner_;
};

namespace {
// One shared callbacks instance for the command characteristics. Lives for the
// life of the process (NimBLE keeps a raw pointer to it).
CommandCallbacks *g_command_callbacks = nullptr;
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

  if (g_command_callbacks == nullptr)
    g_command_callbacks = new CommandCallbacks(this);
  command_->setCallbacks(g_command_callbacks);
  vibration_command_->setCallbacks(g_command_callbacks);

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

void Switch2Pro::on_command_write(const uint8_t *data, size_t len) {
  if (len < COMMAND_HEADER_SIZE) {
    logger_.warn("short command write ({} bytes)", len);
    return;
  }
  const auto command = static_cast<Command>(data[0]);
  const uint8_t subcommand = data[3];
  const uint8_t *payload = data + COMMAND_HEADER_SIZE;
  const size_t payload_len = len - COMMAND_HEADER_SIZE;

  switch (command) {
  case Command::PAIRING:
    handle_pairing(static_cast<PairingSub>(subcommand), payload, payload_len);
    break;
  default:
    // Init / flash-read / feature-select / LEDs / firmware-update are staged in
    // the next milestone; log so captures against a real console are legible.
    logger_.debug("unhandled command 0x{:02x} sub 0x{:02x} ({} payload bytes)",
                  static_cast<uint8_t>(command), subcommand, payload_len);
    break;
  }
}

void Switch2Pro::handle_pairing(PairingSub sub, const uint8_t *payload, size_t len) {
  switch (sub) {
  case PairingSub::EXCHANGE_ADDRESSES: {
    // Console sends its host BD_ADDR(es); remember the first (byte-reversed).
    if (len >= 6) {
      for (size_t i = 0; i < 6; ++i)
        host_addr_[i] = payload[5 - i];
    }
    logger_.info("pairing: exchange addresses");
    // TODO(milestone-2): reply with our own address in a 0x15/0x01 response.
    break;
  }
  case PairingSub::EXCHANGE_KEYS: {
    // Console sends A1; LTK = A1 ⊕ B1. We reply with the fixed B1.
    if (len >= 16) {
      std::array<uint8_t, 16> a1{};
      std::copy(payload, payload + 16, a1.begin());
      ltk_ = PairingCrypto::derive_ltk(a1);
      logger_.info("pairing: exchange keys, LTK derived");
    }
    // TODO(milestone-2): reply with CONTROLLER_KEY_B1 in a 0x15/0x04 response.
    break;
  }
  case PairingSub::CONFIRM_LTK: {
    // Console sends challenge A2; reply B2 = AES-ECB(rev(LTK), rev(A2)).
    if (len >= 16) {
      std::array<uint8_t, 16> a2{};
      std::copy(payload, payload + 16, a2.begin());
      const auto b2 = PairingCrypto::confirm(ltk_, a2);
      (void)b2; // TODO(milestone-2): send b2 back in a 0x15/0x02 response.
      logger_.info("pairing: confirm challenge");
    }
    break;
  }
  case PairingSub::FINALISE:
    paired_ = true;
    logger_.info("pairing: finalised — bonded");
    // TODO(milestone-2): persist {host_addr_, ltk_} to NVS for reconnect/wake.
    break;
  default:
    logger_.debug("pairing: unhandled subcommand 0x{:02x}", static_cast<uint8_t>(sub));
    break;
  }
}

void Switch2Pro::notify_response(const std::vector<uint8_t> &response) {
  if (command_response2_ == nullptr)
    return;
  command_response2_->setValue(response.data(), response.size());
  command_response2_->notify();
}

} // namespace espp
