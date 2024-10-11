#include "gfps.hpp"

static espp::Logger logger({.tag = "GFPS BLE", .level = espp::gfps::LOG_LEVEL});

#include "host/ble_hs.h"

static std::vector<uint8_t> REMOTE_PUBLIC_KEY(64, 0);

static const nearby_platform_BleInterface *g_ble_interface = nullptr;
static const nearby_platform_BtInterface *g_bt_interface = nullptr;

static espp::gfps::notify_callback_t g_gfps_notify_cb = nullptr;

// external API

void espp::gfps::init(const espp::gfps::Config &config) {
  // store anything we need from the config
  g_gfps_notify_cb = config.notify;

  // Calls into google/nearby/embedded to initialize the nearby framework, using
  // the platform specific implementation of the nearby API which is in the
  // embedded component.
  nearby_fp_client_Init(nullptr);
  int advertisement_mode =
      NEARBY_FP_ADVERTISEMENT_DISCOVERABLE | NEARBY_FP_ADVERTISEMENT_PAIRING_UI_INDICATOR;
  nearby_fp_client_SetAdvertisement(advertisement_mode);
}

void espp::gfps::deinit() {
  // nearby_fp_client_Deinit();
}

const nearby_platform_BleInterface *espp::gfps::get_ble_interface() { return g_ble_interface; }

const nearby_platform_BtInterface *espp::gfps::get_bt_interface() { return g_bt_interface; }

uint32_t espp::gfps::get_model_id() { return CONFIG_GFPS_MODEL_ID; }

void espp::gfps::set_remote_public_key(const std::vector<uint8_t> &public_key) {
  REMOTE_PUBLIC_KEY = public_key;
}

void espp::gfps::set_remote_public_key(const uint8_t *public_key, size_t length) {
  if (length != REMOTE_PUBLIC_KEY.size()) {
    logger.error("Invalid public key length: {}", length);
    return;
  }
  for (size_t i = 0; i < length; i++) {
    REMOTE_PUBLIC_KEY[i] = public_key[i];
  }
}

std::vector<uint8_t> espp::gfps::get_remote_public_key() {
  return std::vector<uint8_t>(REMOTE_PUBLIC_KEY.begin(), REMOTE_PUBLIC_KEY.end());
}

#if CONFIG_BT_NIMBLE_ENABLED
int espp::gfps::ble_gap_event_handler(ble_gap_event *event, void *arg) {
  struct ble_gap_conn_desc desc;
  switch (event->type) {
  case BLE_GAP_EVENT_ENC_CHANGE: {
    logger.info("BLE_GAP_EVENT_ENC_CHANGE");
    ble_gap_conn_find(event->enc_change.conn_handle, &desc);
    auto addr = NimBLEAddress(desc.peer_ota_addr);
    bool pairing_succeeded = event->enc_change.status == 0;
    if (pairing_succeeded) {
      logger.info("Pairing succeeded with {}", addr.toString());
      if (g_bt_interface) {
        g_bt_interface->on_paired(uint64_t(addr));
      }
    } else {
      logger.error("Pairing failed with {}", addr.toString());
      if (g_bt_interface) {
        g_bt_interface->on_pairing_failed(uint64_t(addr));
      }
    }
    // set the IO capability of the device (GFPS uses the passkey through a
    // sidechannel; when the PASSKEY event triggers it then causes other
    // devices that pair later to show a passkey. We don't want this, and
    // ensuring we set the mode to IO_CAP_NONE fixes that)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    break;
  }
  default:
    break;
  }
  return 0;
}
#endif // CONFIG_BT_NIMBLE_ENABLED

/// Nearby implementation

// Gets BLE address.
uint64_t nearby_platform_GetBleAddress() {
  // get the mac address of the radio
#if CONFIG_BT_NIMBLE_ENABLED
  // explicitly get the random address here, since the GetPublicAddress function
  // will call NimBLEDevice::getAddress() which will return the public address
  uint64_t address = 0;
  auto rc = ble_hs_id_copy_addr(BLE_ADDR_RANDOM, (uint8_t *)&address, nullptr);
  if (rc != 0) {
    logger.error("Failed to get ble address");
    return 0;
  }
  logger.debug("radio mac address: {:#x}", address);
  return address;
#else
  return 0;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

// Sets BLE address. Returns address after change, which may be different than
// requested address.
//
// address - BLE address to set.
uint64_t nearby_platform_SetBleAddress(uint64_t address) {
  // Since we don't really have to set the requested address, just return the
  // current address
  return nearby_platform_GetBleAddress();
}

#ifdef NEARBY_FP_HAVE_BLE_ADDRESS_ROTATION
// Rotates BLE address to a random resolvable private address (RPA). Returns
// address after change.
uint64_t nearby_platform_RotateBleAddress() {
  logger.warn("RotateBleAddress not implemented");
  // TODO: implement
  return 0;
}
#endif /* NEARBY_FP_HAVE_BLE_ADDRESS_ROTATION */

// Gets the PSM - Protocol and Service Mulitplexor - assigned to Fast Pair's
// Message Stream.
// To support Message Stream for BLE devices, Fast Pair will build and maintain
// a BLE L2CAP channel for sending and receiving messages. The PSM can be
// dynamic or fixed.
// Returns a 16 bit PSM number or a negative value on error. When a valid PSM
// number is returned, the device must be ready to accept L2CAP connections.
int32_t nearby_platform_GetMessageStreamPsm() {
  logger.warn("GetMessageStreamPsm not implemented");
  // TODO: implement
  return -1;
}

// Sends a notification to the connected GATT client.
//
// peer_address   - Address of peer.
// characteristic - Characteristic UUID
// message        - Message buffer to transmit.
// length         - Length of message in buffer.
nearby_platform_status nearby_platform_GattNotify(uint64_t peer_address,
                                                  nearby_fp_Characteristic characteristic,
                                                  const uint8_t *message, size_t length) {
  logger.debug("GattNotify: peer_address={:#x}, characteristic={}, length={}", peer_address,
               characteristic, length);
  if (!g_gfps_notify_cb) {
    logger.error("No notify callback set");
    return kNearbyStatusError;
  }
  bool success = g_gfps_notify_cb(characteristic, message, length);
  if (!success) {
    logger.error("notify failed");
    return kNearbyStatusError;
  }
  return kNearbyStatusOK;
}

// Sets the Fast Pair advertisement payload and starts advertising at a given
// interval.
//
// payload  - Advertising data.
// length   - Length of data.
// interval - Advertising interval code.
nearby_platform_status nearby_platform_SetAdvertisement(const uint8_t *payload, size_t length,
                                                        nearby_fp_AvertisementInterval interval) {
  logger.info("Setting advertisement, interval code: {}", (int)interval);

  // For information of the contents of the payload, see:
  // https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf
  // The payload is a length byte, followed by the data type and data bytes, in
  // sequence

  // TODO: do we really need to do anything with the advertising interval?
  // set the advertising interval
  return kNearbyStatusOK;
}

// Initializes BLE
//
// ble_interface - GATT read and write callbacks structure.
nearby_platform_status nearby_platform_BleInit(const nearby_platform_BleInterface *ble_interface) {
  logger.info("Initializing BLE");
  // save the ble_interface (on_gatt_write and on_gatt_read callbacks) for later
  g_ble_interface = ble_interface;
  return kNearbyStatusOK;
}

// Returns Fast Pair Model Id.
uint32_t nearby_platform_GetModelId() { return CONFIG_GFPS_MODEL_ID; }

// Returns tx power level.
int8_t nearby_platform_GetTxLevel() {
#if CONFIG_BT_NIMBLE_ENABLED
  auto tx_power = NimBLEDevice::getPower();
  return tx_power;
#else
  return 0;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

// Returns public BR/EDR address.
// On a BLE-only device, return the public identity address.
uint64_t nearby_platform_GetPublicAddress() {
#if CONFIG_BT_NIMBLE_ENABLED
  auto address = NimBLEDevice::getAddress();
  logger.info("GetPublicAddress: {}", address.toString());
  return uint64_t(address);
#else
  return 0;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

// Initializes BT
nearby_platform_status nearby_platform_BtInit(const nearby_platform_BtInterface *bt_interface) {
  logger.info("Initializing BT (unused, but necessary)");
  // save the bt_interface (on pairing failed / succeeded) for later
  g_bt_interface = bt_interface;
  return kNearbyStatusOK;
}

// Returns the secondary identity address.
// Some devices, such as ear-buds, can advertise two identity addresses. In this
// case, the Seeker pairs with each address separately but treats them as a
// single logical device set.
// Return 0 if this device does not have a secondary identity address.
uint64_t nearby_platform_GetSecondaryPublicAddress() {
  logger.warn("GetSecondaryPublicAddress not implemented");
  return 0;
}

// Returns passkey used during pairing
uint32_t nearby_platfrom_GetPairingPassKey() {
#if CONFIG_BT_NIMBLE_ENABLED
  auto passkey = NimBLEDevice::getSecurityPasskey();
  logger.info("GetPairingPassKey: {}", passkey);
  return passkey;
#else
  return 0;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

// Provides the passkey received from the remote party.
// The system should compare local and remote party and accept/decline pairing
// request accordingly.
//
// passkey - Passkey
void nearby_platform_SetRemotePasskey(uint32_t passkey) {
  logger.info("SetRemotePasskey: {}", passkey);
#if CONFIG_BT_NIMBLE_ENABLED
  bool accept = passkey == NimBLEDevice::getSecurityPasskey();
  if (accept) {
    logger.info("Accepting pairing request, passkey matches");
  } else {
    logger.error("Declining pairing request, passkey does not match");
  }
  // get the connection info for the remote party (assume only one, so first
  // index)
  auto conn_info = NimBLEDevice::getServer()->getPeerInfo(0);
  // Now actually respond to the pairing request
  NimBLEDevice::injectConfirmPIN(conn_info, accept);
#endif // CONFIG_BT_NIMBLE_ENABLED
}

// Sends a pairing request to the Seeker
//
// remote_party_br_edr_address - BT address of peer.
nearby_platform_status nearby_platform_SendPairingRequest(uint64_t remote_party_br_edr_address) {
  logger.warn("SendPairingRequest not implemented");
  // TODO: implement
  return kNearbyStatusOK;
}

// Switches the device capabilities field back to default so that new
// pairings continue as expected.
nearby_platform_status nearby_platform_SetDefaultCapabilities() {
  logger.info("SetDefaultCapabilities - setting LE_AUTH_BOND and IO_CAP_NONE");
#if CONFIG_BT_NIMBLE_ENABLED
  bool bonding = true;
  bool mitm = false;
  bool secure = true;
  NimBLEDevice::setSecurityAuth(bonding, mitm, secure);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif // CONFIG_BT_NIMBLE_ENABLED
  return kNearbyStatusOK;
}

// Switches the device capabilities field to Fast Pair required configuration:
// DisplayYes/No so that `confirm passkey` pairing method will be used.
nearby_platform_status nearby_platform_SetFastPairCapabilities() {
  logger.info("SetFastPairCapabilities - setting LE_AUTH_REQ_SC_MITM_BOND and IO_CAP_IO");
#if CONFIG_BT_NIMBLE_ENABLED
  bool bonding = true;
  bool mitm = true;
  bool secure = true;
  NimBLEDevice::setSecurityAuth(bonding, mitm, secure);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif // CONFIG_BT_NIMBLE_ENABLED
  return kNearbyStatusOK;
}

// Sets null-terminated device name string in UTF-8 encoding
//
// name - Zero terminated string name of device.
nearby_platform_status nearby_platform_SetDeviceName(const char *name) {
  logger.info("SetDeviceName: {}", name);
#if CONFIG_BT_NIMBLE_ENABLED
  NimBLEDevice::setDeviceName(name);
#endif // CONFIG_BT_NIMBLE_ENABLED
  return kNearbyStatusOK;
}

// Gets null-terminated device name string in UTF-8 encoding
// pass buffer size in char, and get string length in char.
//
// name   - Buffer to return name string.
// length - On input, the size of the name buffer.
//          On output, returns size of name in buffer.
nearby_platform_status nearby_platform_GetDeviceName(char *name, size_t *length) {
  logger.warn("GetDeviceName not implemented");
  // TODO: implement
  return kNearbyStatusOK;
}

// Returns true if the device is in pairing mode (either fast-pair or manual).
bool nearby_platform_IsInPairingMode() {
  // TODO: implement
  return true;
}
