#include "ble_gatt_server_callbacks.hpp"
#include "ble_gatt_server.hpp"

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

using namespace espp;

void BleGattServerCallbacks::onConnect(NimBLEServer *server, NimBLEConnInfo &conn_info) {
  if (server_ && server_->callbacks_.connect_callback) {
    server_->callbacks_.connect_callback(conn_info);
  }
}

void BleGattServerCallbacks::onDisconnect(NimBLEServer *server, NimBLEConnInfo &conn_info,
                                          int reason) {
  if (server_ && server_->callbacks_.disconnect_callback) {
    // convert the reason to a espp::BleGattServer::DisconnectReason. For more
    // information, see
    // https://mynewt.apache.org/latest/network/ble_hs/ble_hs_return_codes.html
    BleGattServer::DisconnectReason disconnect_reason = BleGattServer::DisconnectReason::UNKNOWN;
    switch (reason) {
    case BLE_HS_ETIMEOUT_HCI:
    case (0x200 + BLE_ERR_CONN_SPVN_TMO):
    case (0x200 + BLE_ERR_CONN_ACCEPT_TMO):
      disconnect_reason = BleGattServer::DisconnectReason::TIMEOUT;
      break;
    case BLE_HS_EAUTHEN:
    case BLE_HS_EAUTHOR:
    case BLE_HS_EENCRYPT:
    case (0x200 + BLE_ERR_AUTH_FAIL):
      disconnect_reason = BleGattServer::DisconnectReason::AUTHENTICATION_FAILURE;
      break;
    case (0x0200 + BLE_ERR_CONN_TERM_MIC):
      disconnect_reason = BleGattServer::DisconnectReason::CONNECTION_TERMINATED;
      break;
    case (0x0200 + BLE_ERR_REM_USER_CONN_TERM):
      disconnect_reason = BleGattServer::DisconnectReason::REMOTE_USER_TERMINATED;
      break;
    case (0x0200 + BLE_ERR_RD_CONN_TERM_RESRCS):
    case (0x0200 + BLE_ERR_RD_CONN_TERM_PWROFF):
      disconnect_reason = BleGattServer::DisconnectReason::REMOTE_DEVICE_TERMINATED;
      break;
    case (0x0200 + BLE_ERR_CONN_TERM_LOCAL):
      disconnect_reason = BleGattServer::DisconnectReason::LOCAL_USER_TERMINATED;
      break;
    default:
      fmt::print("Unknown disconnect reason: {}\n", reason);
      disconnect_reason = BleGattServer::DisconnectReason::UNKNOWN;
      break;
    }
    server_->callbacks_.disconnect_callback(conn_info, disconnect_reason);
  }
}

void BleGattServerCallbacks::onAuthenticationComplete(NimBLEConnInfo &conn_info) {
  if (server_ && server_->callbacks_.authentication_complete_callback) {
    server_->callbacks_.authentication_complete_callback(conn_info);
  }
}
uint32_t BleGattServerCallbacks::onPassKeyDisplay() {
  if (server_ && server_->callbacks_.get_passkey_callback) {
    return server_->callbacks_.get_passkey_callback();
  } else {
    return NimBLEDevice::getSecurityPasskey();
  }
}
void BleGattServerCallbacks::onConfirmPassKey(NimBLEConnInfo &conn_info, uint32_t pass_key) {
  if (server_ && server_->callbacks_.confirm_passkey_callback) {
    server_->callbacks_.confirm_passkey_callback(conn_info, pass_key);
  } else {
    NimBLEDevice::injectConfirmPasskey(conn_info, pass_key == NimBLEDevice::getSecurityPasskey());
  }
}

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

void BleGattServerAdvertisingCallbacks::onStopped(NimBLEExtAdvertising *pAdv, int reason,
                                                  uint8_t inst_id) {
  if (server_ && server_->callbacks_.advertisement_stopped_callback) {
    server_->callbacks_.advertisement_stopped_callback(pAdv, reason, inst_id);
  }
}

void BleGattServerAdvertisingCallbacks::onScanRequest(NimBLEExtAdvertising *pAdv, uint8_t inst_id,
                                                      NimBLEAddress addr) {
  if (server_ && server_->callbacks_.scan_request_callback) {
    server_->callbacks_.scan_request_callback(pAdv, inst_id, addr);
  }
}

#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
