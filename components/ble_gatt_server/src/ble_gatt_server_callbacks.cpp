#include "ble_gatt_server_callbacks.hpp"
#include "ble_gatt_server.hpp"

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

void espp::BleGattServerCallbacks::onConnect(NimBLEServer *server, NimBLEConnInfo &conn_info) {
  if (server_ && server_->callbacks_.connect_callback) {
    server_->callbacks_.connect_callback(conn_info);
  }
}

void espp::BleGattServerCallbacks::onDisconnect(NimBLEServer *server, NimBLEConnInfo &conn_info,
                                                int reason) {
  if (server_ && server_->callbacks_.disconnect_callback) {
    server_->callbacks_.disconnect_callback(conn_info);
  }
}

void espp::BleGattServerCallbacks::onAuthenticationComplete(NimBLEConnInfo &conn_info) {
  if (server_ && server_->callbacks_.authentication_complete_callback) {
    server_->callbacks_.authentication_complete_callback(conn_info);
  }
}
uint32_t espp::BleGattServerCallbacks::onPassKeyRequest() {
  if (server_ && server_->callbacks_.get_passkey_callback) {
    return server_->callbacks_.get_passkey_callback();
  } else {
    return NimBLEDevice::getSecurityPasskey();
  }
}
bool espp::BleGattServerCallbacks::onConfirmPIN(uint32_t pass_key) {
  if (server_ && server_->callbacks_.confirm_passkey_callback) {
    return server_->callbacks_.confirm_passkey_callback(pass_key);
  } else {
    return pass_key == NimBLEDevice::getSecurityPasskey();
  }
}

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
