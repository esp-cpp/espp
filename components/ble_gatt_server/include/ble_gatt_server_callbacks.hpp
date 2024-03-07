#pragma once

#include "NimBLEDevice.h"

namespace espp {
class BleGattServer;

/// Class for handling GATT server callbacks, as required by NimBLE
/// \note This class is not intended to be used directly by the user
class BleGattServerCallbacks : public NimBLEServerCallbacks {
public:
  virtual void onConnect(NimBLEServer *server, NimBLEConnInfo &conn_info) override;
  virtual void onDisconnect(NimBLEServer *server, NimBLEConnInfo &conn_info, int reason) override;
  virtual void onAuthenticationComplete(NimBLEConnInfo &conn_info) override;
  virtual uint32_t onPassKeyRequest() override;
  virtual bool onConfirmPIN(uint32_t pass_key) override;

protected:
  friend class BleGattServer;
  BleGattServerCallbacks(BleGattServer *server)
      : server_(server) {}
  BleGattServer *server_{nullptr};
};
} // namespace espp
