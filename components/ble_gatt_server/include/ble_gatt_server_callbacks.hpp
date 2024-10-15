#pragma once

#include <sdkconfig.h>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

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
  virtual uint32_t onPassKeyDisplay() override;
  virtual void onConfirmPIN(NimBLEConnInfo &conn_info, uint32_t pass_key) override;

protected:
  friend class BleGattServer;
  explicit BleGattServerCallbacks(BleGattServer *server)
      : server_(server) {}
  BleGattServer *server_{nullptr};
};

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
/// Class for handling GATT server advertising callbacks, as required by NimBLE
/// \note This class is not intended to be used directly by the user
/// \note This class is only available if CONFIG_BT_NIMBLE_EXT_ADV is enabled
class BleGattServerAdvertisingCallbacks : public NimBLEExtAdvertisingCallbacks {
public:
  virtual void onStopped(NimBLEExtAdvertising *pAdv, int reason, uint8_t inst_id) override;
  virtual void onScanRequest(NimBLEExtAdvertising *pAdv, uint8_t inst_id,
                             NimBLEAddress addr) override;

protected:
  friend class BleGattServer;
  explicit BleGattServerAdvertisingCallbacks(BleGattServer *server)
      : server_(server) {}
  BleGattServer *server_{nullptr};
};
#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
