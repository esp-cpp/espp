#pragma once

#include <sdkconfig.h>

#include <ctime>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

#include "base_component.hpp"

#include "battery_service.hpp"
#include "ble_appearances.hpp"
#include "ble_gatt_server_callbacks.hpp"
#include "device_info_service.hpp"

namespace espp {
/// BLE GATT Server
/// This class is responsible for creating and managing the BLE GATT server.
/// It is responsible for handling the connection and disconnection of clients.
/// It is also responsible for handling the pairing process.
/// It also manages the different services that are part of the GATT server.
///
/// \section ble_gatt_server_ex1 BLE GATT Server Example
/// \snippet ble_gatt_server_example.cpp ble gatt server example
class BleGattServer : public BaseComponent {
public:
#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  typedef NimBLEExtAdvertisement AdvertisedData;
#else
  typedef NimBLEAdvertisementData AdvertisedData;
#endif

  /// @brief Callback for when a device connects to the GATT server.
  typedef std::function<void(NimBLEConnInfo &)> connect_callback_t;

  /// @brief Callback for when a device disconnects from the GATT server.
  typedef std::function<void(NimBLEConnInfo &)> disconnect_callback_t;

  /// @brief Callback for when a device completes authentication.
  /// @param conn_info The connection information for the device.
  typedef std::function<void(NimBLEConnInfo &)> authentication_complete_callback_t;

  /// @brief Callback for the passkey.
  /// @return The passkey for the device.
  typedef std::function<uint32_t(void)> get_passkey_callback_t;

  /// @brief Callback for confirming the passkey.
  /// @param passkey The passkey for the device.
  /// @return Whether the passkey is confirmed.
  typedef std::function<bool(uint32_t)> confirm_passkey_callback_t;

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Callback for when advertising is stopped.
  /// This callback is called when advertising is stopped.
  /// @param advertising Pointer to the advertising object.
  /// @param reason The reason for stopping advertising.
  /// @param instance The advertising instance that was stopped.
  typedef std::function<void(NimBLEExtAdvertising *, int, uint8_t)>
      advertisement_stopped_callback_t;

  /// @brief Callback for when a scan request is received.
  /// This callback is called when a scan request is received.
  /// @param advertising Pointer to the advertising object.
  /// @param instance The advertising instance that received the scan request.
  /// @param address The address of the device that sent the scan request.
  typedef std::function<void(NimBLEExtAdvertising *, uint8_t, NimBLEAddress)>
      scan_request_callback_t;
#endif // CONFIG_BT_NIMBLE_EXT_ADV

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Callback for when advertising is complete.
  /// This callback is called when advertising is complete.
  /// @param advertising Pointer to the advertising object.
  /// @note This is called when advertising is complete, not when the device is
  ///       actually advertising. It will not be called if the advertisement
  ///       duration is 0 (i.e. no timeout).
  typedef std::function<void(NimBLEAdvertising *)> advertisement_complete_callback_t;
#endif // !CONFIG_BT_NIMBLE_EXT_ADV

  /// @brief Convert an interval in milliseconds to units of 0.625 ms.
  /// @param interval_ms The interval in milliseconds.
  /// @return The interval in units of 0.625 ms.
  static constexpr uint16_t interval_ms_to_units(uint16_t interval_ms) {
    return interval_ms * 8 / 5;
  }

  /// @brief Convert an interval in units of 0.625 ms to milliseconds.
  /// @param interval_units The interval in units of 0.625 ms.
  /// @return The interval in milliseconds.
  static constexpr uint16_t interval_units_to_ms(uint16_t interval_units) {
    return interval_units * 5 / 8;
  }

  /// @brief Callbacks for the GATT server.
  struct Callbacks {
    connect_callback_t connect_callback =
        nullptr; ///< Callback for when a device connects to the GATT server.
    disconnect_callback_t disconnect_callback =
        nullptr; ///< Callback for when a device disconnects from the GATT server.
    authentication_complete_callback_t authentication_complete_callback =
        nullptr; ///< Callback for when a device completes authentication.
    get_passkey_callback_t get_passkey_callback =
        nullptr; ///< Callback for getting the passkey. If not set, will simply
                 ///  return NimBLEDevice::getSecurityPasskey().
    confirm_passkey_callback_t confirm_passkey_callback =
        nullptr; ///< Callback for confirming the passkey. If not set, will
                 ///  simply compare the passkey to NimBLEDevice::getSecurityPasskey().
#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    advertisement_complete_callback_t advertisement_complete_callback =
        nullptr; ///< Callback for when advertising is complete. NOTE: this is
                 ///  called when advertising is complete, not when the device
                 ///  is actually advertising. It will not be called if
                 ///  the advertisement duration is 0 (i.e. no timeout)
#endif           // !CONFIG_BT_NIMBLE_EXT_ADV
#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    advertisement_stopped_callback_t advertisement_stopped_callback =
        nullptr; ///< Callback for when advertising is stopped.
    scan_request_callback_t scan_request_callback =
        nullptr; ///< Callback for when a scan request is received.
#endif           // CONFIG_BT_NIMBLE_EXT_ADV
  };

  /// @brief Configuration for the GATT server.
  struct Config {
    Callbacks callbacks; ///< The callbacks for the GATT server.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The verbosity of the logger_.
  };

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Advertising parameters for the device.
  /// This struct contains the advertising parameters for the device.
  /// @see start_advertising
  /// @note This struct is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used. Otherwise, use the
  ///       NimBLEExtAdvertisement class.
  struct AdvertisingParameters {
    bool connectable = true;             ///< Whether the device should be connectable
    uint16_t min_interval_ms = 20;       ///< Minimum advertising interval. NOTE: this
                                         ///< is in milliseconds, but will be converted
                                         ///< to units of 0.625 ms. If the value is
                                         ///< not a multiple of 0.625 ms, it will be
                                         ///< rounded to the nearest multiple and a log
                                         ///< message will be printed.
    uint16_t max_interval_ms = 40;       ///< Maximum advertising interval. NOTE: this
                                         ///< is in milliseconds, but will be converted
                                         ///< to units of 0.625 ms. If the value is
                                         ///< not a multiple of 0.625 ms, it will be
                                         ///< rounded to the nearest multiple and a log
                                         ///< message will be printed.
    bool include_tx_power = false;       ///< Whether to include the TX power level
    bool scan_response = false;          ///< Whether the device should include scan response data
    bool scan_request_whitelist = false; ///< Whether the device should use the
                                         ///< white list when scanning
    bool connect_whitelist = false;      ///< Whether the device should use the white
                                         ///< list when connecting
    uint32_t duration_ms = 0;            ///< Advertising duration (in ms, 0 for no timeout)
    NimBLEAddress *directed_address = nullptr; ///< Address to direct advertising to, if any
  };
#endif // !CONFIG_BT_NIMBLE_EXT_ADV

  /// Constructor for the GATT server.
  BleGattServer()
      : BaseComponent("BleGattServer", espp::Logger::Verbosity::WARN) {}

  /// Constructor
  /// @param config The configuration for the GATT server.
  explicit BleGattServer(const Config &config)
      : BaseComponent("BleGattServer", config.log_level)
      , callbacks_(config.callbacks) {}

  /// Destructor
  ~BleGattServer() { deinit(); }

  /// Get whether the GATT server is connected to any clients.
  /// @return Whether the GATT server is connected to any clients.
  bool is_connected() const {
    if (!server_) {
      return false;
    }
    return server_->getConnectedCount() > 0;
  }

  /// Set the callbacks for the GATT server.
  /// @param callbacks The callbacks for the GATT server.
  void set_callbacks(const Callbacks &callbacks) { callbacks_ = callbacks; }

  /// Initialize the GATT server
  /// This method creates the GATT server and sets the callbacks to this class.
  void init(const std::string &device_name) {
    logger_.info("Initializing GATT server with device name: '{}'", device_name);
    // create the device
    NimBLEDevice::init(device_name);

    // create the server
    logger_.info("Creating server");
    server_ = NimBLEDevice::createServer();
    if (!server_) {
      logger_.error("Failed to create server");
      return;
    }

    // set the server callbacks
    server_->setCallbacks(new BleGattServerCallbacks(this));

    logger_.info("Creating device info service");
    // create the device info service
    device_info_service_.init(server_);

    logger_.info("Creating battery service");
    // create the battery service
    battery_service_.init(server_);
  }

  /// Deinitialize the GATT server
  /// This method deletes the server and all associated objects.
  /// It also invalidates any references/pointers to the server.
  /// @note After calling this method, any references/pointers to the server
  ///       and any created services/characteristics will be invalid.
  /// @note This method should only be called after NimBLEDevice::deinit(true)
  ///       has been called, since that will free the memory used by the server.
  /// @note This method will also deinitialize the device info and battery
  ///       services.
  void deinit() {
    // if true, deletes all server/advertising/scan/client objects which
    // invalidates any references/pointers to them
    bool clear_all = true;
    NimBLEDevice::deinit(clear_all);
    // now deinitialize the services
    device_info_service_.deinit();
    battery_service_.deinit();
  }

  /// Start the services
  /// This method starts the device info and battery services.
  void start_services() {
    device_info_service_.start();
    battery_service_.start();
  }

  /// Start the server
  /// This method starts the GATT server.
  /// This method must be called after the server has been initialized, and
  /// after any services / characteristics have been added to the server.
  void start() {
    if (!server_) {
      logger_.error("Server not created");
      return;
    }
    server_->start();
  }

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Set whether to advertise on disconnect
  /// @param advertise_on_disconnect Whether to advertise on disconnect
  /// @note This method is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used. Otherwise, you will
  ///       have to manually start advertising after disconnecting.
  void set_advertise_on_disconnect(bool advertise_on_disconnect) {
    if (!server_) {
      logger_.error("Server not created");
      return;
    }
    // set whether to advertise on disconnect
    server_->advertiseOnDisconnect(advertise_on_disconnect);
  }
#endif // !CONFIG_BT_NIMBLE_EXT_ADV

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Set the advertisement data for the device.
  /// @param advertising_data The advertising data for the device.
  /// @param instance The advertising instance to set the data for.
  void set_advertisement_data(const AdvertisedData &advertising_data, uint8_t instance = 0);

  /// Set the scan response data for the device.
  /// @param scan_response_data The scan response data for the device.
  /// @param instance The advertising instance to set the data for.
  void set_scan_response_data(const AdvertisedData &scan_response_data, uint8_t instance = 0);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Set the advertisement data for the device.
  /// @param advertising_data The advertising data for the device.
  void set_advertisement_data(const AdvertisedData &advertising_data);

  /// Set the scan response data for the device.
  /// @param scan_response_data The scan response data for the device.
  void set_scan_response_data(const AdvertisedData &scan_response_data);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

  /// Stop advertising
  /// This method stops advertising.
  void stop_advertising() {
    if (!server_) {
      logger_.error("Server not created");
      return;
    }
    auto advertising = NimBLEDevice::getAdvertising();
    if (!advertising) {
      logger_.error("Advertising not created");
      return;
    }
    logger_.info("Stopping advertising");
    advertising->stop();
  }

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Stop advertising
  /// This method stops advertising.
  /// @param instance The advertising instance to stop.
  void stop_advertising(uint8_t instance);

  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param duration_ms The duration of the advertising in milliseconds. If 0,
  ///                    the advertising will not timeout. If non-zero, the
  ///                    advertising will stop after the specified duration.
  /// @param instance The advertising instance to start.
  void start_advertising(uint32_t duration_ms = 0, uint8_t instance = 0);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param params The advertising parameters for the device.
  /// @note This method is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used. Otherwise, use the
  ///       NimBLEExtAdvertisement class for advertising and setting the
  ///       advertising parameters.
  void start_advertising(const AdvertisingParameters &params);

  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param duration_ms The duration of the advertising in milliseconds.
  ///                   If 0, the advertising will not timeout. If non-zero,
  ///                   the advertising will stop after the specified duration.
  /// @param directed_address The address to direct advertising to, if any.
  /// @note This method is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used.
  void start_advertising(uint32_t duration_ms = 0, NimBLEAddress *directed_address = nullptr);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

  /// @brief Get the GATT server.
  /// @return The GATT server.
  NimBLEServer *server() const { return server_; }

  /// @brief Get the device info service.
  DeviceInfoService &device_info_service() { return device_info_service_; }

  /// @brief Get the battery service.
  BatteryService &battery_service() { return battery_service_; }

  /// @brief Set the name of the device.
  /// @param device_name The name of the device.
  void set_device_name(const std::string &device_name) { NimBLEDevice::setDeviceName(device_name); }

  /// @brief Set the passkey for the device.
  /// @param passkey The passkey for the device.
  void set_passkey(uint32_t passkey) { NimBLEDevice::setSecurityPasskey(passkey); }

  /// Set the security settings for the device.
  /// @param bonding Whether to use bonding.
  /// @param mitm Man-in-the-middle protection.
  /// @param secure Whether to use secure connections.
  void set_security(bool bonding, bool mitm, bool secure) {
    NimBLEDevice::setSecurityAuth(bonding, mitm, secure);
  }

  /// Set the IO capabilities for the device.
  /// @param io_capabilities The IO capabilities for the device.
  /// @see BLE_HS_IO_NO_INPUT_OUTPUT
  /// @see BLE_HS_IO_DISPLAY_ONLY
  /// @see BLE_HS_IO_DISPLAY_YESNO
  /// @see BLE_HS_IO_KEYBOARD_ONLY
  /// @see BLE_HS_IO_NO_INPUT_OUTPUT
  /// @see BLE_HS_IO_KEYBOARD_DISPLAY
  void set_io_capabilities(uint8_t io_capabilities) {
    NimBLEDevice::setSecurityIOCap(io_capabilities);
  }

  /// Set the initial key distribution for the device.
  /// @param key_distribution The initial key distribution for the device.
  /// @see BLE_SM_PAIR_KEY_DIST_ENC
  /// @see BLE_SM_PAIR_KEY_DIST_ID
  void set_init_key_distribution(uint8_t key_distribution) {
    NimBLEDevice::setSecurityInitKey(key_distribution);
  }

  /// Set the response key distribution for the device.
  /// @param key_distribution The response key distribution for the device.
  /// @see BLE_SM_PAIR_KEY_DIST_ENC
  /// @see BLE_SM_PAIR_KEY_DIST_ID
  void set_resp_key_distribution(uint8_t key_distribution) {
    NimBLEDevice::setSecurityRespKey(key_distribution);
  }

  /// Get the paired devices
  /// @return The paired devices as a vector of Addresses.
  std::vector<NimBLEAddress> get_paired_devices() {
    std::vector<NimBLEAddress> paired_devices;
    auto num_bonds = NimBLEDevice::getNumBonds();
    for (int i = 0; i < num_bonds; i++) {
      auto bond_addr = NimBLEDevice::getBondedAddress(i);
      paired_devices.push_back(bond_addr);
    }
    return paired_devices;
  }

  /// Get the connected devices
  /// @return The connected devices as a vector of Addresses.
  std::vector<NimBLEAddress> get_connected_devices() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<NimBLEAddress> connected_devices;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerIDInfo(peer_id);
      connected_devices.push_back(peer.getAddress());
    }
    return connected_devices;
  }

  /// Disconnect from all devices
  /// This method disconnects from all devices that are currently connected.
  /// @return The Addresses of the devices that were disconnected from.
  std::vector<NimBLEAddress> disconnect_all() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<NimBLEAddress> disconnected_devices;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerIDInfo(peer_id);
      disconnected_devices.push_back(peer.getAddress());
      server_->disconnect(peer_id);
    }
    return disconnected_devices;
  }

protected:
  friend class BleGattServerCallbacks;
  friend class BleGattServerAdvertisingCallbacks;

  Callbacks callbacks_{};                 ///< The callbacks for the GATT server.
  NimBLEServer *server_{nullptr};         ///< The GATT server.
  DeviceInfoService device_info_service_; ///< The device info service.
  BatteryService battery_service_;        ///< The battery service.
};
} // namespace espp

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
// for easy printing of the advertising parameters using libfmt
template <> struct fmt::formatter<espp::BleGattServer::AdvertisingParameters> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::BleGattServer::AdvertisingParameters &advertising_params,
              FormatContext &ctx) {
    if (advertising_params.directed_address) {
      return fmt::format_to(
          ctx.out(),
          "AdvertisingParameters{{min_interval_ms: {}, max_interval_ms: {}, include_tx_power: {}, "
          "scan_response: {}, duration_ms: {}, directed_address: {}}}",
          advertising_params.min_interval_ms, advertising_params.max_interval_ms,
          advertising_params.include_tx_power, advertising_params.scan_response,
          advertising_params.duration_ms, advertising_params.directed_address->toString());
    }
    return fmt::format_to(
        ctx.out(),
        "AdvertisingParameters{{min_interval_ms: {}, max_interval_ms: {}, include_tx_power: {}, "
        "scan_response: {}, duration_ms: {}, directed_address: {}}}",
        advertising_params.min_interval_ms, advertising_params.max_interval_ms,
        advertising_params.include_tx_power, advertising_params.scan_response,
        advertising_params.duration_ms, fmt::ptr(advertising_params.directed_address));
  }
};
#endif // !CONFIG_BT_NIMBLE_EXT_ADV

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
