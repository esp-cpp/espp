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
  /// @brief Disconnect reasons for the GATT server.
  /// This enum represents the different reasons for disconnection that will be
  /// passed to the disconnect callback.
  ///
  /// This enum is a simplification of the NimBLE disconnect reasons, and is
  /// meant to provide a more user-friendly way to handle disconnections.
  /// For more information about the possible reasons for disconnection, see
  /// https://mynewt.apache.org/latest/network/ble_hs/ble_hs_return_codes.html.
  enum class DisconnectReason : uint8_t {
    UNKNOWN,                  ///< Unknown reason for disconnection
    TIMEOUT,                  ///< Disconnected due to timeout
    CONNECTION_TERMINATED,    ///< Disconnected due to the connection being terminated
    REMOTE_USER_TERMINATED,   ///< Disconnected due to the remote user terminating the connection
    REMOTE_DEVICE_TERMINATED, ///< Disconnected due to the remote device terminating the connection
                              ///< (low resources or power off)
    LOCAL_USER_TERMINATED,    ///< Disconnected due to the local user terminating the connection
    AUTHENTICATION_FAILURE,   ///< Disconnected due to an authentication failure
  };

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  typedef NimBLEExtAdvertisement AdvertisedData;
#else
  typedef NimBLEAdvertisementData AdvertisedData;
#endif

  /// @brief Callback for when a device connects to the GATT server.
  /// @param conn_info The connection information for the device.
  typedef std::function<void(NimBLEConnInfo &)> connect_callback_t;

  /// @brief Callback for when a device disconnects from the GATT server.
  /// @param conn_info The connection information for the device.
  /// @param reason The reason for the disconnection.
  typedef std::function<void(NimBLEConnInfo &, DisconnectReason reason)> disconnect_callback_t;

  /// @brief Callback for when a device completes authentication.
  /// @param conn_info The connection information for the device.
  typedef std::function<void(const NimBLEConnInfo &)> authentication_complete_callback_t;

  /// @brief Callback to retrieve the passkey for the device.
  /// @return The passkey for the device.
  typedef std::function<uint32_t(void)> get_passkey_callback_t;

  /// @brief Callback for confirming the passkey.
  /// @param conn_info The connection information for the device.
  /// @param passkey The passkey for the device.
  typedef std::function<void(const NimBLEConnInfo &conn_info, uint32_t)> confirm_passkey_callback_t;

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// @brief Callback for when advertising is stopped.
  /// This callback is called when advertising is stopped.
  /// @param advertising Pointer to the advertising object.
  /// @param reason The reason for stopping advertising.
  /// @param instance The advertising instance that was stopped.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
  typedef std::function<void(NimBLEExtAdvertising *, int, uint8_t)>
      advertisement_stopped_callback_t;

  /// @brief Callback for when a scan request is received.
  /// This callback is called when a scan request is received.
  /// @param advertising Pointer to the advertising object.
  /// @param instance The advertising instance that received the scan request.
  /// @param address The address of the device that sent the scan request.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
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
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is not enabled.
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
        nullptr; ///< Callback for getting the passkey.
                 /// @note If not provided, will simply return
                 /// NimBLEDevice::getSecurityPasskey().
    confirm_passkey_callback_t confirm_passkey_callback =
        nullptr; ///< Callback for confirming the passkey.
                 /// @note Within this function or some point after this call,
                 /// the user should call
                 /// NimBLEDevice::injectConfirmPIN(conn_info, true/false) to
                 /// confirm or reject the passkey.
                 /// @note If not provided, will simply call
                 /// NimBLEDevice::injectConfirmPIN(conn_info, passkey ==
                 /// NimBLEDevice::getSecurityPasskey()).
#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    advertisement_complete_callback_t advertisement_complete_callback =
        nullptr; ///< Callback for when advertising is complete.
                 ///  @note This is called when advertising is complete, not
                 ///        when the device is actually advertising. It will not
                 ///        be called if the advertisement duration is 0 (i.e.
                 ///        no timeout)
                 ///  @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV
                 ///        is not enabled.
#endif           // !CONFIG_BT_NIMBLE_EXT_ADV
#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
    advertisement_stopped_callback_t advertisement_stopped_callback =
        nullptr; ///< Callback for when advertising is stopped.
                 ///  @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV
                 ///        is enabled.
    scan_request_callback_t scan_request_callback =
        nullptr; ///< Callback for when a scan request is received.
                 ///  @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV
                 ///        is enabled.
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
  /// @note This struct is only available when CONFIG_BT_NIMBLE_EXT_ADV is not
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
  /// It also initializes the device info and battery services.
  /// @param device_name The name of the device.
  /// @note This method must be called before creating any other services or
  ///       characteristics.
  /// @note This method must be called before starting the server.
  /// @return Whether the GATT server was initialized successfully.
  bool init(const std::string &device_name) {
    if (server_) {
      logger_.info("Server already created, not initializing again");
      return true;
    }

    logger_.info("Initializing GATT server with device name: '{}'", device_name);
    // create the device
    NimBLEDevice::init(device_name);

    // create the server
    logger_.info("Creating server");
    server_ = NimBLEDevice::createServer();
    if (!server_) {
      logger_.error("Failed to create server");
      return false;
    }

    // set the server callbacks
    server_->setCallbacks(new BleGattServerCallbacks(this));

    logger_.info("Creating device info service");
    // create the device info service
    device_info_service_.init(server_);

    logger_.info("Creating battery service");
    // create the battery service
    battery_service_.init(server_);

    return true;
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
    if (!server_) {
      logger_.info("Server is nullptr; already deinitialized, not deinitializing again");
      return;
    }

    // deinitialize the services
    device_info_service_.deinit();
    battery_service_.deinit();
    // if true, deletes all server/advertising/scan/client objects which
    // invalidates any references/pointers to them
    bool clear_all = true;
    NimBLEDevice::deinit(clear_all);
    // clear the server
    server_ = nullptr;
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
  /// @return Whether the server was started successfully.
  bool start() {
    if (!server_) {
      logger_.error("Server not created");
      return false;
    }
    server_->start();
    return true;
  }

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Set whether to advertise on disconnect
  /// @param advertise_on_disconnect Whether to advertise on disconnect
  /// @note This method is only available when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used. Otherwise, you will have to
  ///       manually start advertising after disconnecting.
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
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
  void set_advertisement_data(const AdvertisedData &advertising_data, uint8_t instance = 0);

  /// Set the scan response data for the device.
  /// @param scan_response_data The scan response data for the device.
  /// @param instance The advertising instance to set the data for.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
  void set_scan_response_data(const AdvertisedData &scan_response_data, uint8_t instance = 0);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Set the advertisement data for the device.
  /// @param advertising_data The advertising data for the device.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is disabled.
  void set_advertisement_data(const AdvertisedData &advertising_data);

  /// Set the scan response data for the device.
  /// @param scan_response_data The scan response data for the device.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is disabled.
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
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
  void stop_advertising(uint8_t instance);

  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param duration_ms The duration of the advertising in milliseconds. If 0,
  ///                    the advertising will not timeout. If non-zero, the
  ///                    advertising will stop after the specified duration.
  /// @param instance The advertising instance to start.
  /// @return Whether advertising was started successfully.
  /// @note This is only available when CONFIG_BT_NIMBLE_EXT_ADV is enabled.
  bool start_advertising(uint32_t duration_ms = 0, uint8_t instance = 0);
#endif // CONFIG_BT_NIMBLE_EXT_ADV

#if !CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)
  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param params The advertising parameters for the device.
  /// @return Whether advertising was started successfully.
  /// @note This method is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used. Otherwise, use the
  ///       NimBLEExtAdvertisement class for advertising and setting the
  ///       advertising parameters.
  bool start_advertising(const AdvertisingParameters &params);

  /// Start Advertising using the previously set advertising data
  /// This method simply starts advertising using the previously set advertising
  /// data.
  /// @param duration_ms The duration of the advertising in milliseconds.
  ///                   If 0, the advertising will not timeout. If non-zero,
  ///                   the advertising will stop after the specified duration.
  /// @param directed_address The address to direct advertising to, if any.
  /// @return Whether advertising was started successfully.
  /// @note This method is only used when CONFIG_BT_NIMBLE_EXT_ADV is not
  ///       enabled, ane legacy advertising is used.
  bool start_advertising(uint32_t duration_ms = 0, NimBLEAddress *directed_address = nullptr);
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

  /// Get the connected device addresses
  /// @return The addresses for the connected devices as a vector.
  std::vector<NimBLEAddress> get_connected_device_addresses() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<NimBLEAddress> connected_addresses;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerInfoByHandle(peer_id);
      connected_addresses.push_back(peer.getAddress());
    }
    return connected_addresses;
  }

  /// Get the NimBLEConnInfo objects for the connected devices
  /// @return The connected devices info as a vector of NimBLEConnInfo.
  std::vector<NimBLEConnInfo> get_connected_device_infos() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<NimBLEConnInfo> connected_devices_info;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerInfoByHandle(peer_id);
      connected_devices_info.push_back(peer);
    }
    return connected_devices_info;
  }

  /// Get the connected device name
  /// @param conn_info The connection information for the device.
  /// @return The connected device name.
  std::string get_connected_device_name(NimBLEConnInfo &conn_info) {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    // since this connection is handled by the server, we won't manually
    // connect, and instead inform the client that we are already connected
    // using this conn handle
    auto client = server_->getClient(conn_info);
    // refresh the services
    client->getServices(true);
    // now get Generic Access Service
    auto gas = client->getService(NimBLEUUID("1800"));
    if (!gas) {
      logger_.error("Failed to get Generic Access Service");
      return {};
    }
    // now get the Device Name characteristic
    auto name_char = gas->getCharacteristic(NimBLEUUID("2A00"));
    if (!name_char) {
      logger_.error("Failed to get Device Name characteristic");
      return {};
    }
    // make sure we can read it
    if (!name_char->canRead()) {
      logger_.error("Failed to read Device Name characteristic");
      return {};
    }
    // and read it
    auto name = name_char->readValue();
    return name;
  }

  /// Get the connected device names
  /// @return The connected device names as a vector of strings.
  /// @note This method will connect to each device to get the device name.
  ///       This may take some time if there are many devices connected.
  std::vector<std::string> get_connected_device_names() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<std::string> connected_device_names;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerInfoByHandle(peer_id);
      auto peer_name = get_connected_device_name(peer);
      if (!peer_name.empty()) {
        connected_device_names.push_back(peer_name);
      } else {
        logger_.error("Failed to get device name for connected device {}",
                      peer.getAddress().toString());
      }
    }
    logger_.info("Connected device names: {}", connected_device_names);
    return connected_device_names;
  }

  /// Get the RSSI of the connected device
  /// @param conn_info The connection information for the device.
  /// @return The RSSI of the connected device.
  int get_connected_device_rssi(NimBLEConnInfo &conn_info) {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    if (!is_connected()) {
      logger_.error("Not connected to any devices");
      return {};
    }
    auto peer_address = conn_info.getAddress();
    logger_.debug("Getting RSSI for connected device {}", peer_address.toString());
    // since this connection is handled by the server, we won't manually
    // connect, and instead inform the client that we are already connected
    // using this conn handle
    auto client = server_->getClient(conn_info);
    // and read the RSSI from the client
    auto rssi = client->getRssi();
    logger_.info("RSSI for connected device {}: {}", peer_address.toString(), rssi);
    return rssi;
  }

  /// Get the RSSI of the connected devices
  /// @return The RSSI of the connected devices as a vector.
  /// @note This method will connect to each device to get the RSSI.
  ///       This may take some time if there are many devices connected.
  std::vector<int> get_connected_devices_rssi_values() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<int> connected_device_rssi;
    auto peer_ids = server_->getPeerDevices();
    for (const auto &peer_id : peer_ids) {
      auto peer = server_->getPeerInfoByHandle(peer_id);
      auto peer_rssi = get_connected_device_rssi(peer);
      connected_device_rssi.push_back(peer_rssi);
    }
    logger_.info("Connected device RSSI: {}", connected_device_rssi);
    return connected_device_rssi;
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
      auto peer = server_->getPeerInfoByHandle(peer_id);
      disconnected_devices.push_back(peer.getAddress());
      server_->disconnect(peer_id);
    }
    return disconnected_devices;
  }

  /// Unpair all devices
  /// This method unpairs all devices that are currently paired.
  /// @return The Addresses of the devices that were unpaired.
  std::vector<NimBLEAddress> unpair_all() {
    if (!server_) {
      logger_.error("Server not created");
      return {};
    }
    std::vector<NimBLEAddress> unpaired_devices;
    auto num_bonds = NimBLEDevice::getNumBonds();
    logger_.info("Unpairing {} devices", num_bonds);
    for (int i = 0; i < num_bonds; i++) {
      auto bond_addr = NimBLEDevice::getBondedAddress(i);
      logger_.debug("Unpairing device {}", bond_addr.toString());
      bool success = NimBLEDevice::deleteBond(bond_addr);
      if (!success) {
        logger_.error("Failed to unpair device {}", bond_addr.toString());
      } else {
        unpaired_devices.push_back(bond_addr);
      }
    }
    return unpaired_devices;
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
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::BleGattServer::AdvertisingParameters &advertising_params,
              FormatContext &ctx) const {
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

// for easy printing of the DisconnectReason enum using libfmt
template <> struct fmt::formatter<espp::BleGattServer::DisconnectReason> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::BleGattServer::DisconnectReason &reason, FormatContext &ctx) const {
    switch (reason) {
    case espp::BleGattServer::DisconnectReason::UNKNOWN:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    case espp::BleGattServer::DisconnectReason::TIMEOUT:
      return fmt::format_to(ctx.out(), "TIMEOUT");
    case espp::BleGattServer::DisconnectReason::CONNECTION_TERMINATED:
      return fmt::format_to(ctx.out(), "CONNECTION_TERMINATED");
    case espp::BleGattServer::DisconnectReason::REMOTE_USER_TERMINATED:
      return fmt::format_to(ctx.out(), "REMOTE_USER_TERMINATED");
    case espp::BleGattServer::DisconnectReason::REMOTE_DEVICE_TERMINATED:
      return fmt::format_to(ctx.out(), "REMOTE_DEVICE_TERMINATED");
    case espp::BleGattServer::DisconnectReason::LOCAL_USER_TERMINATED:
      return fmt::format_to(ctx.out(), "LOCAL_USER_TERMINATED");
    case espp::BleGattServer::DisconnectReason::AUTHENTICATION_FAILURE:
      return fmt::format_to(ctx.out(), "AUTHENTICATION_FAILURE");
    default:
      return fmt::format_to(ctx.out(), "INVALID");
    }
  }
};

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
