#pragma once

#include <sdkconfig.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <string>

#include <sdkconfig.h>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

#include "base_component.hpp"

namespace espp {
/// HID Service
/// This class is responsible for creating and managing the HID service.
/// It is responsible for creating and managing the required characteristics
/// per the HID specification.
/// It allows arbitrary input/output/feature HID reports.
///
/// NOTE: this is a simplified version of NimBLEHIDDevice, which does not
/// include the DeviceInfoService and BatteryService internally, as
/// espp::BleGattServer already provides the DeviceInfoService and
/// BatteryService.
///
/// If you need the DeviceInfoService and BatteryService, you can access
/// them through espp::BleGattServer.
///
/// @see BleGattServer
///
/// \section hid_service_ex1 HID Service Example
/// \snippet hid_service_example.cpp hid service example
class HidService : public espp::BaseComponent {
public:
  /// @brief Constructor
  /// @param log_level The verbosity of the logger
  explicit HidService(espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("HidService", log_level) {}

  /// @brief Constructor
  /// @param server The BLE server to use for the HID service.
  /// @param log_level The verbosity of the logger
  explicit HidService(NimBLEServer *server,
                      espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("HidService", log_level) {
    make_service(server);
  }

  /// @brief Destructor
  ~HidService() {
    // NOTE: We don't have to delete the service, as the server will
    //       delete it when it is deleted.
  }

  /// @brief Initialize the HID service.
  void init(NimBLEServer *server) { make_service(server); }

  /// @brief Deinitialize the HID service.
  /// @note This should only be called after NimBLEDevice::deinit(true) has been
  ///       called, since that will free the memory used by the service.
  void deinit() {
    service_ = nullptr;
    hid_info_ = nullptr;
    report_map_ = nullptr;
    control_ = nullptr;
    protocol_mode_ = nullptr;
  }

  /// @brief Start the HID service.
  void start() {
    if (service_) {
      service_->start();
    }
  }

  /// @brief Get the HID service.
  /// @return The HID service.
  NimBLEService *service() { return service_; }

  /// @brief Get the UUID of the HID service.
  /// @return The UUID of the HID service.
  NimBLEUUID uuid() {
    if (service_) {
      return service_->getUUID();
    }
    return NimBLEUUID(SERVICE_UUID);
  }

  /// @brief Set the report map for the HID service.
  /// @param report_map The report map as a vector of bytes.
  void set_report_map(const std::vector<uint8_t> &report_map) {
    if (report_map_) {
      report_map_->setValue(report_map);
    } else {
      logger_.error("Report map characteristic not created");
    }
  }

  /// @brief Set the report map for the HID service.
  /// @param report_map The report map as a string view.
  void set_report_map(std::string_view report_map) {
    if (report_map_) {
      report_map_->setValue(report_map);
    } else {
      logger_.error("Report map characteristic not created");
    }
  }

  /// @brief Set the report map for the HID service.
  /// @param report_map The report map bytes
  /// @param report_map_len The length of the report map.
  /// @note The report map is a descriptor that describes the format of
  ///       the HID reports. It is used to tell the host how to interpret
  ///       the reports that are sent by the HID device.
  void set_report_map(const uint8_t *report_map, size_t report_map_len) {
    if (report_map_) {
      report_map_->setValue(report_map, report_map_len);
    } else {
      logger_.error("Report map characteristic not created");
    }
  }

  /// @brief Set the HID information for the HID service.
  /// @param country The country code.
  /// @param flags The HID information flags.
  void set_info(uint8_t country, uint8_t flags) {
    if (hid_info_) {
      uint8_t info[] = {0x11, 0x1, country, flags};
      hid_info_->setValue(info, sizeof(info));
    } else {
      logger_.error("HID information characteristic not created");
    }
  }

  /// @brief Get the control characteristic for the HID service.
  /// @return The control characteristic.
  NimBLECharacteristic *get_control() { return control_; }

  /// @brief Get the protocol mode characteristic for the HID service.
  /// @return The protocol mode characteristic.
  NimBLECharacteristic *get_protocol_mode() { return protocol_mode_; }

  /// @brief Create an input report characteristic.
  /// @param report_id The report ID. This should be the same as the report
  ///                  ID in the report descriptor for the input object that
  ///                  is related to the characteristic.
  /// @return The input report characteristic.
  NimBLECharacteristic *input_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(input_report_characteristics_mutex_);

    // look up the report ID in the list of input report characteristics
    for (const auto &report_char : input_report_characteristics_) {
      // cppcheck-suppress useStlAlgorithm
      if (report_char.first == report_id) {
        return report_char.second;
      }
    }

    // we got here, so the report ID was not found
    auto input_report_char = service_->createCharacteristic(
        NimBLEUUID(REPORT_UUID),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC);
    auto desc = input_report_char->createDescriptor(
        NimBLEUUID(REPORT_DESCRIPTOR_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC);

    // set the report ID and the report type in the descriptor
    uint8_t desc_value[] = {report_id, 0x01};
    desc->setValue(desc_value, sizeof(desc_value));

    // now add the report ID to the list of input report characteristics
    input_report_characteristics_.emplace_back(report_id, input_report_char);

    return input_report_char;
  }

  /// @brief Remove an input report characteristic.
  /// @param report_id The report ID of the input report characteristic to
  ///        remove.
  /// @note This will delete the characteristic, and remove it from the list.
  ///       Any stored pointers to the characteristic will be invalid after this
  ///       call.
  void remove_input_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(input_report_characteristics_mutex_);
    auto it =
        std::find_if(input_report_characteristics_.begin(), input_report_characteristics_.end(),
                     [report_id](const ReportCharacteristic &report_char) {
                       return report_char.first == report_id;
                     });
    if (it != input_report_characteristics_.end()) {
      static constexpr bool delete_char = true;
      service_->removeCharacteristic(it->second, delete_char);
      input_report_characteristics_.erase(it);
    }
  }

  /// @brief Create an output report characteristic.
  /// @param report_id The report ID. This should be the same as the report
  ///                ID in the report descriptor for the output object that
  ///                is related to the characteristic.
  /// @return The output report characteristic.
  NimBLECharacteristic *output_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(output_report_characteristics_mutex_);

    // look up the report ID in the list of output report characteristics
    for (const auto &report_char : output_report_characteristics_) {
      // cppcheck-suppress useStlAlgorithm
      if (report_char.first == report_id) {
        return report_char.second;
      }
    }

    // we got here, so the report ID was not found
    auto output_report_char = service_->createCharacteristic(
        NimBLEUUID(REPORT_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                     NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::READ_ENC |
                                     NIMBLE_PROPERTY::WRITE_ENC);
    auto desc = output_report_char->createDescriptor(
        NimBLEUUID(REPORT_DESCRIPTOR_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::READ_ENC |
                                                NIMBLE_PROPERTY::WRITE_ENC);

    // set the report ID and the report type in the descriptor
    uint8_t desc_value[] = {report_id, 0x02};
    desc->setValue(desc_value, sizeof(desc_value));

    // now add the report ID to the list of output report characteristics
    output_report_characteristics_.emplace_back(report_id, output_report_char);

    return output_report_char;
  }

  /// @brief Remove an output report characteristic.
  /// @param report_id The report ID of the output report characteristic to
  ///        remove.
  /// @note This will delete the characteristic, and remove it from the list.
  ///       Any stored pointers to the characteristic will be invalid after this
  ///       call.
  void remove_output_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(output_report_characteristics_mutex_);
    auto it =
        std::find_if(output_report_characteristics_.begin(), output_report_characteristics_.end(),
                     [report_id](const ReportCharacteristic &report_char) {
                       return report_char.first == report_id;
                     });
    if (it != output_report_characteristics_.end()) {
      static constexpr bool delete_char = true;
      service_->removeCharacteristic(it->second, delete_char);
      output_report_characteristics_.erase(it);
    }
  }

  /// @brief Create a feature report characteristic.
  /// @param report_id The report ID. This should be the same as the report
  ///               ID in the report descriptor for the feature object that
  ///               is related to the characteristic.
  /// @return The feature report characteristic.
  NimBLECharacteristic *feature_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(feature_report_characteristics_mutex_);

    // look up the report ID in the list of feature report characteristics
    for (const auto &report_char : feature_report_characteristics_) {
      // cppcheck-suppress useStlAlgorithm
      if (report_char.first == report_id) {
        return report_char.second;
      }
    }

    // we got here, so the report ID was not found
    auto feature_report_char = service_->createCharacteristic(
        NimBLEUUID(REPORT_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                     NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC);
    auto desc = feature_report_char->createDescriptor(
        NimBLEUUID(REPORT_DESCRIPTOR_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::READ_ENC |
                                                NIMBLE_PROPERTY::WRITE_ENC);

    // set the report ID and the report type in the descriptor
    uint8_t desc_value[] = {report_id, 0x03};
    desc->setValue(desc_value, sizeof(desc_value));

    // now add the report ID to the list of feature report characteristics
    feature_report_characteristics_.emplace_back(report_id, feature_report_char);

    return feature_report_char;
  }

  /// @brief Remove a feature report characteristic.
  /// @param report_id The report ID of the feature report characteristic to
  ///        remove.
  /// @note This will delete the characteristic, and remove it from the list.
  ///       Any stored pointers to the characteristic will be invalid after this
  ///       call.
  void remove_feature_report(uint8_t report_id) {
    std::lock_guard<std::mutex> lock(feature_report_characteristics_mutex_);
    auto it =
        std::find_if(feature_report_characteristics_.begin(), feature_report_characteristics_.end(),
                     [report_id](const ReportCharacteristic &report_char) {
                       return report_char.first == report_id;
                     });
    if (it != feature_report_characteristics_.end()) {
      static constexpr bool delete_char = true;
      service_->removeCharacteristic(it->second, delete_char);
      feature_report_characteristics_.erase(it);
    }
  }

protected:
  static constexpr uint16_t SERVICE_UUID = 0x1812;
  static constexpr uint16_t HID_INFORMATION_UUID = 0x2a4a;
  static constexpr uint16_t REPORT_MAP_UUID = 0x2a4b;
  static constexpr uint16_t HID_CONTROL_POINT_UUID = 0x2a4c;
  static constexpr uint16_t PROTOCOL_MODE_UUID = 0x2a4e;

  static constexpr uint16_t REPORT_UUID = 0x2a4d;
  static constexpr uint16_t REPORT_DESCRIPTOR_UUID = 0x2908;

  void make_service(NimBLEServer *server) {
    service_ = server->createService(NimBLEUUID(SERVICE_UUID));
    if (!service_) {
      logger_.error("Failed to create service");
      return;
    }

    hid_info_ =
        service_->createCharacteristic(NimBLEUUID(HID_INFORMATION_UUID), NIMBLE_PROPERTY::READ);
    report_map_ =
        service_->createCharacteristic(NimBLEUUID(REPORT_MAP_UUID), NIMBLE_PROPERTY::READ);
    control_ = service_->createCharacteristic(NimBLEUUID(HID_CONTROL_POINT_UUID),
                                              NIMBLE_PROPERTY::WRITE_NR);
    protocol_mode_ = service_->createCharacteristic(
        NimBLEUUID(PROTOCOL_MODE_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);

    // set the default protocol mode to report mode (0x01) since it is
    // the most common mode
    const uint8_t pMode[] = {0x01};
    protocol_mode_->setValue(pMode, 1);
  }

  typedef std::pair<uint8_t, NimBLECharacteristic *> ReportCharacteristic;

  std::mutex input_report_characteristics_mutex_;
  std::vector<ReportCharacteristic> input_report_characteristics_;
  std::mutex output_report_characteristics_mutex_;
  std::vector<ReportCharacteristic> output_report_characteristics_;
  std::mutex feature_report_characteristics_mutex_;
  std::vector<ReportCharacteristic> feature_report_characteristics_;

  NimBLEService *service_{nullptr};
  NimBLECharacteristic *hid_info_{nullptr};      // 0x2a4a
  NimBLECharacteristic *report_map_{nullptr};    // 0x2a4b
  NimBLECharacteristic *control_{nullptr};       // 0x2a4c
  NimBLECharacteristic *protocol_mode_{nullptr}; // 0x2a4e
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
