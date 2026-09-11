#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <esp_mac.h>
#include <esp_random.h>
#include <esp_timer.h>

#include "base_component.hpp"
#include "hid-rp-switch-pro.hpp"
#include "high_resolution_timer.hpp"

#include "detail/switch_controller_protocol.hpp"
#include "detail/switch_pro_spi_rom_data.hpp"

namespace espp {
/// @brief Nintendo Switch Pro controller (NS1) USB emulation protocol engine.
///
/// This class implements the Nintendo Switch Pro controller's USB HID handshake
/// and input-report protocol so an ESP device can present itself to a Nintendo
/// Switch (NS1) as a Pro Controller. It is **transport-light**: it owns the
/// controller state + the request/response state machine but performs no USB I/O
/// itself. Drive it from a USB HID interface (e.g. the espp `usb_device`
/// component's HID function): feed host OUTPUT reports to `on_hid_report()` and
/// send the returned reply (and periodic `get_input_report()` reports) back as
/// HID INPUT reports. See the example.
///
/// The HID report descriptor, input-report packing, and the SPI-ROM
/// calibration/config blobs come from the espp `hid-rp` component
/// (`switch_pro_descriptor()`, `SwitchProGamepadInputReport`).
///
/// @note **Emulation only.** A real Switch only binds a device advertising
///       Nintendo's Pro Controller USB VID/PID (0x057E / 0x2009) and identity
///       strings (exposed here as constants). Use these to emulate/test against a
///       Switch you own; do not ship a product impersonating Nintendo hardware.
///
/// \section switch_pro_ex1 Switch Pro example
/// \snippet switch_pro_example.cpp switch_pro example
class SwitchPro : public espp::BaseComponent {
public:
  /// The hid-rp standard input report type (report id 0x30).
  using InputReport = espp::SwitchProGamepadInputReport<>;

  /// A HID report to send to the host: {report id, report bytes}.
  using ReportData = std::pair<uint8_t, std::vector<uint8_t>>;

  /// @brief Configuration for the SwitchPro engine.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  // --- Nintendo Switch Pro USB identity (EMULATION ONLY; see the class note) ---
  static constexpr uint16_t vid = 0x057E;        ///< Nintendo USB VID.
  static constexpr uint16_t pid = 0x2009;        ///< Switch Pro Controller USB PID.
  static constexpr uint16_t bcd_device = 0x0200; ///< bcdDevice.
  static constexpr uint16_t bcd_usb = 0x0200;    ///< bcdUSB (USB 2.0).
  static constexpr const char *manufacturer_name = "Nintendo Co., Ltd."; ///< iManufacturer.
  static constexpr const char *product_name = "Pro Controller";          ///< iProduct.

  explicit SwitchPro(const Config &config);

  /// @brief The HID report descriptor bytes (the full Switch Pro descriptor).
  /// @return A copy of the report descriptor bytes.
  std::vector<uint8_t> get_report_descriptor() const {
    return std::vector<uint8_t>(report_descriptor_.begin(), report_descriptor_.end());
  }

  /// @brief The report id of the standard input report (0x30).
  uint8_t input_report_id() const { return InputReport::ID; }

  /// @brief Whether the handshake has progressed far enough that the host has
  ///        enabled input reports (i.e. it is meaningful to stream them).
  bool is_ready() const { return hid_ready_; }

  /// @brief Kick off the initialization sequence.
  ///
  /// Call once when the USB device is attached/mounted. Returns the initial
  /// device-info report the controller sends unprompted to start the handshake
  /// (report id 0x81), or std::nullopt if none.
  std::optional<ReportData> on_attach();

  /// @brief Handle a host OUTPUT report (host -> device) and produce the reply.
  ///
  /// Feed every OUTPUT report the host sends (report ids 0x80 init, 0x01 output /
  /// subcommand, 0x10 rumble) here; byte 0 of @p data must be the report id.
  /// Returns the INPUT report to send back (report id + bytes), or std::nullopt if
  /// no reply is warranted.
  /// @param report_id The HID report id (ignored; the type is read from data[0]).
  /// @param data The received report bytes (data[0] is the report id/type).
  /// @param len Number of bytes at @p data.
  std::optional<ReportData> on_hid_report(uint8_t report_id, const uint8_t *data, size_t len);

  /// @brief The current standard input report bytes (report id 0x30 payload).
  /// @return The report bytes, or an empty vector if not ready yet.
  std::vector<uint8_t> get_input_report() const;

  /// @brief Thread-safely mutate the input report (buttons / joysticks / dpad).
  ///
  /// The callback receives the underlying `hid-rp` input report; use its setters
  /// (e.g. `set_button_a()`, `set_left_joystick()`, `set_dpad()`). Housekeeping
  /// fields (USB-powered, battery, connection info) are (re)applied afterward so a
  /// caller-issued `reset()` does not clear them.
  /// @param fn Callback that mutates the input report.
  void update_input_report(const std::function<void(InputReport &)> &fn);

  /// @brief Set the reported battery level (0-100).
  void set_battery_level(uint8_t level);

  /// @brief Set the trigger-buttons-elapsed times (subcommand 0x04 reply), in
  ///        units of 10 ms, order L,R,ZL,ZR,SL,SR,HOME. Optional; defaults to 0.
  void set_trigger_elapsed_times(const std::array<uint16_t, 7> &times_10ms);

protected:
  static constexpr auto report_descriptor_ = espp::switch_pro_descriptor();

  // Joy-Con uses 4.96 ms as the timer tick rate for the input-report counter.
  static constexpr uint64_t counter_period_us = 4960;

  // Apply the housekeeping fields the report needs (called with the mutex held).
  void apply_housekeeping();

  // --- request/response handlers (ported from the reference protocol) ---------
  /// @brief Build the device-info payload (report 0x81, command 0x01): device
  ///        type + our MAC. Sent both proactively on attach and in reply to the
  ///        host's 0x80 0x01 device-info request.
  std::vector<uint8_t> device_info_report() const;
  ReportData process_command(const uint8_t *data, size_t len);
  void set_subcommand_reply(std::vector<uint8_t> &report);
  void set_unknown_subcommand(std::vector<uint8_t> &report, uint8_t subcommand_id);
  void set_standard_input_report(std::vector<uint8_t> &report);
  void set_device_info(std::vector<uint8_t> &report);
  void set_shipment(std::vector<uint8_t> &report);
  void toggle_imu(std::vector<uint8_t> &report, sp::Message &message);
  void set_imu_data(std::vector<uint8_t> &report);
  void spi_read(std::vector<uint8_t> &report, sp::Message &message);
  void set_mode(std::vector<uint8_t> &report, sp::Message &message);
  void set_trigger_buttons(std::vector<uint8_t> &report);
  void enable_vibration(std::vector<uint8_t> &report);
  void set_player_lights(std::vector<uint8_t> &report, sp::Message &message);
  void set_nfc_ir_state(std::vector<uint8_t> &report);
  void set_nfc_ir_config(std::vector<uint8_t> &report);

  /// Read emulated SPI flash memory into @p response.
  /// @param bank The bank (high address byte) to read from.
  /// @param reg The register (low address byte) to read from.
  /// @param read_length The number of bytes to read.
  /// @param response Destination buffer (must hold at least @p read_length bytes).
  /// @return Number of bytes read (0 on failure).
  uint8_t spi_read_impl(uint8_t bank, uint8_t reg, uint8_t read_length, uint8_t *response);

  std::array<uint8_t, 6> mac_address_{0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  std::array<uint8_t, std::size(sp::spi_rom_data_60)> spi_rom_factory_data_{};
  std::array<uint8_t, std::size(sp::spi_rom_data_80)> spi_rom_user_data_{};

  bool hid_ready_ = false;      // set after device info has been queried / USB HID enabled
  uint8_t battery_level_ = 100; // reported battery percentage (re-applied by apply_housekeeping)

  uint8_t input_report_mode_ = 0; // standard (0x30), nfc/ir (0x31), simpleHID (0x3F)
  uint8_t player_number_ = 0;     // valid values are 1, 2, 3, and 4
  bool vibration_enabled_ = false;
  uint8_t vibrator_report_{0}; // randomly selected from sp::vibrator_bytes
  bool imu_enabled_ = false;
  uint8_t input_report_id_ = 0x21;
  sp::TriggerTimes trigger_times_{};

  InputReport input_report_;
  mutable std::recursive_mutex input_report_mutex_;

  espp::HighResolutionTimer counter_timer_{{
      .name = "Switch Pro Counter Timer",
      .callback =
          [this]() {
            std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
            input_report_.increment_counter();
          },
  }};
}; // class SwitchPro
} // namespace espp
