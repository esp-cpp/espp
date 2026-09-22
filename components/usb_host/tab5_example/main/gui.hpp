#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <string_view>

#include "lvgl.h"

#include "logger.hpp"
#include "task.hpp"

#include "spacemouse_decoder.hpp"

/// The LVGL UI for the USB HID host example on the M5Stack Tab5. Follows the
/// espp example Gui pattern: every LVGL object is built in init_ui() (split
/// into small init_* functions), the class owns the task that pumps
/// lv_task_handler(), and a recursive mutex guards every LVGL call so the USB
/// host's dispatch task can call the public setters directly.
///
/// Layout (portrait or landscape, sized from the display at runtime):
/// * a device card: connection state, product / manufacturer, VID:PID,
///   interface / protocol, report descriptor size, report rate;
/// * six axis bars (Tx Ty Tz / Rx Ry Rz) with numeric readouts, driven by
///   the SpaceMouse decoder when a 3Dconnexion device is attached;
/// * a row of button indicators;
/// * a "last report" line with the raw bytes of the most recent Input report
///   (every HID device, SpaceMouse or not).
class Gui {
public:
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// What the device card shows.
  struct DeviceInfo {
    bool connected{false};
    std::string product;
    std::string manufacturer;
    uint16_t vid{0};
    uint16_t pid{0};
    uint8_t interface_number{0};
    uint8_t protocol{0};
    size_t report_descriptor_bytes{0};
    bool is_spacemouse{false};
    bool is_keyboard{false}; ///< boot-protocol keyboard: the virtual keyboard is shown
  };

  explicit Gui(const Config &config)
      : logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    update_task_.start();
  }

  ~Gui() {
    update_task_.stop();
    deinit_ui();
  }

  /// Update the device card. Thread-safe.
  void set_device(const DeviceInfo &info);

  /// Update the six axis bars + buttons from a decoded SpaceMouse state.
  /// Thread-safe.
  void set_spacemouse_state(const SpaceMouseDecoder::State &state);

  /// Update the virtual keyboard from a boot-protocol keyboard report: the
  /// modifier bit-set (byte 0) and the pressed key usage ids (bytes 2..7).
  /// Thread-safe.
  void set_keyboard_state(uint8_t modifiers, std::span<const uint8_t> keys);

  /// Show the raw bytes of the latest Input report and the measured report
  /// rate. Thread-safe.
  void set_last_report(std::span<const uint8_t> report, float reports_per_second);

  /// Set the one-line status / instruction text at the bottom. Thread-safe.
  void set_status_text(std::string_view text);

protected:
  static constexpr size_t kAxisCount = 6;
  static constexpr size_t kButtonCount = SpaceMouseDecoder::kButtonCount;

  void init_ui();
  void deinit_ui();
  void init_device_card(lv_obj_t *parent);
  void init_axes(lv_obj_t *parent);
  void init_keyboard(lv_obj_t *parent);
  void init_buttons(lv_obj_t *parent);
  void init_report_line(lv_obj_t *parent);

  bool update(std::mutex &m, std::condition_variable &cv);

  std::recursive_mutex mutex_;
  espp::Logger logger_;

  lv_obj_t *root_{nullptr};
  lv_obj_t *axes_panel_{nullptr};
  lv_obj_t *keyboard_panel_{nullptr};
  /// virtual keyboard keys by HID usage id (Keyboard/Keypad page, 0xE0..0xE7 =
  /// the modifier keys); nullptr for usages not on the layout
  std::array<lv_obj_t *, 256> keyboard_keys_{};
  lv_obj_t *device_title_{nullptr};
  lv_obj_t *device_detail_{nullptr};
  lv_obj_t *device_state_dot_{nullptr};
  std::array<lv_obj_t *, kAxisCount> axis_bars_{};
  std::array<lv_obj_t *, kAxisCount> axis_values_{};
  std::array<lv_obj_t *, kButtonCount> button_leds_{};
  lv_obj_t *report_label_{nullptr};
  lv_obj_t *rate_label_{nullptr};
  lv_obj_t *status_label_{nullptr};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 6 * 1024}}};
};
