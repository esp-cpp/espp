#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "lvgl.h"

#include "hid-rp-report-map.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "spacemouse_decoder.hpp"

/// The LVGL UI for the USB HID host example on the M5Stack Tab5. Follows the
/// espp example Gui pattern: every LVGL object is built in init_ui() (split
/// into small init_* functions), the class owns the task that pumps
/// lv_task_handler(), and a recursive mutex guards every LVGL call so the USB
/// host's dispatch task can call the public setters directly.
///
/// Layout (portrait or landscape, sized from the display at runtime): a device
/// card on top, a "last report" box at the bottom, and between them a
/// scrollable column of panels, one per kind of HID interface, of which only
/// the kinds currently attached are shown:
/// * SpaceMouse: six axis bars (Tx Ty Tz / Rx Ry Rz) with numeric readouts and
///   a row of button indicators;
/// * keyboard: a virtual keyboard whose pressed keys and modifiers light up;
/// * mouse: a pad with a dot that follows the accumulated motion, a wheel
///   readout and button indicators;
/// * gamepad: two stick pads, the d-pad, and indicators for the face,
///   shoulder, stick-click and menu buttons.
class Gui {
public:
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// What an opened HID interface was recognised as.
  enum class Kind { Unknown, SpaceMouse, Keyboard, Mouse, Gamepad };
  static const char *kind_name(Kind kind);

  /// One opened HID interface of the device, as listed on the card.
  struct Interface {
    uint8_t interface_number{0};
    uint8_t protocol{0};
    size_t report_descriptor_bytes{0};
    Kind kind{Kind::Unknown};
  };

  /// What the device card shows: the device, and every interface opened on it.
  struct DeviceInfo {
    bool connected{false};
    std::string product;
    std::string manufacturer;
    uint16_t vid{0};
    uint16_t pid{0};
    std::vector<Interface> interfaces;
    bool has(Kind kind) const;
  };

  /// What the mouse panel shows: the dot position is the motion accumulated by
  /// the caller (clamped to +-kMouseRange), the wheel likewise.
  struct MouseState {
    int32_t x{0};
    int32_t y{0};
    int32_t wheel{0};
    uint8_t buttons{0}; ///< bit0 left, bit1 right, bit2 middle, ...
  };
  static constexpr int32_t kMouseRange = 1000;

  explicit Gui(const Config &config)
      : logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    update_task_.start();
  }

  ~Gui() {
    update_task_.stop();
    deinit_ui();
  }

  /// Update the device card and show the panels for the kinds it carries.
  /// Thread-safe.
  void set_device(const DeviceInfo &info);

  /// Update the six axis bars + buttons from a decoded SpaceMouse state.
  /// Thread-safe.
  void set_spacemouse_state(const SpaceMouseDecoder::State &state);

  /// Update the virtual keyboard from a decoded keyboard report (any usage
  /// on the Keyboard/Keypad page, modifiers included). Thread-safe.
  void set_keyboard_state(const espp::hid_rp::KeyboardReport &report);

  /// Update the mouse panel. Thread-safe.
  void set_mouse_state(const MouseState &state);

  /// Update the gamepad panel from a decoded gamepad report. Thread-safe.
  void set_gamepad_state(const espp::hid_rp::GamepadReport &report);

  /// Show the raw bytes of the latest Input report and the measured report
  /// rate. Thread-safe.
  void set_last_report(std::span<const uint8_t> report, float reports_per_second);

  /// Set the one-line status / instruction text at the bottom. Thread-safe.
  void set_status_text(std::string_view text);

protected:
  static constexpr size_t kAxisCount = 6;
  static constexpr size_t kButtonCount = SpaceMouseDecoder::kButtonCount;
  static constexpr size_t kMouseButtonCount = 5;
  /// gamepad indicators: 0..3 the face buttons (south, east, west, north),
  /// then L1 R1 L2 R2 L3 R3 Select Start Home (see kGamepadButtonNames)
  static constexpr size_t kGamepadButtonCount = 13;

  void init_ui();
  void deinit_ui();
  void init_device_card(lv_obj_t *parent);
  lv_obj_t *init_panel(lv_obj_t *parent, const char *title);
  void init_axes(lv_obj_t *parent);
  void init_keyboard(lv_obj_t *parent);
  void init_mouse(lv_obj_t *parent);
  void init_gamepad(lv_obj_t *parent);
  void init_report_line(lv_obj_t *parent);
  /// A square pad with a dot, for a mouse position or a gamepad stick.
  struct Pad {
    lv_obj_t *obj{nullptr};
    lv_obj_t *dot{nullptr};
    int size{0}; ///< the pad's side, which the dot's travel is computed from
  };
  Pad make_pad(lv_obj_t *parent, int size);
  /// A labelled indicator square (button LED).
  lv_obj_t *make_led(lv_obj_t *parent, const char *label, int size);
  static void set_led(lv_obj_t *led, bool on);
  static void move_dot(const Pad &pad, int32_t x, int32_t y, int32_t range);
  void reset_panels();

  bool update(std::mutex &m, std::condition_variable &cv);

  std::recursive_mutex mutex_;
  espp::Logger logger_;

  lv_obj_t *root_{nullptr};
  lv_obj_t *panels_{nullptr}; ///< the scrollable column the kind panels live in
  lv_obj_t *axes_panel_{nullptr};
  lv_obj_t *buttons_panel_{nullptr};
  lv_obj_t *keyboard_panel_{nullptr};
  lv_obj_t *mouse_panel_{nullptr};
  lv_obj_t *gamepad_panel_{nullptr};
  /// virtual keyboard keys by HID usage id (Keyboard/Keypad page, 0xE0..0xE7 =
  /// the modifier keys); nullptr for usages not on the layout
  std::array<lv_obj_t *, 256> keyboard_keys_{};
  lv_obj_t *device_title_{nullptr};
  lv_obj_t *device_detail_{nullptr};
  lv_obj_t *device_state_dot_{nullptr};
  std::array<lv_obj_t *, kAxisCount> axis_bars_{};
  std::array<lv_obj_t *, kAxisCount> axis_values_{};
  std::array<lv_obj_t *, kButtonCount> button_leds_{};
  // mouse panel
  Pad mouse_pad_;
  lv_obj_t *mouse_wheel_bar_{nullptr};
  lv_obj_t *mouse_wheel_value_{nullptr};
  lv_obj_t *mouse_position_{nullptr};
  std::array<lv_obj_t *, kMouseButtonCount> mouse_leds_{};
  // gamepad panel
  Pad left_pad_;
  Pad right_pad_;
  lv_obj_t *stick_values_{nullptr};
  std::array<lv_obj_t *, 4> dpad_leds_{}; ///< up, down, left, right
  std::array<lv_obj_t *, kGamepadButtonCount> gamepad_leds_{};
  // last values shown, so an unchanged state does not invalidate anything
  std::array<int16_t, kAxisCount> shown_axes_{};
  std::array<bool, kButtonCount> shown_buttons_{};
  std::array<bool, 256> shown_keys_{};
  // the device the card shows, so an interface being added to the same device
  // does not reset the panels or the scroll position
  bool shown_connected_{false};
  uint16_t shown_vid_{0};
  uint16_t shown_pid_{0};
  MouseState shown_mouse_{};
  espp::hid_rp::GamepadReport shown_gamepad_{};
  std::array<bool, kGamepadButtonCount> shown_gamepad_buttons_{};
  std::string shown_report_hex_;
  int shown_rate_{-1};
  std::string shown_status_;
  lv_obj_t *report_label_{nullptr};
  lv_obj_t *rate_label_{nullptr};
  lv_obj_t *status_label_{nullptr};

  // lv_task_handler() lays out the panels on this task's stack: the gamepad
  // panel (two pads, two 3x3 grids and a wrapping block of indicators) needs
  // well over the 6 KB the SpaceMouse-only layout got by with
  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 16 * 1024}}};
};
