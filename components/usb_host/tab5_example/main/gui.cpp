#include "gui.hpp"

#include <algorithm>
#include <cstdio>

#include "format.hpp"

namespace {
constexpr int kPad = 12;
constexpr const char *kAxisNames[] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};
constexpr lv_palette_t kAxisPalettes[] = {LV_PALETTE_BLUE,   LV_PALETTE_BLUE,   LV_PALETTE_BLUE,
                                          LV_PALETTE_ORANGE, LV_PALETTE_ORANGE, LV_PALETTE_ORANGE};
} // namespace

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x101418), 0);

  // one column filling the screen, sized from the live display so it works in
  // either orientation
  root_ = lv_obj_create(screen);
  lv_obj_set_size(root_, lv_display_get_horizontal_resolution(nullptr),
                  lv_display_get_vertical_resolution(nullptr));
  lv_obj_set_style_bg_opa(root_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(root_, 0, 0);
  lv_obj_set_style_pad_all(root_, kPad, 0);
  lv_obj_set_style_pad_row(root_, kPad, 0);
  lv_obj_set_flex_flow(root_, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

  init_device_card(root_);
  init_axes(root_);
  init_keyboard(root_);
  init_buttons(root_);
  init_report_line(root_);
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
  root_ = nullptr;
}

void Gui::init_device_card(lv_obj_t *parent) {
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_width(card, LV_PCT(100));
  lv_obj_set_height(card, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x1c2330), 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_pad_all(card, kPad, 0);
  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(card, 4, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title_row = lv_obj_create(card);
  lv_obj_set_width(title_row, LV_PCT(100));
  lv_obj_set_height(title_row, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(title_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(title_row, 0, 0);
  lv_obj_set_style_pad_all(title_row, 0, 0);
  lv_obj_set_flex_flow(title_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(title_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(title_row, 10, 0);
  lv_obj_clear_flag(title_row, LV_OBJ_FLAG_SCROLLABLE);

  device_state_dot_ = lv_obj_create(title_row);
  lv_obj_set_size(device_state_dot_, 18, 18);
  lv_obj_set_style_radius(device_state_dot_, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(device_state_dot_, 0, 0);
  lv_obj_set_style_bg_color(device_state_dot_, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_clear_flag(device_state_dot_, LV_OBJ_FLAG_SCROLLABLE);

  device_title_ = lv_label_create(title_row);
  lv_obj_set_style_text_font(device_title_, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(device_title_, lv_color_white(), 0);
  lv_label_set_text(device_title_, "No USB HID device");

  device_detail_ = lv_label_create(card);
  lv_obj_set_width(device_detail_, LV_PCT(100));
  lv_label_set_long_mode(device_detail_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(device_detail_, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(device_detail_, "Plug a device into the USB-A port.");
}

void Gui::init_axes(lv_obj_t *parent) {
  lv_obj_t *panel = lv_obj_create(parent);
  axes_panel_ = panel;
  lv_obj_set_width(panel, LV_PCT(100));
  lv_obj_set_flex_grow(panel, 1);
  lv_obj_set_style_bg_color(panel, lv_color_hex(0x1c2330), 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_radius(panel, 10, 0);
  lv_obj_set_style_pad_all(panel, kPad, 0);
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

  for (size_t i = 0; i < kAxisCount; ++i) {
    lv_obj_t *row = lv_obj_create(panel);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = lv_label_create(row);
    lv_obj_set_width(name, 44);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(name, lv_palette_main(kAxisPalettes[i]), 0);
    lv_label_set_text(name, kAxisNames[i]);

    // a centered bar: range symmetric about 0, the indicator grows from the
    // middle in either direction
    lv_obj_t *bar = lv_bar_create(row);
    lv_obj_set_flex_grow(bar, 1);
    lv_obj_set_height(bar, 26);
    lv_bar_set_mode(bar, LV_BAR_MODE_SYMMETRICAL);
    lv_bar_set_range(bar, -SpaceMouseDecoder::kAxisMax, SpaceMouseDecoder::kAxisMax);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x2a3444), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_palette_main(kAxisPalettes[i]), LV_PART_INDICATOR);
    axis_bars_[i] = bar;

    lv_obj_t *value = lv_label_create(row);
    lv_obj_set_width(value, 70);
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(value, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(value, lv_color_white(), 0);
    lv_label_set_text(value, "0");
    axis_values_[i] = value;
  }
}

namespace {
/// One key of the virtual keyboard: label, HID usage id, width in quarter-units.
struct KeySpec {
  const char *label;
  uint8_t usage;
  uint8_t width; // quarter key units (4 = one key)
};
// A compact US layout. Usage ids are the HID Keyboard/Keypad page; 0xE0..0xE7
// are the modifier keys (reported as bits, mapped to these usages here).
constexpr KeySpec kRow0[] = {{"Esc", 0x29, 4}, {"F1", 0x3A, 4}, {"F2", 0x3B, 4},  {"F3", 0x3C, 4},
                             {"F4", 0x3D, 4},  {"F5", 0x3E, 4}, {"F6", 0x3F, 4},  {"F7", 0x40, 4},
                             {"F8", 0x41, 4},  {"F9", 0x42, 4}, {"F10", 0x43, 4}, {"F11", 0x44, 4},
                             {"F12", 0x45, 4}, {"Del", 0x4C, 4}};
constexpr KeySpec kRow1[] = {{"`", 0x35, 4}, {"1", 0x1E, 4},   {"2", 0x1F, 4}, {"3", 0x20, 4},
                             {"4", 0x21, 4}, {"5", 0x22, 4},   {"6", 0x23, 4}, {"7", 0x24, 4},
                             {"8", 0x25, 4}, {"9", 0x26, 4},   {"0", 0x27, 4}, {"-", 0x2D, 4},
                             {"=", 0x2E, 4}, {"Bksp", 0x2A, 8}};
constexpr KeySpec kRow2[] = {{"Tab", 0x2B, 6}, {"q", 0x14, 4}, {"w", 0x1A, 4}, {"e", 0x08, 4},
                             {"r", 0x15, 4},   {"t", 0x17, 4}, {"y", 0x1C, 4}, {"u", 0x18, 4},
                             {"i", 0x0C, 4},   {"o", 0x12, 4}, {"p", 0x13, 4}, {"[", 0x2F, 4},
                             {"]", 0x30, 4},   {"\\", 0x31, 6}};
constexpr KeySpec kRow3[] = {{"Caps", 0x39, 7}, {"a", 0x04, 4}, {"s", 0x16, 4}, {"d", 0x07, 4},
                             {"f", 0x09, 4},    {"g", 0x0A, 4}, {"h", 0x0B, 4}, {"j", 0x0D, 4},
                             {"k", 0x0E, 4},    {"l", 0x0F, 4}, {";", 0x33, 4}, {"'", 0x34, 4},
                             {"Enter", 0x28, 9}};
constexpr KeySpec kRow4[] = {{"Shift", 0xE1, 9}, {"z", 0x1D, 4}, {"x", 0x1B, 4},
                             {"c", 0x06, 4},     {"v", 0x19, 4}, {"b", 0x05, 4},
                             {"n", 0x11, 4},     {"m", 0x10, 4}, {",", 0x36, 4},
                             {".", 0x37, 4},     {"/", 0x38, 4}, {"Shift", 0xE5, 11}};
constexpr KeySpec kRow5[] = {
    {"Ctrl", 0xE0, 5}, {"Win", 0xE3, 5}, {"Alt", 0xE2, 5}, {"Space", 0x2C, 25}, {"Alt", 0xE6, 5},
    {"Ctrl", 0xE4, 5}, {"<", 0x50, 4},   {"^", 0x52, 4},   {"v", 0x51, 4},      {">", 0x4F, 4}};
constexpr std::span<const KeySpec> kRows[] = {kRow0, kRow1, kRow2, kRow3, kRow4, kRow5};
constexpr uint32_t kKeyColor = 0x2a3444;
constexpr uint8_t kFirstModifierUsage = 0xE0;
} // namespace

void Gui::init_keyboard(lv_obj_t *parent) {
  lv_obj_t *panel = lv_obj_create(parent);
  keyboard_panel_ = panel;
  lv_obj_set_width(panel, LV_PCT(100));
  lv_obj_set_flex_grow(panel, 1);
  lv_obj_set_style_bg_color(panel, lv_color_hex(0x1c2330), 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_radius(panel, 10, 0);
  lv_obj_set_style_pad_all(panel, kPad, 0);
  lv_obj_set_style_pad_row(panel, 6, 0);
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN); // shown when a keyboard is attached

  for (const auto &row_keys : kRows) {
    lv_obj_t *row = lv_obj_create(panel);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, 46);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 4, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    for (const auto &spec : row_keys) {
      lv_obj_t *key = lv_obj_create(row);
      lv_obj_set_height(key, LV_PCT(100));
      lv_obj_set_flex_grow(key, spec.width);
      lv_obj_set_style_radius(key, 6, 0);
      lv_obj_set_style_border_width(key, 0, 0);
      lv_obj_set_style_pad_all(key, 0, 0);
      lv_obj_set_style_bg_color(key, lv_color_hex(kKeyColor), 0);
      lv_obj_clear_flag(key, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_t *label = lv_label_create(key);
      lv_obj_set_style_text_color(label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
      lv_label_set_text(label, spec.label);
      lv_obj_center(label);
      keyboard_keys_[spec.usage] = key;
    }
  }
}

void Gui::set_keyboard_state(uint8_t modifiers, std::span<const uint8_t> keys) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  std::array<bool, 256> pressed{};
  for (uint8_t bit = 0; bit < 8; ++bit)
    if (modifiers & (1u << bit))
      pressed[kFirstModifierUsage + bit] = true;
  for (uint8_t usage : keys)
    if (usage != 0)
      pressed[usage] = true;
  for (size_t usage = 0; usage < keyboard_keys_.size(); ++usage) {
    if (pressed[usage] == shown_keys_[usage])
      continue;
    shown_keys_[usage] = pressed[usage];
    if (auto *key = keyboard_keys_[usage]) {
      lv_obj_set_style_bg_color(
          key, pressed[usage] ? lv_palette_main(LV_PALETTE_GREEN) : lv_color_hex(kKeyColor), 0);
    }
  }
}

void Gui::init_buttons(lv_obj_t *parent) {
  lv_obj_t *row = lv_obj_create(parent);
  lv_obj_set_width(row, LV_PCT(100));
  lv_obj_set_height(row, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(row, lv_color_hex(0x1c2330), 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_radius(row, 10, 0);
  lv_obj_set_style_pad_all(row, kPad, 0);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 8, 0);
  lv_obj_set_style_pad_row(row, 8, 0);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(row);
  lv_obj_set_style_text_color(title, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(title, "Buttons");

  for (size_t i = 0; i < kButtonCount; ++i) {
    lv_obj_t *led = lv_obj_create(row);
    lv_obj_set_size(led, 34, 34);
    lv_obj_set_style_radius(led, 8, 0);
    lv_obj_set_style_border_width(led, 0, 0);
    lv_obj_set_style_bg_color(led, lv_color_hex(0x2a3444), 0);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *n = lv_label_create(led);
    lv_obj_set_style_text_color(n, lv_palette_lighten(LV_PALETTE_GREY, 1), 0);
    lv_label_set_text_fmt(n, "%d", static_cast<int>(i + 1));
    lv_obj_center(n);
    button_leds_[i] = led;
  }
}

void Gui::init_report_line(lv_obj_t *parent) {
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_set_width(box, LV_PCT(100));
  lv_obj_set_height(box, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(box, lv_color_hex(0x1c2330), 0);
  lv_obj_set_style_border_width(box, 0, 0);
  lv_obj_set_style_radius(box, 10, 0);
  lv_obj_set_style_pad_all(box, kPad, 0);
  lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(box, 4, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

  rate_label_ = lv_label_create(box);
  lv_obj_set_style_text_color(rate_label_, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(rate_label_, "Last report: -");

  report_label_ = lv_label_create(box);
  lv_obj_set_width(report_label_, LV_PCT(100));
  lv_label_set_long_mode(report_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(report_label_, &lv_font_unscii_16, 0);
  lv_obj_set_style_text_color(report_label_, lv_palette_lighten(LV_PALETTE_GREEN, 3), 0);
  lv_label_set_text(report_label_, "");

  status_label_ = lv_label_create(box);
  lv_obj_set_width(status_label_, LV_PCT(100));
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(status_label_, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(status_label_, "");
}

void Gui::set_device(const DeviceInfo &info) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  lv_obj_set_style_bg_color(
      device_state_dot_,
      lv_palette_main(info.connected ? (info.is_spacemouse ? LV_PALETTE_GREEN : LV_PALETTE_BLUE)
                                     : LV_PALETTE_GREY),
      0);
  // a keyboard gets the virtual keyboard where the axes normally are
  if (info.connected && info.is_keyboard) {
    lv_obj_add_flag(axes_panel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(keyboard_panel_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(axes_panel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(keyboard_panel_, LV_OBJ_FLAG_HIDDEN);
  }
  for (auto *key : keyboard_keys_)
    if (key)
      lv_obj_set_style_bg_color(key, lv_color_hex(kKeyColor), 0);
  // a new device repaints everything once
  shown_keys_.fill(false);
  shown_axes_.fill(0);
  shown_buttons_.fill(false);
  shown_report_hex_.clear();
  shown_rate_ = -1;
  if (!info.connected) {
    lv_label_set_text(device_title_, "No USB HID device");
    lv_label_set_text(device_detail_, "Plug a device into the USB-A port.");
    for (auto *bar : axis_bars_)
      lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    for (auto *value : axis_values_)
      lv_label_set_text(value, "0");
    for (auto *led : button_leds_)
      lv_obj_set_style_bg_color(led, lv_color_hex(0x2a3444), 0);
    return;
  }
  const std::string title = info.product.empty() ? "USB HID device" : info.product;
  lv_label_set_text(device_title_, title.c_str());
  const std::string detail =
      fmt::format("{}VID {:#06x}  PID {:#06x}  interface {}  protocol {}  descriptor {} B{}",
                  info.manufacturer.empty() ? "" : info.manufacturer + "   ", info.vid, info.pid,
                  info.interface_number, info.protocol, info.report_descriptor_bytes,
                  info.is_spacemouse ? "\n3Dconnexion SpaceMouse: decoding 6-DoF reports"
                  : info.is_keyboard ? "\nKeyboard (boot protocol): pressed keys light up below"
                                     : "\nNot a SpaceMouse: showing raw reports only");
  lv_label_set_text(device_detail_, detail.c_str());
}

void Gui::set_spacemouse_state(const SpaceMouseDecoder::State &state) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  const std::array<int16_t, kAxisCount> values = {state.x,  state.y,  state.z,
                                                  state.rx, state.ry, state.rz};
  for (size_t i = 0; i < kAxisCount; ++i) {
    if (values[i] == shown_axes_[i])
      continue; // unchanged: don't invalidate the bar
    shown_axes_[i] = values[i];
    const int v =
        std::clamp<int>(values[i], -SpaceMouseDecoder::kAxisMax, SpaceMouseDecoder::kAxisMax);
    lv_bar_set_value(axis_bars_[i], v, LV_ANIM_OFF);
    lv_label_set_text_fmt(axis_values_[i], "%d", static_cast<int>(values[i]));
  }
  for (size_t i = 0; i < kButtonCount; ++i) {
    if (state.buttons[i] == shown_buttons_[i])
      continue;
    shown_buttons_[i] = state.buttons[i];
    lv_obj_set_style_bg_color(
        button_leds_[i],
        state.buttons[i] ? lv_palette_main(LV_PALETTE_GREEN) : lv_color_hex(0x2a3444), 0);
  }
}

void Gui::set_last_report(std::span<const uint8_t> report, float reports_per_second) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  std::string hex;
  hex.reserve(report.size() * 3);
  for (size_t i = 0; i < report.size(); ++i) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02X%s", report[i], i + 1 < report.size() ? " " : "");
    hex += buf;
  }
  if (hex != shown_report_hex_) {
    shown_report_hex_ = hex;
    lv_label_set_text(report_label_, hex.c_str());
  }
  const int rate = static_cast<int>(reports_per_second + 0.5f);
  if (rate != shown_rate_) {
    shown_rate_ = rate;
    lv_label_set_text_fmt(rate_label_, "Last report: %d bytes   %d reports/s",
                          static_cast<int>(report.size()), rate);
  }
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  if (text == shown_status_)
    return;
  shown_status_ = std::string(text);
  lv_label_set_text(status_label_, shown_status_.c_str());
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    lv_task_handler();
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, std::chrono::milliseconds(16));
  return false;
}
