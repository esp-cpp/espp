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
    const int v =
        std::clamp<int>(values[i], -SpaceMouseDecoder::kAxisMax, SpaceMouseDecoder::kAxisMax);
    lv_bar_set_value(axis_bars_[i], v, LV_ANIM_OFF);
    lv_label_set_text_fmt(axis_values_[i], "%d", static_cast<int>(values[i]));
  }
  for (size_t i = 0; i < kButtonCount; ++i) {
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
  lv_label_set_text(report_label_, hex.c_str());
  lv_label_set_text_fmt(rate_label_, "Last report: %d bytes   %.0f reports/s",
                        static_cast<int>(report.size()), static_cast<double>(reports_per_second));
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  lv_label_set_text(status_label_, std::string(text).c_str());
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
