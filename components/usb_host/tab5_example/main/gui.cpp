#include "gui.hpp"

#include <algorithm>
#include <cstdio>

#include "format.hpp"

namespace {
constexpr int kPad = 12;
constexpr uint32_t kPanelColor = 0x1c2330;
constexpr uint32_t kInactiveColor = 0x2a3444;
constexpr const char *kAxisNames[] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};
constexpr lv_palette_t kAxisPalettes[] = {LV_PALETTE_BLUE,   LV_PALETTE_BLUE,   LV_PALETTE_BLUE,
                                          LV_PALETTE_ORANGE, LV_PALETTE_ORANGE, LV_PALETTE_ORANGE};
constexpr int kPadSize = 180; ///< the square pads (mouse position, gamepad sticks)
constexpr int kDotSize = 18;
constexpr int32_t kWheelRange = 20; ///< wheel bar: +- this many detents
constexpr int kLedSize = 34;
constexpr const char *kMouseButtonNames[] = {"L", "R", "M", "4", "5"};
/// gamepad indicators: face buttons by position first (the Xbox letters for
/// those positions), then the rest in GamepadReport order
constexpr const char *kGamepadButtonNames[] = {"A",  "B",  "X",  "Y",   "L1",    "R1",  "L2",
                                               "R2", "L3", "R3", "Sel", "Start", "Home"};
static_assert(std::size(kGamepadButtonNames) == 13);

lv_obj_t *make_row(lv_obj_t *parent) {
  lv_obj_t *row = lv_obj_create(parent);
  lv_obj_set_width(row, LV_PCT(100));
  lv_obj_set_height(row, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_pad_all(row, 0, 0);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 10, 0);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
  return row;
}

lv_obj_t *make_column(lv_obj_t *parent) {
  lv_obj_t *col = lv_obj_create(parent);
  lv_obj_set_size(col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(col, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_style_pad_all(col, 0, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(col, 8, 0);
  lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);
  return col;
}

lv_obj_t *make_caption(lv_obj_t *parent, const char *text) {
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_color(label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(label, text);
  return label;
}
} // namespace

const char *Gui::kind_name(Kind kind) {
  switch (kind) {
  case Kind::SpaceMouse:
    return "SpaceMouse";
  case Kind::Keyboard:
    return "keyboard";
  case Kind::Mouse:
    return "mouse";
  case Kind::Gamepad:
    return "gamepad";
  default:
    return "other HID (raw reports)";
  }
}

bool Gui::DeviceInfo::has(Kind kind) const {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [kind](const Interface &i) { return i.kind == kind; });
}

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x101418), 0);

  // one column filling the screen, sized from the live display so it works in
  // either orientation
  root_ = lv_obj_create(screen);
  auto *display = lv_display_get_default();
  lv_obj_set_size(root_, lv_display_get_horizontal_resolution(display),
                  lv_display_get_vertical_resolution(display));
  lv_obj_set_style_bg_opa(root_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(root_, 0, 0);
  lv_obj_set_style_pad_all(root_, kPad, 0);
  lv_obj_set_style_pad_row(root_, kPad, 0);
  lv_obj_set_flex_flow(root_, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

  init_device_card(root_);

  // the kind panels share a scrollable column: a composite device (keyboard +
  // mouse + gamepad behind one receiver) shows all of them, and in landscape
  // that is taller than the screen, so the column scrolls by touch
  panels_ = lv_obj_create(root_);
  lv_obj_set_width(panels_, LV_PCT(100));
  lv_obj_set_flex_grow(panels_, 1);
  lv_obj_set_style_bg_opa(panels_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(panels_, 0, 0);
  lv_obj_set_style_pad_all(panels_, 0, 0);
  lv_obj_set_style_pad_row(panels_, kPad, 0);
  lv_obj_set_flex_flow(panels_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(panels_, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(panels_, LV_SCROLLBAR_MODE_AUTO);

  init_axes(panels_);
  init_keyboard(panels_);
  init_mouse(panels_);
  init_gamepad(panels_);
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
  lv_obj_set_style_bg_color(card, lv_color_hex(kPanelColor), 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_pad_all(card, kPad, 0);
  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(card, 4, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title_row = make_row(card);

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

lv_obj_t *Gui::init_panel(lv_obj_t *parent, const char *title) {
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_set_width(panel, LV_PCT(100));
  lv_obj_set_height(panel, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(panel, lv_color_hex(kPanelColor), 0);
  lv_obj_set_style_border_width(panel, 0, 0);
  lv_obj_set_style_radius(panel, 10, 0);
  lv_obj_set_style_pad_all(panel, kPad, 0);
  lv_obj_set_style_pad_row(panel, 8, 0);
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN); // shown while that kind is attached
  make_caption(panel, title);
  return panel;
}

Gui::Pad Gui::make_pad(lv_obj_t *parent, int size) {
  lv_obj_t *pad = lv_obj_create(parent);
  lv_obj_set_size(pad, size, size);
  lv_obj_set_style_bg_color(pad, lv_color_hex(kInactiveColor), 0);
  lv_obj_set_style_border_width(pad, 0, 0);
  lv_obj_set_style_radius(pad, 8, 0);
  lv_obj_set_style_pad_all(pad, 0, 0);
  lv_obj_clear_flag(pad, LV_OBJ_FLAG_SCROLLABLE);
  // crosshair
  for (int i = 0; i < 2; ++i) {
    lv_obj_t *line = lv_obj_create(pad);
    lv_obj_set_size(line, i == 0 ? size : 1, i == 0 ? 1 : size);
    lv_obj_set_pos(line, i == 0 ? 0 : size / 2, i == 0 ? size / 2 : 0);
    lv_obj_set_style_bg_color(line, lv_color_hex(kPanelColor), 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_radius(line, 0, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
  }
  lv_obj_t *dot = lv_obj_create(pad);
  lv_obj_set_size(dot, kDotSize, kDotSize);
  lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(dot, 0, 0);
  lv_obj_set_style_bg_color(dot, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
  Pad result{.obj = pad, .dot = dot, .size = size};
  move_dot(result, 0, 0, 1);
  return result;
}

void Gui::move_dot(const Pad &pad, int32_t x, int32_t y, int32_t range) {
  // pixels from the center to the edge, from the size the pad was made with
  // (its laid-out width is not valid until LVGL has run a layout pass)
  const int travel = (pad.size - kDotSize) / 2;
  const int cx = std::clamp<int32_t>(x, -range, range) * travel / range;
  const int cy = std::clamp<int32_t>(y, -range, range) * travel / range;
  lv_obj_set_pos(pad.dot, travel + cx, travel + cy);
}

lv_obj_t *Gui::make_led(lv_obj_t *parent, const char *label, int size) {
  lv_obj_t *led = lv_obj_create(parent);
  lv_obj_set_size(led, size, size);
  lv_obj_set_style_radius(led, 8, 0);
  lv_obj_set_style_border_width(led, 0, 0);
  lv_obj_set_style_pad_all(led, 0, 0);
  lv_obj_set_style_bg_color(led, lv_color_hex(kInactiveColor), 0);
  lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *n = lv_label_create(led);
  lv_obj_set_style_text_color(n, lv_palette_lighten(LV_PALETTE_GREY, 1), 0);
  lv_label_set_text(n, label);
  lv_obj_center(n);
  return led;
}

void Gui::set_led(lv_obj_t *led, bool on) {
  lv_obj_set_style_bg_color(
      led, on ? lv_palette_main(LV_PALETTE_GREEN) : lv_color_hex(kInactiveColor), 0);
}

void Gui::init_axes(lv_obj_t *parent) {
  lv_obj_t *panel = init_panel(parent, "SpaceMouse: translation (blue) and rotation (orange)");
  axes_panel_ = panel;

  for (size_t i = 0; i < kAxisCount; ++i) {
    lv_obj_t *row = make_row(panel);

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
    lv_obj_set_style_bg_color(bar, lv_color_hex(kInactiveColor), LV_PART_MAIN);
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

  // the SpaceMouse's buttons, in a wrapping row under the axes
  lv_obj_t *buttons = lv_obj_create(panel);
  buttons_panel_ = buttons;
  lv_obj_set_width(buttons, LV_PCT(100));
  lv_obj_set_height(buttons, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(buttons, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(buttons, 0, 0);
  lv_obj_set_style_pad_all(buttons, 0, 0);
  lv_obj_set_flex_flow(buttons, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(buttons, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(buttons, 8, 0);
  lv_obj_set_style_pad_row(buttons, 8, 0);
  lv_obj_clear_flag(buttons, LV_OBJ_FLAG_SCROLLABLE);
  make_caption(buttons, "Buttons");
  for (size_t i = 0; i < kButtonCount; ++i) {
    const std::string label = std::to_string(i + 1);
    button_leds_[i] = make_led(buttons, label.c_str(), kLedSize);
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
// are the modifier keys (their own usages, which the decoder reports like any
// other key).
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
} // namespace

void Gui::init_keyboard(lv_obj_t *parent) {
  lv_obj_t *panel = init_panel(parent, "Keyboard: pressed keys light up");
  keyboard_panel_ = panel;
  lv_obj_set_style_pad_row(panel, 6, 0);

  for (const auto &row_keys : kRows) {
    lv_obj_t *row = lv_obj_create(panel);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, 40);
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
      lv_obj_set_style_bg_color(key, lv_color_hex(kInactiveColor), 0);
      lv_obj_clear_flag(key, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_t *label = lv_label_create(key);
      lv_obj_set_style_text_color(label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
      lv_label_set_text(label, spec.label);
      lv_obj_center(label);
      keyboard_keys_[spec.usage] = key;
    }
  }
}

void Gui::init_mouse(lv_obj_t *parent) {
  lv_obj_t *panel = init_panel(parent, "Mouse: the dot follows the motion; wheel and buttons");
  mouse_panel_ = panel;

  lv_obj_t *row = make_row(panel);
  lv_obj_set_style_pad_column(row, kPad * 2, 0);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  mouse_pad_ = make_pad(row, kPadSize);

  lv_obj_t *col = make_column(row);
  mouse_position_ = lv_label_create(col);
  lv_obj_set_style_text_font(mouse_position_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(mouse_position_, lv_color_white(), 0);
  lv_label_set_text(mouse_position_, "x 0   y 0");

  lv_obj_t *wheel_row = make_row(col);
  lv_obj_set_width(wheel_row, LV_SIZE_CONTENT);
  make_caption(wheel_row, "Wheel");
  mouse_wheel_bar_ = lv_bar_create(wheel_row);
  lv_obj_set_size(mouse_wheel_bar_, 200, 20);
  lv_bar_set_mode(mouse_wheel_bar_, LV_BAR_MODE_SYMMETRICAL);
  lv_bar_set_range(mouse_wheel_bar_, -kWheelRange, kWheelRange);
  lv_bar_set_value(mouse_wheel_bar_, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(mouse_wheel_bar_, lv_color_hex(kInactiveColor), LV_PART_MAIN);
  lv_obj_set_style_bg_color(mouse_wheel_bar_, lv_palette_main(LV_PALETTE_ORANGE),
                            LV_PART_INDICATOR);
  mouse_wheel_value_ = lv_label_create(wheel_row);
  lv_obj_set_width(mouse_wheel_value_, 60);
  lv_obj_set_style_text_font(mouse_wheel_value_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(mouse_wheel_value_, lv_color_white(), 0);
  lv_label_set_text(mouse_wheel_value_, "0");

  lv_obj_t *buttons = make_row(col);
  lv_obj_set_width(buttons, LV_SIZE_CONTENT);
  make_caption(buttons, "Buttons");
  for (size_t i = 0; i < kMouseButtonCount; ++i) {
    mouse_leds_[i] = make_led(buttons, kMouseButtonNames[i], kLedSize);
  }
}

void Gui::init_gamepad(lv_obj_t *parent) {
  lv_obj_t *panel =
      init_panel(parent, "Gamepad: sticks, d-pad and buttons (face buttons by position)");
  gamepad_panel_ = panel;

  // wraps in portrait: [left stick] [d-pad] [right stick] [face buttons] [others]
  lv_obj_t *row = make_row(panel);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_style_pad_column(row, kPad * 2, 0);
  lv_obj_set_style_pad_row(row, kPad, 0);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

  left_pad_ = make_pad(row, kPadSize);

  // d-pad and face buttons: 3x3 grids of indicator cells, laid out by position
  auto make_cross = [&](std::array<lv_obj_t *, 4> &leds, const char *const names[4]) {
    // names: up, down, left, right (or north, south, west, east)
    const int cell = kLedSize + 4;
    lv_obj_t *grid = lv_obj_create(row);
    lv_obj_set_size(grid, cell * 3, cell * 3);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    const int positions[4][2] = {{1, 0}, {1, 2}, {0, 1}, {2, 1}}; // column, row
    for (int i = 0; i < 4; ++i) {
      leds[i] = make_led(grid, names[i], kLedSize);
      lv_obj_set_pos(leds[i], positions[i][0] * cell + 2, positions[i][1] * cell + 2);
    }
  };
  static constexpr const char *kDpadNames[4] = {"^", "v", "<", ">"};
  make_cross(dpad_leds_, kDpadNames);

  right_pad_ = make_pad(row, kPadSize);

  // face buttons: north, south, west, east -> indices 3, 0, 2, 1 of the names
  std::array<lv_obj_t *, 4> face{};
  static constexpr const char *kFaceNames[4] = {kGamepadButtonNames[3], kGamepadButtonNames[0],
                                                kGamepadButtonNames[2], kGamepadButtonNames[1]};
  make_cross(face, kFaceNames);
  gamepad_leds_[3] = face[0]; // north
  gamepad_leds_[0] = face[1]; // south
  gamepad_leds_[2] = face[2]; // west
  gamepad_leds_[1] = face[3]; // east

  // the rest as a wrapping block of indicators
  lv_obj_t *others = lv_obj_create(row);
  lv_obj_set_size(others, (kLedSize + 8) * 3 + 8, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(others, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(others, 0, 0);
  lv_obj_set_style_pad_all(others, 0, 0);
  lv_obj_set_flex_flow(others, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_style_pad_column(others, 8, 0);
  lv_obj_set_style_pad_row(others, 8, 0);
  lv_obj_clear_flag(others, LV_OBJ_FLAG_SCROLLABLE);
  for (size_t i = 4; i < kGamepadButtonCount; ++i) {
    gamepad_leds_[i] = make_led(others, kGamepadButtonNames[i], kLedSize + 8);
  }

  stick_values_ = lv_label_create(panel);
  lv_obj_set_style_text_font(stick_values_, &lv_font_unscii_16, 0);
  lv_obj_set_style_text_color(stick_values_, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_label_set_text(stick_values_, "L 0,0   R 0,0   hat -");
}

void Gui::init_report_line(lv_obj_t *parent) {
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_set_width(box, LV_PCT(100));
  lv_obj_set_height(box, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(box, lv_color_hex(kPanelColor), 0);
  lv_obj_set_style_border_width(box, 0, 0);
  lv_obj_set_style_radius(box, 10, 0);
  lv_obj_set_style_pad_all(box, kPad, 0);
  lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(box, 4, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

  rate_label_ = make_caption(box, "Last report: -");

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

void Gui::reset_panels() {
  for (auto *key : keyboard_keys_)
    if (key)
      set_led(key, false);
  for (auto *bar : axis_bars_)
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
  for (auto *value : axis_values_)
    lv_label_set_text(value, "0");
  for (auto *led : button_leds_)
    set_led(led, false);
  for (auto *led : mouse_leds_)
    set_led(led, false);
  for (auto *led : dpad_leds_)
    set_led(led, false);
  for (auto *led : gamepad_leds_)
    set_led(led, false);
  move_dot(mouse_pad_, 0, 0, 1);
  move_dot(left_pad_, 0, 0, 1);
  move_dot(right_pad_, 0, 0, 1);
  lv_bar_set_value(mouse_wheel_bar_, 0, LV_ANIM_OFF);
  lv_label_set_text(mouse_wheel_value_, "0");
  lv_label_set_text(mouse_position_, "x 0   y 0");
  lv_label_set_text(stick_values_, "L 0,0   R 0,0   hat -");
  shown_keys_.fill(false);
  shown_axes_.fill(0);
  shown_buttons_.fill(false);
  shown_mouse_ = {};
  shown_gamepad_ = {};
  shown_gamepad_buttons_.fill(false);
  shown_report_hex_.clear();
  shown_rate_ = -1;
}

void Gui::set_device(const DeviceInfo &info) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  const bool recognised = info.has(Kind::SpaceMouse) || info.has(Kind::Keyboard) ||
                          info.has(Kind::Mouse) || info.has(Kind::Gamepad);
  lv_obj_set_style_bg_color(device_state_dot_,
                            lv_palette_main(!info.connected              ? LV_PALETTE_GREY
                                            : info.has(Kind::SpaceMouse) ? LV_PALETTE_GREEN
                                            : recognised                 ? LV_PALETTE_BLUE
                                                                         : LV_PALETTE_ORANGE),
                            0);
  // one panel per kind the device carries
  const auto show = [&](lv_obj_t *panel, bool on) {
    if (on)
      lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
  };
  show(axes_panel_, info.connected && info.has(Kind::SpaceMouse));
  show(keyboard_panel_, info.connected && info.has(Kind::Keyboard));
  show(mouse_panel_, info.connected && info.has(Kind::Mouse));
  show(gamepad_panel_, info.connected && info.has(Kind::Gamepad));
  // Only a change of device starts over. A composite device opens its
  // interfaces one at a time, and each one updates the card; wiping the panels
  // (keys held, motion accumulated) and the scroll position for that would be
  // wrong, so those survive while the same device is still attached.
  const bool new_device =
      info.connected != shown_connected_ || info.vid != shown_vid_ || info.pid != shown_pid_;
  shown_connected_ = info.connected;
  shown_vid_ = info.vid;
  shown_pid_ = info.pid;
  if (new_device) {
    lv_obj_scroll_to_y(panels_, 0, LV_ANIM_OFF);
    reset_panels();
  }
  if (!info.connected) {
    lv_label_set_text(device_title_, "No USB HID device");
    lv_label_set_text(device_detail_, "Plug a device into the USB-A port.");
    lv_label_set_text(report_label_, "");
    lv_label_set_text(rate_label_, "Last report: -");
    return;
  }
  const std::string title = info.product.empty() ? "USB HID device" : info.product;
  lv_label_set_text(device_title_, title.c_str());
  std::string detail =
      fmt::format("{}VID {:#06x}  PID {:#06x}",
                  info.manufacturer.empty() ? "" : info.manufacturer + "   ", info.vid, info.pid);
  for (const auto &iface : info.interfaces) {
    detail +=
        fmt::format("\ninterface {}  protocol {}  descriptor {} B  ->  {}", iface.interface_number,
                    iface.protocol, iface.report_descriptor_bytes, kind_name(iface.kind));
  }
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
    lv_label_set_text_fmt(axis_values_[i], "%d", v); // the value the bar shows
  }
  for (size_t i = 0; i < kButtonCount; ++i) {
    if (state.buttons[i] == shown_buttons_[i])
      continue;
    shown_buttons_[i] = state.buttons[i];
    set_led(button_leds_[i], state.buttons[i]);
  }
}

void Gui::set_keyboard_state(const espp::hid_rp::KeyboardReport &report) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  for (size_t usage = 0; usage < keyboard_keys_.size(); ++usage) {
    const bool pressed = report.pressed(static_cast<uint8_t>(usage));
    if (pressed == shown_keys_[usage])
      continue;
    shown_keys_[usage] = pressed;
    if (auto *key = keyboard_keys_[usage])
      set_led(key, pressed);
  }
}

void Gui::set_mouse_state(const MouseState &state) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  if (state.x != shown_mouse_.x || state.y != shown_mouse_.y) {
    move_dot(mouse_pad_, state.x, state.y, kMouseRange);
    lv_label_set_text_fmt(mouse_position_, "x %d   y %d", static_cast<int>(state.x),
                          static_cast<int>(state.y));
  }
  if (state.wheel != shown_mouse_.wheel) {
    lv_bar_set_value(mouse_wheel_bar_, std::clamp<int32_t>(state.wheel, -kWheelRange, kWheelRange),
                     LV_ANIM_OFF);
    lv_label_set_text_fmt(mouse_wheel_value_, "%d", static_cast<int>(state.wheel));
  }
  for (size_t i = 0; i < kMouseButtonCount; ++i) {
    const bool on = (state.buttons >> i) & 1;
    if (on != ((shown_mouse_.buttons >> i) & 1))
      set_led(mouse_leds_[i], on);
  }
  shown_mouse_ = state;
}

void Gui::set_gamepad_state(const espp::hid_rp::GamepadReport &report) {
  const auto &r = report;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!root_)
    return;
  const auto &s = shown_gamepad_;
  if (r.lx != s.lx || r.ly != s.ly)
    move_dot(left_pad_, r.lx, r.ly, 32767);
  if (r.rx != s.rx || r.ry != s.ry)
    move_dot(right_pad_, r.rx, r.ry, 32767);
  if (r.lx != s.lx || r.ly != s.ly || r.rx != s.rx || r.ry != s.ry || r.hat != s.hat) {
    if (r.hat < 0)
      lv_label_set_text_fmt(stick_values_, "L %d,%d   R %d,%d   hat -", r.lx, r.ly, r.rx, r.ry);
    else
      lv_label_set_text_fmt(stick_values_, "L %d,%d   R %d,%d   hat %d", r.lx, r.ly, r.rx, r.ry,
                            static_cast<int>(r.hat));
  }
  const bool dpad[4] = {r.up, r.down, r.left, r.right};
  const bool shown_dpad[4] = {s.up, s.down, s.left, s.right};
  for (int i = 0; i < 4; ++i)
    if (dpad[i] != shown_dpad[i])
      set_led(dpad_leds_[i], dpad[i]);
  const bool buttons[kGamepadButtonCount] = {r.south,  r.east,  r.west, r.north, r.l1,
                                             r.r1,     r.l2,    r.r2,   r.l3,    r.r3,
                                             r.select, r.start, r.home};
  for (size_t i = 0; i < kGamepadButtonCount; ++i) {
    if (buttons[i] != shown_gamepad_buttons_[i]) {
      shown_gamepad_buttons_[i] = buttons[i];
      set_led(gamepad_leds_[i], buttons[i]);
    }
  }
  shown_gamepad_ = r;
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
