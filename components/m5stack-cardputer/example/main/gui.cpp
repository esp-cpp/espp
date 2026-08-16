#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_tabview();
  init_text_tab();
  init_lora_tab();
  init_imu_tab();
  init_gps_tab();
  init_sys_tab();
  init_help_tab();
  init_status_bar();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
  tabview_ = nullptr;
  text_tab_ = nullptr;
  lora_tab_ = nullptr;
  imu_tab_ = nullptr;
  gps_tab_ = nullptr;
  sys_tab_ = nullptr;
  help_tab_ = nullptr;
  textarea_ = nullptr;
  status_label_ = nullptr;
  imu_label_ = nullptr;
  gps_label_ = nullptr;
  sys_label_ = nullptr;
  lora_status_label_ = nullptr;
  lora_log_label_ = nullptr;
}

void Gui::init_tabview() {
  // the tabview fills the screen except for the status bar along the bottom;
  // the tab bar (top) is the page switcher (driven from the keyboard - there
  // is no touchscreen)
  int width = lv_display_get_horizontal_resolution(lv_display_get_default());
  int height = lv_display_get_vertical_resolution(lv_display_get_default());
  tabview_ = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_position(tabview_, LV_DIR_TOP);
  lv_tabview_set_tab_bar_size(tabview_, TAB_BAR_HEIGHT);
  lv_obj_set_size(tabview_, width, height - STATUS_BAR_HEIGHT);
  lv_obj_align(tabview_, LV_ALIGN_TOP_MID, 0, 0);
  text_tab_ = lv_tabview_add_tab(tabview_, "Text");
  lora_tab_ = lv_tabview_add_tab(tabview_, "LoRa");
  imu_tab_ = lv_tabview_add_tab(tabview_, "IMU");
  gps_tab_ = lv_tabview_add_tab(tabview_, "GPS");
  sys_tab_ = lv_tabview_add_tab(tabview_, "Sys");
  help_tab_ = lv_tabview_add_tab(tabview_, "Help");
  // there is no touchscreen, so the content should not be swipe-scrollable
  // (tabs are switched from the keyboard); tighten the tab content padding to
  // make the most of the small screen
  lv_obj_set_style_pad_all(lv_tabview_get_content(tabview_), 2, LV_PART_MAIN);

  // All six tabs do not fit across the 240px screen. LVGL flex-grows the tab
  // buttons to equal width by default, which crams / clips the labels; instead
  // give each button its natural width and let the tab bar scroll
  // horizontally. The active button is scrolled into view on every tab change
  // (see scroll_active_tab_into_view()).
  lv_obj_t *tab_bar = lv_tabview_get_tab_bar(tabview_);
  lv_obj_set_scrollbar_mode(tab_bar, LV_SCROLLBAR_MODE_OFF);
  uint32_t tab_count = lv_tabview_get_tab_count(tabview_);
  for (uint32_t i = 0; i < tab_count; i++) {
    lv_obj_t *btn = lv_tabview_get_tab_button(tabview_, static_cast<int32_t>(i));
    if (btn) {
      lv_obj_set_flex_grow(btn, 0);
      lv_obj_set_width(btn, LV_SIZE_CONTENT);
      lv_obj_set_style_pad_hor(btn, 10, LV_PART_MAIN);
    }
  }
  scroll_active_tab_into_view();
}

void Gui::init_text_tab() {
  textarea_ = lv_textarea_create(text_tab_);
  lv_obj_set_size(textarea_, lv_pct(100), lv_pct(100));
  lv_obj_align(textarea_, LV_ALIGN_TOP_MID, 0, 0);
  lv_textarea_set_placeholder_text(textarea_, "Type here; fn+0 sends over LoRa");
  // the keyboard scanner callback drives this text area; there is no
  // touchscreen so it never needs to be clickable
  lv_obj_remove_flag(textarea_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_state(textarea_, LV_STATE_FOCUSED);
}

void Gui::init_lora_tab() {
  // a status line at the top and a scrolling log of sent / received messages
  lv_obj_set_flex_flow(lora_tab_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(lora_tab_, 4, LV_PART_MAIN);

  lora_status_label_ = lv_label_create(lora_tab_);
  lv_obj_set_width(lora_status_label_, lv_pct(100));
  lv_label_set_long_mode(lora_status_label_, LV_LABEL_LONG_WRAP);
  lv_label_set_text(lora_status_label_, "LoRa: initializing...");

  lora_log_label_ = lv_label_create(lora_tab_);
  lv_obj_set_width(lora_log_label_, lv_pct(100));
  lv_label_set_long_mode(lora_log_label_, LV_LABEL_LONG_WRAP);
  lv_label_set_text(lora_log_label_, "No messages yet.");
}

void Gui::init_imu_tab() {
  imu_label_ = lv_label_create(imu_tab_);
  lv_obj_set_width(imu_label_, lv_pct(100));
  lv_label_set_long_mode(imu_label_, LV_LABEL_LONG_WRAP);
  lv_obj_align(imu_label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(imu_label_, "IMU...");
}

void Gui::init_gps_tab() {
  gps_label_ = lv_label_create(gps_tab_);
  lv_obj_set_width(gps_label_, lv_pct(100));
  lv_label_set_long_mode(gps_label_, LV_LABEL_LONG_WRAP);
  lv_obj_align(gps_label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(gps_label_, "GPS...");
}

void Gui::init_sys_tab() {
  sys_label_ = lv_label_create(sys_tab_);
  lv_obj_set_width(sys_label_, lv_pct(100));
  lv_label_set_long_mode(sys_label_, LV_LABEL_LONG_WRAP);
  lv_obj_align(sys_label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(sys_label_, "System...");
}

void Gui::init_help_tab() {
  auto *label = lv_label_create(help_tab_);
  lv_obj_set_width(label, lv_pct(100));
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(label, HELP_TEXT);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);
}

void Gui::init_status_bar() {
  // a slim bar along the very bottom, outside the tabview, so transient
  // messages stay visible regardless of the active tab
  int width = lv_display_get_horizontal_resolution(lv_display_get_default());
  status_label_ = lv_label_create(lv_screen_active());
  lv_obj_set_size(status_label_, width, STATUS_BAR_HEIGHT);
  lv_obj_align(status_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_CLIP);
  lv_label_set_text(status_label_, "Ready");
}

Gui::Tab Gui::next_tab() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint32_t current = lv_tabview_get_tab_active(tabview_);
  uint32_t next = (current + 1) % static_cast<uint32_t>(Tab::COUNT);
  lv_tabview_set_active(tabview_, next, LV_ANIM_OFF);
  scroll_active_tab_into_view();
  return static_cast<Tab>(next);
}

void Gui::select_tab(Tab tab) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_tabview_set_active(tabview_, static_cast<uint32_t>(tab), LV_ANIM_OFF);
  scroll_active_tab_into_view();
}

void Gui::scroll_active_tab_into_view() {
  if (!tabview_) {
    return;
  }
  lv_obj_t *btn = lv_tabview_get_tab_button(
      tabview_, static_cast<int32_t>(lv_tabview_get_tab_active(tabview_)));
  if (btn) {
    lv_obj_scroll_to_view(btn, LV_ANIM_ON);
  }
}

Gui::Tab Gui::active_tab() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return static_cast<Tab>(lv_tabview_get_tab_active(tabview_));
}

void Gui::add_char(char c) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!textarea_) {
    return;
  }
  if (c == '\b') {
    lv_textarea_delete_char(textarea_);
  } else {
    lv_textarea_add_char(textarea_, c);
  }
}

void Gui::handle_special_key(SpecialKey key) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // On tabs other than Text, the up / down keys scroll the page so its content
  // (the LoRa log, the help text, ...) can be read - there is no touchscreen to
  // drag it. The editing keys only act on the Text tab.
  if (static_cast<Tab>(lv_tabview_get_tab_active(tabview_)) != Tab::TEXT) {
    lv_obj_t *page = active_tab_content();
    if (!page) {
      return;
    }
    static constexpr int scroll_step = 24;
    if (key == SpecialKey::UP) {
      lv_obj_scroll_to_y(page, lv_obj_get_scroll_y(page) - scroll_step, LV_ANIM_ON);
    } else if (key == SpecialKey::DOWN) {
      lv_obj_scroll_to_y(page, lv_obj_get_scroll_y(page) + scroll_step, LV_ANIM_ON);
    }
    return;
  }
  if (!textarea_) {
    return;
  }
  switch (key) {
  case SpecialKey::LEFT:
    lv_textarea_cursor_left(textarea_);
    break;
  case SpecialKey::RIGHT:
    lv_textarea_cursor_right(textarea_);
    break;
  case SpecialKey::UP:
    lv_textarea_cursor_up(textarea_);
    break;
  case SpecialKey::DOWN:
    lv_textarea_cursor_down(textarea_);
    break;
  case SpecialKey::DELETE:
    lv_textarea_delete_char_forward(textarea_);
    break;
  case SpecialKey::ESC:
    lv_textarea_set_text(textarea_, "");
    break;
  default:
    // the other special keys (F1-F12) are handled by the example (tab
    // switching, audio, LoRa)
    break;
  }
}

lv_obj_t *Gui::active_tab_content() {
  switch (static_cast<Tab>(lv_tabview_get_tab_active(tabview_))) {
  case Tab::TEXT:
    return text_tab_;
  case Tab::LORA:
    return lora_tab_;
  case Tab::IMU:
    return imu_tab_;
  case Tab::GPS:
    return gps_tab_;
  case Tab::SYS:
    return sys_tab_;
  case Tab::HELP:
    return help_tab_;
  default:
    return nullptr;
  }
}

std::string Gui::get_text() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!textarea_) {
    return {};
  }
  const char *text = lv_textarea_get_text(textarea_);
  return text ? std::string(text) : std::string();
}

void Gui::clear_text() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (textarea_) {
    lv_textarea_set_text(textarea_, "");
  }
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!status_label_) {
    return;
  }
  const std::string text_str(text);
  lv_label_set_text(status_label_, text_str.c_str());
}

void Gui::set_imu_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!imu_label_) {
    return;
  }
  const std::string text_str(text);
  lv_label_set_text(imu_label_, text_str.c_str());
}

void Gui::set_system_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!sys_label_) {
    return;
  }
  const std::string text_str(text);
  lv_label_set_text(sys_label_, text_str.c_str());
}

void Gui::set_gps_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!gps_label_) {
    return;
  }
  const std::string text_str(text);
  lv_label_set_text(gps_label_, text_str.c_str());
}

void Gui::set_lora_status(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!lora_status_label_) {
    return;
  }
  const std::string text_str(text);
  lv_label_set_text(lora_status_label_, text_str.c_str());
}

void Gui::add_lora_message(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lora_messages_.emplace_front(text);
  while (lora_messages_.size() > MAX_LORA_MESSAGES) {
    lora_messages_.pop_back();
  }
  update_lora_log();
}

void Gui::update_lora_log() {
  if (!lora_log_label_) {
    return;
  }
  if (lora_messages_.empty()) {
    lv_label_set_text(lora_log_label_, "No messages yet.");
    return;
  }
  std::string text;
  for (const auto &message : lora_messages_) {
    if (!text.empty()) {
      text += "\n";
    }
    text += message;
  }
  lv_label_set_text(lora_log_label_, text.c_str());
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    lv_task_handler();
  }
  using namespace std::chrono_literals;
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, 16ms);
  return false; // don't stop the task
}
