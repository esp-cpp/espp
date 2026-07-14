#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_background();
  init_textarea();
  init_status_bar();
  init_imu_label();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
  background_ = nullptr;
  textarea_ = nullptr;
  status_label_ = nullptr;
  imu_label_ = nullptr;
}

void Gui::init_background() {
  background_ = lv_obj_create(lv_screen_active());
  lv_obj_set_size(background_, lv_pct(100), lv_pct(100));
  lv_obj_center(background_);
  lv_obj_set_style_border_width(background_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(background_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(background_, 0, LV_PART_MAIN);
}

void Gui::init_textarea() {
  textarea_ = lv_textarea_create(background_);
  lv_obj_set_size(textarea_, lv_pct(100), lv_pct(80));
  lv_obj_align(textarea_, LV_ALIGN_TOP_MID, 0, 0);
  lv_textarea_set_placeholder_text(textarea_, "Type on the keyboard...");
  // the keyboard scanner callback drives this text area; there is no
  // touchscreen so it never needs to be clickable
  lv_obj_remove_flag(textarea_, LV_OBJ_FLAG_CLICKABLE);
  // make sure the cursor is visible
  lv_obj_add_state(textarea_, LV_STATE_FOCUSED);
}

void Gui::init_status_bar() {
  status_label_ = lv_label_create(background_);
  lv_obj_set_width(status_label_, lv_pct(100));
  lv_obj_align(status_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_CLIP);
  lv_label_set_text(status_label_, "Ready");
}

void Gui::init_imu_label() {
  // floats over the top-right corner of the text area; empty (and therefore
  // invisible) until the first set_imu_text() call
  imu_label_ = lv_label_create(background_);
  lv_obj_align(imu_label_, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_label_set_text(imu_label_, "");
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
    // the other special keys (F1-F12) are just shown in the status bar by
    // the main loop
    break;
  }
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!status_label_) {
    return;
  }
  // string_view is not guaranteed to be null-terminated
  const std::string text_str(text);
  lv_label_set_text(status_label_, text_str.c_str());
}

void Gui::set_imu_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!imu_label_) {
    return;
  }
  // string_view is not guaranteed to be null-terminated
  const std::string text_str(text);
  lv_label_set_text(imu_label_, text_str.c_str());
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
