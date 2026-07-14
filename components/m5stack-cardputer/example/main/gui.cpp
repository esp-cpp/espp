#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_background();
  init_textarea();
  init_status_bar();
  init_imu_popup();
  init_help_panel();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
  background_ = nullptr;
  textarea_ = nullptr;
  status_label_ = nullptr;
  imu_popup_ = nullptr;
  imu_label_ = nullptr;
  help_panel_ = nullptr;
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

void Gui::init_imu_popup() {
  // a small popup in the top-right corner holding the live IMU readings and
  // a hint for how to close it
  imu_popup_ = lv_obj_create(background_);
  lv_obj_set_size(imu_popup_, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(imu_popup_, LV_ALIGN_TOP_RIGHT, -2, 2);
  lv_obj_set_style_pad_all(imu_popup_, 4, LV_PART_MAIN);
  lv_obj_add_flag(imu_popup_, LV_OBJ_FLAG_HIDDEN);
  imu_label_ = lv_label_create(imu_popup_);
  lv_obj_align(imu_label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(imu_label_, "IMU...");
}

void Gui::init_help_panel() {
  // a centered popup listing the controls; hidden until toggled via fn+1.
  // The content is taller than the panel, so the panel is scrollable (see
  // handle_special_key(): the fn+arrow keys scroll it while it is open)
  help_panel_ = lv_obj_create(background_);
  lv_obj_set_size(help_panel_, lv_pct(92), lv_pct(92));
  lv_obj_center(help_panel_);
  lv_obj_set_style_pad_all(help_panel_, 6, LV_PART_MAIN);
  lv_obj_add_flag(help_panel_, LV_OBJ_FLAG_HIDDEN);
  auto *label = lv_label_create(help_panel_);
  // constrain the label to the panel's content width so long lines wrap
  // instead of running off screen
  lv_obj_set_width(label, lv_pct(100));
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(label, HELP_TEXT);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);
}

bool Gui::toggle_imu_visible() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imu_visible_ = !imu_visible_;
  if (imu_popup_) {
    if (imu_visible_) {
      lv_obj_remove_flag(imu_popup_, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(imu_popup_);
    } else {
      lv_obj_add_flag(imu_popup_, LV_OBJ_FLAG_HIDDEN);
    }
  }
  return imu_visible_;
}

bool Gui::toggle_help() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  help_visible_ = !help_visible_;
  if (help_panel_) {
    if (help_visible_) {
      lv_obj_remove_flag(help_panel_, LV_OBJ_FLAG_HIDDEN);
      // make sure the popup is on top of the other UI elements
      lv_obj_move_foreground(help_panel_);
    } else {
      lv_obj_add_flag(help_panel_, LV_OBJ_FLAG_HIDDEN);
    }
  }
  return help_visible_;
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
  if (help_visible_ && help_panel_) {
    // while the help popup is open, the arrow keys scroll it and esc closes
    // it instead of acting on the text area
    switch (key) {
    case SpecialKey::UP:
      lv_obj_scroll_to_y(help_panel_, lv_obj_get_scroll_y(help_panel_) - 20, LV_ANIM_ON);
      return;
    case SpecialKey::DOWN:
      lv_obj_scroll_to_y(help_panel_, lv_obj_get_scroll_y(help_panel_) + 20, LV_ANIM_ON);
      return;
    case SpecialKey::LEFT:
    case SpecialKey::RIGHT:
      // ignored while the help popup is open
      return;
    case SpecialKey::ESC:
      toggle_help();
      return;
    default:
      break;
    }
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
  // append the close hint so the popup is self-documenting (and copy since
  // string_view is not guaranteed to be null-terminated)
  const std::string text_str = std::string(text) + "\nfn+2 to close";
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
