#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_title();
  init_status_lines();
  init_arc();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
}

void Gui::init_title() {
  title_ = lv_label_create(lv_screen_active());
  lv_label_set_text(title_, "espp in QEMU");
  lv_obj_set_style_text_font(title_, &lv_font_montserrat_14, 0);
  lv_obj_align(title_, LV_ALIGN_TOP_MID, 0, 8);
}

void Gui::init_status_lines() {
  // three left-aligned lines under the title; each wraps if it gets long
  auto make_line = [this](int index, const char *initial) {
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, lv_pct(60));
    lv_label_set_text(label, initial);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 8, 36 + index * 22);
    return label;
  };
  uptime_label_ = make_line(0, "Uptime: 0 s");
  ethernet_label_ = make_line(1, "Ethernet: starting");
  sdcard_label_ = make_line(2, "SD card: not mounted");
}

void Gui::init_arc() {
  // a sweeping arc on the right: something that visibly moves every frame
  arc_ = lv_arc_create(lv_screen_active());
  lv_obj_set_size(arc_, 90, 90);
  lv_arc_set_rotation(arc_, 270);
  lv_arc_set_bg_angles(arc_, 0, 360);
  lv_arc_set_range(arc_, 0, 100);
  lv_arc_set_value(arc_, 0);
  lv_obj_remove_style(arc_, nullptr, LV_PART_KNOB);
  lv_obj_clear_flag(arc_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(arc_, LV_ALIGN_RIGHT_MID, -12, 10);
}

void Gui::set_uptime_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(uptime_label_, std::string(text).c_str());
}

void Gui::set_ethernet_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(ethernet_label_, std::string(text).c_str());
}

void Gui::set_sdcard_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(sdcard_label_, std::string(text).c_str());
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    arc_value_ = (arc_value_ + 1) % 101;
    lv_arc_set_value(arc_, arc_value_);
    lv_timer_handler();
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, std::chrono::milliseconds(16));
  return false; // don't stop the task
}
