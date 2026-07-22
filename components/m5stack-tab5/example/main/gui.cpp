#include <algorithm>
#include <utility>

#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_tabview();
  init_draw_tab();
  init_status_tab();
  init_audio_tab();
  init_circle_layer();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
}

void Gui::init_tabview() {
  // the tabview gives each subsystem its own uncrowded page; the tab bar
  // (top) is the page switcher
  tabview_ = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_position(tabview_, LV_DIR_TOP);
  lv_tabview_set_tab_bar_size(tabview_, TAB_BAR_HEIGHT);
  lv_obj_set_size(tabview_, lv_display_get_horizontal_resolution(lv_display_get_default()),
                  lv_display_get_vertical_resolution(lv_display_get_default()));
  draw_tab_ = lv_tabview_add_tab(tabview_, "Draw");
  status_tab_ = lv_tabview_add_tab(tabview_, "Status");
  audio_tab_ = lv_tabview_add_tab(tabview_, "Audio");
  // switching tabs is done with the tab buttons only: disable swipe
  // scrolling of the content so drawing on the Draw tab cannot accidentally
  // change pages
  lv_obj_clear_flag(lv_tabview_get_content(tabview_), LV_OBJ_FLAG_SCROLLABLE);
  // hide the touch-trail overlay whenever a non-drawing tab is shown
  lv_obj_add_event_cb(tabview_, event_callback, LV_EVENT_VALUE_CHANGED, this);
}

void Gui::init_draw_tab() {
  // instructions on the left, with the rotate / brightness / clear buttons
  // on the right
  label_ = lv_label_create(draw_tab_);
  lv_label_set_long_mode(label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(label_, lv_pct(70));
  lv_label_set_text(label_, "");
  lv_obj_align(label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_LEFT, 0);

  struct ButtonSpec {
    lv_obj_t **button;
    const char *symbol;
    int y_offset;
  };
  const ButtonSpec buttons[] = {
      {&rotate_button_, LV_SYMBOL_REFRESH, 0},
      {&brightness_button_, LV_SYMBOL_EYE_OPEN, 60},
      {&clear_button_, LV_SYMBOL_TRASH, 120},
  };
  for (const auto &spec : buttons) {
    *spec.button = lv_btn_create(draw_tab_);
    lv_obj_set_size(*spec.button, 50, 50);
    lv_obj_align(*spec.button, LV_ALIGN_TOP_RIGHT, 0, spec.y_offset);
    lv_obj_t *label = lv_label_create(*spec.button);
    lv_label_set_text(label, spec.symbol);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(*spec.button, event_callback, LV_EVENT_PRESSED, this);
  }
}

void Gui::init_status_tab() {
  status_label_ = lv_label_create(status_tab_);
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(status_label_, lv_pct(100));
  lv_label_set_text(status_label_, "Waiting for data...");
  lv_obj_align(status_label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_LEFT, 0);

  lv_style_init(&kalman_line_style_);
  lv_style_set_line_width(&kalman_line_style_, 8);
  lv_style_set_line_color(&kalman_line_style_, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_line_rounded(&kalman_line_style_, true);

  kalman_line_ = lv_line_create(status_tab_);
  kalman_line_points_[0] = {0, 0};
  kalman_line_points_[1] = {0, 0};
  lv_line_set_points(kalman_line_, kalman_line_points_, 2);
  lv_obj_add_style(kalman_line_, &kalman_line_style_, 0);

  lv_style_init(&madgwick_line_style_);
  lv_style_set_line_width(&madgwick_line_style_, 8);
  lv_style_set_line_color(&madgwick_line_style_, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_line_rounded(&madgwick_line_style_, true);

  madgwick_line_ = lv_line_create(status_tab_);
  madgwick_line_points_[0] = {0, 0};
  madgwick_line_points_[1] = {0, 0};
  lv_line_set_points(madgwick_line_, madgwick_line_points_, 2);
  lv_obj_add_style(madgwick_line_, &madgwick_line_style_, 0);
}

void Gui::init_audio_tab() {
  // a column: status line, volume line, then the buttons in a wrapping row
  lv_obj_set_flex_flow(audio_tab_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(audio_tab_, 12, 0);

  audio_status_label_ = lv_label_create(audio_tab_);
  lv_label_set_long_mode(audio_status_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(audio_status_label_, lv_pct(100));
  lv_label_set_text(audio_status_label_, "Idle");

  audio_label_ = lv_label_create(audio_tab_);
  lv_label_set_long_mode(audio_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(audio_label_, lv_pct(100));
  update_audio_label();

  lv_obj_t *row = lv_obj_create(audio_tab_);
  lv_obj_remove_style_all(row);
  lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_style_pad_column(row, 12, 0);
  lv_obj_set_style_pad_row(row, 12, 0);

  struct ButtonSpec {
    lv_obj_t **button;
    const char *symbol;
  };
  const ButtonSpec buttons[] = {
      {&record_button_, LV_SYMBOL_AUDIO},           {&play_button_, LV_SYMBOL_PLAY},
      {&volume_down_button_, LV_SYMBOL_VOLUME_MID}, {&volume_up_button_, LV_SYMBOL_VOLUME_MAX},
      {&mic_down_button_, LV_SYMBOL_MINUS},         {&mic_up_button_, LV_SYMBOL_PLUS},
  };
  for (const auto &spec : buttons) {
    *spec.button = lv_btn_create(row);
    lv_obj_set_size(*spec.button, 60, 60);
    lv_obj_t *label = lv_label_create(*spec.button);
    lv_label_set_text(label, spec.symbol);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(*spec.button, event_callback, LV_EVENT_PRESSED, this);
  }
  // remember the record / play button labels so set_record_active /
  // set_play_active can swap their symbols
  record_button_label_ = lv_obj_get_child(record_button_, 0);
  play_button_label_ = lv_obj_get_child(play_button_, 0);
  // color the volume buttons so the speaker and microphone pairs are
  // distinguishable from each other
  lv_obj_set_style_bg_color(mic_down_button_, lv_palette_main(LV_PALETTE_TEAL), 0);
  lv_obj_set_style_bg_color(mic_up_button_, lv_palette_main(LV_PALETTE_TEAL), 0);
}

void Gui::update_audio_label() {
  auto &tab5 = espp::M5StackTab5::get();
  lv_label_set_text_fmt(audio_label_,
                        "Speaker %d%% (" LV_SYMBOL_VOLUME_MID "/" LV_SYMBOL_VOLUME_MAX
                        ")\nMic %d%% (teal " LV_SYMBOL_MINUS "/" LV_SYMBOL_PLUS ")",
                        static_cast<int>(tab5.volume()),
                        static_cast<int>(tab5.microphone_volume()));
}

void Gui::init_circle_layer() {
  // a transparent, click-through overlay above the tabview which shows the
  // touch trail (only populated while the Draw tab is active)
  circle_layer_ = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(circle_layer_);
  lv_obj_set_size(circle_layer_, lv_display_get_horizontal_resolution(lv_display_get_default()),
                  lv_display_get_vertical_resolution(lv_display_get_default()));
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(circle_layer_, LV_OPA_TRANSP, 0);
  lv_obj_add_event_cb(circle_layer_, draw_circle_layer, LV_EVENT_DRAW_MAIN, this);
  lv_obj_move_foreground(circle_layer_);
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    lv_task_handler();
    // keep the audio volume label in sync with the live BSP state, so the
    // first press of a volume button doesn't appear to jump from a stale
    // default (the values are set by app_main after the Gui is constructed)
    update_audio_label();
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, std::chrono::milliseconds(16));
  return false; // don't stop the task
}

void Gui::event_callback(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (!gui) {
    return;
  }
  switch (lv_event_get_code(e)) {
  case LV_EVENT_PRESSED:
    gui->on_pressed(e);
    break;
  case LV_EVENT_VALUE_CHANGED:
    gui->on_tab_changed(e);
    break;
  default:
    break;
  }
}

void Gui::on_tab_changed(lv_event_t *e) {
  const auto *target = static_cast<const lv_obj_t *>(lv_event_get_target(e));
  if (target != tabview_) {
    return;
  }
  // the touch trail only belongs to the drawing tab; hide it (and its
  // circles) everywhere else
  if (lv_tabview_get_tab_active(tabview_) == 0) {
    lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(circle_layer_, LV_OBJ_FLAG_HIDDEN);
  }
}

void Gui::on_pressed(lv_event_t *e) {
  const auto *target = static_cast<const lv_obj_t *>(lv_event_get_target(e));
  auto &tab5 = espp::M5StackTab5::get();
  if (target == record_button_) {
    logger_.info("Record button pressed");
    if (record_callback_) {
      record_callback_();
    }
    return;
  }
  if (target == play_button_) {
    logger_.info("Play button pressed");
    if (play_callback_) {
      play_callback_();
    }
    return;
  }
  if (target == volume_down_button_ || target == volume_up_button_) {
    float delta = target == volume_down_button_ ? -10.0f : 10.0f;
    tab5.volume(tab5.volume() + delta);
    logger_.info("Speaker volume: {:.0f}%", tab5.volume());
    update_audio_label();
    return;
  }
  if (target == mic_down_button_ || target == mic_up_button_) {
    float delta = target == mic_down_button_ ? -10.0f : 10.0f;
    tab5.microphone_volume(tab5.microphone_volume() + delta);
    logger_.info("Microphone volume: {:.0f}%", tab5.microphone_volume());
    update_audio_label();
    return;
  }
  if (target == rotate_button_) {
    logger_.info("Rotate button pressed");
    next_rotation();
  } else if (target == brightness_button_) {
    logger_.info("Brightness button pressed");
    cycle_brightness();
  } else if (target == clear_button_) {
    logger_.info("Clear button pressed");
    clear_circles();
  }
}

void Gui::set_record_active(bool active) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(record_button_label_, active ? LV_SYMBOL_STOP : LV_SYMBOL_AUDIO);
  lv_obj_set_style_bg_color(
      record_button_, active ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE),
      0);
}

void Gui::set_play_active(bool active) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(play_button_label_, active ? LV_SYMBOL_STOP : LV_SYMBOL_PLAY);
  lv_obj_set_style_bg_color(
      play_button_, active ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_BLUE),
      0);
}

void Gui::set_audio_status(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(audio_status_label_, std::string(text).c_str());
}

void Gui::set_label_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(label_, std::string(text).c_str());
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(status_label_, std::string(text).c_str());
}

bool Gui::draw_page_active() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return lv_tabview_get_tab_active(tabview_) == 0;
}

void Gui::next_rotation() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clear_circles_impl();
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  lv_display_set_rotation(lv_display_get_default(), rotation);
  // update the size of the screen-filling objects
  int width = lv_display_get_horizontal_resolution(lv_display_get_default());
  int height = lv_display_get_vertical_resolution(lv_display_get_default());
  lv_obj_set_size(tabview_, width, height);
  lv_obj_set_size(circle_layer_, width, height);
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_invalidate(circle_layer_);
}

void Gui::cycle_brightness() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  brightness_index_ = (brightness_index_ + 1) % BRIGHTNESS_LEVELS.size();
  float new_brightness = BRIGHTNESS_LEVELS[brightness_index_];
  espp::M5StackTab5::get().brightness(new_brightness);
  logger_.info("Set brightness to {:.0f}%", new_brightness);
}

void Gui::set_kalman_down(float vx, float vy) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  set_down_line(kalman_line_, kalman_line_points_, vx, vy);
}

void Gui::set_madgwick_down(float vx, float vy) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  set_down_line(madgwick_line_, madgwick_line_points_, vx, vy);
}

void Gui::set_down_line(lv_obj_t *line, lv_point_precise_t *points, float vx, float vy) {
  // remap the vector according to the current display rotation so that the
  // line always points toward physical "down"
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  if (rotation == LV_DISPLAY_ROTATION_90) {
    std::swap(vx, vy);
    vx = -vx;
  } else if (rotation == LV_DISPLAY_ROTATION_180) {
    vx = -vx;
    vy = -vy;
  } else if (rotation == LV_DISPLAY_ROTATION_270) {
    std::swap(vx, vy);
    vy = -vy;
  }
  // draw the line from the center of the Status tab's content area toward
  // "down" (fall back to the display size if the layout hasn't run yet)
  int w = lv_obj_get_content_width(status_tab_);
  int h = lv_obj_get_content_height(status_tab_);
  if (w <= 0 || h <= 0) {
    w = lv_display_get_horizontal_resolution(lv_display_get_default());
    h = lv_display_get_vertical_resolution(lv_display_get_default()) - TAB_BAR_HEIGHT;
  }
  int x0 = w / 2;
  int y0 = h / 2;
  points[0] = {static_cast<lv_value_precise_t>(x0), static_cast<lv_value_precise_t>(y0)};
  points[1] = {static_cast<lv_value_precise_t>(x0 + 50 * vx),
               static_cast<lv_value_precise_t>(y0 + 50 * vy)};
  lv_line_set_points(line, points, 2);
}

void Gui::draw_circle(int x, int y, int radius) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_move_foreground(circle_layer_);
  Circle previous_circle = circles_[next_circle_index_];
  circles_[next_circle_index_] = {.x = x, .y = y, .radius = radius, .visible = true};
  next_circle_index_ = (next_circle_index_ + 1) % circles_.size();
  if (visible_circle_count_ < circles_.size()) {
    visible_circle_count_++;
  }
  if (previous_circle.visible) {
    invalidate_circle_area(previous_circle);
  }
  invalidate_circle_area(circles_[(next_circle_index_ + circles_.size() - 1) % circles_.size()]);
}

void Gui::clear_circles() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clear_circles_impl();
}

void Gui::clear_circles_impl() {
  for (auto &circle : circles_) {
    if (circle.visible) {
      invalidate_circle_area(circle);
    }
    circle.visible = false;
  }
  next_circle_index_ = 0;
  visible_circle_count_ = 0;
}

void Gui::invalidate_circle_area(const Circle &circle) {
  if (!circle_layer_ || circle.radius <= 0) {
    return;
  }
  lv_area_t obj_coords;
  lv_obj_get_coords(circle_layer_, &obj_coords);
  lv_area_t coords = {
      .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
      .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
      .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
      .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
  };
  lv_obj_invalidate_area(circle_layer_, &coords);
}

void Gui::draw_circle_layer(lv_event_t *e) {
  const auto *gui = static_cast<const Gui *>(lv_event_get_user_data(e));
  if (!gui) {
    return;
  }
  gui->draw_circles(e);
}

void Gui::draw_circles(lv_event_t *e) const {
  if (visible_circle_count_ == 0) {
    return;
  }

  auto *obj = static_cast<lv_obj_t *>(lv_event_get_current_target(e));
  auto *layer = lv_event_get_layer(e);
  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);

  lv_draw_rect_dsc_t rect_dsc;
  lv_draw_rect_dsc_init(&rect_dsc);
  rect_dsc.base.layer = layer;
  rect_dsc.radius = LV_RADIUS_CIRCLE;
  rect_dsc.bg_opa = LV_OPA_70;
  rect_dsc.bg_color = lv_color_make(0, 255, 255);
  rect_dsc.border_width = 0;
  rect_dsc.outline_width = 0;
  rect_dsc.shadow_width = 0;

  for (const auto &circle : circles_) {
    if (!circle.visible) {
      continue;
    }
    lv_area_t coords = {
        .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
        .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
        .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
        .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
    };
    lv_draw_rect(layer, &rect_dsc, &coords);
  }
}
