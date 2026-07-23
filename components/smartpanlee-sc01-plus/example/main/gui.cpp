#include "gui.hpp"

#include <algorithm>

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_tabview();
  init_label();
  init_buttons();
  init_audio_controls();
  init_circle_layer();
  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);
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
  audio_tab_ = lv_tabview_add_tab(tabview_, "Audio");
  // switching tabs is done with the tab buttons only: disable swipe
  // scrolling of the content so drawing on the Draw tab cannot accidentally
  // change pages
  lv_obj_clear_flag(lv_tabview_get_content(tabview_), LV_OBJ_FLAG_SCROLLABLE);
  // hide the touch-trail overlay whenever a non-drawing tab is shown
  lv_obj_add_event_cb(tabview_, event_callback, LV_EVENT_VALUE_CHANGED, this);
}

void Gui::init_label() {
  label_ = lv_label_create(draw_tab_);
  lv_label_set_long_mode(label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(label_, lv_pct(75));
  lv_label_set_text(label_, "");
  lv_obj_align(label_, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_LEFT, 0);
}

void Gui::init_buttons() {
  // rotate / clear buttons in the top right of the Draw tab
  rotate_button_ = lv_btn_create(draw_tab_);
  lv_obj_set_size(rotate_button_, 56, 56);
  lv_obj_align(rotate_button_, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_t *rotate_label = lv_label_create(rotate_button_);
  lv_label_set_text(rotate_label, LV_SYMBOL_REFRESH);
  lv_obj_align(rotate_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(rotate_button_, event_callback, LV_EVENT_PRESSED, this);

  clear_button_ = lv_btn_create(draw_tab_);
  lv_obj_set_size(clear_button_, 56, 56);
  lv_obj_align(clear_button_, LV_ALIGN_TOP_RIGHT, 0, 66);
  lv_obj_add_state(clear_button_, LV_STATE_CHECKED); // make the button red
  lv_obj_t *clear_label = lv_label_create(clear_button_);
  lv_label_set_text(clear_label, LV_SYMBOL_TRASH);
  lv_obj_align(clear_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(clear_button_, event_callback, LV_EVENT_PRESSED, this);
}

void Gui::init_audio_controls() {
  // the Audio tab: a column with the volume line, then play / mute / volume
  // buttons in a wrapping row
  lv_obj_set_flex_flow(audio_tab_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(audio_tab_, 12, 0);

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
      {&play_button_, LV_SYMBOL_PLAY},
      {&mute_button_, LV_SYMBOL_MUTE},
      {&volume_down_button_, LV_SYMBOL_VOLUME_MID},
      {&volume_up_button_, LV_SYMBOL_VOLUME_MAX},
  };
  for (const auto &spec : buttons) {
    *spec.button = lv_btn_create(row);
    lv_obj_set_size(*spec.button, 56, 56);
    lv_obj_t *label = lv_label_create(*spec.button);
    lv_label_set_text(label, spec.symbol);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(*spec.button, event_callback, LV_EVENT_PRESSED, this);
  }
}

void Gui::update_audio_label() {
  auto &board = espp::SmartPanleeSc01Plus::get();
  // pass the LVGL symbol as a %s argument rather than concatenating it into the
  // format-string literal: cppcheck cannot expand the LVGL symbol macros and
  // flags the literal concatenation as an unknown macro
  lv_label_set_text_fmt(audio_label_, "%s %d%%%s", LV_SYMBOL_VOLUME_MAX,
                        static_cast<int>(board.volume()), board.is_muted() ? " (muted)" : "");
}

void Gui::init_circle_layer() {
  circle_layer_ = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(circle_layer_);
  lv_obj_set_size(circle_layer_, lv_display_get_horizontal_resolution(lv_display_get_default()),
                  lv_display_get_vertical_resolution(lv_display_get_default()));
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(circle_layer_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(circle_layer_, 0, 0);
  lv_obj_set_style_outline_width(circle_layer_, 0, 0);
  lv_obj_set_style_shadow_width(circle_layer_, 0, 0);
  lv_obj_add_event_cb(circle_layer_, draw_circle_layer, LV_EVENT_DRAW_MAIN, this);
  lv_obj_move_foreground(circle_layer_);
}

void Gui::update_layout() {
  auto &board = espp::SmartPanleeSc01Plus::get();
  int width = board.rotated_display_width();
  int height = board.rotated_display_height();
  lv_obj_set_size(tabview_, width, height);
  lv_obj_set_size(circle_layer_, width, height);
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_move_foreground(circle_layer_);
  lv_obj_invalidate(circle_layer_);
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
  auto &board = espp::SmartPanleeSc01Plus::get();
  if (target == play_button_) {
    logger_.info("Play button pressed");
    if (play_callback_) {
      play_callback_();
    }
    return;
  }
  if (target == mute_button_) {
    board.mute(!board.is_muted());
    logger_.info("Muted: {}", board.is_muted());
    update_audio_label();
    return;
  }
  if (target == volume_down_button_ || target == volume_up_button_) {
    float delta = target == volume_down_button_ ? -10.0f : 10.0f;
    board.volume(board.volume() + delta);
    logger_.info("Speaker volume: {:.0f}%", board.volume());
    update_audio_label();
    return;
  }
  if (target == rotate_button_) {
    logger_.info("Rotate button pressed");
    next_rotation();
  } else if (target == clear_button_) {
    logger_.info("Clear button pressed");
    clear_circles();
  }
}

bool Gui::draw_page_active() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return lv_tabview_get_tab_active(tabview_) == 0;
}

void Gui::set_label_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(label_, std::string(text).c_str());
}

void Gui::next_rotation() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clear_circles_impl();
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  lv_display_set_rotation(lv_display_get_default(), rotation);
  // re-size / re-align the UI to fill the newly-rotated display
  update_layout();
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
