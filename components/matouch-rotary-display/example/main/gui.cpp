#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_background();
  init_label();
  init_buttons();
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

void Gui::init_background() {
  auto &mt_display = espp::MatouchRotaryDisplay::get();
  background_ = lv_obj_create(lv_screen_active());
  lv_obj_set_size(background_, mt_display.lcd_width(), mt_display.lcd_height());
  lv_obj_set_style_bg_color(background_, lv_color_make(0, 0, 0), 0);
}

void Gui::init_label() {
  label_ = lv_label_create(lv_screen_active());
  lv_label_set_text(label_, "");
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
}

void Gui::init_buttons() {
  // a button in the top middle which rotates the display through
  // 0/90/180/270 degrees
  rotate_button_ = lv_btn_create(lv_screen_active());
  lv_obj_set_size(rotate_button_, 50, 50);
  lv_obj_align(rotate_button_, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_t *rotate_label = lv_label_create(rotate_button_);
  lv_label_set_text(rotate_label, LV_SYMBOL_REFRESH);
  lv_obj_align(rotate_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(rotate_button_, event_callback, LV_EVENT_PRESSED, this);

  // a button in the top right which clears the circles
  clear_button_ = lv_btn_create(lv_screen_active());
  lv_obj_set_size(clear_button_, 50, 50);
  lv_obj_align(clear_button_, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_t *clear_label = lv_label_create(clear_button_);
  lv_label_set_text(clear_label, LV_SYMBOL_TRASH);
  lv_obj_align(clear_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(clear_button_, event_callback, LV_EVENT_PRESSED, this);
}

void Gui::init_circle_layer() {
  auto &mt_display = espp::MatouchRotaryDisplay::get();
  circle_layer_ = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(circle_layer_);
  lv_obj_set_size(circle_layer_, mt_display.lcd_width(), mt_display.lcd_height());
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

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    lv_task_handler();
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
  default:
    break;
  }
}

void Gui::on_pressed(lv_event_t *e) {
  const auto *target = static_cast<const lv_obj_t *>(lv_event_get_target(e));
  if (target == rotate_button_) {
    logger_.info("Rotate button pressed");
    next_rotation();
  } else if (target == clear_button_) {
    logger_.info("Clear button pressed");
    clear_circles();
  }
}

void Gui::set_label_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(label_, std::string(text).c_str());
}

void Gui::next_rotation() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto &mt_display = espp::MatouchRotaryDisplay::get();
  clear_circles_impl();
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  lv_display_set_rotation(lv_display_get_default(), rotation);
  // update the size of the screen-filling objects
  lv_obj_set_size(background_, mt_display.rotated_display_width(),
                  mt_display.rotated_display_height());
  lv_obj_set_size(circle_layer_, mt_display.rotated_display_width(),
                  mt_display.rotated_display_height());
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_invalidate(circle_layer_);
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
