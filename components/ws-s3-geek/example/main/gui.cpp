#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_background();
  init_label();
  init_circle_layer();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
}

void Gui::init_background() {
  auto &bsp = Bsp::get();
  background_ = lv_obj_create(lv_screen_active());
  lv_obj_set_size(background_, bsp.lcd_width(), bsp.lcd_height());
  lv_obj_set_style_bg_color(background_, lv_color_make(0, 0, 0), 0);
}

void Gui::init_label() {
  label_ = lv_label_create(lv_screen_active());
  lv_label_set_text(label_, "");
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
}

void Gui::init_circle_layer() {
  auto &bsp = Bsp::get();
  circle_layer_ = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(circle_layer_);
  lv_obj_set_size(circle_layer_, bsp.lcd_width(), bsp.lcd_height());
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

void Gui::set_label_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(label_, std::string(text).c_str());
}

void Gui::next_rotation() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  logger_.info("Setting rotation to {}", static_cast<int>(rotation));
  lv_display_set_rotation(lv_display_get_default(), rotation);
  update_layout();
}

void Gui::update_layout() {
  auto &bsp = Bsp::get();
  int width = bsp.rotated_display_width();
  int height = bsp.rotated_display_height();
  // update the size of the screen-filling objects and re-align the rest
  lv_obj_set_size(background_, width, height);
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(circle_layer_, width, height);
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_move_foreground(circle_layer_);
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
  for (auto &circle : circles_) {
    if (circle.visible) {
      invalidate_circle_area(circle);
    }
    circle.visible = false;
  }
  next_circle_index_ = 0;
  visible_circle_count_ = 0;
}

size_t Gui::circle_count() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return visible_circle_count_;
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
