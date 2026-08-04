#include "gui.hpp"

#include <chrono>

using namespace std::chrono_literals;

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_background();
  init_header();
  init_info_panel();

  // A flexible spacer that fills the space between the stats panel and the
  // buttons, so the buttons stay at the bottom whether or not stats are shown.
  lv_obj_t *spacer = lv_obj_create(lv_screen_active());
  lv_obj_set_width(spacer, 1);
  lv_obj_set_flex_grow(spacer, 1);
  lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(spacer, 0, 0);
  lv_obj_clear_flag(spacer, LV_OBJ_FLAG_SCROLLABLE);

  init_buttons();
}

void Gui::init_background() {
  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(screen, lv_color_black(), 0);
  // a border around the whole screen
  lv_obj_set_style_border_color(screen, lv_color_black(), 0);
  lv_obj_set_style_border_width(screen, 3, 0);
  lv_obj_set_style_pad_all(screen, 12, 0);
  lv_obj_set_style_pad_row(screen, 6, 0);
  // Lay the screen out as a vertical stack, centered horizontally, so it adapts
  // to the current (rotated) resolution instead of using fixed positions.
  lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
}

void Gui::init_header() {
  lv_obj_t *screen = lv_screen_active();

  // Full-width, center-aligned text so updating the clock doesn't re-flow the
  // layout (only the label's own area is redrawn).
  auto centered = [&](lv_obj_t *label, const lv_font_t *font) {
    lv_obj_set_width(label, lv_pct(100));
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
  };

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "LilyGo T5 4.7\" e-paper");
  centered(title, &lv_font_montserrat_20);

  // the live RTC clock, large
  time_label_ = lv_label_create(screen);
  lv_label_set_text(time_label_, "--:--:--");
  centered(time_label_, &lv_font_montserrat_48);

  date_label_ = lv_label_create(screen);
  lv_label_set_text(date_label_, "----/--/--");
  centered(date_label_, &lv_font_montserrat_20);
}

void Gui::init_info_panel() {
  lv_obj_t *screen = lv_screen_active();

  // Status rows created as direct children of the screen flex column (same as
  // the header labels, which render reliably). They are grouped/toggled by
  // hiding them together. Full width with left-aligned text gives a clean left
  // column; the Stats button shows/hides them.
  auto make_row = [&](lv_obj_t **label, const char *initial) {
    *label = lv_label_create(screen);
    lv_label_set_text(*label, initial);
    lv_obj_set_width(*label, lv_pct(96));
    lv_obj_set_style_text_font(*label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_align(*label, LV_TEXT_ALIGN_LEFT, 0);
  };

  make_row(&battery_label_, "Battery: --");
  make_row(&touch_label_, "Touch: --");
  make_row(&io48_label_, "IO48 button: --");
  make_row(&home_label_, "Home button: --");
  make_row(&lora_label_, "LoRa: --");
}

lv_obj_t *Gui::make_button(lv_obj_t *parent, const char *text, lv_obj_t **out_label) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, 148, 70);
  // white with a black border reads cleanly on grayscale e-paper
  lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, lv_color_black(), 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_radius(btn, 6, 0);
  // No pressed-state styling: on e-paper the press/release redraws would each be
  // a full ~refresh, so a single tap would flash 2-3 times. Keeping the button
  // static means only the button's action redraws.

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label, lv_color_black(), 0);
  lv_obj_center(label);

  lv_obj_add_event_cb(btn, event_callback, LV_EVENT_CLICKED, this);
  if (out_label) {
    *out_label = label;
  }
  return btn;
}

void Gui::init_buttons() {
  lv_obj_t *screen = lv_screen_active();

  // A transparent wrapping flex row of buttons. Wrapping + content height lets
  // it re-flow to as many rows as needed when the display is rotated (5 across
  // in landscape, wrapping to 2-3 rows in portrait).
  lv_obj_t *row = lv_obj_create(screen);
  lv_obj_set_width(row, lv_pct(100));
  lv_obj_set_height(row, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_pad_all(row, 0, 0);
  lv_obj_set_style_pad_column(row, 8, 0);
  lv_obj_set_style_pad_row(row, 8, 0);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

  frontlight_button_ = make_button(row, "Light: OFF", &frontlight_button_label_);
  rotate_button_ = make_button(row, "Rotate", nullptr);
  stats_button_ = make_button(row, "Stats", nullptr);
  lora_button_ = make_button(row, "LoRa TX", nullptr);
  refresh_button_ = make_button(row, "Refresh", nullptr);
  off_button_ = make_button(row, "Off", nullptr);

  // The update mode follows the stats panel: GC16 (grayscale) while stats are
  // shown so the small text is legible; DU (fast mono) while hidden.
  board_.set_lvgl_update_mode(stats_shown_ ? MODE_GC16 : MODE_DU);
}

void Gui::set_label_if_changed(lv_obj_t *label, std::string &cache, std::string_view text) {
  if (!label) {
    return;
  }
  // Guard the cache and the LVGL call together - callers come from several tasks
  // (touch callback, sensor-polling loop, LoRa RX, ...).
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (cache == text) {
    return; // no change - avoid a needless e-paper refresh
  }
  cache = std::string(text);
  lv_label_set_text(label, cache.c_str());
}

void Gui::set_time(std::string_view text) { set_label_if_changed(time_label_, time_cache_, text); }
void Gui::set_date(std::string_view text) { set_label_if_changed(date_label_, date_cache_, text); }
void Gui::set_battery(std::string_view text) {
  set_label_if_changed(battery_label_, battery_cache_, text);
}

void Gui::set_touch(int num_points, int x, int y) {
  std::string s;
  if (num_points > 0) {
    s = "Touch: (" + std::to_string(x) + ", " + std::to_string(y) + ")";
  } else {
    s = "Touch: --";
  }
  set_label_if_changed(touch_label_, touch_cache_, s);
}

void Gui::set_io48_button(bool pressed) {
  set_label_if_changed(io48_label_, io48_cache_,
                       pressed ? "IO48 button: pressed" : "IO48 button: released");
}

void Gui::set_home_button(bool pressed) {
  set_label_if_changed(home_label_, home_cache_,
                       pressed ? "Home button: pressed" : "Home button: released");
}

void Gui::set_lora(std::string_view text) {
  set_label_if_changed(lora_label_, lora_cache_, std::string("LoRa: ") + std::string(text));
}

void Gui::request_full_refresh() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  board_.full_refresh();
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    bool relayout = false;
    if (rotate_requested_.exchange(false)) {
      board_.rotate();
      relayout = true;
    }
    if (stats_toggle_requested_.exchange(false)) {
      stats_shown_ = !stats_shown_;
      for (lv_obj_t *label :
           {battery_label_, touch_label_, io48_label_, home_label_, lora_label_}) {
        if (stats_shown_) {
          lv_obj_clear_flag(label, LV_OBJ_FLAG_HIDDEN);
        } else {
          lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
        }
      }
      // Follow the stats panel with the update mode: GC16 renders the small
      // stats text legibly; DU is faster but too coarse for it.
      board_.set_lvgl_update_mode(stats_shown_ ? MODE_GC16 : MODE_DU);
      relayout = true;
    }
    if (relayout) {
      // A layout change (rotation or showing/hiding the stats panel) needs a
      // forced re-layout + render, then a clean full refresh to clear the old
      // content.
      lv_obj_invalidate(lv_screen_active());
      lv_refr_now(board_.lvgl_display());
      board_.full_refresh();
    } else {
      lv_task_handler();
    }
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, 16ms);
  return false; // don't stop the task
}

void Gui::event_callback(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (!gui) {
    return;
  }
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    gui->on_clicked(e);
  }
}

void Gui::on_clicked(lv_event_t *e) {
  const auto *target = static_cast<const lv_obj_t *>(lv_event_get_target(e));
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (target == frontlight_button_) {
    frontlight_on_ = !frontlight_on_;
    board_.set_frontlight(frontlight_on_);
    lv_label_set_text(frontlight_button_label_, frontlight_on_ ? "Light: ON" : "Light: OFF");
    logger_.info("Frontlight {}", frontlight_on_ ? "on" : "off");
  } else if (target == rotate_button_) {
    // Defer the actual rotation to update() - doing it here (inside
    // lv_task_handler's event processing) would re-enter LVGL layout/refresh.
    logger_.info("Rotate requested");
    rotate_requested_ = true;
  } else if (target == stats_button_) {
    logger_.info("Stats toggle requested");
    stats_toggle_requested_ = true;
  } else if (target == lora_button_) {
    logger_.info("LoRa send requested");
    lora_send_requested_ = true;
  } else if (target == refresh_button_) {
    logger_.info("Full refresh");
    board_.full_refresh();
  } else if (target == off_button_) {
    logger_.info("Powering off (ship mode) - press PWR to turn back on");
    board_.shutdown();
  }
}
