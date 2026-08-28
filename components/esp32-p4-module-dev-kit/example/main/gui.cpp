#include <algorithm>
#include <cstring>
#include <utility>

#include <esp_heap_caps.h>

#include "gui.hpp"

using Board = espp::Esp32P4ModuleDevKit;

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_tabview();
  init_labels();
  init_buttons();
  init_audio_controls();
  init_camera_tab();
  init_circle_layer();
  ui_ready_ = true;
}

void Gui::init_tabview() {
  // the tabview gives each subsystem its own uncrowded page; the tab bar
  // (top) is the page switcher
  tabview_ = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_position(tabview_, LV_DIR_TOP);
  lv_tabview_set_tab_bar_size(tabview_, TAB_BAR_HEIGHT);
  lv_obj_set_size(tabview_, lv_display_get_horizontal_resolution(lv_display_get_default()),
                  lv_display_get_vertical_resolution(lv_display_get_default()));
  status_tab_ = lv_tabview_add_tab(tabview_, "Status");
  audio_tab_ = lv_tabview_add_tab(tabview_, "Audio");
  camera_tab_ = lv_tabview_add_tab(tabview_, "Camera");
  // The tab pages themselves are scrollable by default; disable that so drags
  // inside a page don't rubber-band/scroll the content (matches the other BSP
  // example GUIs).
  lv_obj_clear_flag(status_tab_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(audio_tab_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(camera_tab_, LV_OBJ_FLAG_SCROLLABLE);
  // switching tabs is done with the tab buttons only: disable swipe
  // scrolling of the content so drawing on the Status tab cannot accidentally
  // change pages
  lv_obj_clear_flag(lv_tabview_get_content(tabview_), LV_OBJ_FLAG_SCROLLABLE);
  // hide the touch-trail overlay whenever a non-drawing tab is shown
  lv_obj_add_event_cb(tabview_, event_callback, LV_EVENT_VALUE_CHANGED, this);
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // Mark the UI down first so a camera frame arriving mid-teardown (the camera
  // task runs independently) is dropped rather than touching freed objects.
  ui_ready_ = false;
  lv_obj_clean(lv_screen_active());
  // lv_obj_clean() deleted every object under the screen; null the pointers we
  // hold so nothing dereferences a freed object. It does not free the canvas's
  // external PSRAM buffer (a member we own), so release that too.
  tabview_ = nullptr;
  status_tab_ = nullptr;
  audio_tab_ = nullptr;
  camera_tab_ = nullptr;
  camera_canvas_ = nullptr;
  camera_label_ = nullptr;
  if (camera_buf_) {
    heap_caps_free(camera_buf_);
    camera_buf_ = nullptr;
  }
  camera_w_ = 0;
  camera_h_ = 0;
}

void Gui::init_labels() {
  // a title label at the top of the Status tab
  title_label_ = lv_label_create(status_tab_);
  lv_label_set_long_mode(title_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(title_label_, lv_pct(100));
  lv_label_set_text(title_label_, "ESP32-P4-Module-DEV-KIT  -  touch to draw!");
  lv_obj_align(title_label_, LV_ALIGN_TOP_MID, 0, 0);

  // a status label showing the live subsystem state, updated via
  // set_status_text()
  status_label_ = lv_label_create(status_tab_);
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(status_label_, lv_pct(100));
  lv_label_set_text(status_label_, "");
  lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_align(status_label_, LV_ALIGN_TOP_LEFT, 0, 32);
}

void Gui::init_buttons() {
  // a button in the top right of the Status tab which rotates the display
  // through 0/90/180/270 degrees
  rotate_button_ = lv_btn_create(status_tab_);
  lv_obj_set_size(rotate_button_, 50, 50);
  lv_obj_align(rotate_button_, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_t *rotate_label = lv_label_create(rotate_button_);
  lv_label_set_text(rotate_label, LV_SYMBOL_REFRESH);
  lv_obj_align(rotate_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(rotate_button_, event_callback, LV_EVENT_CLICKED, this);

  // a button next to it which clears the circles
  clear_button_ = lv_btn_create(status_tab_);
  lv_obj_set_size(clear_button_, 50, 50);
  lv_obj_align(clear_button_, LV_ALIGN_TOP_RIGHT, -58, 0);
  lv_obj_t *clear_label = lv_label_create(clear_button_);
  lv_label_set_text(clear_label, LV_SYMBOL_TRASH);
  lv_obj_align(clear_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(clear_button_, event_callback, LV_EVENT_CLICKED, this);
}

void Gui::init_audio_controls() {
  // the Audio tab: a column with a status line, the volume line, then the
  // buttons in a wrapping row
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
    lv_obj_set_size(*spec.button, 50, 50);
    lv_obj_t *label = lv_label_create(*spec.button);
    lv_label_set_text(label, spec.symbol);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(*spec.button, event_callback, LV_EVENT_CLICKED, this);
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

void Gui::init_camera_tab() {
  // Center the camera feed; show a placeholder label until the first frame
  // arrives. The canvas that displays the live feed is created lazily in
  // set_camera_frame() once the true frame size is known.
  lv_obj_set_flex_flow(camera_tab_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(camera_tab_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(camera_tab_, 0, 0);
  camera_label_ = lv_label_create(camera_tab_);
  lv_label_set_text(camera_label_, "Waiting for camera...");
}

void Gui::update_audio_label() {
  auto &board = Board::get();
  int speaker_volume = static_cast<int>(board.volume());
  int mic_volume = static_cast<int>(board.microphone_volume());
  // this is called every GUI tick; only reformat the label when a value
  // actually changes (lv_label_set_text_fmt formats a new string each call)
  if (speaker_volume == last_speaker_volume_ && mic_volume == last_mic_volume_) {
    return;
  }
  last_speaker_volume_ = speaker_volume;
  last_mic_volume_ = mic_volume;
  // Pass the LVGL symbols as %s arguments rather than concatenating them into
  // the format-string literal: cppcheck cannot expand the LVGL symbol macros
  // and flags the literal concatenation as an unknown macro.
  lv_label_set_text_fmt(audio_label_, "Speaker %d%% (%s/%s)\nMic %d%% (teal %s/%s)", speaker_volume,
                        LV_SYMBOL_VOLUME_MID, LV_SYMBOL_VOLUME_MAX, mic_volume, LV_SYMBOL_MINUS,
                        LV_SYMBOL_PLUS);
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

void Gui::set_camera_frame(const uint8_t *rgb565, int w, int h) {
  if (!rgb565 || w <= 0 || h <= 0) {
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // The camera task may still deliver a frame after the UI is torn down; drop it
  // rather than create objects under a freed camera_tab_.
  if (!ui_ready_ || !camera_tab_) {
    return;
  }
  // (Re)allocate the canvas buffer and (re)create the canvas on the first frame
  // or whenever the frame size changes. The buffer must outlive the canvas, so
  // it is a member kept in PSRAM.
  if (camera_buf_ == nullptr || w != camera_w_ || h != camera_h_) {
    // Allocate the new buffer BEFORE freeing the old one / deleting the canvas,
    // so that on OOM we keep showing the previous frame instead of blanking the
    // tab.
    const size_t bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
    auto *new_buf =
        static_cast<uint8_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!new_buf) {
      return; // out of memory; keep the current canvas / preview
    }
    if (camera_canvas_) {
      lv_obj_del(camera_canvas_);
      camera_canvas_ = nullptr;
    }
    if (camera_buf_) {
      heap_caps_free(camera_buf_);
    }
    camera_buf_ = new_buf;
    camera_w_ = w;
    camera_h_ = h;
    camera_canvas_ = lv_canvas_create(camera_tab_);
    lv_canvas_set_buffer(camera_canvas_, camera_buf_, w, h, LV_COLOR_FORMAT_RGB565);
    // Frame the feed (a border + rounded corners so it reads as a distinct
    // element when it is smaller than the tab).
    lv_obj_set_style_border_width(camera_canvas_, 2, 0);
    lv_obj_set_style_border_color(camera_canvas_, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_radius(camera_canvas_, 6, 0);
    if (camera_label_) {
      lv_obj_add_flag(camera_label_, LV_OBJ_FLAG_HIDDEN);
    }
  }
  std::memcpy(camera_buf_, rgb565, static_cast<size_t>(w) * static_cast<size_t>(h) * 2);
  lv_obj_invalidate(camera_canvas_);
}

void Gui::init_circle_layer() {
  // a transparent, click-through overlay above the tabview which shows the
  // touch trail (only populated while the Status tab is active)
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

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // drain any touch points queued since the last cycle
    {
      std::vector<Circle> points;
      {
        std::lock_guard<std::mutex> plock(pending_points_mutex_);
        points.swap(pending_points_);
      }
      for (const auto &c : points) {
        draw_circle_pending(c);
      }
    }
    lv_task_handler();
    // keep the audio volume label in sync with the live BSP state, so the
    // first press of a volume button doesn't appear to jump from a stale
    // default (the values are set by app_main after the Gui is constructed)
    update_audio_label();
  }
  std::unique_lock<std::mutex> lock(m);
  // stop promptly if Task::stop() notified us (otherwise ~Gui can hang joining
  // the update thread); a plain timeout means keep running
  return cv.wait_for(lock, std::chrono::milliseconds(16)) == std::cv_status::no_timeout;
}

void Gui::event_callback(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (!gui) {
    return;
  }
  switch (lv_event_get_code(e)) {
  case LV_EVENT_CLICKED:
    gui->on_clicked(e);
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
  // the touch trail only belongs to the Status tab; hide it (and its
  // circles) everywhere else
  if (lv_tabview_get_tab_active(tabview_) == 0) {
    lv_obj_clear_flag(circle_layer_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(circle_layer_, LV_OBJ_FLAG_HIDDEN);
  }
}

void Gui::on_clicked(lv_event_t *e) {
  const auto *target = static_cast<const lv_obj_t *>(lv_event_get_target(e));
  auto &board = Board::get();
  if (target == record_button_) {
    logger_.info("Record button clicked");
    if (record_callback_) {
      record_callback_();
    }
    return;
  }
  if (target == play_button_) {
    logger_.info("Play button clicked");
    if (play_callback_) {
      play_callback_();
    }
    return;
  }
  if (target == volume_down_button_ || target == volume_up_button_) {
    float delta = target == volume_down_button_ ? -10.0f : 10.0f;
    board.volume(board.volume() + delta);
    logger_.info("Speaker volume: {:.0f}%", board.volume());
    update_audio_label();
    return;
  }
  if (target == mic_down_button_ || target == mic_up_button_) {
    float delta = target == mic_down_button_ ? -10.0f : 10.0f;
    board.microphone_volume(board.microphone_volume() + delta);
    logger_.info("Microphone volume: {:.0f}%", board.microphone_volume());
    update_audio_label();
    return;
  }
  if (target == rotate_button_) {
    logger_.info("Rotate button clicked");
    next_rotation();
  } else if (target == clear_button_) {
    logger_.info("Clear button clicked");
    clear_circles();
  }
}

void Gui::set_status_text(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(status_label_, std::string(text).c_str());
}

void Gui::set_audio_status(std::string_view text) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_label_set_text(audio_status_label_, std::string(text).c_str());
}

bool Gui::draw_page_active() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return lv_tabview_get_tab_active(tabview_) == 0;
}

void Gui::next_rotation() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto &board = Board::get();
  clear_circles_impl();
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  lv_display_set_rotation(lv_display_get_default(), rotation);
  // update the size of the screen-filling objects
  lv_obj_set_size(tabview_, board.rotated_display_width(), board.rotated_display_height());
  lv_obj_set_size(circle_layer_, board.rotated_display_width(), board.rotated_display_height());
  lv_obj_align(circle_layer_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_move_foreground(circle_layer_);
  lv_obj_invalidate(circle_layer_);
}

void Gui::draw_circle(int x, int y, int radius) {
  // Only queue the point here; the GUI update task drains the queue under the
  // LVGL mutex. Taking mutex_ directly would block the caller (the touch poll
  // task) for the duration of lv_task_handler() rendering, collapsing the
  // touch sample rate.
  std::lock_guard<std::mutex> lock(pending_points_mutex_);
  // Bound the queue so it cannot grow without limit if the GUI task stalls or
  // the producers outpace the drain: drop the oldest point to keep the newest,
  // and log drops at most once a second (from this producer context a per-drop
  // log would itself become the bottleneck).
  if (pending_points_.size() >= MAX_PENDING_POINTS) {
    pending_points_.erase(pending_points_.begin());
    ++dropped_points_;
    auto now = std::chrono::steady_clock::now();
    if (now - last_drop_log_ >= std::chrono::seconds(1)) {
      last_drop_log_ = now;
      logger_.warn("Pending touch-point queue full; dropped {} oldest point(s)", dropped_points_);
      dropped_points_ = 0;
    }
  }
  pending_points_.push_back({.x = x, .y = y, .radius = radius, .visible = true});
}

void Gui::draw_circle_pending(const Circle &c) {
  // caller holds mutex_
  lv_obj_move_foreground(circle_layer_);
  int x = c.x, y = c.y, radius = c.radius;
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
