#include <algorithm>
#include <cstring>
#include <utility>

#include <esp_heap_caps.h>

#include "gui.hpp"

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_tabview();
  init_draw_tab();
  init_status_tab();
  init_audio_tab();
  init_camera_tab();
  init_circle_layer();
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  lv_obj_clean(lv_screen_active());
  // lv_obj_clean() deletes the canvas object but not its external PSRAM buffer,
  // which is a member we own; free it so tearing down / recreating the Gui does
  // not leak PSRAM.
  camera_canvas_ = nullptr;
  if (camera_buf_) {
    heap_caps_free(camera_buf_);
    camera_buf_ = nullptr;
  }
  camera_w_ = 0;
  camera_h_ = 0;
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
  camera_tab_ = lv_tabview_add_tab(tabview_, "Camera");
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
  int speaker_volume = static_cast<int>(tab5.volume());
  int mic_volume = static_cast<int>(tab5.microphone_volume());
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

void Gui::init_camera_tab() {
  // Center the camera feed; show a placeholder label until the first frame
  // arrives. The canvas that displays the live feed is created lazily in
  // set_camera_frame() once the true frame size is known. The settings controls
  // float on top of the feed (see build_camera_controls).
  lv_obj_set_flex_flow(camera_tab_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(camera_tab_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // No padding, so a full-size feed can sit flush in the top-left corner.
  lv_obj_set_style_pad_all(camera_tab_, 0, 0);
  camera_label_ = lv_label_create(camera_tab_);
  lv_label_set_text(camera_label_, "Waiting for camera...");
  build_camera_controls();
}

void Gui::build_camera_controls() {
  // A gear button in the top-right corner, floating (not part of the tab's flex
  // layout) so it overlays the feed and is always visible. Tapping it toggles
  // the settings panel.
  camera_settings_btn_ = lv_btn_create(camera_tab_);
  lv_obj_add_flag(camera_settings_btn_, LV_OBJ_FLAG_FLOATING);
  lv_obj_set_size(camera_settings_btn_, 48, 48);
  lv_obj_align(camera_settings_btn_, LV_ALIGN_TOP_RIGHT, -8, 8);
  lv_obj_t *gear = lv_label_create(camera_settings_btn_);
  lv_label_set_text(gear, LV_SYMBOL_SETTINGS);
  lv_obj_center(gear);
  lv_obj_add_event_cb(camera_settings_btn_, camera_settings_toggle_cb, LV_EVENT_CLICKED, this);

  // The panel: floating, semi-transparent, sized to its content so it stays
  // compact. Starts collapsed (hidden).
  camera_panel_ = lv_obj_create(camera_tab_);
  lv_obj_add_flag(camera_panel_, LV_OBJ_FLAG_FLOATING);
  lv_obj_set_size(camera_panel_, 240, LV_SIZE_CONTENT);
  lv_obj_align(camera_panel_, LV_ALIGN_TOP_RIGHT, -8, 64);
  lv_obj_set_flex_flow(camera_panel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(camera_panel_, 8, 0);
  lv_obj_set_style_bg_opa(camera_panel_, LV_OPA_80, 0);
  lv_obj_add_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *title = lv_label_create(camera_panel_);
  lv_label_set_text(title, "Camera settings");

  // Image size dropdown.
  lv_obj_t *size_label = lv_label_create(camera_panel_);
  lv_label_set_text(size_label, "Image size");
  camera_scale_dd_ = lv_dropdown_create(camera_panel_);
  lv_dropdown_set_options(camera_scale_dd_, "Full\nHalf\nQuarter");
  lv_dropdown_set_selected(camera_scale_dd_, 1); // Half (the BSP default)
  lv_obj_set_width(camera_scale_dd_, lv_pct(100));
  lv_obj_add_event_cb(camera_scale_dd_, camera_control_event_cb, LV_EVENT_VALUE_CHANGED, this);

  // A labeled switch row helper.
  auto add_switch = [&](const char *text, bool on) {
    lv_obj_t *row = lv_obj_create(camera_panel_);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_t *l = lv_label_create(row);
    lv_label_set_text(l, text);
    lv_obj_t *sw = lv_switch_create(row);
    if (on) {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(sw, camera_control_event_cb, LV_EVENT_VALUE_CHANGED, this);
    return sw;
  };
  camera_mirror_sw_ = add_switch("Mirror", false);
  camera_flip_sw_ = add_switch("Flip", false);
}

void Gui::camera_settings_toggle_cb(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (!gui || !gui->camera_panel_) {
    return;
  }
  if (lv_obj_has_flag(gui->camera_panel_, LV_OBJ_FLAG_HIDDEN)) {
    lv_obj_clear_flag(gui->camera_panel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(gui->camera_panel_);
  } else {
    lv_obj_add_flag(gui->camera_panel_, LV_OBJ_FLAG_HIDDEN);
  }
}

void Gui::camera_control_event_cb(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (gui) {
    gui->sync_camera_controls();
  }
}

void Gui::sync_camera_controls() {
  // Read every widget into camera_controls_ and push it to the BSP. (Called
  // from an LVGL event, i.e. already under the GUI mutex via lv_task_handler.)
  using Scale = espp::M5StackTab5::CameraScale;
  uint32_t scale_idx = lv_dropdown_get_selected(camera_scale_dd_);
  camera_controls_.scale = (scale_idx == 0)   ? Scale::FULL
                           : (scale_idx == 2) ? Scale::QUARTER
                                              : Scale::HALF;
  camera_controls_.hmirror = lv_obj_has_state(camera_mirror_sw_, LV_STATE_CHECKED);
  camera_controls_.vflip = lv_obj_has_state(camera_flip_sw_, LV_STATE_CHECKED);
  espp::M5StackTab5::get().set_camera_controls(camera_controls_);
}

void Gui::raise_camera_controls() {
  // Keep the overlay above the camera canvas, which is (re)created lazily and
  // would otherwise cover the controls.
  if (camera_settings_btn_) {
    lv_obj_move_foreground(camera_settings_btn_);
  }
  if (camera_panel_) {
    lv_obj_move_foreground(camera_panel_);
  }
}

void Gui::camera_canvas_drag_cb(lv_event_t *e) {
  auto *canvas = static_cast<lv_obj_t *>(lv_event_get_target(e));
  lv_indev_t *indev = lv_indev_active();
  if (!canvas || !indev) {
    return;
  }
  // Move the feed by the drag delta since the last event.
  lv_point_t vect;
  lv_indev_get_vect(indev, &vect);
  lv_obj_t *parent = lv_obj_get_parent(canvas);
  int32_t x = lv_obj_get_x(canvas) + vect.x;
  int32_t y = lv_obj_get_y(canvas) + vect.y;
  // Keep the feed within the tab; if it is larger than the tab, the range goes
  // negative so its far edges can still be panned into view.
  int32_t range_x = lv_obj_get_width(parent) - lv_obj_get_width(canvas);
  int32_t range_y = lv_obj_get_height(parent) - lv_obj_get_height(canvas);
  int32_t lo_x = LV_MIN(0, range_x), hi_x = LV_MAX(0, range_x);
  int32_t lo_y = LV_MIN(0, range_y), hi_y = LV_MAX(0, range_y);
  x = x < lo_x ? lo_x : (x > hi_x ? hi_x : x);
  y = y < lo_y ? lo_y : (y > hi_y ? hi_y : y);
  lv_obj_set_pos(canvas, x, y);
}

void Gui::set_camera_frame(const uint8_t *rgb565, int w, int h) {
  if (!rgb565 || w <= 0 || h <= 0) {
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    // element when it is smaller than the tab), and make it draggable within the
    // tab. LV_OBJ_FLAG_FLOATING takes it out of the tab's flex layout so it can
    // be freely positioned; the border brightens while it is pressed to hint
    // that it can be moved.
    lv_obj_add_flag(camera_canvas_, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(camera_canvas_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_border_width(camera_canvas_, 2, 0);
    lv_obj_set_style_border_color(camera_canvas_, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_border_color(camera_canvas_, lv_palette_main(LV_PALETTE_BLUE),
                                  LV_STATE_PRESSED);
    lv_obj_set_style_radius(camera_canvas_, 6, 0);
    lv_obj_add_event_cb(camera_canvas_, camera_canvas_drag_cb, LV_EVENT_PRESSING, this);
    // Center the feed when it fits within the tab; otherwise pin it to the
    // top-left so it fills from the corner (drag to pan the overflow).
    lv_obj_update_layout(camera_tab_);
    const int32_t tab_w = lv_obj_get_width(camera_tab_);
    const int32_t tab_h = lv_obj_get_height(camera_tab_);
    const bool fits = (w <= tab_w) && (h <= tab_h);
    lv_obj_set_pos(camera_canvas_, fits ? (tab_w - w) / 2 : 0, fits ? (tab_h - h) / 2 : 0);
    if (camera_label_) {
      lv_obj_add_flag(camera_label_, LV_OBJ_FLAG_HIDDEN);
    }
    // the canvas was just created on top of the settings overlay; restore the
    // overlay to the foreground so the gear button / panel stay tappable.
    raise_camera_controls();
  }
  std::memcpy(camera_buf_, rgb565, static_cast<size_t>(w) * static_cast<size_t>(h) * 2);
  lv_obj_invalidate(camera_canvas_);
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
