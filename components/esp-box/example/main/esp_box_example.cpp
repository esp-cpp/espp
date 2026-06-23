#include <array>
#include <chrono>
#include <stdlib.h>
#include <vector>

#include "esp-box.hpp"

#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
struct Circle {
  int x{0};
  int y{0};
  int radius{0};
  bool visible{false};
};
static std::array<Circle, MAX_CIRCLES> circles;
static size_t next_circle_index = 0;
static size_t visible_circle_count = 0;
static std::vector<uint8_t> audio_bytes;
static lv_obj_t *circle_layer = nullptr;

static std::recursive_mutex lvgl_mutex;
static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::EspBox &box);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP BOX Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp box example]
  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = box.touchpad_convert(touch);
    auto touchpad_data = box.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
      }
      // if there is a touch point, draw a circle and play a click sound
      if (touchpad_data.num_touch_points > 0) {
        play_click(box);
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  // initialize the sound
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // make the filter we'll use for the IMU to compute the orientation
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);
  static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
  static espp::MadgwickFilter f(beta);

  using Imu = espp::EspBox::Imu;
  auto kalman_filter_fn = [](float dt, const Imu::Value &accel,
                             const Imu::Value &gyro) -> Imu::Value {
    // Apply Kalman filter
    float accelRoll = atan2(accel.y, accel.z);
    float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y)}, dt);
    kf.update({accelRoll, accelPitch});
    float roll, pitch;
    std::tie(roll, pitch) = kf.get_state();
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = 0.0f;
    return orientation;
  };

  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel,
                               const Imu::Value &gyro) -> Imu::Value {
    // Apply Madgwick filter
    f.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y),
             espp::deg_to_rad(gyro.z));
    float roll, pitch, yaw;
    f.get_euler(roll, pitch, yaw);
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = espp::deg_to_rad(roll);
    orientation.pitch = espp::deg_to_rad(pitch);
    orientation.yaw = espp::deg_to_rad(yaw);
    return orientation;
  };

  // initialize the IMU
  if (!box.initialize_imu(kalman_filter_fn)) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, box.lcd_width(), box.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  if (!initialize_circle_layer(box.lcd_width(), box.lcd_height())) {
    logger.error("Failed to initialize circle layer!");
    return;
  }

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  static std::string label_text =
      "\n\n\n\nTouch the screen!\nPress the home button to clear circles.";
  lv_label_set_text(label, label_text.c_str());
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

  /*Create style*/
  static lv_style_t style_line0;
  lv_style_init(&style_line0);
  lv_style_set_line_width(&style_line0, 8);
  lv_style_set_line_color(&style_line0, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_line_rounded(&style_line0, true);

  // make a line for showing the direction of "down"
  lv_obj_t *line0 = lv_line_create(lv_screen_active());
  static lv_point_precise_t line_points0[] = {{0, 0}, {box.lcd_width(), box.lcd_height()}};
  lv_line_set_points(line0, line_points0, 2);
  lv_obj_add_style(line0, &style_line0, 0);

  /*Create style*/
  static lv_style_t style_line1;
  lv_style_init(&style_line1);
  lv_style_set_line_width(&style_line1, 8);
  lv_style_set_line_color(&style_line1, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_line_rounded(&style_line1, true);

  // make a line for showing the direction of "down"
  lv_obj_t *line1 = lv_line_create(lv_screen_active());
  static lv_point_precise_t line_points1[] = {{0, 0}, {box.lcd_width(), box.lcd_height()}};
  lv_line_set_points(line1, line_points1, 2);
  lv_obj_add_style(line1, &style_line1, 0);

  static auto rotate_display = [&]() {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
    clear_circles();
    static auto rotation = LV_DISPLAY_ROTATION_0;
    rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
    lv_display_t *disp = lv_display_get_default();
    lv_disp_set_rotation(disp, rotation);
    // update the size of the screen
    lv_obj_set_size(bg, box.rotated_display_width(), box.rotated_display_height());
    if (circle_layer) {
      lv_obj_set_size(circle_layer, box.rotated_display_width(), box.rotated_display_height());
      lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
      lv_obj_invalidate(circle_layer);
    }
  };

  // add a button in the top left which (when pressed) will rotate the display
  // through 0, 90, 180, 270 degrees
  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 50, 50);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_t *label_btn = lv_label_create(btn);
  lv_label_set_text(label_btn, LV_SYMBOL_REFRESH);
  // center the text in the button
  lv_obj_align(label_btn, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(
      btn, [](auto event) { rotate_display(); }, LV_EVENT_PRESSED, nullptr);

  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

  // initialize the touchpad after the circle canvas exists so touch events can
  // update the trail immediately.
  if (!box.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }
  lv_obj_move_foreground(circle_layer);

  // start a simple thread to do the lv_task_handler every 16ms
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                          .stack_size_bytes = 6 * 1024,
                      }});
  lv_task.start();

  // load the audio file (wav file bundled in memory)
  size_t wav_size = 0;
  size_t wav_sample_rate = 0;
  if (!load_audio(wav_size, wav_sample_rate)) {
    logger.error("Failed to load audio file!");
    return;
  }
  logger.info("Loaded {} bytes of audio", wav_size);

  logger.info("Setting audio sample rate to {} Hz", wav_sample_rate);
  box.audio_sample_rate(wav_sample_rate);

  // unmute the audio and set the volume to 60%
  box.mute(false);
  box.volume(60.0f);

  // set the display brightness to be 75%
  box.brightness(75.0f);

  // make a task to read out the IMU data and print it to console
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 10ms);
         }
         static auto &box = espp::EspBox::get();
         static auto imu = box.imu();

         auto now = esp_timer_get_time(); // time in microseconds
         static auto t0 = now;
         auto t1 = now;
         float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
         t0 = t1;

         std::error_code ec;
         // update the imu data
         if (!imu->update(dt, ec)) {
           return false;
         }
         // get accel
         auto accel = imu->get_accelerometer();
         auto gyro = imu->get_gyroscope();
         auto temp = imu->get_temperature();
         auto orientation = imu->get_orientation();
         auto gravity_vector = imu->get_gravity_vector();

         auto box_type = box.box_type();
         if (box_type == espp::EspBox::BoxType::BOX) {
           std::swap(gravity_vector.x, gravity_vector.y);
           gravity_vector.y = -gravity_vector.y;
         }

         // now update the gravity vector line to show the direction of "down"
         // taking into account the configured rotation of the display
         auto rotation = lv_display_get_rotation(lv_display_get_default());
         if (rotation == LV_DISPLAY_ROTATION_90) {
           std::swap(gravity_vector.x, gravity_vector.y);
           gravity_vector.x = -gravity_vector.x;
         } else if (rotation == LV_DISPLAY_ROTATION_180) {
           gravity_vector.x = -gravity_vector.x;
           gravity_vector.y = -gravity_vector.y;
         } else if (rotation == LV_DISPLAY_ROTATION_270) {
           std::swap(gravity_vector.x, gravity_vector.y);
           gravity_vector.y = -gravity_vector.y;
         }

         std::string text = fmt::format("{}\n\n\n\n\n", label_text);
         text += fmt::format("Accel: {:02.2f} {:02.2f} {:02.2f}\n", accel.x, accel.y, accel.z);
         text += fmt::format("Gyro: {:03.2f} {:03.2f} {:03.2f}\n", espp::deg_to_rad(gyro.x),
                             espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
         text += fmt::format("Angle: {:03.2f} {:03.2f}\n", espp::rad_to_deg(orientation.roll),
                             espp::rad_to_deg(orientation.pitch));
         text += fmt::format("Temp: {:02.1f} C\n", temp);

         // use the pitch to to draw a line on the screen indiating the
         // direction from the center of the screen to "down"
         int x0 = box.rotated_display_width() / 2;
         int y0 = box.rotated_display_height() / 2;

         int x1 = x0 + 50 * gravity_vector.x;
         int y1 = y0 + 50 * gravity_vector.y;

         static lv_point_precise_t line_points0[] = {{x0, y0}, {x1, y1}};
         line_points0[0].x = x0;
         line_points0[0].y = y0;
         line_points0[1].x = x1;
         line_points0[1].y = y1;

         // Now show the madgwick filter
         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
         float roll = madgwick_orientation.roll;
         float pitch = madgwick_orientation.pitch;
         [[maybe_unused]] float yaw = madgwick_orientation.yaw;
         float vx = sin(pitch);
         float vy = -cos(pitch) * sin(roll);
         [[maybe_unused]] float vz = -cos(pitch) * cos(roll);

         if (box_type == espp::EspBox::BoxType::BOX) {
           std::swap(vx, vy);
           vy = -vy;
         }

         // now update the line to show the direction of "down" based on the
         // configured rotation of the display
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

         x1 = x0 + 50 * vx;
         y1 = y0 + 50 * vy;

         static lv_point_precise_t line_points1[] = {{x0, y0}, {x1, y1}};
         line_points1[0].x = x0;
         line_points1[0].y = y0;
         line_points1[1].x = x1;
         line_points1[1].y = y1;

         std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
         lv_label_set_text(label, text.c_str());
         lv_line_set_points(line0, line_points0, 2);
         lv_line_set_points(line1, line_points1, 2);

         return false;
       },
       .task_config = {
           .name = "IMU",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 0,
       }});
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [esp box example]
}

static bool initialize_circle_layer(int width, int height) {
  if (circle_layer) {
    return true;
  }

  circle_layer = lv_obj_create(lv_screen_active());
  if (!circle_layer) {
    return false;
  }
  lv_obj_remove_style_all(circle_layer);
  lv_obj_set_size(circle_layer, width, height);
  lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(circle_layer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(circle_layer, 0, 0);
  lv_obj_set_style_outline_width(circle_layer, 0, 0);
  lv_obj_set_style_shadow_width(circle_layer, 0, 0);
  lv_obj_add_event_cb(circle_layer, draw_circle_layer, LV_EVENT_DRAW_MAIN, nullptr);
  return true;
}

static void draw_circle_layer(lv_event_t *event) {
  if (visible_circle_count == 0) {
    return;
  }

  auto *obj = static_cast<lv_obj_t *>(lv_event_get_current_target(event));
  auto *layer = lv_event_get_layer(event);
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

  for (const auto &circle : circles) {
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

static void invalidate_circle_area(const Circle &circle) {
  if (!circle_layer || circle.radius <= 0) {
    return;
  }

  lv_area_t obj_coords;
  lv_obj_get_coords(circle_layer, &obj_coords);
  lv_area_t coords = {
      .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
      .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
      .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
      .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
  };
  lv_obj_invalidate_area(circle_layer, &coords);
}

static void draw_circle(int x0, int y0, int radius) {
  if (!circle_layer) {
    return;
  }
  lv_obj_move_foreground(circle_layer);
  Circle previous_circle = circles[next_circle_index];
  circles[next_circle_index] = {.x = x0, .y = y0, .radius = radius, .visible = true};
  next_circle_index = (next_circle_index + 1) % circles.size();
  if (visible_circle_count < circles.size()) {
    visible_circle_count++;
  }
  if (previous_circle.visible) {
    invalidate_circle_area(previous_circle);
  }
  invalidate_circle_area(circles[(next_circle_index + circles.size() - 1) % circles.size()]);
}

static void clear_circles() {
  for (auto &circle : circles) {
    if (circle.visible) {
      invalidate_circle_area(circle);
    }
    circle.visible = false;
  }
  next_circle_index = 0;
  visible_circle_count = 0;
}

static bool load_audio(size_t &out_size, size_t &out_sample_rate) {
  // if the audio_bytes vector is already populated, return the size
  if (audio_bytes.size() > 0) {
    return true;
  }

  // load the audio data. these are configured in the CMakeLists.txt file

  extern const uint8_t click_wav_start[] asm("_binary_click_wav_start");
  extern const uint8_t click_wav_end[] asm("_binary_click_wav_end");
  audio_bytes = std::vector<uint8_t>(click_wav_start, click_wav_end);
  // ensure we have at least a wav header
  if (audio_bytes.size() < 44) {
    audio_bytes.clear();
    return false;
  }
  // get the sample rate from the wav header (bytes 24-27)
  uint32_t sample_rate = *(reinterpret_cast<const uint32_t *>(&audio_bytes[24]));
  // set the audio sample rate accordingly
  // decode the wav file header (first 44 bytes) and remove it
  if (audio_bytes.size() > 44) {
    audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + 44);
  }
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}

static void play_click(espp::EspBox &box) {
  // use the box.play_audio() function to play a sound, breaking it into
  // audio_buffer_size chunks
  auto audio_buffer_size = box.audio_buffer_size();
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    size_t bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    box.play_audio(audio_bytes.data() + offset, bytes_to_play);
    offset += bytes_to_play;
  }
}
