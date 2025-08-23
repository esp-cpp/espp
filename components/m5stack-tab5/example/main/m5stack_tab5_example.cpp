/**
 * @file m5stack_tab5_example.cpp
 * @brief M5Stack Tab5 BSP Example
 *
 * This example demonstrates the comprehensive functionality of the M5Stack Tab5
 * development board including display, touch, audio, camera, IMU, power management,
 * and communication interfaces.
 */

#include <chrono>
#include <deque>
#include <stdlib.h>
#include <vector>

#include "m5stack-tab5.hpp"

#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::deque<lv_obj_t *> circles;
static std::vector<uint8_t> audio_bytes;

static std::recursive_mutex lvgl_mutex;
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

static size_t load_audio();
static void play_click(espp::M5StackTab5 &tab5);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "M5Stack Tab5 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [m5stack tab5 example]
  espp::M5StackTab5 &tab5 = espp::M5StackTab5::get();
  tab5.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on M5Stack Tab5");

  // first let's get the internal i2c bus and probe for all devices on the bus
  logger.info("Probing internal I2C bus...");
  auto &i2c = tab5.internal_i2c();
  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 0; address < 128; address++) {
    if (i2c.probe_device(address)) {
      found_addresses.push_back(address);
    }
  }
  logger.info("Found devices at addresses: {::#02x}", found_addresses);

  // Initialize the IO expanders
  logger.info("Initializing IO expanders...");
  if (!tab5.initialize_io_expanders()) {
    logger.error("Failed to initialize IO expanders!");
    return;
  }

  logger.info("Initializing lcd...");
  // initialize the LCD
  if (!tab5.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  // initialize the display with a pixel buffer (Tab5 is 1280x720 with 2 bytes per pixel)
  logger.info("Initializing display...");
  auto pixel_buffer_size = tab5.display_width() * 10;
  if (!tab5.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = tab5.touchpad_convert(touch);
    auto touchpad_data = tab5.touchpad_convert(touch);
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
        play_click(tab5);
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  logger.info("Initializing touch...");
  if (!tab5.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touch!");
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

  using Imu = espp::M5StackTab5::Imu;
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

  logger.info("Initializing IMU...");
  // initialize the IMU
  if (!tab5.initialize_imu(kalman_filter_fn)) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  logger.info("Initializing sound...");
  // initialize the sound
  if (!tab5.initialize_audio()) {
    logger.error("Failed to initialize sound!");
    return;
  }

  // Brightness control with button
  logger.info("Initializing button...");
  auto button_callback = [&](const auto &state) {
    logger.info("Button state: {}", state.active);
    if (state.active) {
      // Cycle through brightness levels: 25%, 50%, 75%, 100%
      static int brightness_level = 0;
      float brightness_values[] = {0.25f, 0.5f, 0.75f, 1.0f};
      brightness_level = (brightness_level + 1) % 4;
      float new_brightness = brightness_values[brightness_level];
      tab5.brightness(new_brightness);
      logger.info("Set brightness to {:.0f}%", new_brightness * 100);
    }
  };
  if (!tab5.initialize_button(button_callback)) {
    logger.warn("Failed to initialize button");
  }

  logger.info("Setting up LVGL UI...");
  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, tab5.display_width(), tab5.display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

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
  static lv_point_precise_t line_points0[] = {{0, 0},
                                              {tab5.display_width(), tab5.display_height()}};
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
  static lv_point_precise_t line_points1[] = {{0, 0},
                                              {tab5.display_width(), tab5.display_height()}};
  lv_line_set_points(line1, line_points1, 2);
  lv_obj_add_style(line1, &style_line1, 0);

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
      btn,
      [](auto event) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
        static auto rotation = LV_DISPLAY_ROTATION_0;
        rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
        lv_display_t *disp = lv_display_get_default();
        lv_disp_set_rotation(disp, rotation);
      },
      LV_EVENT_PRESSED, nullptr);

  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

  // start a simple thread to do the lv_task_handler every 16ms
  logger.info("Starting LVGL task...");
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
                          .stack_size_bytes = 10 * 1024,
                      }});
  lv_task.start();

  // load the audio and play it once as a test
  logger.info("Loading audio...");
  auto num_bytes_loaded = load_audio();
  logger.info("Loaded {} bytes of audio", num_bytes_loaded);

  // unmute the audio and set the volume to 60%
  tab5.mute(false);
  tab5.volume(60.0f);

  // set the brightness to 75%
  tab5.brightness(75.0f);

  // make a task to read out the IMU data and print it to console
  logger.info("Starting IMU task...");
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 10ms);
         }
         static auto &tab5 = espp::M5StackTab5::get();
         static auto imu = tab5.imu();

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
         int x0 = tab5.display_width() / 2;
         int y0 = tab5.display_height() / 2;

         int x1 = x0 + 50 * gravity_vector.x;
         int y1 = y0 + 50 * gravity_vector.y;

         static lv_point_precise_t line_points0[] = {{x0, y0}, {x1, y1}};
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
  //! [m5stack tab5 example]
}

static void draw_circle(int x0, int y0, int radius) {
  lv_obj_t *circle = lv_obj_create(lv_scr_act());
  lv_obj_set_size(circle, radius * 2, radius * 2);
  lv_obj_set_pos(circle, x0 - radius, y0 - radius);
  lv_obj_set_style_radius(circle, radius, 0);
  lv_obj_set_style_bg_opa(circle, LV_OPA_50, 0);
  lv_obj_set_style_border_width(circle, 0, 0);
  lv_obj_set_style_bg_color(circle, lv_color_hex(0xFF0000), 0); // Red color

  circles.push_back(circle);

  // Limit the number of circles to prevent memory issues
  if (circles.size() > MAX_CIRCLES) {
    lv_obj_del(circles.front());
    circles.pop_front();
  }
}

static void clear_circles() {
  for (auto circle : circles) {
    lv_obj_del(circle);
  }
  circles.clear();
}

static size_t load_audio() {
  // load the audio data
  extern const uint8_t click_wav_start[] asm("_binary_click_wav_start");
  extern const uint8_t click_wav_end[] asm("_binary_click_wav_end");
  size_t click_wav_size = click_wav_end - click_wav_start;
  fmt::print("Click wav size: {} bytes\n", click_wav_size);
  audio_bytes = std::vector<uint8_t>(click_wav_start, click_wav_end);
  return audio_bytes.size();
}

static void play_click(espp::M5StackTab5 &tab5) {
  if (audio_bytes.size() > 0) {
    tab5.play_audio(audio_bytes);
  }
}
