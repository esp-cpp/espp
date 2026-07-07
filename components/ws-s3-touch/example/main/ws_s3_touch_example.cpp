#include <chrono>
#include <cmath>
#include <stdlib.h>
#include <utility>

#include "ws-s3-touch.hpp"

#include "gui.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Waveshare S3 Touch Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [ws-s3-touch example]
  using Bsp = espp::WsS3Touch;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the buzzer
  if (!bsp.initialize_buzzer()) {
    logger.error("Failed to initialize buzzer!");
    return;
  }
  // initialize the LCD
  if (!bsp.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = bsp.lcd_width() * 50;
  // initialize the LVGL display
  if (!bsp.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the RTC
  if (!bsp.initialize_rtc()) {
    logger.error("Failed to initialize RTC!");
    return;
  }
  // now set the time on the RTC
  std::tm timeinfo{};
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 42;
  timeinfo.tm_hour = 13;
  timeinfo.tm_mday = 24;
  timeinfo.tm_mon = 10;           // 0-11, so 10 is November
  timeinfo.tm_year = 2023 - 1900; // years since 1900
  std::mktime(&timeinfo);

  std::error_code ec;
  bsp.rtc()->set_time(timeinfo, ec);
  if (ec) {
    logger.error("Failed to set RTC time: {}", ec.message());
    return;
  }

  // make the filter we'll use for the IMU to compute the orientation
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);
  static constexpr float beta = 0.9f; // higher = more accelerometer, lower = more gyro
  static espp::MadgwickFilter f(beta);

  using Imu = Bsp::Imu;
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
  if (!bsp.initialize_imu(kalman_filter_fn)) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  logger.info("Initialization complete, starting LVGL!");

  // create the GUI: builds the UI (labels, buttons, gravity lines, circle
  // layer) and starts the task which updates LVGL. All of its public methods
  // are thread-safe, so the touch callback and the RTC / IMU tasks below can
  // call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  static const std::string instructions =
      fmt::format("\n\n\n\nTouch the screen!\nPress the boot button or the {} button to clear "
                  "circles.\nPress the {} button to rotate the display.",
                  LV_SYMBOL_TRASH, LV_SYMBOL_REFRESH);
  gui.set_label_text(instructions);

  // initialize the touchpad; each touch draws a circle (and drives the
  // buzzer), while the touchpad's button clears the circles
  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = bsp.touchpad_convert(touch);
    auto touchpad_data = bsp.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        gui.clear_circles();
      }
      // if there is a touch point, draw a circle and play a click sound
      if (touchpad_data.num_touch_points > 0) {
        // set the PWM / frequency for the buzzer based on the touch point (x -> pwm, y ->
        // frequency)
        float pwm =
            touchpad_data.x / static_cast<float>(bsp.lcd_width()) * 100.0f; // scale to 0-100%
        // scale frequency to be in range [50 Hz, 10 KHz]
        static constexpr float min_frequency_hz = 50.0f;
        static constexpr float max_frequency_hz = 10000.0f;
        // make it a logarithmic scale so that the frequency is more sensitive to
        // the lower end of the touchpad
        float frequency_hz =
            min_frequency_hz * std::pow(max_frequency_hz / min_frequency_hz,
                                        touchpad_data.y / static_cast<float>(bsp.lcd_height()));
        bsp.buzzer(pwm, frequency_hz);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      } else {
        // if there are no touch points, stop the buzzer
        bsp.buzzer(0.0f);
      }
    }
  };
  if (!bsp.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // initialize the button; while pressed, clear the circles on the screen and
  // play a click sound
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      logger.info("Button pressed");
      gui.clear_circles();
      // play a click sound
      bsp.buzzer(50.0f, 1000.0f); // 50% duty cycle, 1 kHz frequency
    } else {
      logger.info("Button released");
      // stop the buzzer
      bsp.buzzer(0.0f); // stop the buzzer
    }
  };
  if (!bsp.initialize_button(on_button_pressed)) {
    logger.error("Failed to initialize button!");
    return;
  }

  // set the display brightness to be 75%
  bsp.brightness(75.0f);

  // make a task to read the RTC and update the clock label with it
  espp::Task rtc_task({.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
                         auto start = std::chrono::steady_clock::now();
                         static auto &bsp = Bsp::get();
                         static auto rtc = bsp.rtc();
                         std::error_code ec;
                         std::tm timeinfo = rtc->get_time(ec);
                         if (ec) {
                           logger.error("Failed to get RTC time: {}", ec.message());
                         } else {
                           // update the clock label with the current time
                           gui.set_clock_text(fmt::format(
                               "{:02d}:{:02d}:{:02d} - {:02d}/{:02d}/{:04d}", timeinfo.tm_hour,
                               timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday,
                               timeinfo.tm_mon + 1, timeinfo.tm_year + 1900));
                         }
                         std::unique_lock<std::mutex> lock(m);
                         cv.wait_until(lock, start + 1s);
                         return false;
                       },
                       .task_config = {
                           .name = "rtc_task",
                           .stack_size_bytes = 4 * 1024,
                       }});
  rtc_task.start();

  // make a task to read out the IMU data and update the GUI with it
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 10ms);
         }
         static auto &bsp = Bsp::get();
         static auto imu = bsp.imu();

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

         // NOTE: because of the mounting of the IMU w.r.t the mounting of the
         // screen we have to rotate the axes.
         std::swap(gravity_vector.x, gravity_vector.y);
         gravity_vector.y = -gravity_vector.y;

         std::string text = fmt::format("{}\n\n\n\n\n", instructions);
         text += fmt::format("Accel: {:02.2f} {:02.2f} {:02.2f}\n", accel.x, accel.y, accel.z);
         text += fmt::format("Gyro: {:03.2f} {:03.2f} {:03.2f}\n", espp::deg_to_rad(gyro.x),
                             espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
         text += fmt::format("Angle: {:03.2f} {:03.2f}\n", espp::rad_to_deg(orientation.roll),
                             espp::rad_to_deg(orientation.pitch));
         text += fmt::format("Temp: {:02.1f} C\n", temp);

         // Now show the madgwick filter's estimate of "down"
         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
         float roll = madgwick_orientation.roll;
         float pitch = madgwick_orientation.pitch;
         float vx = sin(pitch);
         float vy = -cos(pitch) * sin(roll);

         // NOTE: because of the mounting of the IMU w.r.t the mounting of the
         // screen we have to rotate the axes.
         std::swap(vx, vy);
         vy = -vy;

         // update the GUI with the new data; the Gui handles remapping the
         // vectors for the current display rotation
         gui.set_label_text(text);
         gui.set_kalman_down(gravity_vector.x, gravity_vector.y);
         gui.set_madgwick_down(vx, vy);

         return false;
       },
       .task_config = {
           .name = "IMU",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 0,
       }});
  imu_task.start();

  logger.info("Example started, waiting for touch events...");

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [ws-s3-touch example]
}
