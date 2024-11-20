#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "ads1x15.hpp"
#include "controller.hpp"
#include "i2c.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // This example shows creating a digital controller without specifying all the
  // pins
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting digital controller example, press start & select together to quit!\n");
    //! [digital controller example]
    // make the controller - NOTE: this was designed for connecting the Sparkfun
    // Joystick Shield to the ESP32 S3 BOX
    espp::Controller controller(espp::Controller::DigitalConfig{
        // buttons short to ground, so they are active low. this will enable the
        // GPIO_PULLUP and invert the logic
        .active_low = true,
        .gpio_a = 38,      // D3 on the joystick shield
        .gpio_b = 39,      // D5 on the joystick shield
        .gpio_start = 42,  // D4 on the joystick shield
        .gpio_select = 21, // D6 on the joystick shield
        .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the controller and print
    // the state
    auto task_fn = [&quit_test, &controller](std::mutex &m, std::condition_variable &cv) {
      controller.update();
      bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
      bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
      bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
      bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
      fmt::print("Controller buttons:\n"
                 "\tA:      {}\n"
                 "\tB:      {}\n"
                 "\tSelect: {}\n"
                 "\tStart:  {}\n",
                 is_a_pressed, is_b_pressed, is_select_pressed, is_start_pressed);
      quit_test = is_start_pressed && is_select_pressed;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Controller Task",
                                    .stack_size_bytes = 6 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [digital controller example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  std::this_thread::sleep_for(500ms);

  // This example shows using 2 analog pins with the controller for the x/y
  // joystick
  {
    std::atomic<bool> quit_test = false;
    fmt::print(
        "Starting analog joystick controller example, press start & select together to quit!\n");
    //! [analog controller example]
    // make the adc we'll be reading from
    std::vector<espp::AdcConfig> channels{
        {.unit = ADC_UNIT_2,
         .channel = ADC_CHANNEL_1, // (x) Analog 0 on the joystick shield
         .attenuation = ADC_ATTEN_DB_12},
        {.unit = ADC_UNIT_2,
         .channel = ADC_CHANNEL_2, // (y) Analog 1 on the joystick shield
         .attenuation = ADC_ATTEN_DB_12}};
    espp::OneshotAdc adc(espp::OneshotAdc::Config{
        .unit = ADC_UNIT_2,
        .channels = channels,
    });
    // make the function which will get the raw data from the ADC and convert to
    // uncalibrated [-1,1]
    auto read_joystick = [&adc, &channels](float *x, float *y) -> bool {
      auto maybe_x_mv = adc.read_mv(channels[0]);
      auto maybe_y_mv = adc.read_mv(channels[1]);
      if (maybe_x_mv.has_value() && maybe_y_mv.has_value()) {
        auto x_mv = maybe_x_mv.value();
        auto y_mv = maybe_y_mv.value();
        *x = (x_mv / 1700.0f - 1.0f);
        *y = (y_mv / 1700.0f - 1.0f);
        return true;
      }
      return false;
    };
    // make the controller - NOTE: this was designed for connecting the Sparkfun
    // Joystick Shield to the ESP32 S3 BOX
    espp::Controller controller(espp::Controller::AnalogJoystickConfig{
        // buttons short to ground, so they are active low. this will enable the
        // GPIO_PULLUP and invert the logic
        .active_low = true,
        .gpio_a = 38,               // D3 on the joystick shield
        .gpio_b = 39,               // D5 on the joystick shield
        .gpio_x = -1,               // we're using this as start...
        .gpio_y = -1,               // we're using this as select...
        .gpio_start = 42,           // D4 on the joystick shield
        .gpio_select = 21,          // D6 on the joystick shield
        .gpio_joystick_select = -1, // D2 on the joystick shield
        .joystick_config = {.x_calibration = {.center = 0.0f,
                                              .center_deadband = 0.2f,
                                              .minimum = -1.0f,
                                              .maximum = 1.0f},
                            .y_calibration = {.center = 0.0f,
                                              .center_deadband = 0.2f,
                                              .minimum = -1.0f,
                                              .maximum = 1.0f},
                            .get_values = read_joystick,
                            .log_level = espp::Logger::Verbosity::WARN},
        .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the controller and print
    // the state
    auto task_fn = [&quit_test, &controller](std::mutex &m, std::condition_variable &cv) {
      controller.update();
      bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
      bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
      bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
      bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
      bool is_up_pressed = controller.is_pressed(espp::Controller::Button::UP);
      bool is_down_pressed = controller.is_pressed(espp::Controller::Button::DOWN);
      bool is_left_pressed = controller.is_pressed(espp::Controller::Button::LEFT);
      bool is_right_pressed = controller.is_pressed(espp::Controller::Button::RIGHT);
      fmt::print("Controller buttons:\n"
                 "\tA:      {}\n"
                 "\tB:      {}\n"
                 "\tSelect: {}\n"
                 "\tStart:  {}\n"
                 "\tUp:     {}\n"
                 "\tDown:   {}\n"
                 "\tLeft:   {}\n"
                 "\tRight:  {}\n",
                 is_a_pressed, is_b_pressed, is_select_pressed, is_start_pressed, is_up_pressed,
                 is_down_pressed, is_left_pressed, is_right_pressed);
      quit_test = is_start_pressed && is_select_pressed;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Controller Task",
                                    .stack_size_bytes = 6 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [analog controller example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  std::this_thread::sleep_for(500ms);

  // This example shows using the i2c adc (ads1x15) with the controller for
  // the x/y joystick
  {
    std::atomic<bool> quit_test = false;
    fmt::print(
        "Starting i2c adc joystick controller example, press start & select together to quit!\n");
    //! [i2c analog controller example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    // make the actual ads class
    espp::Ads1x15 ads(espp::Ads1x15::Ads1015Config{
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3)});
    // make the task which will get the raw data from the I2C ADC and convert to
    // uncalibrated [-1,1]
    std::atomic<float> joystick_x{0};
    std::atomic<float> joystick_y{0};
    auto ads_read_task_fn = [&joystick_x, &joystick_y, &ads](std::mutex &m,
                                                             std::condition_variable &cv) {
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 20ms);
      }
      std::error_code ec;
      auto x_mv = ads.sample_mv(1, ec);
      if (ec) {
        fmt::print("error reading x: {}\n", ec.message());
        return false;
      }
      auto y_mv = ads.sample_mv(0, ec);
      if (ec) {
        fmt::print("error reading y: {}\n", ec.message());
        return false;
      }
      joystick_x.store(x_mv / 1700.0f - 1.0f);
      // y is inverted so negate it
      joystick_y.store(-(y_mv / 1700.0f - 1.0f));
      // we don't want to stop, so return false
      return false;
    };
    auto ads_task = espp::Task::make_unique({.callback = ads_read_task_fn,
                                             .task_config =
                                                 {
                                                     .name = "ADS Task",
                                                     .stack_size_bytes{4 * 1024},
                                                 },
                                             .log_level = espp::Logger::Verbosity::INFO});
    ads_task->start();
    // make the read joystick function used by the controller
    auto read_joystick = [&joystick_x, &joystick_y](float *x, float *y) -> bool {
      *x = joystick_x.load();
      *y = joystick_y.load();
      return true;
    };
    // make the controller - NOTE: this was designed for connecting the Adafruit
    // JoyBonnet to the ESP32 S3 BOX
    espp::Controller controller(espp::Controller::AnalogJoystickConfig{
        // buttons short to ground, so they are active low. this will enable the
        // GPIO_PULLUP and invert the logic
        .active_low = true,
        .gpio_a = 38,      // pin 32 on the joybonnet
        .gpio_b = 39,      // pin 31 on the joybonnet
        .gpio_x = -1,      // pin 36 on the joybonnet
        .gpio_y = -1,      // pin 33 on the joybonnet
        .gpio_start = 42,  // pin 37 on the joybonnet
        .gpio_select = 21, // pin 38 on the joybonnet
        .gpio_joystick_select = -1,
        .joystick_config = {.x_calibration = {.center = 0.0f,
                                              .center_deadband = 0.2f,
                                              .minimum = -1.0f,
                                              .maximum = 1.0f},
                            .y_calibration = {.center = 0.0f,
                                              .center_deadband = 0.2f,
                                              .minimum = -1.0f,
                                              .maximum = 1.0f},
                            .get_values = read_joystick,
                            .log_level = espp::Logger::Verbosity::WARN},
        .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the controller and print
    // the state
    auto task_fn = [&quit_test, &controller](std::mutex &m, std::condition_variable &cv) {
      controller.update();
      bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
      bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
      bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
      bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
      bool is_up_pressed = controller.is_pressed(espp::Controller::Button::UP);
      bool is_down_pressed = controller.is_pressed(espp::Controller::Button::DOWN);
      bool is_left_pressed = controller.is_pressed(espp::Controller::Button::LEFT);
      bool is_right_pressed = controller.is_pressed(espp::Controller::Button::RIGHT);
      fmt::print("Controller buttons:\n"
                 "\tA:      {}\n"
                 "\tB:      {}\n"
                 "\tSelect: {}\n"
                 "\tStart:  {}\n"
                 "\tUp:     {}\n"
                 "\tDown:   {}\n"
                 "\tLeft:   {}\n"
                 "\tRight:  {}\n",
                 is_a_pressed, is_b_pressed, is_select_pressed, is_start_pressed, is_up_pressed,
                 is_down_pressed, is_left_pressed, is_right_pressed);
      quit_test = is_start_pressed && is_select_pressed;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, return false
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Controller Task",
                                    .stack_size_bytes = 6 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [i2c analog controller example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Controller example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
