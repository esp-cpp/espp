#include <chrono>
#include <vector>

#include "joystick.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Joystick", .level = espp::Logger::Verbosity::DEBUG});
  logger.info("Running joystick example");
  {
    logger.info("Starting circular / square joystick example");
    //! [circular joystick example]
    float min = 0;
    float max = 255;
    float center = 127;
    float deadband_percent = 0.1;
    float deadband = deadband_percent * (max - min);

    // circular joystick
    espp::Joystick js1({
        .x_calibration = {.center = center, .minimum = min, .maximum = max},
        .y_calibration = {.center = center, .minimum = min, .maximum = max},
        .type = espp::Joystick::Type::CIRCULAR,
        .center_deadzone_radius = deadband_percent,
        .range_deadzone = deadband_percent,
    });
    // square joystick (for comparison)
    espp::Joystick js2({
        .x_calibration = {.center = center,
                          .center_deadband = deadband,
                          .minimum = min,
                          .maximum = max,
                          .range_deadband = deadband},
        .y_calibration = {.center = center,
                          .center_deadband = deadband,
                          .minimum = min,
                          .maximum = max,
                          .range_deadband = deadband},
    });
    // now make a loop where we update the raw valuse and print out the joystick values
    fmt::print("raw x, raw y, js1 x, js1 y, js2 x, js2 y\n");
    for (float x = min - 10.0f; x <= max + 10.0f; x += 10.0f) {
      for (float y = min - 10.0f; y <= max + 10.0f; y += 10.0f) {
        js1.update(x, y);
        js2.update(x, y);
        fmt::print("{}, {}, {}, {}, {}, {}\n", x, y, js1.x(), js1.y(), js2.x(), js2.y());
      }
    }
    //! [circular joystick example]
  }

  {
    logger.info("Starting joystick deadzone (.5, .5) example");
    //! [joystick deadzone example]
    float min = 0;
    float max = 255;
    float center = 127;
    float center_deadzone_radius = 0.5;
    float range_deadzone_radius = 0.5;

    // circular joystick
    espp::Joystick js1({
        .x_calibration = {.center = center, .minimum = min, .maximum = max},
        .y_calibration = {.center = center, .minimum = min, .maximum = max},
        .type = espp::Joystick::Type::CIRCULAR,
        .center_deadzone_radius = center_deadzone_radius,
        .range_deadzone = range_deadzone_radius,
    });
    // now make a loop where we update the raw valuse and print out the joystick values
    fmt::print("raw x, raw y, js x, js y\n");
    for (float x = min - 10.0f; x <= max + 10.0f; x += 10.0f) {
      for (float y = min - 10.0f; y <= max + 10.0f; y += 10.0f) {
        js1.update(x, y);
        fmt::print("{}, {}, {}, {}\n", x, y, js1.x(), js1.y());
      }
    }
    // update the deadzones to be very large (overlapping)
    logger.info("Setting deadzones to overlap");
    js1.set_center_deadzone_radius(0.75);
    js1.set_range_deadzone(0.75);
    // and run the loop again
    fmt::print("raw x, raw y, js x, js y\n");
    for (float x = min - 10.0f; x <= max + 10.0f; x += 10.0f) {
      for (float y = min - 10.0f; y <= max + 10.0f; y += 10.0f) {
        js1.update(x, y);
        fmt::print("{}, {}, {}, {}\n", x, y, js1.x(), js1.y());
      }
    }
    // extreme case: set both deadzones to be greater than 1, which should
    // mean they are both clamped to 1 by joystick.
    logger.info("Setting deadzones to be greater than 1");
    js1.set_center_deadzone_radius(2);
    js1.set_range_deadzone(2);
    // and run the loop again
    fmt::print("raw x, raw y, js x, js y\n");
    for (float x = min - 10.0f; x <= max + 10.0f; x += 10.0f) {
      for (float y = min - 10.0f; y <= max + 10.0f; y += 10.0f) {
        js1.update(x, y);
        fmt::print("{}, {}, {}, {}\n", x, y, js1.x(), js1.y());
      }
    }
    //! [joystick deadzone example]
  }

  {
    logger.info("Starting ADC joystick example");
    //! [adc joystick example]
    static constexpr adc_unit_t ADC_UNIT = CONFIG_EXAMPLE_ADC_UNIT == 1 ? ADC_UNIT_1 : ADC_UNIT_2;
    static constexpr adc_channel_t ADC_CHANNEL_X = (adc_channel_t)CONFIG_EXAMPLE_ADC_CHANNEL_X;
    static constexpr adc_channel_t ADC_CHANNEL_Y = (adc_channel_t)CONFIG_EXAMPLE_ADC_CHANNEL_Y;

    std::vector<espp::AdcConfig> channels{
        {.unit = ADC_UNIT, .channel = ADC_CHANNEL_X, .attenuation = ADC_ATTEN_DB_12},
        {.unit = ADC_UNIT, .channel = ADC_CHANNEL_Y, .attenuation = ADC_ATTEN_DB_12}};
    espp::OneshotAdc adc({
        .unit = ADC_UNIT_2,
        .channels = channels,
    });
    auto read_joystick = [&adc, &channels](float *x, float *y) -> bool {
      // this will be in mv
      auto maybe_x_mv = adc.read_mv(channels[0]);
      auto maybe_y_mv = adc.read_mv(channels[1]);
      if (maybe_x_mv.has_value() && maybe_y_mv.has_value()) {
        auto x_mv = maybe_x_mv.value();
        auto y_mv = maybe_y_mv.value();
        *x = (float)(x_mv);
        *y = (float)(y_mv);
        return true;
      }
      return false;
    };
    espp::Joystick js1({
        // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
        .x_calibration =
            {.center = 1700.0f, .center_deadband = 100.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .y_calibration =
            {.center = 1700.0f, .center_deadband = 100.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .get_values = read_joystick,
    });
    espp::Joystick js2({
        // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
        .x_calibration =
            {.center = 1700.0f, .center_deadband = 0.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .y_calibration =
            {.center = 1700.0f, .center_deadband = 0.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .type = espp::Joystick::Type::CIRCULAR,
        .center_deadzone_radius = 0.1f,
        .get_values = read_joystick,
    });
    auto task_fn = [&js1, &js2](std::mutex &m, std::condition_variable &cv) {
      js1.update();
      js2.update();
      fmt::print("{}, {}\n", js1, js2);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Joystick"},
                            .log_level = espp::Logger::Verbosity::INFO});
    fmt::print("js1 x, js1 y, js2 x, js2 y\n");
    task.start();
    //! [adc joystick example]

    // loop forever to let the task run and user to play with the joystick
    while (true) {
      std::this_thread::sleep_for(1s);
    }
  }

  fmt::print("Joystick example complete!\n");
}
