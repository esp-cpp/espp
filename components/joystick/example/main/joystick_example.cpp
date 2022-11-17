#include <chrono>
#include <vector>

#include "joystick.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 5;

  {
    fmt::print("Running ADC joystick for {} seconds\n", num_seconds_to_run);
    //! [adc joystick example]
    std::vector<espp::AdcConfig> channels{
      {
        .unit = ADC_UNIT_2,
        .channel = ADC_CHANNEL_1,
        .attenuation = ADC_ATTEN_DB_11
      },
      {
        .unit = ADC_UNIT_2,
        .channel = ADC_CHANNEL_2,
        .attenuation = ADC_ATTEN_DB_11
      }
    };
    espp::OneshotAdc adc({
        .unit = ADC_UNIT_2,
        .channels = channels,
      });
    auto read_joystick = [&adc, &channels](float *x, float *y) -> bool {
      // this will be in mv
      auto maybe_x_mv = adc.read_mv(channels[0].channel);
      auto maybe_y_mv = adc.read_mv(channels[1].channel);
      if (maybe_x_mv.has_value() && maybe_y_mv.has_value()) {
        auto x_mv = maybe_x_mv.value();
        auto y_mv = maybe_y_mv.value();
        // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
        *x = (float)(x_mv) / 1700.0f - 1.0f;
        *y = (float)(y_mv) / 1700.0f - 1.0f;
        return true;
      }
      return false;
    };
    espp::Joystick joystick({
        .x_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
        .y_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
        .get_values = read_joystick,
      });
    auto task_fn = [&joystick](std::mutex& m, std::condition_variable& cv) {
      joystick.update();
      fmt::print("Joystick values: {}\n", joystick.position().to_string());
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
    };
    auto task = espp::Task({
        .name = "Joystick",
        .callback = task_fn,
        .log_level = espp::Logger::Verbosity::INFO
      });
    task.start();
    //! [adc joystick example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("Joystick example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
