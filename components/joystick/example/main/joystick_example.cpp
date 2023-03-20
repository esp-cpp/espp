#include <chrono>
#include <vector>

#include "joystick.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Running joystick example\n");
    //! [adc joystick example]
    std::vector<espp::AdcConfig> channels{
      {
        .unit = ADC_UNIT_2,
        .channel = ADC_CHANNEL_9,  // Qt Py ESP32 PICO A0
        .attenuation = ADC_ATTEN_DB_11
      },
      {
        .unit = ADC_UNIT_2,
        .channel = ADC_CHANNEL_8,  // Qt Py ESP32 PICO A1
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
        *x = (float)(x_mv);
        *y = (float)(y_mv);
        return true;
      }
      return false;
    };
    espp::Joystick js1({
        // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
        .x_calibration = {.center = 1700.0f, .deadband = 100.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .y_calibration = {.center = 1700.0f, .deadband = 100.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .get_values = read_joystick,
      });
    espp::Joystick js2({
        // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
        .x_calibration = {.center = 1700.0f, .deadband = 0.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .y_calibration = {.center = 1700.0f, .deadband = 0.0f, .minimum = 0.0f, .maximum = 3300.0f},
        .deadzone = espp::Joystick::Deadzone::CIRCULAR,
        .deadzone_radius = 0.1f,
        .get_values = read_joystick,
      });
    auto task_fn = [&js1, &js2](std::mutex& m, std::condition_variable& cv) {
      js1.update();
      js2.update();
      fmt::print("{:.2f},{:.2f},{:.2f},{:.2f}\n",
                 js1.x(), js1.y(),
                 js2.x(), js2.y());
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({
        .name = "Joystick",
        .callback = task_fn,
        .log_level = espp::Logger::Verbosity::INFO
      });
    fmt::print("js1 x, js1 y, js2 x, js2 y\n");
    task.start();
    //! [adc joystick example]
    while (true) {
      std::this_thread::sleep_for(1s);
    }
  }

  fmt::print("Joystick example complete!\n");
}
