#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "gaussian.hpp"
#include "led.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting led example!\n");
  {
    //! [linear led example]
    fmt::print("Starting linear led example!\n");
    float num_seconds_to_run = 10.0f;
    int led_fade_time_ms = 1000;
    std::vector<espp::Led::ChannelConfig> led_channels{{
        .gpio = 2,
        .channel = LEDC_CHANNEL_5,
        .timer = LEDC_TIMER_2,
    }};
    espp::Led led(espp::Led::Config{
        .timer = LEDC_TIMER_2,
        .frequency_hz = 5000,
        .channels = led_channels,
        .duty_resolution = LEDC_TIMER_10_BIT,
    });
    auto led_channel = led_channels[0].channel;
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      // if we can change the led, the previous fade is done
      if (led.can_change(led_channel)) {
        auto maybe_duty = led.get_duty(led_channel);
        if (maybe_duty.has_value()) {
          auto current_duty = maybe_duty.value();
          float new_duty = (current_duty < 50.0f) ? 100.0f : 0.0f;
          // start a new fade to the opposite duty cycle
          led.set_fade_with_time(led_channel, new_duty, led_fade_time_ms);
        }
      }
      // sleep for a little bit
      std::this_thread::sleep_for(100ms);
      // update our elapsed time
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
    }
    //! [linear led example]
  }

  {
    //! [breathing led example]
    fmt::print("Starting gaussian led example!\n");
    float breathing_period = 3.5f; // seconds
    float num_periods_to_run = 2.0f;
    std::vector<espp::Led::ChannelConfig> led_channels{{
        .gpio = 2,
        .channel = LEDC_CHANNEL_5,
        .timer = LEDC_TIMER_2,
    }};
    espp::Led led(espp::Led::Config{
        .timer = LEDC_TIMER_2,
        .frequency_hz = 5000,
        .channels = led_channels,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .clock_config = LEDC_USE_RC_FAST_CLK,
    });
    espp::Gaussian gaussian({.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f});
    auto breathe = [&gaussian, &breathing_period]() -> float {
      static auto breathing_start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - breathing_start).count();
      float t = std::fmod(elapsed, breathing_period) / breathing_period;
      return gaussian(t);
    };
    auto led_callback = [&breathe, &led, &led_channels](auto &m, auto &cv) -> bool {
      led.set_duty(led_channels[0].channel, 100.0f * breathe());
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 10ms);
      return false;
    };
    auto led_task =
        espp::Task::make_unique({.callback = led_callback, .task_config = {.name = "breathe"}});
    led_task->start();
    float wait_time = num_periods_to_run * breathing_period;
    fmt::print("Sleeping for {:.1f}s...\n", wait_time);
    std::this_thread::sleep_for(wait_time * 1.0s);
    //! [breathing led example]
  }

  fmt::print("LED example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
