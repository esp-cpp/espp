#include <chrono>
#include <vector>

#include "continuous_adc.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 5;

  {
    fmt::print("Reading oneshot adc for {} seconds\n", num_seconds_to_run);
    //! [oneshot adc example]
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
    auto task_fn = [&adc, &channels](std::mutex& m, std::condition_variable& cv) {
      for (auto &conf : channels) {
        auto channel = conf.channel;
        auto maybe_mv = adc.read_mv(channel);
        if (maybe_mv.has_value()) {
          fmt::print("Channel {}: {} mV\n", (int)channel, maybe_mv.value());
        } else {
          fmt::print("Channel {}: no value!\n", (int)channel);
        }
      }
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
        .name = "Oneshot ADC",
        .callback = task_fn,
        .log_level = espp::Logger::Verbosity::INFO
      });
    task.start();
    //! [oneshot adc example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  {
    fmt::print("Reading continuous adc for {} seconds\n", num_seconds_to_run);
    //! [continuous adc example]
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
    // this initailizes the DMA and filter task for the continuous adc
    espp::ContinuousAdc adc({
        .sample_rate_hz = 20*1000,
        .channels = channels,
        .convert_mode = ADC_CONV_SINGLE_UNIT_2, // or BOTH_UNIT, ALTER_UNIT, SINGLE_UNIT_1
        .window_size_bytes = 1024,
        .log_level = espp::Logger::Verbosity::WARN
      });
    auto task_fn = [&adc, &channels](std::mutex& m, std::condition_variable& cv) {
      for (auto &conf : channels) {
        auto channel = conf.channel;
        auto maybe_mv = adc.get_mv(channel);
        if (maybe_mv.has_value()) {
          fmt::print("Channel {}: {} mV\n", (int)channel, maybe_mv.value());
        } else {
          fmt::print("Channel {}: no value!\n", (int)channel);
        }
        auto maybe_rate = adc.get_rate(channel);
        if (maybe_rate.has_value()) {
          fmt::print("Channel {}: {} Hz\n", (int)channel, maybe_rate.value());
        } else {
          fmt::print("Channel {}: no rate!\n", (int)channel);
        }
      }
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
        .name = "Read ADC",
        .callback = task_fn,
        .log_level = espp::Logger::Verbosity::INFO
      });
    task.start();
    //! [continuous adc example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("ADC example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
