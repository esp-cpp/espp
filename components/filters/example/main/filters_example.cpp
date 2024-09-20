#include <chrono>
#include <vector>

#include "esp_random.h"

#include "butterworth_filter.hpp"
#include "lowpass_filter.hpp"
#include "simple_lowpass_filter.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

/**
 * @brief Get a random number in the range [-1.0f, 1.0f]
 * @return Random floating point number.
 */
float get_random() { return ((float)esp_random() / (float)UINT32_MAX) * 2.0f - 1.0f; }

extern "C" void app_main(void) {
  {
    size_t num_seconds_to_run = 10;

    fmt::print("Running both Lowpass and Butterworth Filter for {} seconds\n", num_seconds_to_run);
    //! [filter example]
    static constexpr float sample_freq_hz = 50.0f;
    static constexpr float filter_cutoff_freq_hz = 2.0f;
    static constexpr float normalized_cutoff_frequency =
        2.0f * filter_cutoff_freq_hz / sample_freq_hz;
    espp::LowpassFilter lpf({
        .normalized_cutoff_frequency = normalized_cutoff_frequency,
        .q_factor = 1.0f,
    });
    espp::SimpleLowpassFilter slpf({
        .time_constant = normalized_cutoff_frequency,
    });
    static constexpr size_t ORDER = 2;
    // NOTE: using the Df2 since it's hardware accelerated :)
    espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2> butterworth(
        {.normalized_cutoff_frequency = normalized_cutoff_frequency});
    fmt::print("{}\n", butterworth);
    static auto start = std::chrono::high_resolution_clock::now();
    auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
      auto now = std::chrono::high_resolution_clock::now();
      float seconds = std::chrono::duration<float>(now - start).count();
      // use time to create a stairstep function
      constexpr float noise_scale = 0.2f;
      float input = floor(seconds) + get_random() * noise_scale;
      float slpf_output = slpf.update(input);
      float lpf_output = lpf.update(input);
      float bwf_output = butterworth.update(input);
      fmt::print("{:.03f}, {:.03f}, {:.03f}, {:.03f}, {:.03f}\n", seconds, input, slpf_output,
                 lpf_output, bwf_output);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, now + std::chrono::duration<float>(1.0f / sample_freq_hz));
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.name = "Lowpass Filter",
                            .callback = task_fn,
                            .log_level = espp::Logger::Verbosity::INFO});
    fmt::print("% time (s), input, simple_lpf_output, lpf_output, bwf_output\n");
    task.start();
    //! [filter example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("Filters example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
