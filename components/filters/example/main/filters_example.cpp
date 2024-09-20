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
    espp::ButterworthFilter<1, espp::BiquadFilterDf1> bwf_df1_o1(
        {.normalized_cutoff_frequency = normalized_cutoff_frequency});
    espp::ButterworthFilter<2, espp::BiquadFilterDf1> bwf_df1_o2(
        {.normalized_cutoff_frequency = normalized_cutoff_frequency});
    // NOTE: using the Df2 since it's hardware accelerated :)
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> bwf_df2_o2(
        {.normalized_cutoff_frequency = normalized_cutoff_frequency});
    espp::ButterworthFilter<4, espp::BiquadFilterDf2> bwf_df2_o4(
        {.normalized_cutoff_frequency = normalized_cutoff_frequency});
    static auto start = std::chrono::high_resolution_clock::now();
    auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
      auto now = std::chrono::high_resolution_clock::now();
      float seconds = std::chrono::duration<float>(now - start).count();
      // use time to create a stairstep function
      constexpr float noise_scale = 0.2f;
      float input = floor(seconds) + get_random() * noise_scale;
      fmt::print("{:.03f}, {:.03f}, {:.03f}, {:.03f}, "
                 "{:.03f}, {:.03f}, {:.03f}, {:.03f}\n",
                 seconds, input, slpf.update(input), lpf.update(input), bwf_df1_o1.update(input),
                 bwf_df1_o2.update(input), bwf_df2_o2.update(input), bwf_df2_o4.update(input));
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
    fmt::print("% time (s), input, simple_lpf_output, lpf_output, "
               "bwf_df1_o1_output, bwf_df1_o2_output, bwf_df2_o2_output, "
               "bwf_df2_o4_output\n");
    task.start();
    //! [filter example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("Filters example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
