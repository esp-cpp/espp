#pragma once

#include <stdlib.h>

#if defined(ESP_PLATFORM)
#include <esp_timer.h>
#else
#include <chrono>
#endif

#include "esp_dsp.h"

namespace espp {
/**
 *  @brief Simple lowpass filter using a time constant and a stored value.
 */
class SimpleLowpassFilter {
public:
  /**
   *  @brief Configuration for the lowpass filter.
   */
  struct Config {
    float time_constant = 0.0f; /**< Time constant of the filter. */
  };

  /**
   * @brief Initialize the lowpass filter coefficients based on the config.
   * @param config Configuration struct.
   */
  explicit SimpleLowpassFilter(const Config &config)
      : time_constant_(config.time_constant) {}

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float update(const float input) {
    float output;
#if defined(ESP_PLATFORM)
    uint64_t time = esp_timer_get_time();
    float dt = (time - prev_time_) / 1e6f;
    prev_time_ = time;
#else
    auto time = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(time - prev_time_).count();
    prev_time_ = time;
#endif
    if (dt <= 0) {
      return prev_output_;
    }
    float alpha = dt / (time_constant_ + dt);
    output = alpha * input + (1.0f - alpha) * prev_output_;
    prev_output_ = output;
    return output;
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float operator()(float input) { return update(input); }

  friend struct fmt::formatter<SimpleLowpassFilter>;

protected:
  float time_constant_ = 0.0f; /**< Time constant of the filter. */
  float prev_output_ = 0.0f;   /**< Previous output of the filter. */
#if defined(ESP_PLATFORM)
  uint64_t prev_time_ = 0; /**< Previous time the filter was updated. */
#else
  std::chrono::time_point<std::chrono::high_resolution_clock> prev_time_{
      std::chrono::high_resolution_clock::now()}; /**< Previous time the filter was updated. */
#endif
};
} // namespace espp

// for allowing easy serialization/printing of the
// espp::SimpleLowpassFilter
template <> struct fmt::formatter<espp::SimpleLowpassFilter> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::SimpleLowpassFilter const &f, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "SimpleLowpassFilter - {}", f.time_constant_);
  }
};
