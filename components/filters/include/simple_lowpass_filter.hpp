#pragma once

#include <stdlib.h>

#if defined(ESP_PLATFORM)
#include <esp_timer.h>
#else
#include <chrono>
#endif

#include "format.hpp"

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
   * @brief Construct a new SimpleLowpassFilter object
   */
  SimpleLowpassFilter() = default;

  /**
   * @brief Initialize the lowpass filter based on the config.
   * @param config Configuration struct.
   */
  explicit SimpleLowpassFilter(const Config &config);

  /**
   * @brief Set the time constant of the filter.
   * @param time_constant Time constant of the filter.
   */
  void set_time_constant(const float time_constant);

  /**
   * @brief Get the time constant of the filter.
   * @return Time constant of the filter.
   */
  float get_time_constant() const;

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input, time, and history.
   */
  float update(const float input);

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input, time, and history.
   */
  float operator()(float input);

  /**
   * @brief Reset the filter to its initial state.
   */
  void reset();

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

#include "simple_lowpass_filter_formatters.hpp"
