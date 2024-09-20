#pragma once

#include <stdlib.h>

#if defined(ESP_PLATFORM)
#include "esp_dsp.h"
#endif

#include "format.hpp"

namespace espp {
/**
 *  @brief Lowpass infinite impulse response (IIR) filter.
 */
class LowpassFilter {
public:
  /**
   *  @brief Configuration for the lowpass filter.
   */
  struct Config {
    float
        normalized_cutoff_frequency; /**< Filter cutoff frequency in the range [0.0, 0.5] (normalizd
                                        to sample frequency, = 2 * f_cutoff / f_sample). */
    float
        q_factor; /**< Quality (Q) factor of the filter. The higher the Q the better the filter. */
  };

  /**
   * @brief Default constructor.
   */
  LowpassFilter() = default;

  /**
   * @brief Initialize the lowpass filter coefficients based on the config.
   * @param config Configuration struct.
   */
  explicit LowpassFilter(const Config &config);

  /**
   * @brief Set the filter coefficients based on the config.
   * @param config Configuration struct.
   */
  void configure(const Config &config);

  /**
   * @brief Filter the input samples, updating internal state, and writing the
   *        filtered values to the data pointed to by output.
   * @param input Pointer to (floating point) array of new samples of the input data
   * @param output Pointer to (floating point) array which will be filled with
   *        the filtered input.
   * @param length Number of samples, should be >= length of input & output memory.
   * @note On ESP32, the input and output arrays must have
   *       __attribute__((aligned(16))) to ensure proper alignment for the ESP32
   *       DSP functions.
   */
  void update(const float *input, float *output, size_t length);

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float update(const float input);

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float operator()(float input);

  /**
   * @brief Reset the filter state to zero.
   */
  void reset();

  friend struct fmt::formatter<LowpassFilter>;

protected:
  void init(const Config &config);

  float coeffs_[5] = {0, 0, 0, 0, 0};
  float state_[5] = {0, 0, 0, 0, 0};
};
} // namespace espp

#include "lowpass_filter_formatters.hpp"
