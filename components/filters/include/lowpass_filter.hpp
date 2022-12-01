#pragma once

#include <stdlib.h>

#include "esp_dsp.h"

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
      float normalized_cutoff_frequency; /**< Filter cutoff frequency in the range [0.0, 0.5] (normalizd to sample frequency, = 2 * f_cutoff / f_sample). */
      float q_factor; /**< Quality (Q) factor of the filter. The higher the Q the better the filter. */
    };

    /**
     * @brief Initialize the lowpass filter coefficients based on the config.
     * @param config Configuration struct.

     */
    LowpassFilter(const Config& config) {
      init(config);
    }

    /**
     * @brief Filter the input samples, updating internal state, and writing the
     *        filtered values to the data pointed to by output.
     * @param input Pointer to (floating point) array of new samples of the input data
     * @param output Pointer to (floating point) array which will be filled with
     *        the filtered input.
     * @param length Number of samples, should be >= length of input & output memory.
     */
    void update(const float *input, float* output, size_t length) {
      dsps_biquad_f32(input, output, length, coeffs_, state_);
    }

    /**
     * @brief Filter the signal sampled by input, updating internal state, and
     *        returning the filtered output.
     * @param input New sample of the input data.
     * @return Filtered output based on input and history.
     */
    float update(const float input) {
      float output;
      dsps_biquad_f32(&input, &output, 1, coeffs_, state_);
      return output;
    }

    /**
     * @brief Filter the signal sampled by input, updating internal state, and
     *        returning the filtered output.
     * @param input New sample of the input data.
     * @return Filtered output based on input and history.
     */
    float operator() (float input) {
      return update(input);
    }

  protected:
    void init(const Config& config) {
      dsps_biquad_gen_lpf_f32(coeffs_, config.normalized_cutoff_frequency, config.q_factor);
    }

    float coeffs_[5];
    float state_[2];
  };
}
